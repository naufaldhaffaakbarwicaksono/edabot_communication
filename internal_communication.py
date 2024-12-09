import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from message_filters import ApproximateTimeSynchronizer, Subscriber, Cache
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Twist
from rcl_interfaces.msg import Log

import socket
import json
import threading
import configs
from configs import States
from datetime import datetime, timezone, timedelta
from utils import ros_msg_to_dict, dict_to_ros_msg
from edabot_protocol import send_packet, receive_packet


class RosPubHandler(Node):
    def __init__(self):
        super().__init__("edabot_com_pub")

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish_cmd_vel(self, payload: Twist):
        self.pub_cmd_vel.publish(payload)

    def check_topic(self, topic):
        topic_list = self.get_topic_names_and_types()
        topic_exists = any(topic[0] == topic for topic in topic_list)


class RosSubHandler(Node):
    def __init__(self):
        super().__init__("edabot_com_sub")

        sub_imu = Subscriber(self, Imu, "/imu")
        sub_odom = Subscriber(self, Odometry, "/odom")
        sub_local_costmap = Subscriber(
            self,
            OccupancyGrid,
            "/local_costmap/costmap"
            if not configs.is_simulation
            else "/local_costmap/costmap",
        )
        sub_global_costmap = Subscriber(
            self,
            OccupancyGrid,
            "/global_costmap/costmap"
            if not configs.is_simulation
            else "/global_costmap/costmap",
        )

        self.cache_local_costmap = Cache(sub_local_costmap, cache_size=1)
        self.cache_global_costmap = Cache(sub_global_costmap, cache_size=1)

        sync_topics = ApproximateTimeSynchronizer(
            [sub_imu, sub_odom], queue_size=10, slop=0.1
        )
        sync_topics.registerCallback(self.sync_callback)

        self.create_subscription(Log, "/rosout", self.rosout_callback, 10)
        self.latest_log = self.log_default

        self._json_data = {}

    def sync_callback(self, imu: Imu, odom: Odometry):
        local_costmap: OccupancyGrid = self.cache_local_costmap.getLast()
        global_costmap: OccupancyGrid = self.cache_global_costmap.getLast()
        if not all([imu, odom, local_costmap, global_costmap]):
            self.get_logger().warn("Missing data in callback")
            return

        now = datetime.now(timezone.utc)
        if now - self.latest_log["timestamp"] > timedelta(seconds=1):
            self.latest_log = self.clean_log()

        # If not all data is available, send empty fields for missing data
        if not all([imu, odom]):
            self.get_logger().warn("Missing imu or odom data in callback")
            self._json_data = {
                "type": "status",
                "agentId": f"{configs.robot_id}",
                "timestamp": now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z",
                "log": self.latest_log,
                "payload": {
                    "imu": ros_msg_to_dict(imu) if imu else "",
                    "odom": ros_msg_to_dict(odom) if odom else "",
                    "local_costmap": "",
                    "global_costmap": "",
                },
            }
            return

        if not all([local_costmap, global_costmap]):
            self.get_logger().warn("Missing costmap data in callback")
            self._json_data = {
                "type": "status",
                "agentId": f"{configs.robot_id}",
                "timestamp": now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z",
                "log": self.latest_log,
                "payload": {
                    "imu": ros_msg_to_dict(imu),
                    "odom": ros_msg_to_dict(odom),
                    "local_costmap": "",
                    "global_costmap": "",
                },
            }
            return

        # All data is available, include the latest log
        self._json_data = {
            "type": "status",
            "agentId": f"{configs.robot_id}",
            "timestamp": now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z",
            "log": self.latest_log,
            "payload": {
                "imu": ros_msg_to_dict(imu),
                "odom": ros_msg_to_dict(odom),
                "local_costmap": ros_msg_to_dict(local_costmap),
                "global_costmap": ros_msg_to_dict(global_costmap),
            },
        }

    def rosout_callback(self, msg: Log):
        if msg.level >= configs.log_level:
            self.latest_log = {
                "level": msg.level,
                "msg": msg.msg,
                "name": msg.name,
                "timestamp": datetime.now(timezone.utc),
            }

    def clean_log(self):
        return {
            "level": 0,
            "msg": "",
            "name": "",
            "timestamp": datetime.now(timezone.utc),
        }

    @property
    def json_data(self):
        data = self._json_data.copy()
        self._json_data.clear()
        return data


class InternalCom:
    def __init__(self, no_thread=False):
        # Initialize the client socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.client_socket.bind((configs.orin_ip, configs.orin_port))
        if configs.bypass_integrity:
            self.client_socket.connect((configs.raspi_ip_internal, configs.raspi_port_internal))
        else:
            self.client_socket.connect((configs.zinq_ip, configs.zinq_port_secure))

        # Initialize ROS 2 context once globally
        rclpy.init()
        self.publisher_node = RosPubHandler()
        self.subscriber_node = RosSubHandler()

        # Start threads for receiving and sending messages
        self.start()

    def start(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self.publisher_node)
        executor.add_node(self.subscriber_node)

        threading.Thread(target=self.receive_messages, daemon=True).start()
        threading.Thread(target=self.send_messages, daemon=True).start()

        try:
            # Let the executor manage spinning
            executor.spin()
        except KeyboardInterrupt:
            print("Shutting down communication.")
        finally:
            executor.shutdown()
            self.client_socket.close()

    def receive_messages(self):
        try:
            latest_message = None
            while True:
                message = receive_packet(self.client_socket)
                if message:
                    latest_message = json.loads(message)

                if latest_message:
                    cmd_vel = dict_to_ros_msg(Twist, latest_message)
                    self.publisher_node.publish_cmd_vel(cmd_vel)
                    print(f"Processed Latest Server Data: {latest_message}")
                    latest_message = None
        except (ConnectionResetError, KeyboardInterrupt) as e:
            default_msg = Twist()
            self.publisher_node.publish_cmd_vel(default_msg)
            print(f"Receive error: {e}")
        finally:
            print("Server disconnected.")

    def send_messages(self):
        try:
            while True:
                if configs.state == States.INIT:
                    continue

                data = json.dumps(self.subscriber_node.json_data)
                if data.encode() != b"{}":
                    if not send_packet(self.client_socket, data):
                        print("Send Failed")
        except (ConnectionResetError, BrokenPipeError, KeyboardInterrupt) as e:
            print(f"Send error: {e}")
        finally:
            print("Connection closed.")


if __name__ == "__main__":
    internal_com = InternalCom()
