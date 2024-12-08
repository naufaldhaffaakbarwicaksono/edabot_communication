import rclpy
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, Subscriber, Cache
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Twist

import socket
import json
import threading
import configs
from utils import ros_msg_to_dict, dict_to_ros_msg
from edabot_protocol import send_packet, receive_packet


class RosPubHandler(Node):
    def __init__(self):
        super().__init__("edabot_com_pub")

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish_cmd_vel(self, payload):
        data = Twist()

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

        self._json_data = {}

    def sync_callback(self, imu: Imu, odom: Odometry):
        local_costmap: OccupancyGrid = self.cache_local_costmap.getLast()
        global_costmap: OccupancyGrid = self.cache_global_costmap.getLast()
        if not all([imu, odom, local_costmap, global_costmap]):
            self.get_logger().warn("Missing data in callback")
            return

        self._json_data = {
            "imu": ros_msg_to_dict(imu),
            "odom": ros_msg_to_dict(odom),
            "local_costmap": ros_msg_to_dict(local_costmap),
            "global_costmap": ros_msg_to_dict(global_costmap),
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
        self.client_socket.connect((configs.zinq_ip, configs.zinq_port))

        # Initialize ROS 2 context once globally
        rclpy.init()
        self.publisher_node = RosPubHandler()
        self.subscriber_node = RosSubHandler()

        # Start threads for receiving and sending messages
        if not no_thread:
            threading.Thread(target=self.receive_messages).start()
            threading.Thread(target=self.send_messages).start()

    def cleanup(self):
        """Cleanup resources."""
        try:
            self.publisher_node.destroy_node()
            self.subscriber_node.destroy_node()
        finally:
            rclpy.shutdown()
            self.client_socket.close()

    def receive_messages(self):
        try:
            while True:
                message = receive_packet(self.client_socket)
                if not message:
                    rclpy.spin_once(self.publisher_node, timeout_sec=1)
                    continue
                self.publisher_node.publish(message)
                rclpy.spin_once(self.publisher_node, timeout_sec=1)
                print(f"Server: {message}")
        except (ConnectionResetError, KeyboardInterrupt) as e:
            print(f"Receive error: {e}")
        finally:
            print("Server disconnected.")

    def send_messages(self):
        try:
            while True:
                data = json.dumps(self.subscriber_node.json_data)
                if data.encode() != b"{}":
                    if not send_packet(self.client_socket, data):
                        print("Send Failed")
                rclpy.spin_once(self.subscriber_node, timeout_sec=3)
        except (ConnectionResetError, BrokenPipeError, KeyboardInterrupt) as e:
            print(f"Send error: {e}")
        finally:
            print("Connection closed.")


if __name__ == "__main__":
    internal_com = InternalCom()
