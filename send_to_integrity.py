import rclpy
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, Subscriber, Cache
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Pose

import socket
import json
import threading
from dotenv import load_dotenv
import os
import pprint

load_dotenv()


def ros_message_to_dict(msg):
    """
    Recursively convert a ROS 2 message to a dictionary.
    """
    result = {}
    for field_name in msg.get_fields_and_field_types():
        value = getattr(msg, field_name)
        # If the value is a ROS message, recursively convert it
        if hasattr(value, "get_fields_and_field_types"):
            result[field_name] = ros_message_to_dict(value)
        # If the value is an array, handle each element
        elif isinstance(value, (list, tuple)):
            result[field_name] = [
                ros_message_to_dict(item)
                if hasattr(item, "get_fields_and_field_types")
                else item
                for item in value
            ]
        else:
            result[field_name] = value
    return result


class SendToIntegrity(Node):
    def __init__(self):
        super().__init__("topic_to_tcp_node")

        tcp_ip = os.getenv("TCP_IP", "127.0.0.1")
        tcp_port = int(os.getenv("TCP_PORT", 5000))

        self.server_address = (tcp_ip, tcp_port)
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_client.connect(self.server_address)
        self.get_logger().info(f"Connected to TCP server at {self.server_address}")

        self.sub_imu = Subscriber(self, Imu, "/imu")
        self.sub_odom = Subscriber(self, Odometry, "/odom")
        self.sub_local_costmap = Subscriber(
            self, OccupancyGrid, "/local_costmap/costmap"
        )
        self.sub_global_costmap = Subscriber(
            self, OccupancyGrid, "/global_costmap/costmap"
        )

        self.cache_local_costmap = Cache(self.sub_local_costmap, cache_size=1)
        self.cache_global_costmap = Cache(self.sub_global_costmap, cache_size=1)

        self.sync_topics = ApproximateTimeSynchronizer(
            [self.sub_imu, self.sub_odom], queue_size=10, slop=0.1
        )
        self.sync_topics.registerCallback(self.sync_callback)

        self.json_data = {}
        self.lock = threading.Lock()

    def sync_callback(self, imu: Imu, odom: Odometry):
        local_costmap: OccupancyGrid = self.cache_local_costmap.getLast()
        global_costmap: OccupancyGrid = self.cache_global_costmap.getLast()
        if (
            imu is None
            or odom is None
            or local_costmap is None
            or global_costmap is None
        ):
            return

        with self.lock:
            self.json_data = {
                "imu": {
                    "orientation": ros_message_to_dict(imu.orientation),
                    "angular_velocity": ros_message_to_dict(imu.angular_velocity),
                    "linear_acceleration": ros_message_to_dict(imu.linear_acceleration),
                },
                "odom": {
                    "pose": ros_message_to_dict(odom.pose.pose),
                    "twist": ros_message_to_dict(odom.twist.twist),
                },
                "local_costmap": {
                    "info": ros_message_to_dict(local_costmap.info),
                    "data": list(local_costmap.data),
                },
                "global_costmap": {
                    "info": ros_message_to_dict(global_costmap.info),
                    "data": list(global_costmap.data),
                },
            }
            self.save_and_send()

    def save_and_send(self):
        try:
            file_content = json.dumps(self.json_data)

            self.tcp_client.sendall(file_content.encode("utf-8"))

            self.json_data.clear()
        except Exception as e:
            self.get_logger().error(f"Error saving or sending data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SendToIntegrity()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
