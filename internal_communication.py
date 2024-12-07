import rclpy
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, Subscriber, Cache
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid

from utils import ros_message_to_dict
import socket
import json
import multiprocessing
import configs
from edabot_protocol import create_packet


class RosHandler(Node):
    def __init__(self):
        super().__init__("topic_to_tcp_node")
        sub_imu = Subscriber(self, Imu, "/imu")
        sub_odom = Subscriber(self, Odometry, "/odom")
        sub_local_costmap = Subscriber(
            self,
            OccupancyGrid,
            "/local_costmap/costmap"
            if configs.env == "production"
            else "/local_costmap/costmap",
        )
        sub_global_costmap = Subscriber(
            self,
            OccupancyGrid,
            "/global_costmap/costmap"
            if configs.env == "production"
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

    @property
    def json_data(self):
        data = self._json_data.copy()
        self._json_data.clear()
        return data


class InternalCom:
    def __init__(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        client_socket.bind(("0.0.0.0", 12341))
        client_socket.connect(("127.0.0.1", 9999))

        # Provide a name for this client
        client_name = "orin"
        client_socket.send(client_name.encode("utf-8"))
        print(f"Connected to server as {client_name}")

        # Create separate processes for receiving and sending messages
        receive_process = multiprocessing.Process(
            target=self.receive_messages, args=(client_socket,)
        )
        send_process = multiprocessing.Process(
            target=self.send_messages, args=(client_socket,)
        )

        receive_process.start()
        send_process.start()

        # Wait for both processes to complete
        receive_process.join()
        send_process.join()

    def receive_messages(self, sock):
        while True:
            try:
                message = sock.recv(1024).decode("utf-8")
                if not message:
                    break
                print(f"Server: {message}")
            except ConnectionResetError:
                break
        print("Server disconnected.")
        sock.close()

    def send_messages(self, sock):
        try:
            rclpy.init()
            node = RosHandler()
            while True:
                try:
                    rclpy.spin_once(node, timeout_sec=1)
                    data = node.json_data
                    if data:
                        sock.sendall(data.encode())
                except (ConnectionResetError, BrokenPipeError):
                    break
        except KeyboardInterrupt:
            node.get_logger().info("Node stopped by user")
        finally:
            node.destroy_node()
            rclpy.shutdown()
        print("Connection closed.")
        sock.close()


if __name__ == "__main__":
    internal_com = InternalCom()
