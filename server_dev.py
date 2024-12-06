import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Pose
import json
import socket
import threading

# Read IP and port from server_ip.txt
with open("server_ip.txt", "r") as f:
    SERVER_IP = f.readline().strip()
    SERVER_PORT = int(f.readline().strip())

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
print(f"Binding {SERVER_IP}:{SERVER_PORT}")
sock.bind((SERVER_IP, SERVER_PORT))
sock.listen(1)
try:
    conn, addr = sock.accept()
    client_socket = conn
except socket.error as e:
    print(f"Socket error: {e}")
    exit(1)

default_control_json = {"command": "keyboard", "linear_x": 0.0, "angular_z": 0.0}

control_json = default_control_json.copy()

# Global lock for control_json
control_json_lock = threading.Lock()

# Global lock for client_socket
client_socket_lock = threading.Lock()


class StatusSubscriber(Node):
    def __init__(self):
        super().__init__("status_subscriber")
        self.last_imu_msg = None
        self.last_odom_msg = None
        self.last_cmd_vel_msg = None
        self.last_map_msg = None
        self.last_pose_msg = None
        self.map_received = False
        self.robot_data_json = None
        self.previous_robot_data_json = None

        callback_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Imu, "imu", self.imu_callback, 10, callback_group=callback_group
        )
        self.create_subscription(
            Odometry, "odom", self.odom_callback, 10, callback_group=callback_group
        )
        self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10, callback_group=callback_group
        )
        self.create_subscription(
            OccupancyGrid, "map", self.map_callback, 10, callback_group=callback_group
        )
        self.create_subscription(
            PoseStamped,
            "/zed/zed_node/pose",
            self.pose_callback,
            10,
            callback_group=callback_group,
        )

        self.create_timer(0.5, self.timer_callback, callback_group=callback_group)

    def imu_callback(self, msg):
        self.last_imu_msg = msg
        self.get_logger().info("IMU data received")

    def odom_callback(self, msg):
        self.last_odom_msg = msg
        self.get_logger().info("Odometry data received")

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel_msg = msg
        self.get_logger().info("Cmd_vel data received")

    def map_callback(self, msg):
        self.last_map_msg = msg
        if not self.map_received:
            self.map_received = True
            self.get_logger().info("Map data received and cached for the first time.")
        self.get_logger().info("Map data received")

    def pose_callback(self, msg):
        self.last_pose_msg = msg
        self.get_logger().info("Pose data received")

    def timer_callback(self):
        self.get_logger().info("Getting robot data...")
        self.get_robot_data_json()

    def get_robot_data_json(self):
        imu_data = {
            "orientation": {
                "x": self.last_imu_msg.orientation.x if self.last_imu_msg else 0.0,
                "y": self.last_imu_msg.orientation.y if self.last_imu_msg else 0.0,
                "z": self.last_imu_msg.orientation.z if self.last_imu_msg else 0.0,
                "w": self.last_imu_msg.orientation.w if self.last_imu_msg else 0.0,
            },
            "angular_velocity": {
                "x": self.last_imu_msg.angular_velocity.x if self.last_imu_msg else 0.0,
                "y": self.last_imu_msg.angular_velocity.y if self.last_imu_msg else 0.0,
                "z": self.last_imu_msg.angular_velocity.z if self.last_imu_msg else 0.0,
            },
            "linear_acceleration": {
                "x": self.last_imu_msg.linear_acceleration.x
                if self.last_imu_msg
                else 0.0,
                "y": self.last_imu_msg.linear_acceleration.y
                if self.last_imu_msg
                else 0.0,
                "z": self.last_imu_msg.linear_acceleration.z
                if self.last_imu_msg
                else 0.0,
            },
        }

        odometry_data = {
            "pose": {
                "position": {
                    "x": self.last_odom_msg.pose.pose.position.x
                    if self.last_odom_msg
                    else 0.0,
                    "y": self.last_odom_msg.pose.pose.position.y
                    if self.last_odom_msg
                    else 0.0,
                    "z": self.last_odom_msg.pose.pose.position.z
                    if self.last_odom_msg
                    else 0.0,
                },
                "orientation": {
                    "x": self.last_odom_msg.pose.pose.orientation.x
                    if self.last_odom_msg
                    else 0.0,
                    "y": self.last_odom_msg.pose.pose.orientation.y
                    if self.last_odom_msg
                    else 0.0,
                    "z": self.last_odom_msg.pose.pose.orientation.z
                    if self.last_odom_msg
                    else 0.0,
                    "w": self.last_odom_msg.pose.pose.orientation.w
                    if self.last_odom_msg
                    else 0.0,
                },
            },
            "twist": {
                "linear": {
                    "x": self.last_odom_msg.twist.twist.linear.x
                    if self.last_odom_msg
                    else 0.0,
                    "y": self.last_odom_msg.twist.twist.linear.y
                    if self.last_odom_msg
                    else 0.0,
                    "z": self.last_odom_msg.twist.twist.linear.z
                    if self.last_odom_msg
                    else 0.0,
                },
                "angular": {
                    "x": self.last_odom_msg.twist.twist.angular.x
                    if self.last_odom_msg
                    else 0.0,
                    "y": self.last_odom_msg.twist.twist.angular.y
                    if self.last_odom_msg
                    else 0.0,
                    "z": self.last_odom_msg.twist.twist.angular.z
                    if self.last_odom_msg
                    else 0.0,
                },
            },
        }

        cmd_vel_data = {
            "linear": {
                "x": self.last_cmd_vel_msg.linear.x if self.last_cmd_vel_msg else 0.0,
                "y": self.last_cmd_vel_msg.linear.y if self.last_cmd_vel_msg else 0.0,
                "z": self.last_cmd_vel_msg.linear.z if self.last_cmd_vel_msg else 0.0,
            },
            "angular": {
                "x": self.last_cmd_vel_msg.angular.x if self.last_cmd_vel_msg else 0.0,
                "y": self.last_cmd_vel_msg.angular.y if self.last_cmd_vel_msg else 0.0,
                "z": self.last_cmd_vel_msg.angular.z if self.last_cmd_vel_msg else 0.0,
            },
        }

        map_data = {
            "info": {
                "resolution": self.last_map_msg.info.resolution
                if self.last_map_msg
                else 0.0,
                "width": self.last_map_msg.info.width if self.last_map_msg else 0,
                "height": self.last_map_msg.info.height if self.last_map_msg else 0,
                "origin": {
                    "position": {
                        "x": self.last_map_msg.info.origin.position.x
                        if self.last_map_msg
                        else 0.0,
                        "y": self.last_map_msg.info.origin.position.y
                        if self.last_map_msg
                        else 0.0,
                        "z": self.last_map_msg.info.origin.position.z
                        if self.last_map_msg
                        else 0.0,
                    },
                    "orientation": {
                        "x": self.last_map_msg.info.origin.orientation.x
                        if self.last_map_msg
                        else 0.0,
                        "y": self.last_map_msg.info.origin.orientation.y
                        if self.last_map_msg
                        else 0.0,
                        "z": self.last_map_msg.info.origin.orientation.z
                        if self.last_map_msg
                        else 0.0,
                        "w": self.last_map_msg.info.origin.orientation.w
                        if self.last_map_msg
                        else 0.0,
                    },
                },
            },
            "data": list(self.last_map_msg.data) if self.last_map_msg else [],
        }

        pose_data = {
            "header": {
                "stamp": {
                    "sec": self.last_pose_msg.header.stamp.sec
                    if self.last_pose_msg
                    else 0,
                    "nanosec": self.last_pose_msg.header.stamp.nanosec
                    if self.last_pose_msg
                    else 0,
                },
                "frame_id": self.last_pose_msg.header.frame_id
                if self.last_pose_msg
                else "",
            },
            "pose": {
                "position": {
                    "x": self.last_pose_msg.pose.position.x
                    if self.last_pose_msg
                    else 0.0,
                    "y": self.last_pose_msg.pose.position.y
                    if self.last_pose_msg
                    else 0.0,
                    "z": self.last_pose_msg.pose.position.z
                    if self.last_pose_msg
                    else 0.0,
                },
                "orientation": {
                    "x": self.last_pose_msg.pose.orientation.x
                    if self.last_pose_msg
                    else 0.0,
                    "y": self.last_pose_msg.pose.orientation.y
                    if self.last_pose_msg
                    else 0.0,
                    "z": self.last_pose_msg.pose.orientation.z
                    if self.last_pose_msg
                    else 0.0,
                    "w": self.last_pose_msg.pose.orientation.w
                    if self.last_pose_msg
                    else 0.0,
                },
            },
        }

        robot_data = {
            "imu": imu_data,
            "odometry": odometry_data,
            "cmd_vel": cmd_vel_data,
            "map": map_data,
            "position_robot": pose_data,
        }

        pose_to_other_robot = {
            "command": "other",
            "robot": "pose_robot1",  # harusnya dari file txt namanya
            "pose": pose_data,
        }

        self.robot_data_json = json.dumps(robot_data)
        self.get_logger().debug("Robot data processed")


class HandleReceive(Node):
    def __init__(self, status_subscriber):
        super().__init__("handle_receive")
        self.status_subscriber = status_subscriber
        callback_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.5, self.timer_callback, callback_group=callback_group)
        self.get_logger().info(f"Connection established with {addr}")
        self.send_buffer = b""

    def timer_callback(self):
        self.get_logger().info("Sending robot data...")
        if self.status_subscriber.robot_data_json:
            current_data = self.status_subscriber.robot_data_json.encode("utf-8")
            header = b"\xfe\x01"
            payload = current_data
            payload_length = len(payload).to_bytes(4, byteorder="big")
            data_send = header + payload_length + payload
            self.send_data(data_send)
            self.status_subscriber.previous_robot_data_json = (
                self.status_subscriber.robot_data_json
            )
            self.get_logger().info("Robot data sent")

    def send_data(self, data):
        self.send_buffer += data
        while self.send_buffer:
            try:
                with client_socket_lock:
                    sent = conn.send(self.send_buffer)
                self.send_buffer = self.send_buffer[sent:]
            except BlockingIOError:
                break


class ReceivePublish(Node):
    def __init__(self):
        super().__init__("data_sender")
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pose_publisher = self.create_publisher(
            Float64MultiArray, "/send_goal_pose", 10
        )
        self.dynamic_publishers = {}
        self.first_input_received = False
        callback_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.5, self.timer_callback, callback_group=callback_group)

    def timer_callback(self):
        global control_json, default_control_json
        self.get_logger().debug("Publishing control data...")
        with control_json_lock:
            if not self.first_input_received:
                self.process_control_data(default_control_json)
            else:
                self.process_control_data(control_json)

    def process_control_data(self, control_json):
        if control_json["command"] == "keyboard":
            twist = Twist()
            twist.linear.x = float(control_json.get("linear_x", 0.0))
            twist.angular.z = float(control_json.get("angular_z", 0.0))
            self.cmd_vel_publisher.publish(twist)
            self.first_input_received = True
        elif control_json["command"] == "coordinate":
            goal_pose = Float64MultiArray()
            goal_pose.data = [
                float(control_json.get("coordinate_x", 0.0)),
                float(control_json.get("coordinate_y", 0.0)),
                float(control_json.get("heading", 0.0)),
            ]
            self.goal_pose_publisher.publish(goal_pose)
        elif control_json["command"] == "other":
            robot_topic = f"/{control_json['robot']}"
            if robot_topic not in self.dynamic_publishers:
                self.dynamic_publishers[robot_topic] = self.create_publisher(
                    Pose, robot_topic, 10
                )
            pose_msg = Pose()
            pose_msg.position.x = control_json["pose"]["position"]["x"]
            pose_msg.position.y = control_json["pose"]["position"]["y"]
            pose_msg.position.z = control_json["pose"]["position"]["z"]
            pose_msg.orientation.x = control_json["pose"]["orientation"]["x"]
            pose_msg.orientation.y = control_json["pose"]["orientation"]["y"]
            pose_msg.orientation.z = control_json["pose"]["orientation"]["z"]
            pose_msg.orientation.w = control_json["pose"]["orientation"]["w"]
            self.dynamic_publishers[robot_topic].publish(pose_msg)
            self.get_logger().info(f"Published pose data to {robot_topic}")
        else:
            self.get_logger().info("Unknown control data type.")


class ReceiveControl(Node):
    def __init__(self):
        super().__init__("receive_control")
        callback_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.1, self.receive_data, callback_group=callback_group)
        client_socket.setblocking(False)
        self.get_logger().info("ReceiveControl node started")

    def receive_data(self):
        global control_json
        try:
            with client_socket_lock:
                data_forward_tcp = client_socket.recv(4096)
            if data_forward_tcp:
                control_data = json.loads(data_forward_tcp.decode("utf-8"))
                with control_json_lock:
                    control_json = control_data
                self.get_logger().info("Control data received and updated")
        except BlockingIOError:
            pass
        except json.JSONDecodeError:
            self.get_logger().error("Received invalid JSON data")
        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")


def main(args=None):
    rclpy.init(args=args)
    status_subscriber = StatusSubscriber()
    handle_receive = HandleReceive(status_subscriber)
    receive_publish = ReceivePublish()
    receive_control = ReceiveControl()
    executor = MultiThreadedExecutor()
    executor.add_node(status_subscriber)
    executor.add_node(handle_receive)
    executor.add_node(receive_publish)
    executor.add_node(receive_control)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
