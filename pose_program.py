import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class PoseRelayNode(Node):
    def __init__(self):
        super().__init__('pose_relay_node')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10)
        self.subscription  

        self.publisher = self.create_publisher(PoseStamped, '/zed/zed_node/pose', 10)

    def amcl_pose_callback(self, msg):
        # Membuat pesan PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose.position.x = msg.pose.pose.position.x
        pose_stamped.pose.position.y = msg.pose.pose.position.y
        pose_stamped.pose.position.z = msg.pose.pose.position.z
        pose_stamped.pose.orientation.x = msg.pose.pose.orientation.x
        pose_stamped.pose.orientation.y = msg.pose.pose.orientation.y
        pose_stamped.pose.orientation.z = msg.pose.pose.orientation.z
        pose_stamped.pose.orientation.w = msg.pose.pose.orientation.w

        # Publish pesan ke /zed/zed_node/pose
        self.publisher.publish(pose_stamped)
        self.get_logger().info("Relayed pose from /amcl_pose to /zed/zed_node/pose")

def main(args=None):
    rclpy.init(args=args)
    node = PoseRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
