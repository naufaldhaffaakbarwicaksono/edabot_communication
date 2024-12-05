import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler

class GoalPoseConverter(Node):
    def __init__(self):
        super().__init__('goal_pose_converter')
        
        # Subscriber for /send_goal_pose
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/send_goal_pose',
            self.goal_pose_callback,
            10)
        
        # Publisher for /goal_pose
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def goal_pose_callback(self, msg):
        # Extract position and heading from the message
        x, y, heading = msg.data
        
        # Create PoseStamped message for /goal_pose
        pose_msg = PoseStamped()
        
        # Fill in header information
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # You can change this to your desired frame
        
        # Set position
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0  # Assuming a 2D plane

        # Set orientation from heading (yaw angle)
        quaternion = quaternion_from_euler(0, 0, heading)
        pose_msg.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        # Publish the PoseStamped message to /goal_pose
        self.publisher.publish(pose_msg)
        self.get_logger().info(f'Published to /goal_pose: Position ({x}, {y}, 0.0) Orientation ({quaternion[0]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]})')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseConverter()
    rclpy.spin(node)
    
    # Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
