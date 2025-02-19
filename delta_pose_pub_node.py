import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        
        # Create a publisher that publishes Pose messages to the '/pose_topic'
        self.publisher = self.create_publisher(Pose, '/pose_topic', 10)
        
        # Timer to publish the pose every 1 second
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        # Create a Pose message
        msg = Pose()

        # Set position (x, y, z)
        msg.position.x = 0.3076
        msg.position.y = 0.0001
        msg.position.z = 0.4365
        
        # Set orientation as quaternion (a, b, c, w)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0  # This is the identity quaternion (no rotation)

        # Publish the Pose message
        self.publisher.publish(msg)

        # Log to show the message being published
        self.get_logger().info(f'Publishing Pose: Position: ({msg.position.x}, {msg.position.y}, {msg.position.z}), '
                               f'Orientation: ({msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w})')


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the PosePublisherNode
    node = PosePublisherNode()

    # Spin the node to keep publishing messages
    rclpy.spin(node)

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
