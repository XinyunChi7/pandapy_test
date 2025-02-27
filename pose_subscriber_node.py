import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import numpy as np

'''
4 by 4 transformation matrix for IK solver
'''

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__('pose_subscriber_node')

        # Create a subscriber to listen to the Pose messages
        self.subscription = self.create_subscription(
            Pose,
            '/pose_topic',  # Topic to subscribe to
            self.pose_callback,
            10  # Queue size
        )

        # Create a publisher to publish the 4x4 transformation matrix
        self.matrix_publisher = self.create_publisher(
            Float64MultiArray,
            '/transformation_matrix',  # Topic to publish the 4x4 matrix
            10  # Queue size
        )

    def pose_callback(self, msg: Pose):
        # Extract position and orientation (quaternion) from the Pose message
        position = msg.position
        orientation = msg.orientation

        # Extract the position (x, y, z)
        x, y, z = position.x, position.y, position.z

        # Extract the quaternion (a, b, c, w)
        a, b, c, w = orientation.x, orientation.y, orientation.z, orientation.w

        # Convert quaternion to a rotation matrix
        rot_matrix = self.quaternion_to_rotation_matrix(a, b, c, w)

        # Create the 4x4 transformation matrix
        transformation_matrix = np.eye(4)  # Start with an identity matrix

        # Fill the rotation part (3x3 matrix)
        transformation_matrix[:3, :3] = rot_matrix

        # Fill the position part (translation)
        transformation_matrix[0, 3] = x
        transformation_matrix[1, 3] = y
        transformation_matrix[2, 3] = z

        # Convert the matrix to a flat list (1D array) to publish
        matrix_data = Float64MultiArray()
        matrix_data.data = transformation_matrix.flatten().tolist()  # Flatten to 1D list

        # Publish the transformation matrix
        self.matrix_publisher.publish(matrix_data)

        # Log the transformation matrix for debugging
        self.get_logger().info(f'Published Transformation Matrix:\n{transformation_matrix}')

    def quaternion_to_rotation_matrix(self, a, b, c, w):
        # Calculate the 3x3 rotation matrix from quaternion (a, b, c, w)
        # Quaternion to Rotation Matrix formula:
        R = np.array([
            [1 - 2 * (b**2 + c**2), 2 * (a * b - c * w), 2 * (a * c + b * w)],
            [2 * (a * b + c * w), 1 - 2 * (a**2 + c**2), 2 * (b * c - a * w)],
            [2 * (a * c - b * w), 2 * (b * c + a * w), 1 - 2 * (a**2 + b**2)]
        ])
        return R


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the PoseSubscriberNode
    node = PoseSubscriberNode()

    # Spin the node to keep listening for messages
    rclpy.spin(node)

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
