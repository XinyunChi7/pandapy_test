import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import numpy as np
import transforms3d as t3d

hostname = '192.168.1.11'
username = 'panda'
password = 'panda1234'
import logging
logging.basicConfig(level=logging.INFO)

import panda_py
desk = panda_py.Desk(hostname, username, password, platform="fr3")
desk.unlock()
desk.activate_fci()

from panda_py import libfranka
panda = panda_py.Panda(hostname)
gripper = libfranka.Gripper(hostname)


'''
Move_to_joint_position()
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
        self.xyz_scale = 100.0
        # panda.move_to_start()
        self.moved = False
        # Create a publisher to publish the 4x4 transformation matrix
        self.matrix_publisher = self.create_publisher(
            Float64MultiArray,
            '/transformation_matrix',  # Topic to publish the 4x4 matrix
            10  # Queue size
        )

    def pose_callback(self, msg: Pose):
        # Extract position and orientation (quaternion) from the Pose message

        ori_q = panda.get_orientation() # (quaternion) 
        pos = panda.get_position() 
        
        if self.moved == False:
            self.moved = True
            self.get_logger().info(f"message pos {msg.position}, ori {msg.orientation}")

            delta_pos = np.array([msg.position.x, msg.position.y, msg.position.z])

            ori_r, ori_p, ori_y = t3d.euler.quat2euler(ori_q)
            # Extract the quaternion (a, b, c, w)
            self.get_logger().info(f"initial pos {pos}, ori {ori_q}")
    
            delta_ori= [msg.orientation.x, msg.orientation.y, msg.orientation.z]
            ori_rpy = [ori_r, ori_p, ori_y] + delta_ori 

            orientation = t3d.euler.euler2quat(ori_rpy[0], ori_rpy[1], ori_rpy[2])
            position = np.array(pos) + delta_pos/self.xyz_scale 

            orientation = ori_q + [0.0, 0.0, 0.0, 0.1]

            pos = np.array(position)
            ori = np.array(orientation)

            q = panda_py.ik(pos, ori)
            print("Solved IK", q)

            panda.move_to_joint_position(q, speed_factor = 0.01)
            

            ori = panda.get_orientation()
            pos = panda.get_position() 

            self.get_logger().info(f"after move pos {pos}, ori {ori}")
            


    
    



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
