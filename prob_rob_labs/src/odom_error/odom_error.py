import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Vector3
import numpy as np
import math

from tf_transformations import euler_from_quaternion


heartbeat_period = 0.05

class OdomError(Node):

    def __init__(self):
        super().__init__('odom_error')
        self.log = self.get_logger()
        self.create_subscription(PoseStamped, '/tb3/ground_truth/pose', self.ground_truth_pose_callback, 10)
        self.create_subscription(Odometry, '/ekf_odom', self.ekf_odom_callback, 10)

        self.pub_odom_error = self.create_publisher(Vector3, '/odom_error', 10)

        self.ground_truth_pose = None
        self.ground_truth_orientation = None
        self.ekf_odom_pose = None
        self.ekf_odom_orientation = None

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def ground_truth_pose_callback(self, msg):
        self.ground_truth_pose = msg.pose
        self.ground_truth_orientation = msg.pose.orientation
        self.log.info(f"the ground_truth_timestamp is {msg.header.stamp}")
        # self.log.info(f"the ground_truth_pose is {self.ground_truth_pose}")

    def ekf_odom_callback(self, msg):
        self.ekf_odom_pose = msg.pose.pose
        self.ekf_odom_orientation = msg.pose.pose.orientation
        self.log.info(f"the ekf_odom_timestamp is {msg.header.stamp}")
        # self.log.info(f"the ekf_odom_pose is {self.ekf_odom_pose}")    
    
    def heartbeat(self):
        # Check if both poses are available
        if (self.ground_truth_pose is None or self.ground_truth_orientation is None or 
            self.ekf_odom_pose is None or self.ekf_odom_orientation is None):
            return
            
        # cal the euclidean distance
        x_error = self.ground_truth_pose.position.x - self.ekf_odom_pose.position.x
        y_error = self.ground_truth_pose.position.y - self.ekf_odom_pose.position.y
        euc_dis = np.sqrt(x_error**2 + y_error**2)

        # cal the orientation error
        # Convert quaternion to yaw angle
        yaw_gt_quat = [self.ground_truth_orientation.x, 
                       self.ground_truth_orientation.y, 
                       self.ground_truth_orientation.z, 
                       self.ground_truth_orientation.w]
        yaw_ekf_quat = [self.ekf_odom_orientation.x, 
                        self.ekf_odom_orientation.y, 
                        self.ekf_odom_orientation.z, 
                        self.ekf_odom_orientation.w]

        yaw_gt = euler_from_quaternion(yaw_gt_quat)[2]
        yaw_ekf = euler_from_quaternion(yaw_ekf_quat)[2]
        
        yaw_error = (yaw_ekf - yaw_gt + np.pi) % (2 * np.pi) - np.pi

        # self.log.info(f"the euclidean distance is {euc_dis}")
        # self.log.info(f"the orientation error is {yaw_error}")
        
        # Publish error message
        error_msg = Vector3()
        error_msg.x = euc_dis
        error_msg.y = 0.0
        error_msg.z = yaw_error
        self.pub_odom_error.publish(error_msg)

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    odom_error = OdomError()
    odom_error.spin()
    odom_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
