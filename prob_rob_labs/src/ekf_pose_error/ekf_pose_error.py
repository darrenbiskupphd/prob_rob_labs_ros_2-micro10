import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3


heartbeat_period = 0.1

class EkfPoseError(Node):

    def __init__(self):
        super().__init__('ekf_pose_error')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.gt_sub = self.create_subscription(PoseStamped, '/tb3/ground_truth/pose', self.gt_callback, 10)
        self.ekf_sub = self.create_subscription(PoseStamped, '/ekf_pose', self.ekf_pose_callback, 10)
        self.gt_pose = None
        self.ekf_pose = None

        self.error_pub = self.create_publisher(Vector3, '/ekf_error', 10)

    def gt_callback(self, msg):
        self.gt_pose = msg

    def ekf_pose_callback(self, msg): ## this code is driven by ekf pose publisher timing
        self.ekf_pose = msg
        if self.gt_pose is not None:
            self.calculate_error()

    def theta_from_quat(self, q):
        siny_cosp = 2*(q.w*q.z+q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    #this will also pub to /ekf_error topic
    def calculate_error(self):
        error_x = self.gt_pose.position.x - self.ekf_pose.position.x
        error_y = self.gt_pose.position.y - self.ekf_pose.position.y
        error_theta = self.theta_from_quat(self.gt_pose.orientation) - self.theta_from_quat(self.ekf_pose.orientation)

        msg = Vector3()
        msg.x = error_x
        msg.y = error_y
        msg.z = error_theta
        self.error_pub.publish(msg)


    def heartbeat(self):
        # self.log.info('heartbeat')
        pass

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ekf_pose_error = EkfPoseError()
    ekf_pose_error.spin()
    ekf_pose_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
