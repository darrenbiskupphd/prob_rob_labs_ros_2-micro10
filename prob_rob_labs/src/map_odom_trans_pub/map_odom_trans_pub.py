import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_matrix, translation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Transform

import tf2_ros


odom_base_period = 0.02
map_odom_period = 1/30.0

class MapOdomTransPub(Node):

    def __init__(self):
        super().__init__('map_odom_trans_pub')
        self.log = self.get_logger()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)


        self.timer = self.create_timer(odom_base_period, self.odom_base_callback)
        self.odom_base_trans = None

        self.ekf_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.ekf_pose_callback, 10)
        self.ekf_pose = None
        self.map_base_trans = None

        self.timer2 = self.create_timer(map_odom_period, self.map_odom_callback)
        self.map_odom_trans = None
        

    def ekf_pose_callback(self, msg):
        if self.odom_base_trans is None:
            return
        
        self.ekf_pose = msg
        t = [self.ekf_pose.pose.pose.position.x, self.ekf_pose.pose.pose.position.y, self.ekf_pose.pose.pose.position.z]
        q = [self.ekf_pose.pose.pose.orientation.x, self.ekf_pose.pose.pose.orientation.y, self.ekf_pose.pose.pose.orientation.z, self.ekf_pose.pose.pose.orientation.w]
        self.map_base_trans = self._from_pose_to_matrix(t, q)
        self.map_odom_trans = np.dot(self.map_base_trans, np.linalg.inv(self.odom_base_trans))
        # self.log.info(f"map_odom trans successful")

    def odom_base_callback(self):
        trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        # self.log.info(f"odom - base_footprint: {trans.transform}")
        t = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        self.odom_base_trans = self._from_pose_to_matrix(t, q)
        # self.log.info(f"odom_base trans successful")

    def map_odom_callback(self):
        if self.map_odom_trans is None:
            return
        
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'odom'
        trans = Transform()
        trans.translation.x = self.map_odom_trans[0, 3]
        trans.translation.y = self.map_odom_trans[1, 3]
        trans.translation.z = self.map_odom_trans[2, 3]

        rot = quaternion_from_matrix(self.map_odom_trans)
        trans.rotation.x = rot[0]
        trans.rotation.y = rot[1]
        trans.rotation.z = rot[2]
        trans.rotation.w = rot[3]
        msg.transform = trans
        self.tf_broadcaster.sendTransform(msg)


    def _from_pose_to_matrix(self, pose, rot):
        mat_t = translation_matrix(pose)
        mat_r = quaternion_matrix(rot)
        mat_trans = np.dot(mat_t, mat_r)
        return mat_trans



    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    map_odom_trans_pub = MapOdomTransPub()
    map_odom_trans_pub.spin()
    map_odom_trans_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
