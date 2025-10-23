import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


heartbeat_period = 0.05

class GtSubPub(Node):

    def __init__(self):
        super().__init__('gt_sub_pub')
        self.log = self.get_logger()
        self.declare_parameter('reference_frame', 'odom')
        self.ref_frame = self.get_parameter('reference_frame').value

        self.sub_ground_truth = self.create_subscription(LinkStates, '/gazebo/link_states', self.sub_ground_truth_callback, 10)

        self.pub_ground_truth_pose = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.pub_ground_truth_twist = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)


        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    
    def sub_ground_truth_callback(self, msg):
        # self.log.info('sub_ground_truth_callback')
        if 'waffle_pi::base_footprint' in msg.name:
            id = msg.name.index('waffle_pi::base_footprint')
            self.pose = PoseStamped()
            self.pose.header.stamp = self.get_clock().now().to_msg()
            self.pose.header.frame_id = self.ref_frame
            self.pose.pose = msg.pose[id]

            self.twist = TwistStamped()
            self.twist.header.stamp = self.get_clock().now().to_msg()
            self.twist.header.frame_id = self.ref_frame
            self.twist.twist = msg.twist[id]

    #         self.pub_gt()
    # def pub_gt(self):
    #     self.pub_ground_truth_pose.publish(self.pose)
    #     self.pub_ground_truth_twist.publish(self.twist)
    def heartbeat(self):
        # self.log.info('heartbeat')
        self.pub_ground_truth_pose.publish(self.pose)
        self.pub_ground_truth_twist.publish(self.twist)

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    gt_sub_pub = GtSubPub()
    gt_sub_pub.spin()
    gt_sub_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
