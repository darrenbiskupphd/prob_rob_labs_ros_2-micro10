import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


heartbeat_period = 1/30.0

class MapOdomTransPub(Node):

    def __init__(self):
        super().__init__('map_odom_trans_pub')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.ekf_pose_sub = self.create_subscription(PoseStamped, '/ekf_pose', self.ekf_pose_callback, 10)
        self.ekf_pose = None

    def ekf_pose_callback(self, msg):
        self.ekf_pose = msg

    def heartbeat(self):
        self.log.info('heartbeat')

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
