import numpy as np
import rclpy
from rclpy.node import Node

from prob_rob_msgs.msg import Point2DArrayStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray

heartbeat_period = 0.1

class LandmarkEstimator(Node):

    def __init__(self):
        super().__init__('landmark_estimator')
        self.log = self.get_logger()
        # self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        
        self.landmark_pixel_axis = -1
        self.landmark_height_pixels = -1
        
        self.declare_parameter('cylinder_height', 0.5)
        self.declare_parameter('cylinder_radius', 0.1)
        self.declare_parameter('cylinder_color', "cyan")
        color = self.get_parameter('cylinder_color').value
        topic_name = f'/vision_{color}/corners'

        self.pub_bearing_flag = False

        self.corner_sub = self.create_subscription(Point2DArrayStamped, topic_name, self.estimate_landmark_height_pos, 10) #time stamp for every 0.006s
        self.camera_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.estimate_dist_bearing, 10) #time stamp for every 0.006s
        self.bearing_dist_pub = self.create_publisher(Float32MultiArray, 'bearing_dist', 10)


    def estimate_landmark_height_pos(self, msg):
        if len(msg.points) < 5:
            self.pub_bearing_flag = False
        else:
            self.pub_bearing_flag = True
            min_x = 100000000.0
            min_y = 100000000.0
            max_x = 0.0
            max_y = 0.0
            for point in msg.points:
                min_x = min(point.x, min_x)
                min_y = min(point.y, min_y)
                max_x = max(point.x, max_x)
                max_y = max(point.y, max_y)

            pixel_width = max_x - min_x
            pixel_height = max_y - min_y
            image_ratio = pixel_height / pixel_width
            actual_ratio = self.get_parameter('cylinder_height').value / (self.get_parameter('cylinder_radius').value*2)
            if abs(image_ratio - actual_ratio) > 0.2:
                self.pub_bearing_flag = False
                return

            # self.log.info(f"Center X: {(max_x + min_x)/2}, Height: {(max_y - min_y)}")
            self.landmark_pixel_axis = (max_x + min_x)/2
            self.landmark_height_pixels = pixel_height

    def estimate_dist_bearing(self, msg):
        if self.pub_bearing_flag:
            p = msg.k
            fx = p[0]
            cx = p[2]
            fy = p[4]
            cy = p[5]
            theta = np.arctan2(cx - self.landmark_pixel_axis, fx)
            h = self.get_parameter('cylinder_height').value
            d = h*(fy/self.landmark_height_pixels*np.cos(theta))

            msg = Float32MultiArray()
            msg.data = [theta, d]
            self.bearing_dist_pub.publish(msg)


    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    landmark_estimator = LandmarkEstimator()
    landmark_estimator.spin()
    landmark_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
