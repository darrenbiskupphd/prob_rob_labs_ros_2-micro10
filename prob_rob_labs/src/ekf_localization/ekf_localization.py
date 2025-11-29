import numpy as np
import rclpy
from rclpy.node import Node
import yaml

from prob_rob_msgs.msg import Point2DArrayStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from functools import partial


heartbeat_period = 0.1

class EkfLocalization(Node):

    def __init__(self):
        super().__init__('ekf_localization')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        
        self.p = np.array([1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0])
        self.declare_parameter('map_path', '')
        map_path = self.get_parameter('map_path').value
        self.landmarks = {}
        with open(map_path, "r") as f:
            self.landmarks = yaml.safe_load(f)
        
        self.subscribers = []
        for color, data in self.landmarks['landmarks'].items():
            topic_name = f'/vision_{color}/corners'
            callback_with_color = partial(self.estimate_landmark_bearing_dist, landmark_color=color)
            sub = self.create_subscription(Point2DArrayStamped, topic_name, callback_with_color, 10)
            self.subscribers.append(sub)

        self.camera_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

    def estimate_landmark_bearing_dist(self, msg, landmark_color):
        landmark_info = self.landmarks['landmarks'][landmark_color]
        height = landmark_info['height']
        radius = landmark_info['radius']

        ### copied in from lab 5
        if len(msg.points) >= 5:
            min_x = 100000000.0
            min_y = 100000000.0
            max_x = 0.0
            max_y = 0.0
            for point in msg.points: # obtain bounding box
                min_x = min(point.x, min_x)
                min_y = min(point.y, min_y)
                max_x = max(point.x, max_x)
                max_y = max(point.y, max_y)

            pixel_width = max_x - min_x
            pixel_height = max_y - min_y
            image_ratio = pixel_height / pixel_width
            actual_ratio = height / (radius*2)
            if abs(image_ratio - actual_ratio) > 0.16:
                return # exit if too close to edge

            landmark_pixel_axis = (max_x + min_x)/2
            landmark_height_pixels = pixel_height
            
            fx = self.p[0]
            cx = self.p[2]
            fy = self.p[4]
            cy = self.p[5]
            theta = np.arctan2(cx - landmark_pixel_axis, fx)
            d = height * fy / (landmark_height_pixels * np.cos(theta))

            ## now from lab 5, estimate distance and bearing variances
            bearing_variance = 0.0004488*d**2 - 0.0005*d - 0.0003
            dist_variance = 3.892*np.exp(-0.000169*d) - 3.884
            self.get_logger().info(f"Landmark {landmark_color}: d={d:.2f}, theta={theta:.2f}")

            

    def camera_info_callback(self, msg):
        self.p = msg.k
        self.destroy_subscription(self.camera_sub)
        self.camera_sub = None

    def heartbeat(self):
        # self.log.info('heartbeat')
        pass

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ekf_localization = EkfLocalization()
    ekf_localization.spin()
    ekf_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
