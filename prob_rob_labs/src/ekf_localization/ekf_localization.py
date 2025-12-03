import numpy as np
import rclpy
from rclpy.node import Node
import yaml

from prob_rob_msgs.msg import Point2DArrayStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from functools import partial


heartbeat_period = 0.1

class EkfLocalization(Node):

    def __init__(self):
        super().__init__('ekf_localization')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        
        self.state = np.array([0,0,0]) # x,y,theta
        self.state_cov = np.diag([0.1,0.1,0.1])
        self.last_time = 0
        self.last_vel = np.array([0,0])
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

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.camera_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.ekf_pose_pub = self.create_publisher(Pose, '/ekf_pose', 10)

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

            ### now do prediction using last_v
            ## and then innovation step
            ## update last_time
            
    def odom_callback(self,msg):
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = msg_time - self.last_time
        if dt < 0: #sample arrived late
            return
        v = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z 
        self.predict(v,wz,dt)
        self.last_vel = np.array([v,wz])
        self.last_time = msg_time
    
    def predict(self, v, wz, dt):
        x,y,theta = self.state
        G = np.eye(3)

        if abs(wz) < 1e-4:
            new_x = x + v*dt*np.cos(theta)
            new_y = y + v*dt*np.sin(theta)
            new_theta = theta
            G[0,2] = -v*dt*np.sin(theta)
            G[1,2] = v*dt*np.cos(theta)
        else:
            new_theta = theta + wz*dt
            new_x = x - (v/wz)*np.sin(theta) + (v/wz)*np.sin(new_theta)
            new_y = y + (v/wz)*np.cos(theta) - (v/wz)*np.cos(new_theta)
            G[0,2] = -(v/wz)*np.cos(theta) + (v/wz)*np.cos(new_theta)
            G[1,2] = -(v/wz)*np.sin(theta) + (v/wz)*np.sin(new_theta)

        Rt = np.diag([0.1, 0.1, 0.1]) * dt
        self.state = np.array([new_x, new_y, new_theta])
        self.state_cov = G @ self.state_cov @ G.T + Rt
        self.get_logger().info(f"predicted state: {self.state}")

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
