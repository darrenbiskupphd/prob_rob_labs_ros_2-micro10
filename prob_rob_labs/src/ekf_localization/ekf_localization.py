import numpy as np
import rclpy
from rclpy.node import Node
import yaml

from prob_rob_msgs.msg import Point2DArrayStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from functools import partial


heartbeat_period = 0.1

class EkfLocalization(Node):

    def __init__(self):
        super().__init__('ekf_localization')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        
        self.state = np.array([-1.5,0,0]) # x,y,theta
        self.state_cov = np.diag([0.1,0.1,0.1])
        self.last_time = None
        self.last_twist = np.array([0,0])
        self.last_S_twist = np.diag([0.1,0.1])

        self.p = np.eye(3).flatten()
        self.alphas = np.array([0.01, 0.01, 0.01, 0.01])*200 # parameters for twist covariance
        self.declare_parameter('map_path', '')
        map_path = self.get_parameter('map_path').value
        self.landmarks = {}
        with open(map_path, "r") as f:
            self.landmarks = yaml.safe_load(f)
        
        self.subscribers = []
        for color, data in self.landmarks['landmarks'].items():
            topic_name = f'/vision_{color}/corners'
            callback_with_color = partial(self.update_on_landmark, landmark_color=color)
            sub = self.create_subscription(Point2DArrayStamped, topic_name, callback_with_color, 10)
            self.get_logger().info(f'Subscribed to {topic_name}')
            self.subscribers.append(sub)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.camera_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.ekf_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)

    def unwrap_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def update_on_landmark(self, msg, landmark_color):
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = msg_time
            return
        
        dt = msg_time - self.last_time
        if dt < 0:
            return

        landmark_info = self.landmarks['landmarks'][landmark_color]
        height = landmark_info['height']
        radius = landmark_info['radius']

        ### copied in from lab 5
        if len(msg.points) >= 4:
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
            cx_opt = self.p[2]
            fy = self.p[4]
            cy_opt = self.p[5]
            theta_meas = np.arctan2(cx_opt - landmark_pixel_axis, fx) # [FIX] Renamed to theta_meas
            d = height * fy / (landmark_height_pixels * np.cos(theta_meas)) + radius

            ## now from lab 5, estimate distance and bearing variances
            bearing_variance = 0.0004488*d**2 - 0.0005*d - 0.0003
            dist_variance = 3.892*np.exp(-0.000169*d) - 3.884
            Q_t = np.diag([dist_variance, bearing_variance]) 

            ### now do prediction using last_twist and last_S_twist 
            self.predict(*self.last_twist, self.last_S_twist, dt)

            ## and then innovation step
            CAMERA_OFFSET_X = 0.076 # the \camera_joint_rgb frame is located at [0.076, 0, 0.09301] m in the robot base frame
            rx, ry, rtheta = self.state # robot state
            cx = rx + CAMERA_OFFSET_X * np.cos(rtheta)
            cy = ry + CAMERA_OFFSET_X * np.sin(rtheta)
            
            mx = landmark_info['x']
            my = landmark_info['y']
            
            dx = mx - cx
            dy = my - cy
            q = dx**2 + dy**2
            
            # Predicted measurement (z_hat)
            z_hat = np.array([np.sqrt(q), self.unwrap_angle(np.arctan2(dy, dx) - rtheta)])
            
            dcx = -CAMERA_OFFSET_X * np.sin(rtheta)
            dcy =  CAMERA_OFFSET_X * np.cos(rtheta)

            # Jacobian H (Directly transcribed)
            H = np.array([
                [-dx/np.sqrt(q), -dy/np.sqrt(q), (-dx*dcx - dy*dcy)/np.sqrt(q)      ],
                [ dy/q,      -dx/q,      ( dy*dcx - dx*dcy)/q - 1.0     ]
            ])

            S = H @ self.state_cov @ H.T + Q_t
            K = self.state_cov @ H.T @ np.linalg.pinv(S)
            z = np.array([d, theta_meas])

            innovation = z - z_hat
            innovation[1] = self.unwrap_angle(innovation[1])

            self.state = self.state + K @ innovation
            self.state[2] = self.unwrap_angle(self.state[2]) # Keep state normalized
            
            self.state_cov = (np.eye(3) - K @ H) @ self.state_cov

            #only update last time if successful measurement update
            self.publish_pose()
            self.last_time = msg_time
            
    def odom_callback(self, msg):
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v, wz = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        # this didnt work, just gives hardcoded vals
        # S = np.diag([msg.twist.covariance[0], msg.twist.covariance[35]])
        # instead we use the book's way of characterizing twist covariance
        S = np.diag([self.alphas[0]*v**2 + self.alphas[1]*wz**2, self.alphas[2]*v**2 + self.alphas[3]*wz**2])
        if self.last_time is not None:
            dt = msg_time - self.last_time
            if dt > 0:
                self.predict(v, wz, S, dt)

        self.last_time, self.last_twist, self.last_S_twist = (msg_time, np.array([v, wz]), S)
        # self.publish_pose()
        
    def predict(self, v, wz, S_twist, dt):
        x, y, theta = self.state
        G = np.eye(3)
        Vt = np.zeros((3, 2))

        if abs(wz) < 1e-3:
            new_x = x + v * dt * np.cos(theta)
            new_y = y + v * dt * np.sin(theta)
            new_theta = theta
            
            G[0, 2] = -v * dt * np.sin(theta)
            G[1, 2] =  v * dt * np.cos(theta)

            Vt[0, 0] = dt * np.cos(theta)  # dx/dv
            Vt[1, 0] = dt * np.sin(theta)  # dy/dv
            Vt[2, 1] = dt        
        else:
            new_theta = theta + wz * dt
            new_x = x - v / wz * np.sin(theta) + v / wz * np.sin(new_theta)
            new_y = y + v / wz * np.cos(theta) - v / wz * np.cos(new_theta)

            G[0, 2] = -v / wz * np.cos(theta) + v / wz * np.cos(new_theta)
            G[1, 2] = -v / wz * np.sin(theta) + v / wz * np.sin(new_theta)

            Vt[0, 0] = (-np.sin(theta) + np.sin(new_theta)) / wz
            Vt[1, 0] = ( np.cos(theta) - np.cos(new_theta)) / wz
            Vt[2, 0] = 0
            Vt[0, 1] = (v * dt * np.cos(new_theta) / wz) - (v * (np.sin(new_theta) - np.sin(theta)) / (wz**2))
            Vt[1, 1] = (v * dt * np.sin(new_theta) / wz) - (v * (np.cos(theta) - np.cos(new_theta)) / (wz**2))
            Vt[2, 1] = dt

        self.state = np.array([new_x, new_y, new_theta])
        self.state_cov = G @ self.state_cov @ G.T + Vt @ S_twist @ Vt.T

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.pose.pose.orientation.w = np.cos(self.state[2]*0.5)
        msg.pose.pose.orientation.z = np.sin(self.state[2]*0.5)

        cov = np.zeros((6, 6))
        # x
        cov[0, 0] = self.state_cov[0, 0] 
        cov[0, 1] = self.state_cov[0, 1] 
        cov[0, 5] = self.state_cov[0, 2] 
        
        # y
        cov[1, 0] = self.state_cov[1, 0] 
        cov[1, 1] = self.state_cov[1, 1] 
        cov[1, 5] = self.state_cov[1, 2] 
        
        # theta
        cov[5, 0] = self.state_cov[2, 0] 
        cov[5, 1] = self.state_cov[2, 1] 
        cov[5, 5] = self.state_cov[2, 2] 
        
        msg.pose.covariance = cov.flatten().tolist()
        
        self.ekf_pose_pub.publish(msg)

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
