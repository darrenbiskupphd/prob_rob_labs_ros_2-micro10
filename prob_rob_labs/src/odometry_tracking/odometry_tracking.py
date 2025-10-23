import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Imu, JointState

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile

from message_filters import Subscriber, ApproximateTimeSynchronizer

from tf_transformations import quaternion_from_euler




heartbeat_period = 0.1

class OdometryTracking(Node):

    def __init__(self):
        super().__init__('odometry_tracking')
        self.log = self.get_logger()


        self.imu_sub = Subscriber(self, Imu, '/imu', qos_profile=QoSProfile(depth=10)) #time stamp for every 0.006s
        self.joint_state_sub = Subscriber(self, JointState, '/joint_states', qos_profile=QoSProfile(depth=10)) # time stamp for every 0.034s

        self.vel_cam_sub = self.create_subscription(Twist, '/vel_cam', self.vel_cam_callback, 10)

        self.ekf_odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

        self.sync = ApproximateTimeSynchronizer([self.imu_sub, self.joint_state_sub], 10, slop = 0.04)
        self.sync.registerCallback(self.sync_callback)

        # define model parameters
        self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # theta, x, y, v_x, w_z
        self.x_cov = np.diag([1.0, 1.0, 1.0, 0.01, 0.01]) 

        # define input parameters
        self.vel_cam = None
        self.u_v = 0.0
        self.u_w = 0.0
        
        # guess of the input covariance
        self.u_cov = np.diag([0.05, 0.05])

        # define the time
        self.last_update_time = None

        self.tau_v = 0.94
        self.tau_w = 0.19

        # self.timer = self.create_timer(heartbeat_period, self.heartbeat) 


    def sync_callback(self, imu_msg, joint_state_msg):
        self.imu = imu_msg
        self.joint_state = joint_state_msg
        # self.log.info(f'now the imu timestamp is {self.imu.header.stamp}')
        # self.log.info(f'now the joint state timestamp is {self.joint_state.header.stamp}')

        if self.last_update_time is None:
            self.last_update_time = self.imu.header.stamp
        dt = (self.imu.header.stamp.sec + self.imu.header.stamp.nanosec * 1e-9) - \
             (self.last_update_time.sec + self.last_update_time.nanosec * 1e-9)
        self.last_update_time = self.imu.header.stamp
        self.x = self.prediction(dt, self.u_v, self.u_w)
        self.x, self.x_cov = self.update_through_imu(dt, self.imu)

        # self.log.info(f"the x after imu update is {self.x}")
        # self.log.info(f"the x_cov after imu update is {self.x_cov}")
        
        dt = (self.joint_state.header.stamp.sec + self.joint_state.header.stamp.nanosec * 1e-9) - \
             (self.last_update_time.sec + self.last_update_time.nanosec * 1e-9)
        self.last_update_time = self.joint_state.header.stamp
        self.x = self.prediction(dt, self.u_v, self.u_w)
        self.x, self.x_cov = self.update_through_joint_state(dt, self.joint_state)

        # self.log.info(f"the x after joint state update is {self.x}")
        # self.log.info(f"the x_cov after joint state update is {self.x_cov}")
        # self.log.info(f"the x is {self.x}")
        # self.log.info(f"the x_cov is {self.x_cov}")
        # self.log.info(f"the x shape is {self.x.shape}")
        # self.log.info(f"the x_cov shape is {self.x_cov.shape}")
        self.ekf_odom_msg = Odometry()
        self.ekf_odom_msg.header.stamp = self.last_update_time
        self.ekf_odom_msg.header.frame_id = 'odom'
        self.ekf_odom_msg.pose.pose.position.x = self.x[1]
        self.ekf_odom_msg.pose.pose.position.y = self.x[2]
        self.ekf_odom_msg.pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(0, 0, self.x[0])
        # if quaternion[3] < 1e-6:
        #     quaternion = 0.0, 0.0, 0.0, 1.0
        self.ekf_odom_msg.pose.pose.orientation.x = quaternion[0]
        self.ekf_odom_msg.pose.pose.orientation.y = quaternion[1]
        self.ekf_odom_msg.pose.pose.orientation.z = quaternion[2]
        self.ekf_odom_msg.pose.pose.orientation.w = quaternion[3]

        self.ekf_odom_msg.twist.twist.linear.x = self.x[3]
        self.ekf_odom_msg.twist.twist.angular.z = self.x[4]

        pose_cov = np.zeros((6, 6))
        pose_cov[0, 0] = self.x_cov[1, 1]  # x 
        pose_cov[1, 1] = self.x_cov[2, 2]  # y 差
        pose_cov[2, 2] = 1e-6              # z 
        pose_cov[3, 3] = 1e-6              # roll
        pose_cov[4, 4] = 1e-6              # pitch 
        pose_cov[5, 5] = self.x_cov[0, 0]  # yaw
        
        pose_cov[0, 1] = pose_cov[1, 0] = self.x_cov[1, 2]  
        pose_cov[0, 5] = pose_cov[5, 0] = self.x_cov[1, 0]  
        pose_cov[1, 5] = pose_cov[5, 1] = self.x_cov[2, 0] 
 
        self.ekf_odom_msg.pose.covariance = pose_cov.flatten().tolist()
        
        # twist cov
        twist_cov = np.zeros((6, 6))
        twist_cov[0, 0] = self.x_cov[3, 3]  # vx 
        twist_cov[1, 1] = 1e-6              # vy 
        twist_cov[2, 2] = 1e-6              # vz 
        twist_cov[3, 3] = 1e-6              # vroll 
        twist_cov[4, 4] = 1e-6              # vpitch 
        twist_cov[5, 5] = self.x_cov[4, 4]  # vyaw
        
        twist_cov[0, 5] = twist_cov[5, 0] = self.x_cov[3, 4]  # vx-vyaw
        
        self.ekf_odom_msg.twist.covariance = twist_cov.flatten().tolist()

        self.ekf_odom_pub.publish(self.ekf_odom_msg)



    # def heartbeat(self):
        # self.log.info('heartbeat')
    def vel_cam_callback(self, msg):
        self.vel_cam = msg
        self.u_v = self.vel_cam.linear.x
        self.u_w = self.vel_cam.angular.z
        # self.log.info(f"the vel_cam is {self.vel_cam}")

    def prediction(self, dt, u_v, u_w):
        self.x[0] = self.x[0] + self.x[4] * dt
        self.x[1] = self.x[1] + self.x[3] * np.cos(self.x[0]) * dt
        self.x[2] = self.x[2] + self.x[3] * np.sin(self.x[0]) * dt
        self.x[3] = 0.1**(dt/self.tau_v) * self.x[3] + (1 - 0.1**(dt/self.tau_v)) * u_v
        self.x[4] = 0.1**(dt/self.tau_w) * self.x[4] + (1 - 0.1**(dt/self.tau_w)) * u_w

        return self.x
    
    def cal_x_cov_bar(self, dt):
        G_x = np.array([[1, 0, 0, 0, dt],
                        [-dt*np.sin(self.x[0])*self.x[3], 1, 0, dt*np.cos(self.x[0]), 0],
                        [dt*np.cos(self.x[0])*self.x[3], 0, 1, dt*np.sin(self.x[0]), 0],
                        [0, 0, 0, 0.1**(dt/self.tau_v), 0],
                        [0, 0, 0, 0, 0.1**(dt/self.tau_w)]])
        G_u = np.array([[0, 0],
                        [0, 0],
                        [0, 0],
                        [1-0.1**(dt/self.tau_v), 0],
                        [0, 1-0.1**(dt/self.tau_w)]])
        
        x_cov_bar = G_x @ self.x_cov @ G_x.T + G_u @ self.u_cov @ G_u.T
        return x_cov_bar

    def update_through_imu(self, dt, imu_msg):
        x_cov_bar = self.cal_x_cov_bar(dt)
        H_w = np.array([0, 0, 0, 0, 1])
        cov_z_w = imu_msg.angular_velocity_covariance[8]
        K = x_cov_bar @ H_w.T / (H_w @ x_cov_bar @ H_w.T + cov_z_w)
        z_w = imu_msg.angular_velocity.z
        self.x = self.x + K * (z_w - self.x[4])
        # self.log.info(f"K gain shape is {K.shape}")
        # self.log.info(f"H_w shape is {H_w.shape}")
        self.x_cov = (np.eye(5) - np.outer(K, H_w)) @ x_cov_bar
        return self.x, self.x_cov

    def update_through_joint_state(self, dt, joint_state_msg):
        x_cov_bar = self.cal_x_cov_bar(dt)
        H_w = np.array([[0, 0, 0, 30.3030, 2.1742],
                        [0, 0, 0, 30.3030, -2.1742]])
        cov_z_w = np.diag([0.01, 0.01])
        K = x_cov_bar @ H_w.T @ np.linalg.inv(H_w @ x_cov_bar @ H_w.T + cov_z_w)
        z_w_rl = np.array([joint_state_msg.velocity[0], joint_state_msg.velocity[1]])
        self.x = self.x + K @ (z_w_rl - H_w @ self.x)
        # self.log.info(f"K gain shape is {K.shape}")
        # self.log.info(f"H_w shape is {H_w.shape}")
        self.x_cov = (np.eye(5) - K @ H_w) @ x_cov_bar
        return self.x, self.x_cov



    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    odometry_tracking = OdometryTracking()
    odometry_tracking.spin()
    odometry_tracking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
