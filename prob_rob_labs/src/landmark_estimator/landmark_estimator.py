import numpy as np
import rclpy
from rclpy.node import Node

from prob_rob_msgs.msg import Point2DArrayStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_matrix


class LandmarkEstimator(Node):

    def __init__(self):
        super().__init__('landmark_estimator')
        self.log = self.get_logger()
        
        self.landmark_pixel_axis = -1
        self.landmark_height_pixels = -1
        self.d = -1
        self.theta = 0
        self.p = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]

        self.declare_parameter('cylinder_height', 0.5)
        self.declare_parameter('cylinder_radius', 0.1)
        self.declare_parameter('cylinder_color', "cyan")
        color = self.get_parameter('cylinder_color').value
        topic_name = f'/vision_{color}/corners'

        self.pub_bearing_flag = False

        self.corner_sub = self.create_subscription(Point2DArrayStamped, topic_name, self.estimate_landmark_height_pos, 10) #time stamp for every 0.006s
        self.camera_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10) #time stamp for every 0.006s
        self.robot_pose_sub = self.create_subscription(PoseStamped, '/tb3/ground_truth/pose', self.calc_measurement_error, 10)

        self.bearing_est_pub = self.create_publisher(Float32, '/bearing_dist/bearing', 10)
        self.dist_est_pub = self.create_publisher(Float32, '/bearing_dist/dist', 10)
        self.bearing_error_pub = self.create_publisher(Float32, '/bearing_dist/bearing_error', 10)
        self.dist_error_pub = self.create_publisher(Float32, '/bearing_dist/dist_error', 10)



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
            if abs(image_ratio - actual_ratio) > 0.16:
                self.pub_bearing_flag = False
                return

            # self.log.info(f"Center X: {(max_x + min_x)/2}, Height: {(max_y - min_y)}")
            self.landmark_pixel_axis = (max_x + min_x)/2
            self.landmark_height_pixels = pixel_height

            self.estimate_dist_bearing()

    def camera_info_callback(self, msg):
        self.p = msg.k

    def estimate_dist_bearing(self):
        if self.pub_bearing_flag:
            fx = self.p[0]
            cx = self.p[2]
            fy = self.p[4]
            cy = self.p[5]
            self.theta = np.arctan2(cx - self.landmark_pixel_axis, fx)
            h = self.get_parameter('cylinder_height').value
            self.d = h * fy / (self.landmark_height_pixels * np.cos(self.theta))

            self.bearing_est_pub.publish(Float32(data=self.theta))
            self.dist_est_pub.publish(Float32(data=self.d))


    def calc_measurement_error(self, msg):
        pos_cylinder = np.array([0,0,0.25])

        camera_pos_robot_frame = np.array([0.0759997, 0, 0.0930071, 1])
        
        robot_base_pose = msg.pose
        rotation_matrix = quaternion_matrix([robot_base_pose.orientation.x,
                                              robot_base_pose.orientation.y,
                                              robot_base_pose.orientation.z, 
                                              robot_base_pose.orientation.w])[:3, :3]
        base_pose_matrix = np.eye(4)
        base_pose_matrix[:3, :3] = rotation_matrix
        base_pose_matrix[:3, 3] = np.array([robot_base_pose.position.x,
                                          robot_base_pose.position.y,
                                          robot_base_pose.position.z])
        
        cam_pos_global = (base_pose_matrix@camera_pos_robot_frame)[:3]
        cam_to_cyl_xy = pos_cylinder[:2] - cam_pos_global[:2]
        robot_xvec = rotation_matrix[:2, 0]

        dist = np.linalg.norm(cam_to_cyl_xy)
        error_d = abs(dist - self.d)

        # bearing = np.arccos(np.dot(robot_xvec, cam_to_cyl_xy)/(np.linalg.norm(robot_xvec)*dist))
        # if np.cross(robot_xvec, cam_to_cyl_xy) > 0:
        #     bearing *= -1
        
        # error_theta = abs(abs(bearing) - abs(self.theta))
        dot = np.dot(robot_xvec, cam_to_cyl_xy)
        cross = np.cross(robot_xvec, cam_to_cyl_xy)
        bearing_gt = np.arctan2(cross, dot)
        # self.log.info(f"bearing_gt: {bearing_gt}, self.theta: {self.theta}")
        if self.pub_bearing_flag:
            self.bearing_error_pub.publish(Float32(data=abs(bearing_gt - self.theta)))
            self.dist_error_pub.publish(Float32(data=error_d))




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
