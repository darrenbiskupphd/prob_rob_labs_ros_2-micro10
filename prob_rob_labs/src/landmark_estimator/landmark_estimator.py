import rclpy
from rclpy.node import Node

from prob_rob_msgs.msg import Point2DArrayStamped

heartbeat_period = 0.1

class LandmarkEstimator(Node):

    def __init__(self):
        super().__init__('landmark_estimator')
        self.log = self.get_logger()
        # self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        color = 'cyan'
        topic_name = f'/vision_{color}/corners'

        self.corner_sub = self.create_subscription(Point2DArrayStamped, topic_name, self.estimate_landmark_height_pos, 10) #time stamp for every 0.006s

    def estimate_landmark_height_pos(self, msg):
        min_x = 100000000.0
        min_y = 100000000.0
        max_x = 0.0
        max_y = 0.0
        for point in msg.points:
            min_x = min(point.x, min_x)
            min_y = min(point.y, min_y)
            max_x = max(point.x, max_x)
            max_y = max(point.y, max_y)

        # self.log.info(f"Center X: {(max_x + min_x)/2}, Height: {(max_y - min_y)}")

        return (max_x + min_x)/2 , (max_y - min_y)


        
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
