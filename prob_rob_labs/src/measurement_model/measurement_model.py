import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

heartbeat_period = 0.1

class MeasurementModel(Node):

    def __init__(self):
        super().__init__('measurement_model')
        self.log = self.get_logger()
        self.feature_mean_sub = self.create_subscription(Float64, '/feature_mean',self.feature_mean_callback,10)
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.threshold = 260.
        self.buffer_size = 100
        self.obs_buffer = np.ones(self.buffer_size) * 300.0
        self.obs_idx = 0
        

    def feature_mean_callback(self,msg):
        latest_feature_mean = float(msg.data)
        self.obs_buffer[self.obs_idx] = latest_feature_mean
        self.obs_idx += 1
        if self.obs_idx >= self.buffer_size:
            self.obs_idx = 0

    def heartbeat(self):
        self.log.info('heartbeat')
        self.t += heartbeat_period

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    measurement_model = MeasurementModel()
    measurement_model.spin()
    measurement_model.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
