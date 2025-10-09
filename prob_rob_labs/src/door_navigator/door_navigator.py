import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


heartbeat_period = 0.1

class OpenDoorThroughDoor(Node):

    def __init__(self):
        super().__init__('open_door_through_door')
        self.log = self.get_logger()
        self.feature_mean_sub = self.create_subscription(Float64, '/feature_mean',self.feature_mean_callback,10)
        self.latest_feature_mean = 300

        self.declare_parameter("speed_forward", 5.0)
        self.declare_parameter("speed_stop", 0.0)
        self.declare_parameter("torque_open", 2.0)
        self.declare_parameter("torque_close", -2.0)
        self.declare_parameter("threshold", 0.9999)
        self.declare_parameter("belief_threshold", 237.0)

        self.pub_door_torque = self.create_publisher(Float64,'/hinged_glass_door/torque', 10)
        self.pub_robot_vel = self.create_publisher(Twist,'/cmd_vel', 10)

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        
        self.t = 0.0
        self.move_started = 0
        self.state = 0

        self.belief = np.array([0.5, 0.5])
        # Pr(z=o;x=o/c)
        # Pr(z=c;x=o/c)
        self.measurement_model = np.array([[0.9622,0.02],
                                            [0.0378,0.98]])

    def feature_mean_callback(self, msg):
        self.latest_feature_mean = float(msg.data)
        
    def send_torque(self,torque):
        self.pub_door_torque.publish(Float64(data=torque))

    def robot_forward(self,speed):
        msg = Twist()
        msg.linear.x = speed
        self.pub_robot_vel.publish(msg)

    def robot_stop(self):
        msg = Twist()
        speed_stop = self.get_parameter('speed_stop').value
        msg.linear.x = speed_stop
        self.pub_robot_vel.publish(msg)

    def heartbeat(self):
        # self.log.info('heartbeat')
        self.t += heartbeat_period
        belief_threshold = self.get_parameter('belief_threshold').value
        if self.state == 0:
            self.send_torque(self.get_parameter('torque_open').value)

            if self.latest_feature_mean >= belief_threshold:
                self.get_logger().info('The sensor believes the door is closed')
                unnorm = self.measurement_model[1] * self.belief
                belief_new = unnorm / unnorm.sum()
                self.belief = belief_new
            else:
                self.get_logger().info('The sensor believes the door is open')
                unnorm = self.measurement_model[0] * self.belief
                belief_new = unnorm / unnorm.sum()
                self.belief = belief_new
            
            self.get_logger().info(f'The belief now: {self.belief}')
            
            threshold = self.get_parameter('threshold').value
            if self.belief[0] >= threshold:
                self.state = 1
                self.move_started = self.t

        elif self.state == 1:
            self.robot_forward(self.get_parameter('speed_forward').value)
            if self.t - self.move_started > 6.0:
                self.state = 2

        elif self.state == 2:
            self.robot_stop()
            self.send_torque(self.get_parameter('torque_close').value)



        # if self.state == 0:
        #     self.send_torque(self.get_parameter("torque_open").value)
        #     if self.latest_feature_mean < 260.0:
        #         self.state = 1
        #         self.move_started = self.t
        # elif self.state == 1:
        #     self.robot_forward(self.get_parameter("speed_forward").value)
        #     if self.t - self.move_started > 8.0:
        #         self.state = 2
        # elif self.state == 2:
        #     self.robot_stop()
        #     self.send_torque(self.get_parameter("torque_close").value)

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    open_door_through_door = OpenDoorThroughDoor()
    open_door_through_door.spin()
    open_door_through_door.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()