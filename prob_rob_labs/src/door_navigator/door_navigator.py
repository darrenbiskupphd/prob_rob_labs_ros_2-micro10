import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


heartbeat_period = 0.1

class OpenDoorThroughDoor(Node):

    def __init__(self):
        super().__init__('open_door_through_door')
        self.log = self.get_logger()
        self.declare_parameter('time_open', 6.0)
        self.declare_parameter('time_close', 11.0)

        
        self.declare_parameter('speed_forward', 1.2)
        self.declare_parameter('speed_stop', 0.0)
        
        self.declare_parameter('torque_open', 2.0)
        self.declare_parameter('torque_close', -1.8)

        
        self.pub_door_torque = self.create_publisher(Float64,'/hinged_glass_door/torque', 10)
        self.pub_robot_vel = self.create_publisher(Twist,'/cmd_vel', 10)

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        
        self.t = 0.0

        
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
        if self.t < self.get_parameter('time_open').value:
            torque_open = self.get_parameter('torque_open').value
            self.send_torque(torque_open)
            self.robot_stop()

        elif self.t < self.get_parameter('time_close').value:
            speed_forward = self.get_parameter('speed_forward').value
            self.robot_forward(speed_forward)

        else:
            torque_close = self.get_parameter('torque_close').value
            self.send_torque(torque_close)
            self.robot_stop()
            


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