import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('time_open', default_value='6.0',
                              description='time to open the door'),
        DeclareLaunchArgument('time_close', default_value='11.0',
                              description='time to close the door'),
        DeclareLaunchArgument('speed_forward', default_value='1.2',
                              description='speed to forward the robot'),
        DeclareLaunchArgument('speed_stop', default_value='0.0',
                              description='speed to stop the robot'),
        DeclareLaunchArgument('torque_open', default_value='2.0',
                              description='torque to open the door'),
        DeclareLaunchArgument('torque_close', default_value='-1.8',
                              description='torque to close the door'),
        Node(
            package='prob_rob_labs',
            executable='door_navigator',
            name='door_navigator',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'time_open': LaunchConfiguration('time_open')},
            {'time_close': LaunchConfiguration('time_close')},
            {'speed_forward': LaunchConfiguration('speed_forward')},
            {'speed_stop': LaunchConfiguration('speed_stop')},
            {'torque_open': LaunchConfiguration('torque_open')},
            {'torque_close': LaunchConfiguration('torque_close')}]
        )
    ])