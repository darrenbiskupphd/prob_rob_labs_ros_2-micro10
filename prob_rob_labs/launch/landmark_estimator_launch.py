import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('cylinder_height', default_value='0.5',
                              description='height of cylinder landmark'),
        DeclareLaunchArgument('cylinder_radius', default_value='0.1',
                              description='radius of cylinder landmark'),
        DeclareLaunchArgument('cylinder_color', default_value='cyan',
                              description='color of cylinder landmark'),
        Node(
            package='prob_rob_labs',
            executable='landmark_estimator',
            name='landmark_estimator',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'cylinder_height': LaunchConfiguration('cylinder_height'),
                         'cylinder_radius': LaunchConfiguration('cylinder_radius'),
                         'cylinder_color': LaunchConfiguration('cylinder_color')}]
        )
    ])
