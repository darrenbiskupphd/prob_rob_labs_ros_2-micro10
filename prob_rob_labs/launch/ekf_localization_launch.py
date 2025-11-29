import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('map_path', default_value='prob_rob_labs/config/landmarks.yaml',
                              description='path to map yaml file'),

        Node(
            package='prob_rob_labs',
            executable='ekf_localization',
            name='ekf_localization',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'map_path': LaunchConfiguration('map_path')}]
        )
    ])
