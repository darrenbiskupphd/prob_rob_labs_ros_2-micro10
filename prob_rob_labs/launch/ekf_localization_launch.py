import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('prob_rob_labs'),
        'config',
        'landmarks.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('map_path', default_value=config,
                              description='path to map yaml file'),

        Node(
            package='prob_rob_labs',
            executable='ekf_localization',
            name='ekf_localization',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'map_path': LaunchConfiguration('map_path')}]
        )
    ])
