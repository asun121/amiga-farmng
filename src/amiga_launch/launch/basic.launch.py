from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
config = os.path.join(
    get_package_share_directory('amiga_launch'),
    'config',
    'sim_params.yaml'
    )
def generate_launch_description():
    return LaunchDescription([



        Node(
            package='amiga_navigate',
            executable='gps_navigate',
            name='gps_navigate',
            parameters=[config]
        ),
        Node(
            package='amiga_sensors',
            executable='gps',
            name='gps',

        ),
        Node(
            package='amiga_navigate',
            executable='basic_controller',
            name='basic_controller',
        ),

        Node(
            package='amiga_navigate',
            executable='canbus_handler',
            name='canbus_handler'
        )
    ])