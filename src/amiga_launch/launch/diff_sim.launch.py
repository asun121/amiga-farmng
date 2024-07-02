from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amiga_navigate',
            executable='navigate',
            name='navigate'
        ),
        Node(
            package='amiga_navigate',
            executable='pid_controller',
            name='pid_controller',
            parameters=[
                'config/sim_params.yaml'
            ]
        )
    ])
