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
                {'kp_linear': 1.0},
                {'ki_linear': 0.0},
                {'kd_linear': 0.0},
                {'kp_angular': 1.0},
                {'ki_angular': 0.0},
                {'kd_angular': 0.0}
            ]
        )
    ])
