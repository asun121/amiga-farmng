from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amiga_navigate',
            namespace='simbot',
            executable='navigate.py',
            name='navigate'
        ),
        Node(
            package='amiga_navigate',
            namespace='simbot',
            executable='pid_controller.py',
            name='pid_controller'
        )
    ])