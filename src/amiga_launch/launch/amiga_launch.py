from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amiga_navigate',
            executable='pid_controller',
            name='pid_controller',
            namespace='robot1',
            parameters=[
                {'kp': 1.0, 'ki': 0.0, 'kd': 0.0},
                'config/pid_params.yaml' 
            ],
            output='screen'
        ),
        Node(
            package='amiga_navigate',
            executable='navigate',
            name='navigate',
            namespace='robot1',
            output='screen'
        ),
        Node(
            package='amiga_sensors',
            executable='gps',
            name='gps',
            namespace='robot1',
            output='screen'
        )
    ])
