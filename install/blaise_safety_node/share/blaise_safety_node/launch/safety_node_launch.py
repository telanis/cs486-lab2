from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blaise_safety_node',
            executable='safety_node',  # This should match the entry point defined in setup.py
            name='safety_node',
            output='screen',
            parameters=[
                {'ttc': 3.0},
                {'mode': 'sim'},
                {'student': 'unknown'}
            ]
        )
    ])

