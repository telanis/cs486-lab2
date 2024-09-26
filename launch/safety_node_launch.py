import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ttc', default_value='0.0', description='ttc'),
        DeclareLaunchArgument('mode', default_value=' ', description='mode'),
        DeclareLaunchArgument('student', default_value=' ', description='student'),

        Node(
            package='tiana_safety_node',
            executable='tiana_safety_node',
            name='tiana_safety_node',
            output='screen',
            parameters=[{
                'ttc': LaunchConfiguration('ttc'),
                'mode': LaunchConfiguration('mode'),
                'student': LaunchConfiguration('student')
            }]
        )
    ])
