from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_course',
            executable='turtlesim_controller',
            name='turtlesim_controller_node',
            output='screen'
        ),
    ])

