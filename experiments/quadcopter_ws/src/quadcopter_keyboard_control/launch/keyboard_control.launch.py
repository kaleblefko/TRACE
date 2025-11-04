from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="quadcopter_keyboard_control",
            executable="keyboard_controller",
            name='quadcopter_keyboard_controller',
            output='screen',
        ),
    ])