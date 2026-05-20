from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GZ_WORLD = 'small_house_testing'


def _xterm(title: str) -> str:
    return f'xterm -T "{title}" -e'


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 for visualization',
        ),

        # t=0 s — Micro XRCE-DDS Agent (must be up before any PX4 topics flow)
        ExecuteProcess(
            cmd=['xterm', '-T', 'MicroXRCEAgent', '-e',
                 'MicroXRCEAgent', 'udp4', '-p', '8888'],
        ),

        # t=15 s — SLAM occupancy-grid mapper
        TimerAction(period=15.0, actions=[
            Node(
                package='trace',
                executable='mapping',
                name='occupancy_grid_mapper',
                prefix=_xterm('mapping'),
                output='screen',
            ),
        ]),

        # t=25 s — Object detector (gz_world_name tells it which camera topic to use)
        TimerAction(period=25.0, actions=[
            Node(
                package='trace',
                executable='bbox',
                name='object_detector',
                prefix=_xterm('bbox'),
                output='screen',
                parameters=[{'gz_world_name': GZ_WORLD}],
            ),
        ]),

        # t=35 s — SLAM-aware navigator
        TimerAction(period=35.0, actions=[
            Node(
                package='trace',
                executable='nav',
                name='slam_nav',
                prefix=_xterm('nav'),
                output='screen',
            ),
        ]),

        # t=45 s — Detection visualizer
        TimerAction(period=45.0, actions=[
            Node(
                package='trace',
                executable='visualizer',
                name='detection_visualizer',
                prefix=_xterm('visualizer'),
                output='screen',
                parameters=[{'gz_world_name': GZ_WORLD}],
            ),
        ]),

        # t=60 s — Eval pipeline (replaces state_machine + text_input)
        TimerAction(period=60.0, actions=[
            Node(
                package='trace',
                executable='eval',
                name='eval_pipeline',
                prefix=_xterm('eval'),
                output='screen',
            ),
        ]),

        # Optional RViz2
        TimerAction(period=15.0, actions=[
            ExecuteProcess(
                cmd=['xterm', '-T', 'RViz2', '-e', 'rviz2'],
                condition=IfCondition(use_rviz),
            ),
        ]),
    ])
