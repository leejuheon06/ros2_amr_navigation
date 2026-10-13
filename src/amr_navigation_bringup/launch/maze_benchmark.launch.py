import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Load both planners in one Planner Server and benchmark the same query."""

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('amr_navigation_bringup'),
                'launch',
                'maze_localization.launch.py',
            ])
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': PathJoinSubstitution([
                FindPackageShare('amr_navigation_bringup'),
                'config',
                'nav2_benchmark_params.yaml',
            ]),
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    delayed_navigation = TimerAction(period=4.0, actions=[navigation])

    benchmark = Node(
        package='amr_navigation_bringup',
        executable='planning_benchmark.py',
        name='planning_benchmark',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'start_x': 0.0,
            'start_y': 0.0,
            'goal_x': 10.0,
            'goal_y': 8.0,
            'repetitions': 10,
            'output_csv': os.path.join(
                os.path.expanduser('~'),
                'ros2_amr_navigation',
                'results',
                'planner_benchmark.csv',
            ),
        }],
    )

    delayed_benchmark = TimerAction(period=10.0, actions=[benchmark])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('amr_navigation_bringup'),
                'rviz',
                'maze_navigation.rviz',
            ]),
        ],
        parameters=[{'use_sim_time': True}],
    )
    delayed_rviz = TimerAction(period=5.0, actions=[rviz])

    return LaunchDescription([
        localization,
        delayed_navigation,
        delayed_rviz,
        delayed_benchmark,
    ])
