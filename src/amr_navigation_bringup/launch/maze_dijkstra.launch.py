from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('amr_navigation_bringup'),
                    'launch',
                    'maze_navigation.launch.py',
                ])
            ),
            launch_arguments={
                'nav2_params': PathJoinSubstitution([
                    FindPackageShare('amr_navigation_bringup'),
                    'config',
                    'nav2_dijkstra_params.yaml',
                ])
            }.items(),
        )
    ])
