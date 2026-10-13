from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Common end-to-end maze navigation launch.

    Pass nav2_params:=... to choose either the custom A* or custom Dijkstra
    configuration while keeping every other system component identical.
    """

    nav2_params = LaunchConfiguration('nav2_params')

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
            'params_file': nav2_params,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    # Give Gazebo, TF, map_server and ICP a short startup window before Nav2.
    delayed_navigation = TimerAction(
        period=4.0,
        actions=[navigation],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
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
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('amr_navigation_bringup'),
                'config',
                'nav2_astar_params.yaml',
            ]),
        ),
        localization,
        delayed_navigation,
        delayed_rviz,
    ])
