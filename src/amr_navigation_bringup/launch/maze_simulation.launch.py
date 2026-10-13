import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    """Start Gazebo Fortress with the existing AMR model in the new maze world."""

    description_pkg = get_package_share_directory('robot_description')
    bringup_pkg = get_package_share_directory('amr_navigation_bringup')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(
        description_pkg,
        'urdf',
        'amr.urdf.xacro',
    )
    robot_description = xacro.process_file(xacro_file).toxml()

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze_world.sdf',
        description='Gazebo world file in amr_navigation_bringup/worlds',
    )
    x_arg = DeclareLaunchArgument('x', default_value='-5.0')
    y_arg = DeclareLaunchArgument('y', default_value='-4.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.10')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    world_file = PathJoinSubstitution([
        bringup_pkg,
        'worlds',
        LaunchConfiguration('world'),
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'icp_ekf_amr',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    bridge_config = os.path.join(
        bringup_pkg,
        'config',
        'bridge_maze.yaml',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            'config_file:=' + bridge_config,
        ],
    )

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
