from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Launch maze simulation + saved map + the existing ICP and EKF nodes.

    Important architecture note:
    - The existing ICP node remains the map->odom TF authority, exactly as in
      the previous localization project.
    - The existing EKF runs in parallel and publishes its fused estimate.
    """

    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    map_yaml = LaunchConfiguration('map')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('amr_navigation_bringup'),
                'launch',
                'maze_simulation.launch.py',
            ])
        ),
        launch_arguments={
            'world': world,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items(),
    )

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': True,
        }],
    )

    # Configure and activate map_server explicitly because AMCL is not used and
    # Nav2's standard localization launch is intentionally bypassed.
    configure_map_server = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(map_server),
                                transition_id=Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ],
                )
            ],
        )
    )

    activate_map_server = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # Existing ICP implementation: no source changes.
    icp_localization = Node(
        package='icp_localization',
        executable='icp_localization_node',
        name='icp_localization_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Existing EKF implementation: no source changes. It starts after the
    # simulation / odom / IMU / ICP topics have had time to initialize.
    ekf_localization = Node(
        package='ekf_localization',
        executable='ekf_localization_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('ekf_localization'),
                'config',
                'ekf_params.yaml',
            ]),
            {'use_sim_time': True},
        ],
    )

    delayed_ekf = TimerAction(
        period=8.0,
        actions=[ekf_localization],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='maze_world.sdf'),
        DeclareLaunchArgument('x', default_value='-5.0'),
        DeclareLaunchArgument('y', default_value='-4.0'),
        DeclareLaunchArgument('z', default_value='0.10'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('amr_navigation_bringup'),
                'maps',
                'maze_map.yaml',
            ]),
        ),
        simulation,
        map_server,
        configure_map_server,
        activate_map_server,
        icp_localization,
        delayed_ekf,
    ])
