from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, ExecuteProcess
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup_dir = get_package_share_directory('waver_bringup')
    map_file = os.path.join(bringup_dir, 'maps', 'custom_map.yaml')

    # Launch map_server as lifecycle node
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'yaml_filename': map_file
        }]
    )

    # Lifecycle manager to auto-configure + activate map_server
    lifecycle_manager = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server']
                }]
            )
        ]
    )

    # RViz2 node
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'map_server.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        map_server,
        lifecycle_manager,
        rviz
    ])
