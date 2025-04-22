from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Directories
    bringup_dir = get_package_share_directory('waver_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    description_dir = get_package_share_directory('waver_description')

    # Paths
    map_yaml_path = os.path.join(bringup_dir, 'maps', 'custom_map.yaml')
    nav2_param_path = os.path.join(bringup_dir, 'params', 'dwb_nav_params.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'navigation.rviz')
    view_launch_file = os.path.join(description_dir, 'launch', 'view_gazebo.launch.py')

    # Nav2 Bringup Launcher
    nav2_bringup = TimerAction(
    period=3.0,
    actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
                launch_arguments={
                    'map': map_yaml_path,
                    'use_sim_time': use_sim_time,
                    'namespace': namespace,
                    'params_file': nav2_param_path,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ParticleCloud bridge
    particlecloud_to_posearray = Node(
        package='waver_bringup',
        executable='particlecloud_bridge',
        name='particlecloud_bridge',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(view_launch_file),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        nav2_bringup,
        rviz,
        particlecloud_to_posearray
    ])
