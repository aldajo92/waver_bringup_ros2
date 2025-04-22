from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch.substitutions import Command
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Directories
    bringup_dir = get_package_share_directory('waver_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # description_dir = get_package_share_directory('waver_description')

    # Paths
    map_yaml_path = os.path.join(bringup_dir, 'maps', 'house_map.yaml')
    nav2_param_path = os.path.join(bringup_dir, 'params', 'physical_dwb_nav_params.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'physical_navigation.rviz')
    # view_launch_file = os.path.join(description_dir, 'launch', 'view_gazebo.launch.py')
    
    # Robot State Publisher
    xacro_file = os.path.join(
        get_package_share_directory('waver_description'),
        'urdf',
        'waver.xacro'
    )
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[{
                                     'robot_description': Command(['xacro ', xacro_file])
                                     }])

    bringup_dir = get_package_share_directory('waver_bringup')

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
                    'autostart': 'true',
                    'use_composition': 'False',
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
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace'),
        robot_state_publisher,
        nav2_bringup,
        rviz,
        particlecloud_to_posearray
    ])
