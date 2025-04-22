from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import TimerAction, ExecuteProcess
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

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
    map_file = os.path.join(bringup_dir, 'maps', 'house_map.yaml')
    
    # AMCL node for localization
    amcl = Node(
        name='waver_amcl',
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[os.path.join(bringup_dir,'params','physical_waver_amcl_params.yaml')],
        remappings=[
            ('/odom', '/odometry/filtered')
        ]
    )

    # Launch map_server as lifecycle node
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file
        }]
    )

    # Lifecycle manager to auto-configure + activate map_server and amcl
    lifecycle_manager = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server', 'waver_amcl']
                }]
            )
        ]
    )

    # RViz2 node
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'physical_localization.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    ## ParticleCloud to PoseArray conversion
    particlecloud_to_posearray = Node(
        package='waver_bringup',
        executable='particlecloud_bridge',
        name='particlecloud_bridge',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        robot_state_publisher,
        map_server,
        amcl,
        lifecycle_manager,
        rviz,
        particlecloud_to_posearray,
    ])
