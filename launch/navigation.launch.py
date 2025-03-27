from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    waver_description_dir = get_package_share_directory('waver_description')
    view_launch_file = os.path.join(waver_description_dir, 'launch', 'view_gazebo.launch.py')

    bringup_dir = get_package_share_directory('waver_bringup')
    map_file = os.path.join(bringup_dir, 'maps', 'custom_map.yaml')
    
    # AMCL node for localization
    amcl = Node(
        name='waver_amcl',
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[os.path.join(bringup_dir,'params','nav2_params.yaml')],
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
                    'node_names': ['map_server']
                }]
            )
        ]
    )

    # RViz2 node
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'localization.rviz')
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
            default_value='true',
            description='Use simulation time'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(view_launch_file)
        ),
        map_server,
        amcl,
        lifecycle_manager,
        rviz,
        particlecloud_to_posearray,
    ])
