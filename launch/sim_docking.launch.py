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
    
    description_dir = get_package_share_directory('waver_description')
    view_launch_file = os.path.join(description_dir, 'launch', 'view_gazebo.launch.py')

    bringup_dir = get_package_share_directory('waver_bringup')
    map_file = os.path.join(bringup_dir, 'maps', 'custom_map.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'sim_docking.rviz')

    # Get the package share directory
    docking_dir = get_package_share_directory('waver_docking')
    params_file = os.path.join(docking_dir, 'params', 'docking_params.yaml')
    
    # AMCL node for localization
    amcl = Node(
        name='waver_amcl',
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[os.path.join(bringup_dir,'params','waver_amcl_params.yaml')],
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

    # Launch the docking node
    docking_node = Node(
        package='waver_docking',
        executable='docking_node',
        name='docking_node',
        output='screen',
        parameters=[params_file]
    )

    # Launch the interactive points node    
    interactive_points_node = Node(
        package='waver_docking',
        executable='interactive_points_node.py',
        name='interactive_points_node',
        output='screen',
        parameters=[params_file]
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
        docking_node,
        interactive_points_node
    ])
