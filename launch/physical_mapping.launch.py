from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('waver_description'),
        'urdf',
        'waver.xacro'
    )

    # Robot State Publisher
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[{
                                     'robot_description': Command(['xacro ', xacro_file])
                                     }])
    
    # Rviz Configuration
    rviz_config_file = os.path.join(
        get_package_share_directory('waver_bringup'),
        'rviz',
        'mapping.rviz'
    )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file]
    )
    
    slam_toolbox_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'slam_toolbox_mapping': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'scan_topic': 'scan',
                'mode': 'mapping',
                'resolution': 0.05,
                'map_update_interval': 4.0,
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/odom', '/odometry/filtered'),
            ]
        )

    return LaunchDescription([
        robot_state_publisher,
        slam_toolbox_node,
        rviz
    ])
