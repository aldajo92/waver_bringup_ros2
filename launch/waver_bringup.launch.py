from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    waver_description_dir = get_package_share_directory('waver_description')
    view_launch_file = os.path.join(waver_description_dir, 'launch', 'view_gazebo.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(view_launch_file)
        ),
    ])