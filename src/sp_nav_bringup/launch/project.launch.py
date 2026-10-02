from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Include the supplied simulator launch without overriding any of its parameters.
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0.15', '0', '0', '0', '0', 'base_link', 'livox_frame']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sp_nav_sim'), 'launch', 'sim_robot.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sp_nav_bringup'), 'launch', 'sp_nav.launch.py'))),
    ])
