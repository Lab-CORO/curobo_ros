from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return 
    Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0.5', '0', '0.5', '0',
                       '0', '0', 'base_0', 'camera_link']
        ),
    # Include the RealSense camera launch file
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={'clip_distance': '0.8'}.items()
    ),