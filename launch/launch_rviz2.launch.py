from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    urdf_file_name = 'm1013.urdf'

    urdf = Command(['cat ', PathJoinSubstitution([FindPackageShare('curobo_ros'), 'curobo_doosan/src/m1013/', urdf_file_name])])
    rviz_config =  PathJoinSubstitution([FindPackageShare('curobo_ros'), 'rviz/rviz_curobo.rviz'])

    print(rviz_config)
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_gui': 'true'}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])