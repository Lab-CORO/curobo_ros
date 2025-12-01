"""
Launch file for curobo_ros integration tests.

Launches required nodes and runs pytest integration tests.

Usage:
    ros2 launch curobo_ros integration_tests.launch.py
    ros2 launch curobo_ros integration_tests.launch.py test_suite:=fk
    ros2 launch curobo_ros integration_tests.launch.py test_suite:=unified_planner
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
import launch_pytest


def generate_launch_description():
    """Generate launch description for integration tests."""

    # Declare arguments
    test_suite_arg = DeclareLaunchArgument(
        'test_suite',
        default_value='all',
        description='Test suite to run: all, fk, ik, trajectory, obstacles, '
                    'segmentation, unified_planner, mpc, switching, pipeline'
    )

    node_type_arg = DeclareLaunchArgument(
        'node_type',
        default_value='unified',
        description='Node type to test: classic (curobo_gen_traj) or unified (unified_planner)'
    )

    # Get package share directory
    pkg_share = FindPackageShare('curobo_ros')

    # Map test suites to test files
    test_suite = LaunchConfiguration('test_suite')

    # Note: For actual launch, you would include the required nodes here
    # Example:
    # unified_planner_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([pkg_share, 'launch', 'unified_planner.launch.py'])
    #     ])
    # )

    return LaunchDescription([
        test_suite_arg,
        node_type_arg,
        # Add node launches here when ready
        # unified_planner_launch,
        # ReadyToTest(),
    ])


if __name__ == '__main__':
    generate_launch_description()
