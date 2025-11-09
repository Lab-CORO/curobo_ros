#!/usr/bin/env python3
"""
Example script demonstrating batch trajectory generation with constraints using CuRobo ROS.

This script shows three use cases:
1. Single trajectory without constraints
2. Single trajectory with partial pose constraints
3. Batch trajectories with constraints
"""

import rclpy
from rclpy.node import Node
from curobo_msgs.srv import TrajectoryGeneration
from geometry_msgs.msg import Pose
import sys


class TrajectoryTester(Node):
    def __init__(self):
        super().__init__('trajectory_tester')
        self.client = self.create_client(TrajectoryGeneration, '/curobo_gen_traj/generate_trajectory')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for trajectory generation service...')

    def test_single_trajectory(self):
        """Test single trajectory generation without constraints"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Test 1: Single trajectory without constraints")
        self.get_logger().info("=" * 60)

        request = TrajectoryGeneration.Request()

        # Single target pose
        request.target_pose.position.x = 0.5
        request.target_pose.position.y = 0.0
        request.target_pose.position.z = 0.5
        request.target_pose.orientation.w = 1.0
        request.target_pose.orientation.x = 0.0
        request.target_pose.orientation.y = 0.0
        request.target_pose.orientation.z = 0.0

        # No batch targets
        request.batch_targets = []

        # No constraints
        request.hold_partial_pose = False
        request.hold_vec_weight = []

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Success: {response.success}")
            self.get_logger().info(f"Message: {response.message}")
            self.get_logger().info(f"Success indices: {response.success_indices}")
        else:
            self.get_logger().error("Service call failed")

    def test_single_with_constraints(self):
        """Test single trajectory with partial pose constraints"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Test 2: Single trajectory with orientation constraints")
        self.get_logger().info("=" * 60)

        request = TrajectoryGeneration.Request()

        # Single target pose
        request.target_pose.position.x = 0.5
        request.target_pose.position.y = 0.2
        request.target_pose.position.z = 0.5
        request.target_pose.orientation.w = 1.0
        request.target_pose.orientation.x = 0.0
        request.target_pose.orientation.y = 0.0
        request.target_pose.orientation.z = 0.0

        # No batch targets
        request.batch_targets = []

        # Add constraints: hold orientation (qw, qx, qy, qz) but free position (px, py, pz)
        # [qw, qx, qy, qz, px, py, pz]
        request.hold_partial_pose = True
        request.hold_vec_weight = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # Hold orientation only

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Success: {response.success}")
            self.get_logger().info(f"Message: {response.message}")
            self.get_logger().info(f"Success indices: {response.success_indices}")
        else:
            self.get_logger().error("Service call failed")

    def test_batch_trajectories(self):
        """Test batch trajectory generation"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Test 3: Batch trajectories (3 targets)")
        self.get_logger().info("=" * 60)

        request = TrajectoryGeneration.Request()

        # Target pose is ignored when batch_targets is provided
        request.target_pose = Pose()

        # Create batch of 3 target poses
        pose1 = Pose()
        pose1.position.x = 0.5
        pose1.position.y = -0.2
        pose1.position.z = 0.5
        pose1.orientation.w = 1.0

        pose2 = Pose()
        pose2.position.x = 0.5
        pose2.position.y = 0.0
        pose2.position.z = 0.6
        pose2.orientation.w = 1.0

        pose3 = Pose()
        pose3.position.x = 0.5
        pose3.position.y = 0.2
        pose3.position.z = 0.5
        pose3.orientation.w = 1.0

        request.batch_targets = [pose1, pose2, pose3]

        # No constraints for this test
        request.hold_partial_pose = False
        request.hold_vec_weight = []

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Success: {response.success}")
            self.get_logger().info(f"Message: {response.message}")
            self.get_logger().info(f"Success indices: {response.success_indices}")
        else:
            self.get_logger().error("Service call failed")

    def test_batch_with_constraints(self):
        """Test batch trajectory generation with constraints"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Test 4: Batch trajectories with Y-axis constraint")
        self.get_logger().info("=" * 60)

        request = TrajectoryGeneration.Request()

        # Create batch of 2 target poses
        pose1 = Pose()
        pose1.position.x = 0.5
        pose1.position.y = 0.0
        pose1.position.z = 0.4
        pose1.orientation.w = 1.0

        pose2 = Pose()
        pose2.position.x = 0.5
        pose2.position.y = 0.0
        pose2.position.z = 0.6
        pose2.orientation.w = 1.0

        request.batch_targets = [pose1, pose2]

        # Constrain Y position: free orientation, hold linear-y
        # [qw, qx, qy, qz, px, py, pz]
        request.hold_partial_pose = True
        request.hold_vec_weight = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]  # Hold Y position

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Success: {response.success}")
            self.get_logger().info(f"Message: {response.message}")
            self.get_logger().info(f"Success indices: {response.success_indices}")
        else:
            self.get_logger().error("Service call failed")


def main(args=None):
    rclpy.init(args=args)

    tester = TrajectoryTester()

    try:
        # Run all tests
        tester.test_single_trajectory()
        print("\n")

        tester.test_single_with_constraints()
        print("\n")

        tester.test_batch_trajectories()
        print("\n")

        tester.test_batch_with_constraints()
        print("\n")

        tester.get_logger().info("All tests completed!")

    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
