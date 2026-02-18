#!/usr/bin/env python3
"""
Test script for GraspMPCPlanner.

This script demonstrates how to use the Grasp MPC planner to:
1. Add a test cube to the world
2. Execute a grasp sequence with MPC reactive control
3. Verify the grasp was successful

Usage:
    ros2 run curobo_ros grasp_mpc_test.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA
from curobo_msgs.srv import AddObject, GraspObject


class GraspMPCTest(Node):
    """Test node for Grasp MPC functionality."""

    def __init__(self):
        super().__init__('grasp_mpc_test')

        self.get_logger().info("Grasp MPC Test Node Started")

        # Wait for services
        self.get_logger().info("Waiting for services...")

        # Obstacle manager services
        self.add_object_client = self.create_client(
            AddObject,
            '/unified_planner/add_object'
        )
        self.add_object_client.wait_for_service(timeout_sec=5.0)

        # Grasp service
        self.grasp_client = self.create_client(
            GraspObject,
            '/unified_planner/grasp_object'
        )
        self.grasp_client.wait_for_service(timeout_sec=5.0)

        self.get_logger().info("All services ready!")

    def add_test_cube(self, name: str, position: list, size: float = 0.05):
        """
        Add a test cube to the world.

        Args:
            name: Name of the cube
            position: [x, y, z] position
            size: Cube size (default 5cm)

        Returns:
            True if successful
        """
        request = AddObject.Request()
        request.type = AddObject.Request.CUBOID
        request.name = name

        # Set pose
        request.pose.position.x = position[0]
        request.pose.position.y = position[1]
        request.pose.position.z = position[2]
        request.pose.orientation.w = 1.0
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0

        # Set dimensions
        request.dimensions.x = size
        request.dimensions.y = size
        request.dimensions.z = size

        # Set color (blue)
        request.color.r = 0.2
        request.color.g = 0.5
        request.color.b = 0.8
        request.color.a = 1.0

        self.get_logger().info(f"Adding cube '{name}' at {position}")

        future = self.add_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✓ Cube added: {response.message}")
                return True
            else:
                self.get_logger().error(f"✗ Failed to add cube: {response.message}")
                return False
        else:
            self.get_logger().error("✗ Service call failed")
            return False

    def grasp_object(
        self,
        object_name: str,
        grasp_pose: list,
        pre_grasp_offset: list = [0.0, 0.0, 0.10],
        post_grasp_offset: list = [0.0, 0.0, 0.15]
    ):
        """
        Execute grasp sequence.

        Args:
            object_name: Name of object to grasp
            grasp_pose: [x, y, z, qw, qx, qy, qz] grasp pose
            pre_grasp_offset: Approach offset (default 10cm above)
            post_grasp_offset: Retreat offset (default 15cm above)

        Returns:
            True if successful
        """
        request = GraspObject.Request()
        request.object_name = object_name

        # Set grasp pose
        request.target_pose.position.x = grasp_pose[0]
        request.target_pose.position.y = grasp_pose[1]
        request.target_pose.position.z = grasp_pose[2]
        request.target_pose.orientation.w = grasp_pose[3]
        request.target_pose.orientation.x = grasp_pose[4]
        request.target_pose.orientation.y = grasp_pose[5]
        request.target_pose.orientation.z = grasp_pose[6]

        # Set offsets
        request.pre_grasp_offset.x = pre_grasp_offset[0]
        request.pre_grasp_offset.y = pre_grasp_offset[1]
        request.pre_grasp_offset.z = pre_grasp_offset[2]

        request.post_grasp_offset.x = post_grasp_offset[0]
        request.post_grasp_offset.y = post_grasp_offset[1]
        request.post_grasp_offset.z = post_grasp_offset[2]

        request.use_current_state = True

        self.get_logger().info(f"Grasping object '{object_name}'...")
        self.get_logger().info(f"  Grasp pose: {grasp_pose[:3]}")
        self.get_logger().info(f"  Pre-grasp offset: {pre_grasp_offset}")
        self.get_logger().info(f"  Post-grasp offset: {post_grasp_offset}")

        future = self.grasp_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"✓ Grasp successful! Time: {response.execution_time:.2f}s"
                )
                return True
            else:
                self.get_logger().error(f"✗ Grasp failed: {response.message}")
                return False
        else:
            self.get_logger().error("✗ Service call timed out")
            return False

    def run_test(self):
        """Run the complete test scenario."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("GRASP MPC TEST SCENARIO")
        self.get_logger().info("="*60 + "\n")

        # Test scenario:
        # 1. Add a test cube at a reachable position
        # 2. Grasp the cube
        # 3. Verify success

        # Step 1: Add test cube
        self.get_logger().info("[Step 1] Adding test cube...")
        cube_position = [0.5, 0.0, 0.3]  # 50cm forward, centered, 30cm height
        cube_size = 0.05  # 5cm cube

        if not self.add_test_cube("test_cube", cube_position, cube_size):
            self.get_logger().error("Test failed at step 1")
            return False

        # Step 2: Grasp the cube
        self.get_logger().info("\n[Step 2] Grasping cube...")

        # Grasp pose: centered on cube, gripper pointing down
        grasp_pose = [
            cube_position[0],  # x
            cube_position[1],  # y
            cube_position[2],  # z
            0.0,  # qw (pointing down: 180° rotation around Y)
            0.0,  # qx
            -1.0, # qy
            0.0,  # qz
        ]

        if not self.grasp_object("test_cube", grasp_pose):
            self.get_logger().error("Test failed at step 2")
            return False

        # Test completed successfully
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("✓ TEST PASSED - Grasp sequence completed successfully!")
        self.get_logger().info("="*60 + "\n")

        return True


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    test_node = GraspMPCTest()

    try:
        # Run test
        success = test_node.run_test()

        if success:
            test_node.get_logger().info("All tests passed!")
        else:
            test_node.get_logger().error("Tests failed!")

    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
    except Exception as e:
        test_node.get_logger().error(f"Test error: {e}")
        import traceback
        test_node.get_logger().error(traceback.format_exc())
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
