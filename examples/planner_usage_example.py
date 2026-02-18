#!/usr/bin/env python3
"""
Examples of using the unified planner architecture.

This file demonstrates different ways to use the Strategy Pattern
for trajectory planning with cuRobo.
"""

import rclpy
from rclpy.node import Node
import torch

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState

from curobo_ros.planners import PlannerFactory, PlannerManager, ClassicPlanner, MPCPlanner
from curobo_ros.robot.robot_context import RobotContext
from curobo_ros.core.config_wrapper_motion import ConfigWrapperMotion, ConfigWrapperMPC


class PlannerExampleNode(Node):
    """Example node showing different planner usage patterns."""

    def __init__(self):
        super().__init__('planner_example')

        # Setup
        self.tensor_args = TensorDeviceType()
        self.robot_context = RobotContext(self, 0.03)

        # Examples
        self.example_1_direct_usage()
        self.example_2_factory_usage()
        self.example_3_manager_usage()
        self.example_4_switching_planners()

    def example_1_direct_usage(self):
        """Example 1: Direct planner instantiation."""
        self.get_logger().info("=== Example 1: Direct Usage ===")

        # Create config wrapper
        config_wrapper = ConfigWrapperMotion(self, self.robot_context)

        # Create planner directly
        planner = ClassicPlanner(self, config_wrapper)

        # Setup (after warmup)
        # planner.set_motion_gen(self.motion_gen)

        # Plan and execute
        start_state = self._get_current_state()
        goal_pose = self._create_goal_pose([0.5, 0.0, 0.3])

        config = {
            'max_attempts': 1,
            'timeout': 5.0,
            'time_dilation_factor': 0.5
        }

        result = planner.plan(start_state, goal_pose, config)

        if result.success:
            self.get_logger().info(f"Planning succeeded: {result.message}")
            # planner.execute(self.robot_context)
        else:
            self.get_logger().error(f"Planning failed: {result.message}")

    def example_2_factory_usage(self):
        """Example 2: Using PlannerFactory."""
        self.get_logger().info("=== Example 2: Factory Usage ===")

        config_wrapper = ConfigWrapperMotion(self, self.robot_context)

        # Create planner using factory
        planner = PlannerFactory.create_planner('classic', self, config_wrapper)

        self.get_logger().info(f"Created: {planner.get_planner_name()}")
        self.get_logger().info(f"Mode: {planner.get_execution_mode().value}")

        # List available planners
        available = PlannerFactory.get_available_planners()
        self.get_logger().info(f"Available planners: {available}")

    def example_3_manager_usage(self):
        """Example 3: Using PlannerManager for multiple planners."""
        self.get_logger().info("=== Example 3: Manager Usage ===")

        config_wrapper = ConfigWrapperMotion(self, self.robot_context)

        # Create manager
        manager = PlannerManager(self, config_wrapper)

        # Set initial planner
        manager.set_current_planner('classic')

        # Get current planner
        planner = manager.get_current_planner()
        self.get_logger().info(f"Current planner: {planner.get_planner_name()}")

        # Use the planner
        # result = planner.plan(start, goal, config)

    def example_4_switching_planners(self):
        """Example 4: Dynamic planner switching."""
        self.get_logger().info("=== Example 4: Dynamic Switching ===")

        config_wrapper_motion = ConfigWrapperMotion(self, self.robot_context)
        config_wrapper_mpc = ConfigWrapperMPC(self, self.robot_context)

        # Create both planners
        classic = PlannerFactory.create_planner('classic', self, config_wrapper_motion)
        mpc = PlannerFactory.create_planner('mpc', self, config_wrapper_mpc)

        # Use different planners for different scenarios
        start_state = self._get_current_state()

        # Scenario 1: Static environment → use Classic
        if self._is_environment_static():
            self.get_logger().info("Environment is static, using Classic planner")
            planner = classic
        else:
            # Scenario 2: Dynamic environment → use MPC
            self.get_logger().info("Environment is dynamic, using MPC planner")
            planner = mpc

        # Plan with selected planner
        goal = self._create_goal_pose([0.5, 0.0, 0.3])
        config = self._get_config_for_planner(planner)

        # result = planner.plan(start_state, goal, config)

    def example_5_custom_planner(self):
        """Example 5: Registering a custom planner."""
        self.get_logger().info("=== Example 5: Custom Planner ===")

        from curobo_ros.planners import TrajectoryPlanner, ExecutionMode, PlannerResult

        class MyPlanner(TrajectoryPlanner):
            def _get_execution_mode(self):
                return ExecutionMode.OPEN_LOOP

            def get_planner_name(self):
                return "My Custom Planner"

            def plan(self, start, goal, config):
                self.node.get_logger().info("Custom planning logic!")
                return PlannerResult(success=True, message="Custom plan")

            def execute(self, robot_context, goal_handle=None):
                self.node.get_logger().info("Custom execution logic!")
                return True

        # Register custom planner
        PlannerFactory.register_planner('custom', MyPlanner)

        # Use it
        config_wrapper = ConfigWrapperMotion(self, self.robot_context)
        planner = PlannerFactory.create_planner('custom', self, config_wrapper)

        # Plan
        start = self._get_current_state()
        goal = self._create_goal_pose([0.5, 0.0, 0.3])
        result = planner.plan(start, goal, {})

        self.get_logger().info(f"Custom planner result: {result.message}")

    # Helper methods
    def _get_current_state(self) -> JointState:
        """Get current robot joint state."""
        joint_positions = self.robot_context.get_joint_pose()
        return JointState.from_position(
            torch.Tensor([joint_positions]).to(device=self.tensor_args.device)
        )

    def _create_goal_pose(self, position: list) -> Pose:
        """Create a goal pose from position."""
        # Position + quaternion (w, x, y, z)
        pose_list = position + [1.0, 0.0, 0.0, 0.0]
        return Pose.from_list(pose_list)

    def _is_environment_static(self) -> bool:
        """Check if environment has dynamic obstacles."""
        # Implement your logic here
        return True

    def _get_config_for_planner(self, planner):
        """Get appropriate config for planner type."""
        if isinstance(planner, ClassicPlanner):
            return {
                'max_attempts': 1,
                'timeout': 5.0,
                'time_dilation_factor': 0.5
            }
        elif isinstance(planner, MPCPlanner):
            return {
                'convergence_threshold': 0.01,
                'max_iterations': 1000
            }
        return {}


def main():
    rclpy.init()
    node = PlannerExampleNode()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
