#!/usr/bin/env python3
"""
Classic trajectory planner using cuRobo MotionGen.

This planner generates a complete trajectory from start to goal in one shot,
then executes it in open-loop fashion.
"""

import time
from typing import Optional

import torch
from curobo.types.robot import JointState
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

from .trajectory_planner import TrajectoryPlanner, PlannerResult, ExecutionMode


class ClassicPlanner(TrajectoryPlanner):
    """
    Classic motion generation planner.

    Uses cuRobo's MotionGen to generate a complete collision-free trajectory
    from start to goal, which is then executed in open-loop.

    Execution flow:
    1. plan() - Generate full trajectory using MotionGen
    2. execute() - Send trajectory to robot and monitor progress
    """

    def __init__(self, node, config_wrapper):
        """
        Initialize classic planner.

        Args:
            node: ROS2 node
            config_wrapper: ConfigWrapperMotion with MotionGen configured
        """
        super().__init__(node, config_wrapper)

        # Store reference to motion_gen from config wrapper
        # This will be set after warmup
        self.motion_gen = None

        # Store planned trajectory for execution
        self.planned_trajectory = None
        self.start_state = None
        self.goal_pose = None

    def _get_execution_mode(self) -> ExecutionMode:
        """This planner uses open-loop execution."""
        return ExecutionMode.OPEN_LOOP

    def get_planner_name(self) -> str:
        """Return planner name."""
        return "Classic Motion Generation"

    def get_config_parameters(self) -> list:
        """List of ROS parameters used by this planner."""
        return [
            'max_attempts',
            'timeout',
            'time_dilation_factor',
            'voxel_size',
            'collision_activation_distance',
        ]

    def set_motion_gen(self, motion_gen):
        """
        Set the motion generator instance.

        This is called after warmup is complete.

        Args:
            motion_gen: Initialized MotionGen instance
        """
        self.motion_gen = motion_gen

    def plan(self, start_state: JointState, goal_pose: Pose, config: dict) -> PlannerResult:
        """
        Generate a complete trajectory using MotionGen.

        Args:
            start_state: Initial joint configuration
            goal_pose: Target end-effector pose
            config: Dictionary with keys:
                - max_attempts: Number of planning attempts
                - timeout: Planning timeout in seconds
                - time_dilation_factor: Trajectory time scaling

        Returns:
            PlannerResult with trajectory or error
        """
        if self.motion_gen is None:
            return PlannerResult(
                success=False,
                message="MotionGen not initialized. Call set_motion_gen() first.",
            )

        # Store for execution
        self.start_state = start_state
        self.goal_pose = goal_pose

        try:
            # Extract config parameters
            max_attempts = config.get('max_attempts', 1)
            timeout = config.get('timeout', 5.0)
            time_dilation_factor = config.get('time_dilation_factor', 0.5)

            self.node.get_logger().info(
                f"Planning with max_attempts={max_attempts}, "
                f"timeout={timeout}s, time_dilation={time_dilation_factor}"
            )

            # Plan trajectory using MotionGen
            result = self.motion_gen.plan_single(
                start_state,
                goal_pose,
                MotionGenPlanConfig(
                    max_attempts=max_attempts,
                    timeout=timeout,
                    time_dilation_factor=time_dilation_factor,
                ),
            )

            # Check if planning succeeded
            if not result.success.item():
                return PlannerResult(
                    success=False,
                    message=f"Planning failed: {result.status}",
                    metadata={'result': result}
                )

            # Get interpolated trajectory
            self.planned_trajectory = result.get_interpolated_plan()

            self.node.get_logger().info(
                f"Successfully planned trajectory with {len(self.planned_trajectory.position)} waypoints"
            )

            return PlannerResult(
                success=True,
                message="Trajectory planned successfully",
                trajectory=self.planned_trajectory,
                metadata={
                    'num_waypoints': len(self.planned_trajectory.position),
                    'planning_time': result.solve_time,
                }
            )

        except Exception as e:
            self.node.get_logger().error(f"Planning exception: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())

            return PlannerResult(
                success=False,
                message=f"Planning error: {str(e)}",
            )

    def execute(self, robot_context, goal_handle=None) -> bool:
        """
        Execute the planned trajectory in open-loop.

        Sends the full trajectory to the robot and monitors progress.

        Args:
            robot_context: RobotContext for command sending
            goal_handle: Optional ROS action goal handle for feedback

        Returns:
            True if execution completed successfully
        """
        if self.planned_trajectory is None:
            self.node.get_logger().error("No trajectory to execute. Call plan() first.")
            return False

        try:
            # Send trajectory commands to robot context
            traj = self.planned_trajectory
            robot_context.set_command(
                traj.joint_names,
                traj.velocity.tolist(),
                traj.acceleration.tolist(),
                traj.position.tolist()
            )

            # Start trajectory execution
            robot_context.send_trajectrory()

            self.node.get_logger().info("Trajectory execution started")

            # Monitor progress with feedback
            start_time = time.time()
            time_dilation_factor = self.node.get_parameter(
                'time_dilation_factor'
            ).get_parameter_value().double_value

            progression = robot_context.get_progression()

            while progression < 1.0:
                # Check for cancellation
                if goal_handle is not None and not goal_handle.is_active:
                    self.node.get_logger().warn("Trajectory execution cancelled")
                    robot_context.stop_robot()
                    return False

                # Publish feedback at regular intervals
                if (time.time() - start_time) > time_dilation_factor:
                    if goal_handle is not None:
                        from curobo_msgs.action import SendTrajectory
                        feedback_msg = SendTrajectory.Feedback()
                        feedback_msg.step_progression = robot_context.get_progression()
                        goal_handle.publish_feedback(feedback_msg)

                    progression = robot_context.get_progression()
                    self.node.get_logger().info(f"Trajectory progress: {progression*100:.1f}%")
                    start_time = time.time()

            self.node.get_logger().info("Trajectory execution completed")
            return True

        except Exception as e:
            self.node.get_logger().error(f"Execution error: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())
            robot_context.stop_robot()
            return False
