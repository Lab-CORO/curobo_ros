#!/usr/bin/env python3
"""
Abstract base class for planners using cuRobo MotionGen.

This class provides shared infrastructure for all planners that use MotionGen
as their underlying solver. All child planners share the same MotionGen instance
and configuration, avoiding redundant warmup operations.

Architecture:
    TrajectoryPlanner (abstract interface)
        ├── SinglePlanner (MotionGen-based planners) [THIS CLASS]
        │   ├── ClassicPlanner (single-shot planning)
        │   ├── MultiPointPlanner (waypoint planning)
        │   ├── JointSpacePlanner (joint space planning)
        │   └── GraspPlanner (grasp planning)
        └── MPCPlanner (MPC-based, uses MpcSolver instead)
"""

from abc import abstractmethod
from typing import Optional, Any
import time

import torch
from curobo.types.robot import JointState
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenResult

from .trajectory_planner import TrajectoryPlanner, PlannerResult, ExecutionMode
from curobo_msgs.action import SendTrajectory

import traceback


class SinglePlanner(TrajectoryPlanner):
    """
    Abstract base class for MotionGen-based planners.

    This class implements the shared infrastructure that all MotionGen-based
    planners need:
    - Shared MotionGen instance (set once, used by all child planners)
    - Common execution logic for open-loop trajectory execution
    - Trajectory storage and state management
    - Cancellation handling

    Child classes only need to implement:
    - _plan_trajectory(): How to generate the trajectory using MotionGen
    - _process_trajectory(): Optional post-processing of the generated trajectory
    - get_planner_name(): Name of the specific planner

    Key design decisions:
    1. All child planners share the SAME MotionGen instance
       - Warmup is done only ONCE by ConfigWrapperMotion
       - Switching between SinglePlanner children does NOT trigger warmup
       - This saves significant initialization time (~seconds)

    2. All planners use open-loop execution
       - Trajectory is fully generated in plan()
       - Then executed as-is in execute()
       - Different from MPC which uses closed-loop

    3. Thread safety: NOT thread-safe by design
       - MotionGen instance is shared without locks
       - Assumes single-threaded sequential execution
       - If concurrent planning needed, add synchronization in child classes
    """

    # Class-level shared MotionGen instance
    # This is shared across ALL instances of SinglePlanner and its children
    _shared_motion_gen: Optional[MotionGen] = None

    def __init__(self, node, config_wrapper):
        """
        Initialize the planner.

        Args:
            node: ROS2 node for logging and parameters
            config_wrapper: ConfigWrapperMotion with world/robot config
                           (NOT used to create MotionGen, just for world updates)
        """
        super().__init__(node, config_wrapper)

        # Trajectory state (instance-specific)
        self.planned_trajectory = None
        self.start_state = None
        self.goal_pose = None

        # Cancellation flag
        self._cancelled = False

    def _get_execution_mode(self) -> ExecutionMode:
        """
        All SinglePlanner children use open-loop execution.

        The trajectory is fully generated upfront, then executed.
        This is different from MPC which uses closed-loop execution.
        """
        return ExecutionMode.OPEN_LOOP

    @classmethod
    def set_motion_gen(cls, motion_gen: MotionGen):
        """
        Set the shared MotionGen instance.

        This is called ONCE after ConfigWrapperMotion.set_motion_gen_config()
        completes the warmup. All SinglePlanner instances (current and future)
        will use this same MotionGen instance.

        Args:
            motion_gen: Warmed-up MotionGen instance from ConfigWrapperMotion

        Example usage in node initialization:
            >>> config_wrapper = ConfigWrapperMotion(node, robot)
            >>> config_wrapper.set_motion_gen_config(node, None, None)  # Creates and warms up
            >>> SinglePlanner.set_motion_gen(node.motion_gen)  # Share with all planners
        """
        cls._shared_motion_gen = motion_gen

    @property
    def motion_gen(self) -> Optional[MotionGen]:
        """
        Access the shared MotionGen instance.

        Returns:
            Shared MotionGen instance, or None if not yet initialized
        """
        return self._shared_motion_gen

    def cancel(self):
        """
        Cancel the current trajectory execution.

        This sets a flag that breaks the execution loop in execute().
        Called by the node when a cancellation request is received.
        """
        self._cancelled = True
        self.node.get_logger().info(f"{self.get_planner_name()}: Cancellation requested")

    def plan(
        self,
        start_state: JointState,
        goal_pose: Pose,
        config: dict,
        robot_context: Optional[Any] = None
    ) -> PlannerResult:
        """
        Generate a complete trajectory using MotionGen.

        This method orchestrates the planning process:
        1. Validate MotionGen is initialized
        2. Call child class's _plan_trajectory() to generate trajectory
        3. Optionally process trajectory via _process_trajectory()
        4. Send to robot_context for visualization

        Args:
            start_state: Initial joint configuration
            goal_pose: Target end-effector pose (or other goal type depending on child)
            config: Dictionary with planner-specific parameters
                   Common parameters:
                   - max_attempts: Number of planning attempts
                   - timeout: Planning timeout in seconds
                   - time_dilation_factor: Trajectory time scaling
            robot_context: Optional RobotContext for trajectory visualization

        Returns:
            PlannerResult with success status and trajectory or error message
        """
        # Validate MotionGen is initialized
        if self.motion_gen is None:
            return PlannerResult(
                success=False,
                message=(
                    "MotionGen not initialized. "
                    "Call SinglePlanner.set_motion_gen() after warmup."
                ),
            )

        # Store for execution
        self.start_state = start_state
        self.goal_pose = goal_pose

        try:
            # Let child class generate the trajectory using MotionGen
            result = self._plan_trajectory(start_state, goal_pose, config)

            # Check if planning succeeded
            if not result.success.item():
                return PlannerResult(
                    success=False,
                    message=f"Planning failed: {result.status}",
                    metadata={'result': result}
                )

            # Get interpolated trajectory
            self.planned_trajectory = result.get_interpolated_plan()

            # Allow child class to post-process the trajectory
            # (e.g., add grasp commands, modify velocities, etc.)
            self.planned_trajectory = self._process_trajectory(
                self.planned_trajectory,
                config
            )

            self.node.get_logger().info(
                f"{self.get_planner_name()}: Successfully planned trajectory "
                f"with {len(self.planned_trajectory.position)} waypoints"
            )

            # Send trajectory to robot context for visualization
            if robot_context is not None:
                traj = self.planned_trajectory
                robot_context.set_command(
                    traj.joint_names,
                    traj.velocity.tolist(),
                    traj.acceleration.tolist(),
                    traj.position.tolist()
                )
                self.node.get_logger().info(
                    "Trajectory sent to robot context for visualization"
                )

            return PlannerResult(
                success=True,
                message="Trajectory planned successfully",
                trajectory=self.planned_trajectory,
                metadata={
                    'num_waypoints': len(self.planned_trajectory.position),
                    'planning_time': result.solve_time,
                    'planner_type': self.get_planner_name(),
                }
            )

        except Exception as e:
            self.node.get_logger().error(f"Planning exception: {e}")
            self.node.get_logger().error(traceback.format_exc())

            return PlannerResult(
                success=False,
                message=f"Planning error: {str(e)}",
            )

    @abstractmethod
    def _plan_trajectory(
        self,
        start_state: JointState,
        goal_pose: Pose,
        config: dict
    ) -> MotionGenResult:
        """
        Generate trajectory using MotionGen.

        This is the main method that child classes must implement.
        It defines HOW the trajectory is generated using MotionGen.

        Different child planners will call different MotionGen methods:
        - ClassicPlanner: motion_gen.plan_single(start, goal, config)
        - MultiPointPlanner: motion_gen.plan_single_js(start, waypoints, config)
        - JointSpacePlanner: motion_gen.plan_single_js(start, joint_goal, config)
        - GraspPlanner: motion_gen.plan_single(start, grasp_pose, config)

        Args:
            start_state: Initial joint configuration
            goal_pose: Target (interpretation depends on child planner)
            config: Dictionary with planner-specific configuration

        Returns:
            MotionGenResult with trajectory and status

        Example implementation (ClassicPlanner):
            >>> from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
            >>>
            >>> def _plan_trajectory(self, start_state, goal_pose, config):
            >>>     return self.motion_gen.plan_single(
            >>>         start_state,
            >>>         goal_pose,
            >>>         MotionGenPlanConfig(
            >>>             max_attempts=config.get('max_attempts', 1),
            >>>             timeout=config.get('timeout', 5.0),
            >>>             time_dilation_factor=config.get('time_dilation_factor', 0.5),
            >>>         )
            >>>     )
        """
        pass

    def _process_trajectory(self, trajectory: JointState, config: dict) -> JointState:
        """
        Post-process the generated trajectory.

        Override this in child classes if you need to modify the trajectory
        after it's generated. For example:
        - GraspPlanner: Add gripper open/close commands
        - SlowPlanner: Reduce velocities for safety
        - VibrateFilter: Smooth out high-frequency oscillations

        Args:
            trajectory: Raw trajectory from MotionGen
            config: Configuration dictionary

        Returns:
            Processed trajectory (default: unchanged)
        """
        return trajectory

    def execute(self, robot_context, goal_handle=None) -> bool:
        """
        Execute the planned trajectory in open-loop.

        Sends the full pre-computed trajectory to the robot and monitors
        progress until completion or cancellation.

        This implementation is shared by all SinglePlanner children since
        they all use the same open-loop execution pattern.

        Args:
            robot_context: RobotContext for command sending
            goal_handle: Optional ROS action goal handle for feedback

        Returns:
            True if execution completed successfully, False if cancelled or error
        """
        if self.planned_trajectory is None:
            self.node.get_logger().error("No trajectory to execute. Call plan() first.")
            return False

        try:
            # Reset cancellation flag at the start of execution
            self._cancelled = False

            # Start trajectory execution
            robot_context.send_trajectrory()

            self.node.get_logger().info(
                f"{self.get_planner_name()}: Trajectory execution started"
            )

            # Monitor progress with feedback
            start_time = time.time()
            time_dilation_factor = self.node.get_parameter(
                'time_dilation_factor'
            ).get_parameter_value().double_value

            progression = robot_context.get_progression()

            while progression < 1.0 and not self._cancelled:
                # Check for cancellation
                if goal_handle is not None and not goal_handle.is_active:
                    self.node.get_logger().warn("Trajectory execution cancelled")
                    robot_context.stop_robot()
                    return False

                # Publish feedback at regular intervals
                if (time.time() - start_time) > time_dilation_factor:
                    # Check for cancellation again before publishing feedback
                    if goal_handle is not None and not goal_handle.is_active:
                        self.node.get_logger().warn("Trajectory execution cancelled")
                        robot_context.stop_robot()
                        return False

                    if goal_handle is not None:
                        feedback_msg = SendTrajectory.Feedback()
                        feedback_msg.step_progression = robot_context.get_progression()
                        goal_handle.publish_feedback(feedback_msg)

                    progression = robot_context.get_progression()
                    # Only log at significant milestones to reduce spam
                    if progression >= 0.99 or int(progression * 10) != int((progression - 0.1) * 10):
                        self.node.get_logger().info(
                            f"Trajectory progress: {progression*100:.1f}%"
                        )
                    start_time = time.time()

                # Small sleep to prevent busy-waiting
                time.sleep(0.01)

            # Check if we exited due to cancellation or completion
            if self._cancelled:
                self.node.get_logger().info("Trajectory execution cancelled via flag")
                return False

            # Wait for emulator thread to finish updating position
            # This ensures the next planner reads the correct final position
            time.sleep(0.1)
            self.node.get_logger().info(
                f"Trajectory execution completed. "
                f"Final position: {robot_context.get_joint_pose()}"
            )
            return True

        except Exception as e:
            self.node.get_logger().error(f"Execution error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            robot_context.stop_robot()
            return False

    def get_config_parameters(self) -> list:
        """
        Get list of common configuration parameters for SinglePlanner.

        Child classes should override this and call super() to add their own.

        Returns:
            List of parameter names
        """
        return [
            'max_attempts',
            'timeout',
            'time_dilation_factor',
            'voxel_size',
            'collision_activation_distance',
        ]
