#!/usr/bin/env python3
"""
Abstract base class for planners using cuRobo MotionGen.

This class provides shared infrastructure for all planners that use MotionGen
as their underlying solver. All child planners share the same MotionGen instance
and configuration, avoiding redundant warmup operations.

Architecture:
    TrajectoryPlanner (abstract interface)
        ├── SinglePlanner (open-loop, MotionPlanner-based) [THIS CLASS]
        │   ├── ClassicPlanner (single-shot planning)
        │   ├── MultiPointPlanner (waypoint planning)
        │   └── JointSpacePlanner (joint space planning)
        └── ReactiveController (closed-loop control loop)
            └── MPCController (cuRobo ModelPredictiveControl)
"""

from abc import abstractmethod
from typing import Optional, Any
import time

from curobo.types import JointState
from curobo.motion_planner import MotionPlanner
# v2: PoseCostMetric is gone; Cartesian axis constraints use ToolPoseCriteria.
# Not re-exported publicly yet, so import from _src (same pattern as Mapper).
from curobo._src.cost.tool_pose_criteria import ToolPoseCriteria

from .trajectory_planner import TrajectoryPlanner, PlannerResult, ExecutionMode
from curobo_msgs.action import SendTrajectory

import traceback


class SinglePlanner(TrajectoryPlanner):
    """
    Abstract base class for MotionPlanner-based planners (v2).

    This class implements the shared infrastructure that all MotionPlanner-based
    planners need:
    - Shared MotionPlanner instance (set once, used by all child planners)
    - Common execution logic for open-loop trajectory execution
    - Trajectory storage and state management
    - Cancellation handling

    Child classes only need to implement:
    - _plan_trajectory(): How to generate the trajectory using plan_pose()
    - _process_trajectory(): Optional post-processing of the generated trajectory
    - get_planner_name(): Name of the specific planner

    Key design decisions:
    1. All child planners share the SAME MotionPlanner instance
       - Warmup is done only ONCE by ConfigWrapperMotion
       - Switching between SinglePlanner children does NOT trigger warmup
       - This saves significant initialization time (~seconds)

    2. All planners use open-loop execution
       - Trajectory is fully generated in plan()
       - Then executed as-is in execute()
       - Different from MPC which uses closed-loop

    3. Thread safety: NOT thread-safe by design
       - MotionPlanner instance is shared without locks
       - Assumes single-threaded sequential execution
       - If concurrent planning needed, add synchronization in child classes

    v2 notes:
    - MotionGen → MotionPlanner (curobo.motion_planner).
    - MotionGenResult → MotionPlannerResult, but we only use the `.success`,
      `.status`, `.solve_time` duck-typed attributes here.
    - MotionGenPlanConfig is gone: per-call params are kwargs on plan_pose().
    """

    # Class-level shared MotionPlanner instance
    # This is shared across ALL instances of SinglePlanner and its children
    _shared_motion_planner: Optional[MotionPlanner] = None

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
        # Buffer epoch returned by the preview set_command() in plan(), paired
        # with send_trajectrory(expect_epoch=...) in execute() — see M2 in the
        # pre-publication audit: plan() and execute() are separate calls (a
        # service call, then later an action), so another set_command() could
        # otherwise land in between and execute() would send someone else's
        # trajectory.
        self._command_epoch = None

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
    def set_motion_planner(cls, motion_planner: MotionPlanner):
        """
        Set the shared MotionPlanner instance (v2).

        This is called ONCE after ConfigWrapperMotion.set_motion_gen_config()
        completes the warmup. All SinglePlanner instances (current and future)
        will use this same MotionPlanner instance.

        Args:
            motion_planner: Warmed-up MotionPlanner instance from ConfigWrapperMotion

        Example:
            >>> config_wrapper = ConfigWrapperMotion(node, robot)
            >>> config_wrapper.set_motion_gen_config(node, None, None)
            >>> SinglePlanner.set_motion_planner(node.motion_planner)
        """
        cls._shared_motion_planner = motion_planner

    # Legacy alias: keeps older call sites (`set_motion_gen`, `.motion_gen`) working
    # during the v2 transition.
    set_motion_gen = set_motion_planner

    @property
    def motion_planner(self) -> Optional[MotionPlanner]:
        """Access the shared MotionPlanner instance."""
        return self._shared_motion_planner

    @property
    def motion_gen(self) -> Optional[MotionPlanner]:
        """Legacy alias for motion_planner."""
        return self._shared_motion_planner

    # ------------------------------------------------------------------
    # Cartesian trajectory constraints (v2: ToolPoseCriteria)
    # ------------------------------------------------------------------

    def _apply_pose_constraints(self, goal_request) -> bool:
        """Hold Cartesian axes along the whole path, if requested.

        Reads ``goal_request.trajectory_constraints`` (int8[6], order
        ``[theta_x, theta_y, theta_z, x, y, z]``; 1 = lock that axis along the
        path) and sets ``ToolPoseCriteria.non_terminal_pose_axes_weight_factor``
        (order ``[x, y, z, roll, pitch, yaw]``) on the shared MotionPlanner.
        This is the v2 replacement for the removed PoseCostMetric.

        Returns True if constraints were applied (caller must reset afterwards).
        """
        constraints = list(getattr(goal_request, 'trajectory_constraints', []) or [])
        if not any(c == 1 for c in constraints):
            return False
        if len(constraints) != 6:
            self.node.get_logger().warn(
                f"{self.get_planner_name()}: trajectory_constraints must have 6 entries "
                f"[theta_x, theta_y, theta_z, x, y, z], got {len(constraints)} - ignoring."
            )
            return False

        tx, ty, tz, x, y, z = (1.0 if c == 1 else 0.0 for c in constraints)
        axes = [x, y, z, tx, ty, tz]  # ToolPoseCriteria order: x,y,z,roll,pitch,yaw
        tool_frame = self.motion_planner.tool_frames[0]
        self.motion_planner.update_tool_pose_criteria(
            {tool_frame: ToolPoseCriteria(non_terminal_pose_axes_weight_factor=axes)}
        )
        self.node.get_logger().info(
            f"{self.get_planner_name()}: holding axes along path "
            f"(x,y,z,roll,pitch,yaw)={axes}"
        )
        return True

    def _reset_pose_criteria(self) -> None:
        """Restore default (unconstrained) criteria — the MotionPlanner is shared."""
        tool_frame = self.motion_planner.tool_frames[0]
        self.motion_planner.update_tool_pose_criteria({tool_frame: ToolPoseCriteria()})

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
        goal_request: Any,
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
            goal_request: TrajectoryGeneration request containing goal specification
                         Child classes extract what they need (target_pose or target_poses)
            config: Dictionary with planner-specific parameters
                   Common parameters:
                   - max_attempts: Number of planning attempts
            robot_context: Optional RobotContext for trajectory visualization

        Returns:
            PlannerResult with success status and trajectory or error message
        """
        # Validate MotionPlanner is initialized
        if self.motion_planner is None:
            return PlannerResult(
                success=False,
                message=(
                    "MotionPlanner not initialized. "
                    "Call SinglePlanner.set_motion_planner() after warmup."
                ),
            )

        # Store for execution
        self.start_state = start_state
        self.goal_pose = goal_request  # Store request, child classes interpret it
        # Reset: only set below if this call actually binds a fresh
        # set_command() (robot_context is not None). A stale epoch from a
        # PREVIOUS plan() call must not silently guard this one's execute().
        self._command_epoch = None

        try:
            # Let child class generate the trajectory using MotionGen
            result = self._plan_trajectory(start_state, goal_request, config)

            # Check if planning succeeded
            # v2: plan_pose() returns Optional[TrajOptSolverResult] — None on failure
            if result is None:
                return PlannerResult(
                    success=False,
                    message="Planning failed: no solution found (plan_pose returned None)",
                )

            success_val = result.success
            if hasattr(success_val, 'item'):
                success_val = success_val.item()
            if not success_val:
                return PlannerResult(
                    success=False,
                    message=f"Planning failed: {getattr(result, 'status', 'unknown')}",
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

            # v2: position shape can be [B, T, D] — count waypoints on horizon dim.
            _pos = self.planned_trajectory.position
            num_wp = _pos.shape[-2] if _pos.ndim >= 2 else len(_pos)
            self.node.get_logger().info(
                f"{self.get_planner_name()}: Successfully planned trajectory "
                f"with {num_wp} waypoints"
            )

            # Send trajectory to robot context for visualization
            if robot_context is not None:
                traj = self.planned_trajectory
                # v2: position/velocity/acceleration may have shape [B, T, D];
                # robot_context expects [T, D] (one row of floats per waypoint).
                # Flatten all leading dims down to 2 so `.tolist()` yields a
                # list[list[float]] regardless of batch rank.
                def _to_2d_list(t):
                    if t is None:
                        return None
                    while t.ndim > 2:
                        t = t[0]
                    return t.detach().cpu().tolist()

                self.node.get_logger().debug(
                    f"Trajectory shapes - pos: {tuple(traj.position.shape)}, "
                    f"vel: {tuple(traj.velocity.shape) if traj.velocity is not None else None}, "
                    f"acc: {tuple(traj.acceleration.shape) if traj.acceleration is not None else None}"
                )
                pos_list = _to_2d_list(traj.position)
                vel_list = _to_2d_list(traj.velocity)
                acc_list = _to_2d_list(traj.acceleration)

                # Ensure velocity/acceleration arrays align with positions even
                # if the planner omitted them (rare but possible for a stubbed
                # trajectory).
                if vel_list is None:
                    vel_list = [[0.0] * len(pos_list[0]) for _ in pos_list]
                if acc_list is None:
                    acc_list = [[0.0] * len(pos_list[0]) for _ in pos_list]

                self._command_epoch = robot_context.set_command(
                    traj.joint_names,
                    vel_list,
                    acc_list,
                    pos_list,
                )
                self.node.get_logger().info(
                    "Trajectory sent to robot context for visualization"
                )

            return PlannerResult(
                success=True,
                message="Trajectory planned successfully",
                trajectory=self.planned_trajectory,
                metadata={
                    'num_waypoints': num_wp,
                    'planning_time': getattr(result, 'solve_time', 0.0),
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
        goal_request: Any,
        config: dict
    ):
        """
        Generate trajectory using MotionPlanner.plan_pose() (v2).

        Child planners extract different data from goal_request:
        - ClassicPlanner: goal_request.target_pose  → single GoalToolPose
        - MultiPointPlanner: goal_request.target_poses → goalset GoalToolPose
        - JointSpacePlanner: goal_request.target_joints → joint goal

        Args:
            start_state: Initial joint configuration
            goal_request: TrajectoryGeneration request (child extracts what it needs)
            config: Dictionary with planner-specific configuration

        Returns:
            MotionPlannerResult-like object with `.success`, `.status`,
            `.solve_time`, and `.get_interpolated_plan()`.

        Example (ClassicPlanner):
            >>> from curobo.types import ToolPose, GoalToolPose
            >>> goal = GoalToolPose(tool_pose=ToolPose.from_list([
            ...     p.position.x, p.position.y, p.position.z,
            ...     p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z,
            ... ]))
            >>> return self.motion_planner.plan_pose(
            ...     start_state, goal,
            ...     max_attempts=config['max_attempts'],
            ... )
        """
        pass

    def _process_trajectory(self, trajectory: JointState, config: dict) -> JointState:
        """
        Post-process the generated trajectory.

        Override this in child classes if you need to modify the trajectory
        after it's generated. For example:
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

            # Start trajectory execution. expect_epoch guards against another
            # set_command() landing between plan() (which set _command_epoch)
            # and this call — see JointCommandStrategy.buffer_epoch / M2.
            if not robot_context.send_trajectrory(expect_epoch=self._command_epoch):
                self.node.get_logger().error(
                    f"{self.get_planner_name()}: refusing to execute - the "
                    f"planned trajectory was superseded by a newer command "
                    f"before execution started."
                )
                return False

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
                        feedback_msg.state = "EXECUTING"
                        feedback_msg.on_target = False
                        feedback_msg.step_progression = robot_context.get_progression()
                        goal_handle.publish_feedback(feedback_msg)

                    progression = robot_context.get_progression()
                    # Only log at significant milestones to reduce spam
                    # if progression >= 0.99 or int(progression * 10) != int((progression - 0.1) * 10):
                        # self.node.get_logger().info(
                        #     f"Trajectory progress: {progression*100:.1f}%"
                        # )
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
            'time_dilation_factor',
            'voxel_size',
            'collision_activation_distance',
        ]
