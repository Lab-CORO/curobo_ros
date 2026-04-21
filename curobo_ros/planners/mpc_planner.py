#!/usr/bin/env python3
"""
Model Predictive Control (MPC) planner (v2).

v2 notes:
- MpcSolver → ModelPredictiveControl (wired via ConfigWrapperMPC as `node.mpc`).
- Goal API: `Goal(current_state=..., goal_pose=Pose)` is gone. v2 uses
  GoalToolPose wrapping a ToolPose, set via `mpc.update_goal_pose(...)`.
  The explicit `setup_solve_single` / `update_goal` dance is folded into
  a single update call.
- `.step(current_state, ...)` and `.action` / `.metrics.pose_error` accessors
  are unchanged in spirit (duck-typed below).
"""

import time
import traceback
from typing import Optional

import torch

from curobo.types import JointState, ToolPose, GoalToolPose

from .trajectory_planner import TrajectoryPlanner, PlannerResult, ExecutionMode
from curobo_msgs.action import SendTrajectory


class MPCPlanner(TrajectoryPlanner):
    """
    Closed-loop MPC planner built on ModelPredictiveControl (v2).

    Continuously recalculates the optimal action based on the current robot
    state. Unlike open-loop planners, no full trajectory is pre-computed.
    """

    def __init__(self, node, config_wrapper):
        super().__init__(node, config_wrapper)

        self.mpc = None  # set by set_mpc_solver(...)

        self.start_state: Optional[JointState] = None
        self.goal_pose: Optional[GoalToolPose] = None
        self.is_goal_active = False

        # Raw [x,y,z,qw,qx,qy,qz] list written from the ROS thread; consumed
        # on the MPC thread to avoid racing CUDA graph capture.
        self.latest_goal_from_topic = None

        self.mpc_time = []
        self.convergence_threshold = 0.01  # meters
        self.max_iterations = 1000

        # Device/dtype for building tensors on the hot path.
        self._device = getattr(config_wrapper, '_device', torch.device('cuda'))
        self._dtype = getattr(config_wrapper, '_ops_dtype', torch.float32)

    def _get_execution_mode(self) -> ExecutionMode:
        return ExecutionMode.CLOSED_LOOP

    def get_planner_name(self) -> str:
        return "Model Predictive Control (MPC)"

    def get_config_parameters(self) -> list:
        return ['convergence_threshold', 'max_mpc_iterations']

    def set_mpc_solver(self, mpc_solver):
        """Attach the warmed-up v2 ModelPredictiveControl instance."""
        self.mpc = mpc_solver

    # ------------------------------------------------------------------
    # Planning: set up the MPC goal buffer (no full trajectory for MPC).
    # ------------------------------------------------------------------

    def plan(self, start_state: JointState, goal_request, config: dict, robot_context=None) -> PlannerResult:
        if self.mpc is None:
            return PlannerResult(
                success=False,
                message="MPC solver not initialized. Call set_mpc_solver() first.",
            )

        tool_pose = ToolPose.from_list([
            goal_request.target_pose.position.x,
            goal_request.target_pose.position.y,
            goal_request.target_pose.position.z,
            goal_request.target_pose.orientation.w,
            goal_request.target_pose.orientation.x,
            goal_request.target_pose.orientation.y,
            goal_request.target_pose.orientation.z,
        ])
        goal = GoalToolPose(tool_pose=tool_pose)

        self.start_state = start_state
        self.goal_pose = goal

        self.convergence_threshold = config.get('convergence_threshold', 0.01)
        self.max_iterations = config.get('max_iterations', 1000)

        try:
            self.mpc.update_goal_pose(goal)
            self.is_goal_active = True

            if robot_context is not None:
                start_position = start_state.position[0].cpu().tolist()
                joint_names = (
                    robot_context.robot_strategy.get_joint_name()
                    if robot_context.robot_strategy
                    else self.mpc.kinematics.joint_names
                )
                robot_context.set_command(
                    joint_names,
                    [[0.0] * len(start_position)],
                    [[0.0] * len(start_position)],
                    [start_position],
                )
                self.node.get_logger().info(
                    f"MPC: robot init'd at {[f'{x:.3f}' for x in start_position]}"
                )

            self.node.get_logger().info(
                f"MPC goal setup: convergence={self.convergence_threshold}m, "
                f"max_iter={self.max_iterations}"
            )

            return PlannerResult(
                success=True,
                message="MPC goal set",
                trajectory=None,
                metadata={
                    'convergence_threshold': self.convergence_threshold,
                    'max_iterations': self.max_iterations,
                },
            )
        except Exception as e:
            self.node.get_logger().error(f"MPC setup error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            return PlannerResult(success=False, message=f"MPC setup error: {e}")

    def update_goal_pose(self, new_goal: GoalToolPose) -> bool:
        """Update MPC goal during execution (for real-time tracking)."""
        try:
            self.mpc.update_goal_pose(new_goal)
            self.goal_pose = new_goal
            return True
        except Exception as e:
            self.node.get_logger().error(f"Failed to update MPC goal: {e}")
            return False

    # ------------------------------------------------------------------
    # Execution: closed-loop MPC step.
    # ------------------------------------------------------------------

    def execute(self, robot_context, goal_handle=None) -> bool:
        if self.goal_pose is None:
            self.node.get_logger().error("MPC not initialized. Call plan() first.")
            return False

        try:
            converged = False
            tstep = 0
            self.mpc_time = []

            self.node.get_logger().info("Starting MPC execution loop")

            while not converged and self.is_goal_active:
                if goal_handle is not None and not goal_handle.is_active:
                    self.node.get_logger().warn("MPC execution cancelled")
                    robot_context.stop_robot()
                    return False

                # Consume pending topic goal on the MPC thread only.
                if self.latest_goal_from_topic is not None:
                    raw = self.latest_goal_from_topic
                    self.latest_goal_from_topic = None
                    new_goal = GoalToolPose(tool_pose=ToolPose.from_list(raw))
                    self.update_goal_pose(new_goal)

                actual_joint_pose = robot_context.get_joint_pose()
                current_state = JointState.from_position(
                    torch.tensor([actual_joint_pose], dtype=self._dtype, device=self._device)
                )

                st_time = time.time()
                result = self.mpc.step(current_state, 1)
                torch.cuda.synchronize()
                if tstep > 5:
                    self.mpc_time.append(time.time() - st_time)

                self._send_mpc_command(robot_context, result.action)

                if goal_handle is not None and tstep % 10 == 0:
                    feedback_msg = SendTrajectory.Feedback()
                    progress = 1.0 - min(result.metrics.pose_error.item() / 0.1, 1.0)
                    feedback_msg.step_progression = progress
                    goal_handle.publish_feedback(feedback_msg)

                if result.metrics.pose_error.item() < self.convergence_threshold:
                    converged = True
                    self.node.get_logger().info(
                        f"MPC converged at step {tstep} "
                        f"with error {result.metrics.pose_error.item():.4f}m"
                    )

                tstep += 1

            robot_context.stop_robot()

            if self.mpc_time:
                avg_time = sum(self.mpc_time) / len(self.mpc_time)
                self.node.get_logger().info(
                    f"MPC completed: {tstep} steps, avg time={avg_time*1000:.1f}ms/step"
                )

            return converged

        except Exception as e:
            self.node.get_logger().error(f"MPC execution error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            robot_context.stop_robot()
            return False

    def _send_mpc_command(self, robot_context, action_state: JointState):
        """Push a single MPC action to the robot."""
        position = (
            action_state.position[0].cpu().tolist()
            if action_state.position.dim() > 1
            else action_state.position.cpu().tolist()
        )

        if action_state.velocity is not None:
            velocity = (
                action_state.velocity[0].cpu().tolist()
                if action_state.velocity.dim() > 1
                else action_state.velocity.cpu().tolist()
            )
        else:
            velocity = [0.0] * len(position)

        if action_state.acceleration is not None:
            acceleration = (
                action_state.acceleration[0].cpu().tolist()
                if action_state.acceleration.dim() > 1
                else action_state.acceleration.cpu().tolist()
            )
        else:
            acceleration = [0.0] * len(position)

        joint_names = robot_context.get_joint_name()
        robot_context.set_command(
            joint_names, [velocity], [acceleration], [position]
        )
        robot_context.send_trajectrory()

    def cancel(self):
        self.is_goal_active = False
        self.node.get_logger().info("MPC execution cancelled")
