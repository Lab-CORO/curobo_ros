#!/usr/bin/env python3
"""
Model Predictive Control (MPC) planner.

This planner uses cuRobo's MPC solver for real-time closed-loop control,
recalculating the trajectory at each time step based on the current state.
"""

import time
from typing import Optional

import torch
from curobo.types.robot import JointState
from curobo.types.math import Pose
from curobo.rollout.rollout_base import Goal

from .trajectory_planner import TrajectoryPlanner, PlannerResult, ExecutionMode


class MPCPlanner(TrajectoryPlanner):
    """
    Model Predictive Control planner.

    Unlike open-loop planners, MPC continuously recalculates the optimal
    trajectory based on the current robot state, enabling reactive control.

    Execution flow:
    1. plan() - Setup MPC goal buffer
    2. execute() - Iterative loop: sense → replan → act → repeat
    """

    def __init__(self, node, config_wrapper):
        """
        Initialize MPC planner.

        Args:
            node: ROS2 node
            config_wrapper: ConfigWrapperMPC with MpcSolver configured
        """
        super().__init__(node, config_wrapper)

        # Store reference to MPC solver
        self.mpc = None

        # MPC state
        self.goal_buffer = None
        self.start_state = None
        self.goal_pose = None
        self.is_goal_active = False

        # Goal update from topic (for real-time tracking)
        self.latest_goal_from_topic = None

        # Performance tracking
        self.mpc_time = []

        # MPC parameters
        self.convergence_threshold = 0.01  # meters
        self.max_iterations = 1000

    def _get_execution_mode(self) -> ExecutionMode:
        """This planner uses closed-loop execution."""
        return ExecutionMode.CLOSED_LOOP

    def get_planner_name(self) -> str:
        """Return planner name."""
        return "Model Predictive Control (MPC)"

    def get_config_parameters(self) -> list:
        """List of ROS parameters used by this planner."""
        return [
            'convergence_threshold',
            'max_mpc_iterations',
        ]

    def set_mpc_solver(self, mpc_solver):
        """
        Set the MPC solver instance.

        Args:
            mpc_solver: Initialized MpcSolver instance
        """
        self.mpc = mpc_solver

    def plan(self, start_state: JointState, goal_pose: Pose, config: dict, robot_context=None) -> PlannerResult:
        """
        Setup MPC goal buffer for closed-loop execution.

        Unlike open-loop planners, this doesn't generate a full trajectory.
        Instead, it prepares the MPC solver for iterative execution.

        Args:
            start_state: Initial joint configuration
            goal_pose: Target end-effector pose
            config: Dictionary with keys:
                - convergence_threshold: Error threshold for goal (default 0.01)
                - max_iterations: Maximum MPC iterations (default 1000)
            robot_context: Optional RobotContext (not used for MPC, visualizes during execute)

        Returns:
            PlannerResult with goal buffer setup status
        """
        if self.mpc is None:
            return PlannerResult(
                success=False,
                message="MPC solver not initialized. Call set_mpc_solver() first.",
            )

        # Store for execution
        self.start_state = start_state
        self.goal_pose = goal_pose

        # Extract config
        self.convergence_threshold = config.get('convergence_threshold', 0.01)
        self.max_iterations = config.get('max_iterations', 1000)

        try:
            # Create goal
            goal = Goal(
                current_state=start_state,
                goal_pose=goal_pose,
            )

            # Setup MPC goal buffer
            self.goal_buffer = self.mpc.setup_solve_single(goal, 1)

            # Update goal in MPC
            self.mpc.update_goal(self.goal_buffer)

            self.is_goal_active = True

            self.node.get_logger().info(
                f"MPC goal setup: convergence={self.convergence_threshold}m, "
                f"max_iter={self.max_iterations}"
            )

            return PlannerResult(
                success=True,
                message="MPC goal buffer initialized",
                trajectory=None,  # No pre-computed trajectory for MPC
                metadata={
                    'goal_buffer': self.goal_buffer,
                    'convergence_threshold': self.convergence_threshold,
                    'max_iterations': self.max_iterations,
                }
            )

        except Exception as e:
            self.node.get_logger().error(f"MPC setup error: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())

            return PlannerResult(
                success=False,
                message=f"MPC setup error: {str(e)}",
            )

    def update_goal_pose(self, new_goal_pose: Pose) -> bool:
        """
        Update MPC goal during execution (for real-time tracking).

        Args:
            new_goal_pose: New target end-effector pose

        Returns:
            True if goal update succeeded
        """
        try:
            from curobo.rollout.rollout_base import Goal

            # Create new goal with updated pose
            goal = Goal(
                current_state=None,  # Will use current state from MPC loop
                goal_pose=new_goal_pose,
            )

            # Setup and update MPC goal buffer
            new_goal_buffer = self.mpc.setup_solve_single(goal, 1)
            self.mpc.update_goal(new_goal_buffer)

            # Update stored goal
            self.goal_pose = new_goal_pose
            self.goal_buffer = new_goal_buffer

            self.node.get_logger().debug(f"MPC goal updated to position: [{new_goal_pose.position.x:.3f}, {new_goal_pose.position.y:.3f}, {new_goal_pose.position.z:.3f}]")

            return True

        except Exception as e:
            self.node.get_logger().error(f"Failed to update MPC goal: {e}")
            return False

    def execute(self, robot_context, goal_handle=None) -> bool:
        """
        Execute MPC closed-loop control.

        Iteratively:
        1. Get current robot state
        2. Solve MPC optimization
        3. Send next action to robot
        4. Repeat until goal reached or timeout

        Args:
            robot_context: RobotContext for sending commands
            goal_handle: Optional ROS action goal handle for feedback

        Returns:
            True if goal reached successfully
        """
        if self.goal_buffer is None:
            self.node.get_logger().error("MPC not initialized. Call plan() first.")
            return False

        try:
            # Initialize MPC loop
            current_state = self.start_state.clone()
            converged = False
            tstep = 0
            traj_list = []
            self.mpc_time = []

            self.node.get_logger().info("Starting MPC execution loop")

            while not converged and self.is_goal_active:
                # Check for cancellation
                if goal_handle is not None and not goal_handle.is_active:
                    self.node.get_logger().warn("MPC execution cancelled")
                    robot_context.stop_robot()
                    return False

                # Check for goal update from topic (for real-time tracking)
                if self.latest_goal_from_topic is not None:
                    self.update_goal_pose(self.latest_goal_from_topic)
                    self.latest_goal_from_topic = None  # Consumed

                st_time = time.time()

                # MPC optimization step
                result = self.mpc.step(current_state, 1)

                # Synchronize CUDA
                torch.cuda.synchronize()

                # Track timing (skip first few iterations for warmup)
                if tstep > 5:
                    self.mpc_time.append(time.time() - st_time)

                # Update current state with the action
                current_state.copy_(result.action)

                # Store trajectory for visualization
                traj_list.append(result.action.get_state_tensor())

                # Send command to robot in real-time
                # For MPC, we send individual waypoints as they're computed
                self._send_mpc_command(robot_context, result.action)

                # Publish feedback (use generic SendTrajectory feedback)
                if goal_handle is not None and tstep % 10 == 0:
                    from curobo_msgs.action import SendTrajectory
                    feedback_msg = SendTrajectory.Feedback()
                    # SendTrajectory uses step_progression (0.0 to 1.0)
                    # Estimate progress based on pose error reduction
                    progress = 1.0 - min(result.metrics.pose_error.item() / 0.1, 1.0)  # Assume 10cm initial error
                    feedback_msg.step_progression = progress
                    goal_handle.publish_feedback(feedback_msg)

                # Log progress periodically
                if tstep % 10 == 0:
                    avg_time = sum(self.mpc_time[-10:]) / len(self.mpc_time[-10:]) if self.mpc_time else 0
                    self.node.get_logger().info(
                        f"MPC step {tstep}: error={result.metrics.pose_error.item():.4f}m, "
                        f"time={avg_time*1000:.1f}ms"
                    )

                # Check convergence
                if result.metrics.pose_error.item() < self.convergence_threshold:
                    converged = True
                    self.node.get_logger().info(
                        f"MPC converged at step {tstep} with error {result.metrics.pose_error.item():.4f}m"
                    )

                tstep += 1

                # Safety timeout
                if tstep > self.max_iterations:
                    self.node.get_logger().warn(
                        f"MPC max iterations ({self.max_iterations}) reached without convergence"
                    )
                    break

            # Stop robot at end
            robot_context.stop_robot()

            # Log statistics
            if self.mpc_time:
                avg_time = sum(self.mpc_time) / len(self.mpc_time)
                self.node.get_logger().info(
                    f"MPC completed: {tstep} steps, avg time={avg_time*1000:.1f}ms/step"
                )

            return converged

        except Exception as e:
            self.node.get_logger().error(f"MPC execution error: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())
            robot_context.stop_robot()
            return False

    def _send_mpc_command(self, robot_context, action_state: JointState):
        """
        Send a single MPC action to the robot.

        For real-time MPC, we send commands as they're computed rather than
        batching them like in open-loop planning.

        Args:
            robot_context: RobotContext for command sending
            action_state: JointState with position, velocity, acceleration
        """
        # Convert tensors to lists
        # Note: action_state has batch dimension [1, n_joints], need to extract first element
        position = action_state.position[0].cpu().tolist() if action_state.position.dim() > 1 else action_state.position.cpu().tolist()

        if action_state.velocity is not None:
            velocity = action_state.velocity[0].cpu().tolist() if action_state.velocity.dim() > 1 else action_state.velocity.cpu().tolist()
        else:
            velocity = [0.0] * len(position)

        if action_state.acceleration is not None:
            acceleration = action_state.acceleration[0].cpu().tolist() if action_state.acceleration.dim() > 1 else action_state.acceleration.cpu().tolist()
        else:
            acceleration = [0.0] * len(position)

        # Get joint names
        joint_names = robot_context.get_joint_name()

        # Send single command (wrapped in list for trajectory format)
        robot_context.set_command(
            joint_names,
            [velocity],      # List of waypoints (1 waypoint)
            [acceleration],  # List of waypoints (1 waypoint)
            [position]       # List of waypoints (1 waypoint)
        )
        self.node.get_logger().info(f"joint state:{position}")
        robot_context.send_trajectrory()

    def cancel(self):
        """Cancel the current MPC execution."""
        self.is_goal_active = False
        self.node.get_logger().info("MPC execution cancelled")
