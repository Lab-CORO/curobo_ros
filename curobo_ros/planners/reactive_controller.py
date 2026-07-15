#!/usr/bin/env python3
"""
Abstract base class for *reactive* (closed-loop) controllers.

This is the closed-loop sibling of :class:`SinglePlanner` (which is the shared
base for open-loop, MotionPlanner-based planners). It exists because cuRobo
models reactive control and motion generation as **different wrappers over the
same core** (shared robot model + world collision model, see
https://nvlabs.github.io/curobo/latest/concepts/index.html): they differ only by
horizon and update frequency, not by structure. curobo_ros mirrors that: a
reactive controller is a *thin ROS wrapper* around a cuRobo reactive solver
(e.g. ``ModelPredictiveControl``).

Design
------
``ReactiveController`` owns all the ROS / robot / perception plumbing of the
control loop **once**, so a concrete controller only implements the few
cuRobo-specific steps:

    build_solver()        -> create the cuRobo solver from the SHARED context
    setup(state, goal)    -> set the initial goal on the solver
    step(state)           -> one optimization step, returns the next action
    apply_live_goal(raw)  -> retarget the goal during execution
    is_converged()        -> stop condition

Adding a new reactive control = subclass this + register it in
``PlannerFactory._PLANNER_CATALOG``. The same ``SetPlanner`` / ``GetPlanners``
switch then works for it, and it automatically shares the node's single context
(robot, obstacles, scene, collision cache).
"""

import time
import traceback
from abc import abstractmethod
from typing import Any, Optional

import torch
from curobo.types import JointState, Pose, GoalToolPose

from .trajectory_planner import TrajectoryPlanner, PlannerResult, ExecutionMode
from curobo_msgs.action import SendTrajectory


class ReactiveController(TrajectoryPlanner):
    """Closed-loop controller base. Subclasses wrap a cuRobo reactive solver."""

    def __init__(self, node, config_wrapper):
        super().__init__(node, config_wrapper)

        # The cuRobo reactive solver, built lazily from the shared context.
        self.solver = None

        # Goal / loop state.
        self.start_state: Optional[JointState] = None
        self.goal: Any = None
        self.is_goal_active = False

        # Raw [x, y, z, qw, qx, qy, qz] written from the ROS thread (topic) and
        # consumed on the control-loop thread to avoid racing CUDA graph capture.
        self.latest_goal = None

        # Tunables (overwritten from the per-call config in plan()).
        self.convergence_threshold = 0.01      # meters
        self.max_iterations = 1000
        # Refresh the perception ESDF every N steps (0 disables, 1 = every step).
        # À 20 (~3s à 7Hz) le monde de collision du MPC était trop lent pour un
        # obstacle dynamique (main) — le bras touchait avant la mise à jour. 2 ≈
        # rythme caméra (5Hz), possible depuis que la voxelization est sur GPU
        # (plus de fallback CPU ~10s). cf. debug 2026-07-15.
        self.perception_refresh_period = 2

        # Latest scalar position error, written by step(), read by is_converged().
        self._last_position_error = float('inf')
        # Cartesian target (xyz tensor), set by _set_target, read by FK error.
        self._target_position = None
        self._step_times = []
        # Last commanded action — fed back so the next current_state carries
        # velocity/acceleration continuity (without it the solver restarts from
        # rest every step and the arm never builds up motion).
        self._last_action = None
        # Wall-clock of the last status log (throttled, rate-independent).
        self._last_log_time = 0.0

        # Device/dtype for building tensors on the hot path.
        self._device = getattr(config_wrapper, '_device', torch.device('cuda'))
        self._dtype = getattr(config_wrapper, '_ops_dtype', torch.float32)

    def _get_execution_mode(self) -> ExecutionMode:
        return ExecutionMode.CLOSED_LOOP

    # ------------------------------------------------------------------
    # Solver lifecycle
    # ------------------------------------------------------------------

    def ensure_solver(self):
        """Build the cuRobo solver once, lazily, from the shared context."""
        if self.solver is None:
            self.solver = self.build_solver()
        return self.solver

    def rebuild_solver(self):
        """Recreate the solver from scratch (e.g. after a collision-cache change).

        The collision cache size is fixed at solver creation, so a change
        requires a full rebuild rather than a world update.
        """
        self.solver = None
        return self.ensure_solver()

    # ------------------------------------------------------------------
    # cuRobo-specific hooks (implemented by concrete controllers)
    # ------------------------------------------------------------------

    @abstractmethod
    def build_solver(self):
        """Create and return the cuRobo reactive solver from ``self.config_wrapper``.

        The shared context exposes everything needed: ``robot_config_file``,
        ``obstacle_manager.get_scene()``, ``collision_cache``, ``_device`` /
        ``_ops_dtype``. Implementations should also publish the solver where the
        node expects it (e.g. ``self.node.mpc``) so world/cache updates reach it.
        """
        raise NotImplementedError

    @abstractmethod
    def setup(self, start_state: JointState, goal_request: Any) -> bool:
        """Set the initial goal on the solver. Return True on success."""
        raise NotImplementedError

    @abstractmethod
    def step(self, current_state: JointState) -> JointState:
        """Run one optimization step and return the next action JointState.

        Implementations must also update ``self._last_position_error``.
        """
        raise NotImplementedError

    @abstractmethod
    def apply_live_goal(self, raw_goal) -> bool:
        """Retarget the goal from a raw [x,y,z,qw,qx,qy,qz] list during execution."""
        raise NotImplementedError

    def update_world(self, scene) -> None:
        """Push the shared Scene into this controller's collision model.

        Reactive solvers each hold their own collision checker, so the node
        delegates world updates here (instead of reaching into solver internals).
        Default is a no-op; concrete controllers override (e.g. MPC reloads its
        scene_collision_checker, retarget updates its IK solvers).
        """
        return None

    # ------------------------------------------------------------------
    # Shared cuRobo helpers (target pose + FK error) usable by every
    # reactive controller — solver exposes tool_frames and FK either directly
    # (MPC) or via .kinematics (retargeter).
    # ------------------------------------------------------------------

    def _set_target(self, raw) -> GoalToolPose:
        """Store the target xyz (for FK error) and build the tool-pose goal.

        Passes ordered_tool_frames + num_goalset like the official cuRobo reactive
        example so the goal buffer is shaped exactly as the solver expects.
        """
        self._target_position = torch.tensor(
            raw[0:3], dtype=self._dtype, device=self._device
        )
        return GoalToolPose.from_poses(
            {self.solver.tool_frames[0]: Pose.from_list(list(raw))},
            ordered_tool_frames=self.solver.tool_frames,
            num_goalset=1,
        )

    def _compute_ee_position(self, current_state: JointState):
        """Current end-effector position via the solver's forward kinematics."""
        fk = getattr(self.solver, 'compute_kinematics', None)
        if fk is None:
            fk = self.solver.kinematics.compute_kinematics
        kin = fk(current_state)
        return kin.tool_poses.position.reshape(-1, 3)[0]  # [B,H,L,3] -> first link

    def _fk_position_error(self, current_state: JointState) -> float:
        """Real Cartesian distance (m) between the current EE and the target."""
        if self._target_position is None:
            return float('inf')
        try:
            ee = self._compute_ee_position(current_state)
            return float(torch.linalg.norm(ee - self._target_position).item())
        except Exception:
            return float('inf')

    def is_on_target(self) -> bool:
        """Signal (NOT a stop condition): the arm is within tolerance of the goal.

        Reactive control keeps servoing even when on target, so this only drives
        the `on_target` feedback flag — it never ends the control loop.
        """
        return self._last_position_error < self.convergence_threshold

    def get_position_error(self) -> float:
        """Latest scalar position error (meters)."""
        return self._last_position_error

    # ------------------------------------------------------------------
    # Planning: set up the reactive goal (no full trajectory is produced).
    # ------------------------------------------------------------------

    def plan(self, start_state: JointState, goal_request: Any, config: dict,
             robot_context: Optional[Any] = None) -> PlannerResult:
        self.ensure_solver()
        if self.solver is None:
            return PlannerResult(
                success=False,
                message=f"{self.get_planner_name()} solver not initialized.",
            )

        self.convergence_threshold = config.get('convergence_threshold', 0.01)
        self.max_iterations = config.get('max_iterations', 1000)
        self.start_state = start_state

        try:
            with self.node.gpu_lock:
                setup_ok = self.setup(start_state, goal_request)
            if not setup_ok:
                return PlannerResult(success=False, message="Failed to set reactive goal")
            self.is_goal_active = True

            if robot_context is not None:
                self._init_robot_at_start(robot_context, start_state)

            self.node.get_logger().info(
                f"{self.get_planner_name()} goal set: "
                f"convergence={self.convergence_threshold}m, max_iter={self.max_iterations}"
            )
            return PlannerResult(
                success=True,
                message=f"{self.get_planner_name()} goal set",
                trajectory=None,
                metadata={
                    'convergence_threshold': self.convergence_threshold,
                    'max_iterations': self.max_iterations,
                },
            )
        except Exception as e:
            self.node.get_logger().error(f"{self.get_planner_name()} setup error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            return PlannerResult(success=False, message=f"Reactive setup error: {e}")


    def execute(self, robot_context, goal_handle=None) -> bool:
        if not self.is_goal_active or self.solver is None:
            self.node.get_logger().error(
                f"{self.get_planner_name()} not initialized. Call plan() first."
            )
            return False

        try:
            tstep = 0
            self._step_times = []
            self._last_action = None
            self._last_log_time = 0.0
            self.node.get_logger().info(f"Starting {self.get_planner_name()} servo loop")

            # Initialize the solver state from the robot once, then advance it
            # from the solver's own prediction each step (see the loop below).
            current_state = self._read_state(robot_context)

            # Reactive control runs CONTINUOUSLY: reaching the target is only a
            # signal (on_target), never a stop condition. The loop ends solely on
            # cancel (or error). Without an action handle, max_iterations is a
            # safety cap for non-action callers.
            while self.is_goal_active:
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    self.node.get_logger().info(f"{self.get_planner_name()} cancel requested")
                    break
                if goal_handle is None and tstep >= self.max_iterations:
                    break

                # Periodically refresh the perception-based collision world so the
                # controller reacts to obstacles seen by the cameras. Throttled
                # (every N steps) since recomputing the ESDF is heavier than a step.
                if (self.perception_refresh_period > 0
                        and tstep % self.perception_refresh_period == 0
                        and hasattr(self.node, 'refresh_perception_world')):
                    self.node.refresh_perception_world()

                # Consume a pending live goal on the loop thread only.
                if self.latest_goal is not None:
                    raw = self.latest_goal
                    self.latest_goal = None
                    self.apply_live_goal(raw)

                st_time = time.time()
                action = self.step(current_state)  # step() already syncs (FK .item())
                if tstep > 5:
                    self._step_times.append(time.time() - st_time)

                self._send_command(robot_context, action)

                # Close the loop with the REAL robot position (a fast,
                # non-blocking read of the joint_states subscriber's latest
                # cached value — no wait, so no new lag). Purely trusting the
                # solver's own predicted position (position-only, from
                # _state_from_action) drifts from reality once a horizon takes
                # real wall-clock time to execute (observed on hardware: MPC
                # correcting toward an imagined position -> growing tracking
                # error, unstable motion). Velocity/acceleration stay the
                # solver's own prediction — the driver doesn't give reliable
                # velocity feedback, and the MPC needs SOME dynamic-continuity
                # estimate for warm-starting.
                predicted_state = self._state_from_action(action)
                current_state = self._close_state_loop(robot_context, predicted_state)
                self._last_action = action

                if goal_handle is not None and tstep % 5 == 0:
                    self._publish_feedback(goal_handle, action)

                now = time.time()
                if now - self._last_log_time > 1.0:
                    self._last_log_time = now
                    self.node.get_logger().info(
                        f"{self.get_planner_name()}: error="
                        f"{self.get_position_error():.4f}m on_target={self.is_on_target()}"
                    )

                tstep += 1

            robot_context.stop_robot()

            if self._step_times:
                avg_time = sum(self._step_times) / len(self._step_times)
                self.node.get_logger().info(
                    f"{self.get_planner_name()} stopped: {tstep} steps, "
                    f"avg time={avg_time * 1000:.1f}ms/step"
                )

            # A clean stop (cancel / safety cap) is a successful session end.
            return True

        except Exception as e:
            self.node.get_logger().error(f"{self.get_planner_name()} execution error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            robot_context.stop_robot()
            return False

    def _publish_feedback(self, goal_handle, action_state):
        """Publish the reactive status through the action feedback (no status topic)."""
        err = self.get_position_error()
        on_target = self.is_on_target()
        fb = SendTrajectory.Feedback()
        fb.state = "ON_TARGET" if on_target else "TRACKING"
        fb.on_target = bool(on_target)
        fb.position_error = float(err) if err != float('inf') else -1.0
        fb.step_progression = (
            float(1.0 - min(err / 0.1, 1.0)) if err != float('inf') else 0.0
        )
        try:
            pos = action_state.position
            if pos.dim() == 3:
                pos = pos[:, -1, :]  # full-horizon action: report the last (current-target) point
            fb.joint_command.position = (pos[0] if pos.dim() > 1 else pos).cpu().tolist()
        except Exception:
            pass
        goal_handle.publish_feedback(fb)

    def cancel(self):
        self.is_goal_active = False
        self.node.get_logger().info(f"{self.get_planner_name()} execution cancelled")

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------

    def _read_state(self, robot_context) -> JointState:
        """Initial solver state from the robot's joint positions.

        Matches the official cuRobo reactive example: joint_names are labelled and
        velocity/acceleration are explicitly zeroed so the solver's dynamic state
        is well-defined at start (an unlabelled/vel-less state weakens tracking).
        """
        actual_joint_pose = robot_context.get_joint_pose()
        pos = torch.tensor([actual_joint_pose], dtype=self._dtype, device=self._device)
        js = JointState.from_position(pos, joint_names=self.solver.joint_names)
        js.velocity = torch.zeros_like(pos)
        js.acceleration = torch.zeros_like(pos)
        return js

    @staticmethod
    def _row(t):
        """Return tensor as a 2D [1, D] row (or None)."""
        if t is None:
            return None
        t = t.detach().clone()
        return t if t.dim() > 1 else t.unsqueeze(0)

    def _state_from_action(self, action: JointState) -> JointState:
        """Build the next solver state from a commanded action (pos+vel+acc).

        For a full-horizon action (position ``[1, horizon, dof]``, from MPC's
        ``optimize_action_sequence``), only the LAST horizon point is used to
        warm-start the next optimize call — matches cuRobo's own
        reactive_control example's state-continuity pattern.
        """
        pos, vel, acc = action.position, getattr(action, 'velocity', None), getattr(action, 'acceleration', None)
        if pos.dim() == 3:
            pos = pos[:, -1, :]
            vel = vel[:, -1, :] if vel is not None else None
            acc = acc[:, -1, :] if acc is not None else None

        state = JointState.from_position(self._row(pos))
        vel = self._row(vel)
        acc = self._row(acc)
        if vel is not None:
            state.velocity = vel
        if acc is not None:
            state.acceleration = acc
        return state

    def _close_state_loop(self, robot_context, predicted_state: JointState) -> JointState:
        """Replace the predicted POSITION with the robot's real, latest joint
        feedback; keep the solver's own PREDICTED velocity/acceleration.

        ``robot_context.get_joint_pose()`` is a plain attribute read of the
        value already cached by the async joint_states subscriber callback —
        no wait, so no new lag. Without this, the MPC corrects toward a
        purely-imagined position that drifts from where the arm actually is
        once a horizon takes real wall-clock time to execute (observed on
        hardware: growing tracking error, unstable motion).

        Velocity is intentionally kept PREDICTED, not real, even though real
        velocity IS available (dsr_hw_interface2.cpp reads actual_joint_velocity
        from the same real-time struct as position — verified in source).
        Feeding the REAL velocity back here was tried and made things worse:
        the outgoing hardware safety clamp (JointSpeedStrategy) deliberately
        throttles commanded velocity below what the solver just planned: real
        velocity always lags. Reporting that throttled reality back as the
        solver's own state told it "you're going much slower than you
        decided", which the warm-started optimizer (only 25 iterations) can't
        reconcile each cycle without oscillating/diverging. The safety clamp
        legitimately uses real velocity (JointSpeedStrategy._clamp_velocities)
        — that's a downstream actuation limit, not the planner's own model of
        its trajectory, and the two must stay decoupled.
        """
        real_pose = robot_context.get_joint_pose()
        pos = torch.tensor([real_pose], dtype=self._dtype, device=self._device)
        state = JointState.from_position(pos, joint_names=self.solver.joint_names)
        state.velocity = predicted_state.velocity
        state.acceleration = predicted_state.acceleration
        return state

    def _init_robot_at_start(self, robot_context, start_state: JointState):
        """Seed the robot/visualization at the start configuration."""
        start_position = start_state.position[0].cpu().tolist()
        if robot_context.robot_strategy is not None:
            joint_names = robot_context.robot_strategy.get_joint_name()
        else:
            joint_names = self.solver.kinematics.joint_names
        n = len(start_position)
        robot_context.set_command(joint_names, [[0.0] * n], [[0.0] * n], [start_position])
        self.node.get_logger().info(
            f"{self.get_planner_name()}: robot init'd at "
            f"{[f'{x:.3f}' for x in start_position]}"
        )

    def _send_command(self, robot_context, action_state: JointState):
        """Push a control action to the robot.

        Two shapes are supported:
          - Single point: position ``[dof]`` or ``[1, dof]`` -> one
            JointTrajectory point (open-loop planners, RetargetController).
          - Full horizon: position ``[1, horizon, dof]`` (from MPC's
            optimize_action_sequence) -> ALL horizon points are streamed as a
            multi-point JointTrajectory. optimize_action_sequence re-optimizes
            fully every call (slow, ~1s on this hardware); sending only a
            single point per call left the robot idle between calls then
            jumping to a very different velocity — a discontinuity that
            tripped the Doosan's acceleration limit. Streaming the whole
            horizon gives it a smooth sequence to execute in between.
        """
        pos_t, vel_t, acc_t = action_state.position, action_state.velocity, action_state.acceleration

        if pos_t.dim() == 3:
            position = pos_t[0].cpu().tolist()
            velocity = vel_t[0].cpu().tolist() if vel_t is not None else [[0.0] * pos_t.shape[-1]] * pos_t.shape[1]
            acceleration = acc_t[0].cpu().tolist() if acc_t is not None else [[0.0] * pos_t.shape[-1]] * pos_t.shape[1]
        else:
            p = pos_t[0] if pos_t.dim() > 1 else pos_t
            position = [p.cpu().tolist()]
            v = vel_t[0] if (vel_t is not None and vel_t.dim() > 1) else vel_t
            velocity = [v.cpu().tolist() if v is not None else [0.0] * len(position[0])]
            a = acc_t[0] if (acc_t is not None and acc_t.dim() > 1) else acc_t
            acceleration = [a.cpu().tolist() if a is not None else [0.0] * len(position[0])]

        joint_names = robot_context.get_joint_name()
        robot_context.set_command(joint_names, velocity, acceleration, position)
        robot_context.send_trajectrory()
