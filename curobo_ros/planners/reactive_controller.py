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

import threading
import time
import traceback
from abc import abstractmethod
from collections import deque
from contextlib import nullcontext
from typing import Any, Optional

import torch
from curobo.types import JointState, Pose, GoalToolPose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

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


        self._live_goal_lock = threading.Lock()
        self._latest_goal = None
        self._latest_goal_fresh = False

        # Tunables (overwritten from the per-call config in plan()).
        self.convergence_threshold = 0.03      # meters

        self.convergence_threshold_rad = 0.05  # radians

        self.convergence_hold_steps = 1
        self.max_iterations = 1000
        self.perception_refresh_period = 2

        # Latest scalar position error, written by step(), read by is_converged().
        self._last_position_error = float('inf')
        # Signed per-axis component (base frame) of the same error, set
        # alongside _last_position_error wherever a controller computes it
        # (see _fk_position_error_xyz). None until the first real measurement.
        self._last_position_error_xyz: Optional[torch.Tensor] = None
        # Latest scalar ORIENTATION error (rad), written by step().
        self._last_orientation_error = float('inf')
        # INSTANTANEOUS FK error of the controller's own next commanded
        # point (e.g. the very first point of the current plan), as opposed
        # to the FK-measured pair above which is against the real/fed-back
        # state. NOT the solver's raw ``result.position_error`` -- measured
        # (curobo_ros's probe_controller_error.py) to be a horizon-end
        # convergence metric that stays near zero even with the real state
        # far from the goal, so it is useless as a "right now" error. inf
        # means "not available" (default; see get_controller_position_error())
        # — subclasses that expose one overwrite these in step().
        self._last_controller_position_error = float('inf')
        self._last_controller_orientation_error = float('inf')
        # Consecutive steps with BOTH errors inside tolerance; see _update_hold.
        self._hold_count = 0
        # Cartesian target (xyz tensor), set by _set_target, read by FK error.
        self._target_position = None
        # Cartesian target orientation (wxyz quaternion tensor), same source.
        self._target_quaternion = None
        self._step_times = []
        self._last_action = None
        # Wall-clock of the last status log (throttled, rate-independent).
        self._last_log_time = 0.0

        self._timer_cb_group = None
        self._pending_lock = threading.Lock()
        self._pending_cv = threading.Condition(self._pending_lock)
        self._pending_queue = deque()
        self._paced_send_error = False
        self._min_queue_depth_seen = None
        self._starvation_ticks = 0


        self._diag_backpressure_wait_ms = 0.0
        self._diag_perception_ms = 0.0
        self._diag_live_goal_ms = 0.0
        self._diag_loop_iter = 0
        self._diag_cheap_ms_before_resolve = 0.0
        self._diag_batch_wall_ms = 0.0

        self._batch_size = 1

        # Device/dtype for building tensors on the hot path.
        self._device = getattr(config_wrapper, '_device', torch.device('cuda'))
        self._dtype = getattr(config_wrapper, '_ops_dtype', torch.float32)

    def _get_execution_mode(self) -> ExecutionMode:
        return ExecutionMode.CLOSED_LOOP

    # ------------------------------------------------------------------
    # Solver lifecycle
    # ------------------------------------------------------------------

    def ensure_solver(self):
        """Build the cuRobo solver once, lazily, from the shared context.

        Single funnel for both first-time lazy construction AND
        rebuild_solver() (which just clears self.solver and calls this) --
        replaying any currently-attached object's spheres here covers both:
        a solver that never saw the attach because it didn't exist yet, and
        one that lost its sphere state to a rebuild.
        """
        if self.solver is None:
            self.solver = self.build_solver()
            try:
                replay = getattr(self.node, 'replay_attachment', None)
                if replay is not None:
                    replay(self)
            except Exception as e:
                self.node.get_logger().warn(
                    f"{self.get_planner_name()}: attachment replay failed: {e}")
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

    def step_batch(self, current_state: JointState, n: int) -> list:
        """Produce ``n`` actions for one producer-loop iteration (default: call
        ``step()`` n times). Override only when the underlying solver already
        computes all ``n`` actions on a single ``step()`` call and exposes a
        cheaper way to read the rest back out (see LBFGSController's override,
        which skips n-1 redundant solver-side calls this way).
        """
        batch = []
        cheap_ms_accum = 0.0
        for _ in range(n):
            self._diag_cheap_ms_before_resolve = cheap_ms_accum
            t_call = time.monotonic()
            batch.append(self.step(current_state))
            cheap_ms_accum += (time.monotonic() - t_call) * 1000.0
        return batch

    @abstractmethod
    def apply_live_goal(self, raw_goal) -> bool:
        """Retarget the goal from a raw [x,y,z,qw,qx,qy,qz] list during execution."""
        raise NotImplementedError

    def update_world(self, scene) -> None:
        """Push the shared Scene into this controller's collision model.

        """
        return None

    def has_solver(self) -> bool:
        """TrajectoryPlanner override: each reactive controller owns its own
        """
        return self.solver is not None

    def attachment_managers(self) -> list:
        """TrajectoryPlanner override: this controller's cuRobo AttachmentManager(s).

        Covers MPPIController and LBFGSController (both build a
        ModelPredictiveControl / MPCSolver, exposing .core and .ik_solver)
        without either needing its own override. solver.ik_solver.core and
        solver.core resolve to the SAME KinematicsParams in this build (see
        AttachmentServices for the analysis) -- included anyway for
        forward-compatibility, deduped by the caller on
        id(am.kinematics_params).

        RetargetController overrides this to [] -- MotionRetargeter has no
        .core and this default would raise on it.
        """
        if self.solver is None:
            return []
        managers = [self.solver.core.attachment_manager]
        ik_solver = getattr(self.solver, 'ik_solver', None)
        if ik_solver is not None:
            managers.append(ik_solver.core.attachment_manager)
        return managers


    def _set_target(self, raw) -> GoalToolPose:
        """Store the target xyz (for FK error) and build the tool-pose goal.
        """

        self._hold_count = 0
        self._target_position = torch.tensor(
            raw[0:3], dtype=self._dtype, device=self._device
        )

        self._target_quaternion = torch.tensor(
            raw[3:7], dtype=self._dtype, device=self._device
        )
        return GoalToolPose.from_poses(
            {self.solver.tool_frames[0]: Pose.from_list(list(raw))},
            ordered_tool_frames=self.solver.tool_frames,
            num_goalset=1,
        )

    def _compute_ee_pose(self, current_state: JointState):
        """Current end-effector (position, quaternion) via the solver's FK.

        One FK call for both, since the two error metrics are always wanted
        together and the call is the expensive part.
        """
        fk = getattr(self.solver, 'compute_kinematics', None)
        if fk is None:
            fk = self.solver.kinematics.compute_kinematics
        kin = fk(current_state)
        # [B,H,L,*] -> first link of the first batch/horizon entry
        return (kin.tool_poses.position.reshape(-1, 3)[0],
                kin.tool_poses.quaternion.reshape(-1, 4)[0])

    def _compute_ee_position(self, current_state: JointState):
        """Current end-effector position via the solver's forward kinematics."""
        return self._compute_ee_pose(current_state)[0]

    def _fk_position_error_xyz(self, current_state: JointState) -> Optional[torch.Tensor]:
        """Signed per-axis Cartesian error (m), base frame: ee - target.

        None if no target is set or FK fails. ``_fk_position_error`` is this
        vector's norm -- kept as a separate call so callers that only need
        the scalar (most of them) don't have to unpack a tensor, while
        callers reporting per-axis error (_close_state_loop) can call this
        directly instead of triggering a second FK pass.
        """
        if self._target_position is None:
            return None
        try:
            ee = self._compute_ee_position(current_state)
            return ee - self._target_position
        except Exception:
            return None

    def _fk_position_error(self, current_state: JointState) -> float:
        """Real Cartesian distance (m) between the current EE and the target."""
        comp = self._fk_position_error_xyz(current_state)
        if comp is None:
            return float('inf')
        return float(torch.linalg.norm(comp).item())

    def _fk_orientation_error(self, current_state: JointState) -> float:
        """Real angular distance (rad) between the current EE and the target
        """
        if self._target_quaternion is None:
            return float('inf')
        try:
            _, quat = self._compute_ee_pose(current_state)
            dot = torch.dot(quat.reshape(-1), self._target_quaternion.reshape(-1)).abs()
            return float((2.0 * torch.acos(dot.clamp(max=1.0))).item())
        except Exception:
            return float('inf')

    def _within_tolerances(self) -> bool:
        """Both errors inside their tolerance, right now (no hold requirement)."""
        return (self._last_position_error < self.convergence_threshold
                and self._last_orientation_error < self.convergence_threshold_rad)

    def _update_hold(self) -> int:
        """Advance the consecutive-in-tolerance counter. Call once per step.
        """
        if self._within_tolerances():
            self._hold_count += 1
        else:
            self._hold_count = 0
        return self._hold_count

    def is_on_target(self) -> bool:
        """Signal (NOT a stop condition): the arm has CONVERGED on the goal.
        """
        return (self._within_tolerances()
                and self._hold_count >= self.convergence_hold_steps)

    def get_orientation_error(self) -> float:
        """Latest scalar orientation error (radians)."""
        return self._last_orientation_error

    def get_hold_count(self) -> int:
        """Consecutive steps spent inside both tolerances."""
        return self._hold_count

    def get_position_error(self) -> float:
        """Latest scalar position error (meters)."""
        return self._last_position_error

    def get_position_error_xyz(self) -> Optional[torch.Tensor]:
        """Latest signed per-axis position error (meters, base frame), or
        None if the active controller never set it (see _last_position_error_xyz).
        """
        return self._last_position_error_xyz

    def get_controller_position_error(self) -> float:
        """Latest INSTANTANEOUS controller position error (meters), inf if unavailable.

        FK error of the controller's own next commanded point against the
        goal — the solver's best estimate of "how far will I be right after
        this solve", as opposed to the FK-measured error against the
        real/fed-back state returned by ``get_position_error()``. NOT the
        solver's raw internal cost/convergence metric — see the field comment
        in ``SendTrajectory.action``.
        """
        return self._last_controller_position_error

    def get_controller_orientation_error(self) -> float:
        """Latest instantaneous controller orientation error (radians), inf if unavailable."""
        return self._last_controller_orientation_error

    def set_live_goal(self, raw_goal) -> None:
        """Deposit a live goal update. Called from the ROS topic thread.
        """
        with self._live_goal_lock:
            self._latest_goal = list(raw_goal)
            self._latest_goal_fresh = True

    def _take_live_goal(self):
        """Atomically take-and-clear the pending live goal, or None if stale.
        """
        with self._live_goal_lock:
            if not self._latest_goal_fresh:
                return None
            raw = self._latest_goal
            self._latest_goal = None
            self._latest_goal_fresh = False
            return raw

    def _step_guard(self):
        """gpu_lock for a step() that may capture a CUDA graph, else no lock.
        """
        take = getattr(self.node, 'take_graph_capture_pending', None)
        if take is not None and take():
            return self.node.gpu_lock
        return nullcontext()

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
        self.convergence_threshold_rad = config.get('convergence_threshold_rad', 0.05)
        self.convergence_hold_steps = int(config.get('convergence_hold_steps', 1))
        self.max_iterations = config.get('max_iterations', 1000)
        self.start_state = start_state

        try:
            with self.node.gpu_lock:
                setup_ok = self.setup(start_state, goal_request)
            if not setup_ok:
                return PlannerResult(success=False, message="Failed to set reactive goal")

            self._take_live_goal()
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
        """Dispatch to the paced (producer/consumer) or immediate servo loop.

        """
        if not self.is_goal_active or self.solver is None:
            self.node.get_logger().error(
                f"{self.get_planner_name()} not initialized. Call plan() first."
            )
            return False

        interval = getattr(self, '_command_interval', 0.0)
        if interval > 0.0:
            return self._execute_paced(robot_context, goal_handle, interval)
        return self._execute_immediate(robot_context, goal_handle)

    def _execute_immediate(self, robot_context, goal_handle=None) -> bool:
        try:
            tstep = 0
            self._step_times = []
            self._last_action = None
            self._last_log_time = 0.0
            self.node.get_logger().info(f"Starting {self.get_planner_name()} servo loop")

            # Initialize the solver state from the robot once, then advance it
            # from the solver's own prediction each step (see the loop below).
            current_state = self._read_state(robot_context)

            while self.is_goal_active:
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    self.node.get_logger().info(f"{self.get_planner_name()} cancel requested")
                    break
                if goal_handle is None and tstep >= self.max_iterations:
                    break

                if (self.perception_refresh_period > 0
                        and tstep % self.perception_refresh_period == 0
                        and hasattr(self.node, 'refresh_perception_world')):
                    self.node.refresh_perception_world(active_only=True)


                raw = self._take_live_goal()
                if raw is not None:
                    try:
                        with self.node.gpu_lock:
                            self.apply_live_goal(raw)
                    except Exception as e:
                        self.node.get_logger().error(
                            f"{self.get_planner_name()}: live goal rejected "
                            f"({e}) - keeping previous goal",
                            throttle_duration_sec=1.0,
                        )

                st_time = time.time()
                with self._step_guard():
                    action = self.step(current_state)  # step() already syncs (FK .item())
                if tstep > 5:
                    self._step_times.append(time.time() - st_time)

                self._send_command(robot_context, action)

                predicted_state = self._state_from_action(action)
                current_state = self._close_state_loop(robot_context, predicted_state)
                self._last_action = action

                # if goal_handle is not None and tstep % 5 == 0:
                self._publish_feedback(goal_handle, action)

                now = time.time()
                if now - self._last_log_time > 1.0 and bool(self.node.get_parameter('mpc_debug').value):
                    self._last_log_time = now
                    self.node.get_logger().info(
                        f"{self.get_planner_name()}: error="
                        f"{self.get_position_error():.4f}m "
                        f"{self.get_orientation_error():.4f}rad on_target={self.is_on_target()}"
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

    def _execute_paced(self, robot_context, goal_handle, interval: float) -> bool:
        """Producer/consumer servo loop
        """
        try:
            tstep = 0
            loop_iter = 0
            batch_size = max(1, int(getattr(self, '_batch_size', 1)))
            queue_max_depth = batch_size + 1
            self._step_times = []
            self._last_action = None
            self._last_log_time = 0.0
            with self._pending_lock:
                self._pending_queue.clear()
            self._paced_send_error = False
            self._min_queue_depth_seen = None
            self._starvation_ticks = 0
            self.node.get_logger().info(
                f"Starting {self.get_planner_name()} servo loop "
                f"(paced, interval={interval}s, batch={batch_size}, "
                f"queue_max_depth={queue_max_depth})"
            )

            current_state = self._read_state(robot_context)

            if self._timer_cb_group is None:
                self._timer_cb_group = MutuallyExclusiveCallbackGroup()
            timer = self.node.create_timer(
                interval, lambda: self._on_send_tick(robot_context, goal_handle),
                callback_group=self._timer_cb_group,
            )

            try:
                while self.is_goal_active:
                    if goal_handle is not None and goal_handle.is_cancel_requested:
                        self.node.get_logger().info(f"{self.get_planner_name()} cancel requested")
                        break
                    if goal_handle is None and tstep >= self.max_iterations:
                        break
                    if self._paced_send_error:
                        break

                    t_wait0 = time.monotonic()
                    with self._pending_cv:
                        while (len(self._pending_queue) >= queue_max_depth
                               and self.is_goal_active and not self._paced_send_error):
                            if goal_handle is not None and goal_handle.is_cancel_requested:
                                break
                            self._pending_cv.wait(timeout=0.1)
                    self._diag_backpressure_wait_ms = (time.monotonic() - t_wait0) * 1000.0

                    if not self.is_goal_active or self._paced_send_error:
                        continue
                    if goal_handle is not None and goal_handle.is_cancel_requested:
                        continue

                    self._diag_perception_ms = 0.0
                    if (self.perception_refresh_period > 0
                            and loop_iter % self.perception_refresh_period == 0
                            and hasattr(self.node, 'refresh_perception_world')):
                        t_perc0 = time.monotonic()
                        self.node.refresh_perception_world(active_only=True)
                        self._diag_perception_ms = (time.monotonic() - t_perc0) * 1000.0
                    self._diag_loop_iter = loop_iter
                    loop_iter += 1

                    # Under gpu_lock, failure narrowed to the goal — see
                    # _execute_immediate for why.
                    self._diag_live_goal_ms = 0.0
                    raw = self._take_live_goal()
                    if raw is not None:
                        try:
                            t_goal0 = time.monotonic()
                            with self.node.gpu_lock:
                                self.apply_live_goal(raw)
                            self._diag_live_goal_ms = (time.monotonic() - t_goal0) * 1000.0
                        except Exception as e:
                            self.node.get_logger().error(
                                f"{self.get_planner_name()}: live goal rejected "
                                f"({e}) - keeping previous goal",
                                throttle_duration_sec=1.0,
                            )

                    st_time = time.time()
                    with self._step_guard():
                        batch = self.step_batch(current_state, batch_size)
                    if tstep > 5:
                        self._step_times.append(time.time() - st_time)
                    self._diag_batch_wall_ms = (time.time() - st_time) * 1000.0

                    with self._pending_cv:
                        if batch_size == 1:
                            self._pending_queue.clear()
                        self._pending_queue.extend(batch)
                        self._pending_cv.notify()

                    last_action = batch[-1]
                    self._last_action = last_action

                    now = time.time()
                    if now - self._last_log_time > 1.0:
                        self._last_log_time = now
                        with self._pending_cv:
                            queue_depth_now = len(self._pending_queue)
                        self.node.get_logger().info(
                            f"{self.get_planner_name()}: error="
                            f"{self.get_position_error():.4f}m on_target={self.is_on_target()} "
                            f"queue_depth={queue_depth_now}/{queue_max_depth} "
                            f"min_queue_depth_seen={self._min_queue_depth_seen} "
                            f"starvation_ticks={self._starvation_ticks}"
                        )

                    tstep += batch_size

                    predicted_state = self._state_from_action(last_action)
                    current_state = self._close_state_loop(robot_context, predicted_state)
            finally:
                self.node.destroy_timer(timer)

            robot_context.stop_robot()

            if self._step_times:

                avg_batch = sum(self._step_times) / len(self._step_times)
                self.node.get_logger().info(
                    f"{self.get_planner_name()} stopped: {tstep} steps, "
                    f"avg time={avg_batch * 1000 / batch_size:.1f}ms/step"
                    + (f" ({avg_batch * 1000:.1f}ms/batch of {batch_size})" if batch_size > 1 else "")
                    + f", min_queue_depth_seen={self._min_queue_depth_seen}, "
                    f"starvation_ticks={self._starvation_ticks}"
                )

            return not self._paced_send_error

        except Exception as e:
            self.node.get_logger().error(f"{self.get_planner_name()} execution error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            robot_context.stop_robot()
            return False

    def _on_send_tick(self, robot_context, goal_handle):
        """Consumer: send the next queued action, else warn.
        """
        try:
            with self._pending_cv:
                depth = len(self._pending_queue)
                if self._min_queue_depth_seen is None or depth < self._min_queue_depth_seen:
                    self._min_queue_depth_seen = depth
                action = self._pending_queue.popleft() if self._pending_queue else None
                self._pending_cv.notify()

            if action is None:
                self._starvation_ticks += 1
                self.node.get_logger().warn(
                    f"{self.get_planner_name()}: command tick out of time - nothing sent",
                    throttle_duration_sec=1.0,
                )
                return

            self._send_command(robot_context, action)
            if goal_handle is not None:
                self._publish_feedback(goal_handle, action)

        except Exception as e:
            self.node.get_logger().error(f"{self.get_planner_name()} send-tick error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            self._paced_send_error = True

    def _publish_feedback(self, goal_handle, action_state):
        """Publish the reactive status through the action feedback (no status topic)."""
        err = self.get_position_error()
        rot_err = self.get_orientation_error()
        on_target = self.is_on_target()
        fb = SendTrajectory.Feedback()
        fb.state = "ON_TARGET" if on_target else "TRACKING"
        fb.on_target = bool(on_target)
        fb.position_error = float(err) if err != float('inf') else -1.0

        xyz = self.get_position_error_xyz()
        if xyz is not None:
            fb.position_error_x, fb.position_error_y, fb.position_error_z = (
                float(v) for v in xyz.reshape(-1).cpu().tolist())
        else:
            fb.position_error_x = fb.position_error_y = fb.position_error_z = -1e9

        fb.orientation_error = float(rot_err) if rot_err != float('inf') else -1.0

        ctrl_err = self.get_controller_position_error()
        ctrl_rot_err = self.get_controller_orientation_error()
        fb.controller_position_error = float(ctrl_err) if ctrl_err != float('inf') else -1.0
        fb.controller_orientation_error = (
            float(ctrl_rot_err) if ctrl_rot_err != float('inf') else -1.0
        )
        fb.hold_count = int(self.get_hold_count())
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

        real_pose = robot_context.get_joint_pose()
        pos = torch.tensor([real_pose], dtype=self._dtype, device=self._device)
        state = JointState.from_position(pos, joint_names=self.solver.joint_names)
        state.velocity = predicted_state.velocity
        state.acceleration = predicted_state.acceleration
        return state

    def _init_robot_at_start(self, robot_context, start_state: JointState):
        """Seed the robot/visualization at the start configuration."""
        start_position = start_state.position[0].cpu().tolist()
        n = len(start_position)

        robot_context.set_command(None, [[0.0] * n], [[0.0] * n], [start_position])
        self.node.get_logger().info(
            f"{self.get_planner_name()}: robot init'd at "
            f"{[f'{x:.3f}' for x in start_position]}"
        )

    def _send_command(self, robot_context, action_state: JointState):
        """Push a control action to the robot.

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

        robot_context.set_and_send_command(None, velocity, acceleration, position)
