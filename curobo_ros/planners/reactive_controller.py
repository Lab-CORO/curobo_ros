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

        # Raw [x, y, z, qw, qx, qy, qz] written from the ROS thread (topic) and
        # consumed on the control-loop thread to avoid racing CUDA graph capture.
        # The lock only guards the pointer swap (see set_live_goal/_take_live_goal)
        # — apply_live_goal() itself runs outside it, since it does CUDA work.
        self._live_goal_lock = threading.Lock()
        self._latest_goal = None
        self._latest_goal_fresh = False

        # Tunables (overwritten from the per-call config in plan()).
        self.convergence_threshold = 0.01      # meters
        # 0.05 rad = 2.9 deg, matching _ensure_ik_solver's own
        # orientation_tolerance in mpc_planner.py. The IK anchor handed to MPPI
        # is only accurate to that, so a tighter convergence gate here would be
        # a target the pipeline cannot structurally reach -- tighten both or
        # neither.
        self.convergence_threshold_rad = 0.05  # radians
        # Consecutive in-tolerance steps required before on_target goes true.
        # 5 steps ~ 1.2 s at the production mpc_command_interval of 0.24 s.
        # Rationale: measured 2026-08-07, position reached 0.0037 m then drifted
        # back out to 0.0094 m by the end of the run. An instantaneous test
        # reports success on the way through; only a held one means converged.
        self.convergence_hold_steps = 5
        self.max_iterations = 1000
        # Refresh the perception ESDF every N steps (0 disables, 1 = every step).
        # At 20 (~3s at 7Hz) the MPC collision world lagged behind a moving
        # obstacle (a hand) -- the arm made contact before the update landed.
        # 2 is roughly camera rate (5Hz), affordable since voxelization moved to
        # the GPU (no more ~10s CPU fallback). See debug 2026-07-15.
        self.perception_refresh_period = 2

        # Latest scalar position error, written by step(), read by is_converged().
        self._last_position_error = float('inf')
        # Latest scalar ORIENTATION error (rad), written by step().
        self._last_orientation_error = float('inf')
        # Consecutive steps with BOTH errors inside tolerance; see _update_hold.
        self._hold_count = 0
        # Cartesian target (xyz tensor), set by _set_target, read by FK error.
        self._target_position = None
        # Cartesian target orientation (wxyz quaternion tensor), same source.
        self._target_quaternion = None
        self._step_times = []
        # Last commanded action — fed back so the next current_state carries
        # velocity/acceleration continuity (without it the solver restarts from
        # rest every step and the arm never builds up motion).
        self._last_action = None
        # Wall-clock of the last status log (throttled, rate-independent).
        self._last_log_time = 0.0

        # Fixed-interval command pacing (used only when a subclass sets
        # self._command_interval > 0, e.g. MPCController via the
        # mpc_command_interval ROS param). Producer/consumer split: the
        # execute() loop is the producer (solves continuously, never blocks on
        # sending); a per-goal timer is the consumer (sends the next queued
        # action at a fixed cadence, or warns and sends nothing if the queue
        # is empty). Only the producer ever calls step() (CUDA), so the
        # timer's callback group only needs to prevent a slow SEND from
        # overlapping the next tick — no GPU-concurrency concern. cf. debug
        # 2026-07-17.
        self._timer_cb_group = None
        self._pending_lock = threading.Lock()
        # Queue of not-yet-sent actions, one entry per _command_interval tick.
        # _batch_size==1 (the default) makes this behave exactly like the old
        # single-slot pending_action/pending_action_fresh pair: the producer
        # clears+replaces the whole queue every iteration ("latest wins", no
        # resending stale data), so a not-yet-consumed leftover is discarded
        # rather than piling up.
        self._pending_queue = deque()
        self._paced_send_error = False

        # Number of actions produced per producer-loop iteration in
        # _execute_paced (1 = no behavior change, the default -- MPCController
        # never sets this). Set >1 (e.g. LBFGSController, to
        # trajectory_execution_manager.interpolation_steps) when the
        # underlying solver call (optimize_next_action()) only does real GPU
        # work on one call out of every N and silently ignores current_state
        # on the other N-1 (verified in solver_mpc.py: it skips
        # warm_start_solve entirely unless
        # trajectory_execution_manager.has_valid_next_command() is False) --
        # in that case there is no benefit to pacing those N-1 calls to real
        # elapsed time individually. Instead the producer bursts all N calls
        # back-to-back once per _producer_min_interval and hands the whole
        # batch to the consumer timer to drain one-per-tick, so a slow GPU
        # resolve only has to fit inside N * _command_interval instead of a
        # single _command_interval.
        self._batch_size = 1

        # Opt-in minimum wall-clock period between producer-loop iterations in
        # _execute_paced (0.0 = disabled, the default -- no behavior change for
        # MPCController, which never sets this). For _batch_size==1 solvers
        # built on optimize_next_action(), this throttles single-call
        # iterations to stay roughly in step with real elapsed time (cuRobo's
        # internal command buffer, see TrajectoryExecutionManager, has no
        # awareness of wall-clock time by itself). For _batch_size>1
        # (LBFGSController), set this to the FULL batch period (e.g.
        # optimization_dt = interpolation_steps * interpolation_dt) instead:
        # one producer iteration now produces a whole batch, so it's the
        # batch — not each individual call inside it — that must stay paced
        # to real elapsed time. MPCController's optimize_action_sequence()
        # re-solves fully on every call and has no such buffer to desync --
        # hence opt-in, not a change to the base default.
        self._producer_min_interval = 0.0

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
        # Single choke point for every goal change -- setup() and
        # apply_live_goal() both land here, on both reactive controllers. A new
        # target invalidates any hold accumulated against the previous one, so
        # resetting here (rather than in plan()) also covers live retargeting.
        self._hold_count = 0
        self._target_position = torch.tensor(
            raw[0:3], dtype=self._dtype, device=self._device
        )
        # raw is [x, y, z, qw, qx, qy, qz] -- cuRobo's quaternion convention is
        # WXYZ (Pose.from_list below consumes the same seven values in the same
        # order), so raw[3:7] can be stored verbatim.
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

    def _fk_position_error(self, current_state: JointState) -> float:
        """Real Cartesian distance (m) between the current EE and the target."""
        if self._target_position is None:
            return float('inf')
        try:
            ee = self._compute_ee_position(current_state)
            return float(torch.linalg.norm(ee - self._target_position).item())
        except Exception:
            return float('inf')

    def _fk_orientation_error(self, current_state: JointState) -> float:
        """Real angular distance (rad) between the current EE and the target.

        Measured by FK against the goal quaternion, deliberately NOT read from
        ``result.rotation_error``: that field is scaled by roughly 1e-3 relative
        to the true error (verified 2026-08-07 against fk_err_m over three
        decades) AND was written to the CSV with 5 decimals, leaving ~16
        quantisation steps across a whole run — unusable either way.

        Geodesic angle between unit quaternions: theta = 2*acos(|<q1,q2>|).
        The absolute value takes the shorter of the two arcs, since q and -q
        are the same rotation.
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

        Resets to 0 the moment either error leaves tolerance -- that reset is
        the whole point. An instantaneous test passes while the arm is merely
        travelling through the tolerance ball; requiring N consecutive steps
        distinguishes "converged" from "passing by".
        """
        if self._within_tolerances():
            self._hold_count += 1
        else:
            self._hold_count = 0
        return self._hold_count

    def is_on_target(self) -> bool:
        """Signal (NOT a stop condition): the arm has CONVERGED on the goal.

        Requires position AND orientation inside tolerance, held for
        convergence_hold_steps consecutive steps. Reactive control keeps
        servoing afterwards, so this still only drives the `on_target` feedback
        flag -- it never ends the control loop, which exits on cancel or error.

        Callers that end their action on this flag now get a genuine acceptance
        criterion. Before 2026-08-07 it tested instantaneous position only, so a
        goal could report success with the tool mis-aimed, or while merely
        passing through the target on its way back out.
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

    def set_live_goal(self, raw_goal) -> None:
        """Deposit a live goal update. Called from the ROS topic thread.

        Only swaps a pointer under the lock — the expensive retargeting
        (apply_live_goal, CUDA work) happens on the control-loop thread after
        _take_live_goal() hands it off, never here.
        """
        with self._live_goal_lock:
            self._latest_goal = list(raw_goal)
            self._latest_goal_fresh = True

    def _take_live_goal(self):
        """Atomically take-and-clear the pending live goal, or None if stale.

        Replaces the previous test-read-clear sequence (latest_goal is not
        None -> read -> set to None), which raced: a goal written by the ROS
        thread between the read and the clear was silently dropped.
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

        The node flags exactly the step(s) that follow a graph release/rebuild
        (see take_graph_capture_pending) as capture-pending; every other step
        only replays an already-captured graph and doesn't need exclusivity.
        Locking every step would starve the perception thread (its depth
        callback does a non-blocking gpu_lock acquire and drops the frame —
        see camera_depth_map_strategy.py), undoing perception_refresh_period.
        cf. debug 2026-07-28.
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
        self.convergence_hold_steps = int(config.get('convergence_hold_steps', 5))
        self.max_iterations = config.get('max_iterations', 1000)
        self.start_state = start_state

        try:
            with self.node.gpu_lock:
                setup_ok = self.setup(start_state, goal_request)
            if not setup_ok:
                return PlannerResult(success=False, message="Failed to set reactive goal")
            # Discard any live goal left over from a previous session — it must
            # not be silently applied to this new one. A goal arriving AFTER
            # this point (i.e. after setup) is still honoured normally.
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

        Paced mode (self._command_interval > 0, e.g. MPCController via the
        mpc_command_interval ROS param) decouples solve time from send
        cadence — see _execute_paced. Every other caller (interval 0, the
        default) gets the original behavior, untouched, via _execute_immediate.
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

                # Consume a pending live goal on the loop thread only, under
                # gpu_lock: retargeting runs the solver's IK, which can capture
                # a CUDA graph, and capture is process-global (any CUDA op on
                # any thread during it raises cudaErrorStreamCaptureUnsupported
                # and poisons the context). The lock is how the perception
                # thread knows to skip its frame — same invariant as plan()'s
                # setup() call. A raising apply_live_goal (bad pose, IK error)
                # is narrowed to the goal itself — it must not kill the whole
                # session, since the arm should keep servoing the previous
                # goal instead. cf. debug 2026-07-28.
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

    def _execute_paced(self, robot_context, goal_handle, interval: float) -> bool:
        """Producer/consumer servo loop: this loop (producer) solves as fast as
        it can and deposits a batch of _batch_size action(s) under a lock,
        never blocking on sending; a per-goal timer (consumer) sends the next
        queued action at a fixed cadence, or warns and sends nothing if the
        queue is empty (no resending stale data). Only this loop calls step()
        (CUDA) — the timer only pops from the queue and sends, so it never
        contends for the GPU with a slow/cold-start solve. cf. debug 2026-07-17.
        """
        try:
            tstep = 0
            # Producer-iteration counter, separate from tstep: tstep advances
            # by batch_size (4 for LBFGSController) per iteration, which made
            # `tstep % perception_refresh_period` (2) always land on 0 --
            # refresh_perception_world() was firing on every single batch,
            # unthrottled, instead of every perception_refresh_period
            # iterations as configured. Harmless while perception is inactive
            # (current tests), but refresh_perception_world() takes a
            # BLOCKING gpu_lock and will do real ESDF work once cameras/
            # obstacles are live -- exactly the scenario this is being tuned
            # for. cf. debug 2026-08-17.
            loop_iter = 0
            batch_size = max(1, int(getattr(self, '_batch_size', 1)))
            self._step_times = []
            self._last_action = None
            self._last_log_time = 0.0
            with self._pending_lock:
                self._pending_queue.clear()
            self._paced_send_error = False
            self.node.get_logger().info(
                f"Starting {self.get_planner_name()} servo loop "
                f"(paced, interval={interval}s, batch={batch_size})"
            )

            current_state = self._read_state(robot_context)

            if self._timer_cb_group is None:
                self._timer_cb_group = MutuallyExclusiveCallbackGroup()
            timer = self.node.create_timer(
                interval, lambda: self._on_send_tick(robot_context, goal_handle),
                callback_group=self._timer_cb_group,
            )

            # Absolute-deadline anchor for the padding sleep below, instead of
            # a relative "sleep(target - elapsed_this_iteration)" that resets
            # its reference point every loop. A relative scheme silently
            # excludes whatever runs AFTER the sleep (_close_state_loop's
            # real-position read, kept there deliberately for freshness — see
            # its call site below) from the budget: that cost becomes pure
            # add-on every cycle instead of being amortized. Measured
            # 2026-08-17: this is why _producer_min_interval=120ms
            # (LBFGSController) was landing at ~124ms mean with a long tail
            # (up to ~380ms), which the consumer's fixed 30ms send-tick can't
            # absorb -- see "command tick out of time" in _on_send_tick.
            next_deadline = time.time()

            try:
                while self.is_goal_active:
                    if goal_handle is not None and goal_handle.is_cancel_requested:
                        self.node.get_logger().info(f"{self.get_planner_name()} cancel requested")
                        break
                    if goal_handle is None and tstep >= self.max_iterations:
                        break
                    if self._paced_send_error:
                        break

                    # Diagnostic-only phase timers (2026-08-17): dt_step_ms
                    # was landing >120ms on ~50% of LBFGSController batches
                    # even with solve_ms averaging ~35ms (85ms of nominal
                    # slack), including tail spikes to 350-400ms. That's too
                    # much to be sleep-scheduling jitter alone. Rather than
                    # keep guessing, log a phase breakdown whenever an
                    # iteration overruns badly, so the next run's log points
                    # at the actual culprit (candidates: refresh_perception_
                    # world's blocking gpu_lock, live-goal apply, or the
                    # step() batch itself).
                    iter_top = time.time()
                    perc_ms = 0.0
                    if (self.perception_refresh_period > 0
                            and loop_iter % self.perception_refresh_period == 0
                            and hasattr(self.node, 'refresh_perception_world')):
                        t_perc = time.time()
                        self.node.refresh_perception_world()
                        perc_ms = (time.time() - t_perc) * 1000.0
                    loop_iter += 1

                    # Under gpu_lock, failure narrowed to the goal — see
                    # _execute_immediate for why.
                    live_goal_ms = 0.0
                    t_live = time.time()
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
                    live_goal_ms = (time.time() - t_live) * 1000.0

                    st_time = time.time()
                    # current_state is passed unchanged to every call in the
                    # burst. Safe because it's only CONSUMED on the call(s)
                    # that actually re-solve (verified in solver_mpc.py:
                    # optimize_next_action skips warm_start_solve, the only
                    # place current_state is read, whenever
                    # trajectory_execution_manager.has_valid_next_command()
                    # is True) -- the other calls in the batch would ignore
                    # a "fresher" current_state just the same.
                    batch = []
                    # Per-call timing (diagnostic, 2026-08-17): the CSV only
                    # ever recorded solve_ms for the ONE resolving call
                    # (~35ms measured) -- the other batch_size-1 calls' cost
                    # was invisible, yet dt_step_ms averages ~120ms, an ~85ms
                    # gap unaccounted for. Timing each call here (not inside
                    # step()) answers whether the non-resolving calls are
                    # genuinely cheap post-gating (see lbfgs_planner.py's
                    # is_resolving change) or still costing real time in
                    # optimize_next_action() itself (CUDA graph replay,
                    # Python/GIL overhead) -- gating only removed the FK +
                    # RViz-publish work, not the solver call.
                    call_ms = []
                    with self._step_guard():
                        for _ in range(batch_size):
                            t_call = time.time()
                            batch.append(self.step(current_state))
                            call_ms.append((time.time() - t_call) * 1000.0)
                    if tstep > 5:
                        self._step_times.append(time.time() - st_time)

                    with self._pending_lock:
                        self._pending_queue.clear()
                        self._pending_queue.extend(batch)

                    last_action = batch[-1]
                    self._last_action = last_action

                    now = time.time()
                    if now - self._last_log_time > 1.0:
                        self._last_log_time = now
                        self.node.get_logger().info(
                            f"{self.get_planner_name()}: error="
                            f"{self.get_position_error():.4f}m on_target={self.is_on_target()}"
                        )

                    tstep += batch_size

                    # Pad to next_deadline (an absolute, self-correcting
                    # schedule -- see its definition above) so batches into a
                    # fixed-size internal command buffer (e.g.
                    # optimize_next_action) stay roughly in step with real
                    # elapsed time -- see _producer_min_interval's docstring
                    # in __init__. No-op (interval 0.0) for MPCController.
                    # This MUST happen BEFORE _close_state_loop below, not
                    # after: _close_state_loop reads the robot's REAL
                    # position, and that reading feeds the NEXT iteration's
                    # resolving call. Reading it here-then-sleeping left it up
                    # to _producer_min_interval stale by the time it was
                    # actually used -- fine at 30ms (batch_size==1),
                    # destabilizing at 120ms (LBFGSController's
                    # batch_size==4): the resolve anchors to a position the
                    # arm had already moved away from, which (with no cspace
                    # anchor under run_ik=False) let the redundant J4/J6 pair
                    # wind up chasing it. cf. debug 2026-08-17.
                    if self._producer_min_interval > 0.0:
                        next_deadline += self._producer_min_interval
                        remaining = next_deadline - time.time()
                        if remaining > 0.0:
                            time.sleep(remaining)
                        else:
                            # Fell behind by more than one period (e.g. a
                            # slow resolve) -- resync instead of trying to
                            # "catch up" with back-to-back unpaced
                            # iterations, which would defeat the whole point
                            # of pacing to real elapsed time.
                            next_deadline = time.time()

                    t_close = time.time()
                    predicted_state = self._state_from_action(last_action)
                    current_state = self._close_state_loop(robot_context, predicted_state)
                    close_ms = (time.time() - t_close) * 1000.0

                    if self._producer_min_interval > 0.0:
                        total_ms = (time.time() - iter_top) * 1000.0
                        target_ms = self._producer_min_interval * 1000.0
                        # ANY overrun costs a missed consumer tick when
                        # batch_size * _command_interval == _producer_min_interval
                        # (LBFGSController's case: the queue is drained
                        # exactly as fast as it's filled, zero slack) -- so
                        # this must fire on every overrun, not just severe
                        # ones. The old >1.5x gate (180ms) never caught the
                        # ~121-145ms overruns that make up roughly half of
                        # all batches, which is why it logged nothing while
                        # the warning kept firing on hardware. cf. debug
                        # 2026-08-17 (Guillaume caught this).
                        if total_ms > target_ms:
                            self.node.get_logger().warn(
                                f"{self.get_planner_name()}: batch overran "
                                f"({total_ms:.0f}ms vs {target_ms:.0f}ms target) -- "
                                f"per_call_ms={[round(c) for c in call_ms]} "
                                f"perception={perc_ms:.0f}ms "
                                f"live_goal={live_goal_ms:.0f}ms "
                                f"close_state_loop={close_ms:.0f}ms",
                                throttle_duration_sec=1.0,
                            )
            finally:
                self.node.destroy_timer(timer)

            robot_context.stop_robot()

            if self._step_times:
                # Each recorded entry timed one producer iteration, i.e. one
                # batch of batch_size step() calls -- divide back down to a
                # per-step figure so this stays comparable across batch sizes.
                avg_batch = sum(self._step_times) / len(self._step_times)
                self.node.get_logger().info(
                    f"{self.get_planner_name()} stopped: {tstep} steps, "
                    f"avg time={avg_batch * 1000 / batch_size:.1f}ms/step"
                    + (f" ({avg_batch * 1000:.1f}ms/batch of {batch_size})" if batch_size > 1 else "")
                )

            return not self._paced_send_error

        except Exception as e:
            self.node.get_logger().error(f"{self.get_planner_name()} execution error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            robot_context.stop_robot()
            return False

    def _on_send_tick(self, robot_context, goal_handle):
        """Consumer: send the next queued action, else warn.

        Never calls step()/CUDA — only pops from the pending queue and sends.
        Any exception here is caught (never let it escape an rclpy timer
        callback) and signaled to the producer loop via _paced_send_error,
        which checks it every iteration and stops cleanly (mirrors the
        immediate loop's except-block behavior: stop_robot() + return False).
        """
        try:
            with self._pending_lock:
                action = self._pending_queue.popleft() if self._pending_queue else None

            if action is None:
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
        # -1.0 is the "not measurable" sentinel, matching position_error's
        # convention above: inf means no target set or FK failed, and a client
        # must not read that as a perfectly-aligned 0.
        fb.orientation_error = float(rot_err) if rot_err != float('inf') else -1.0
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
        n = len(start_position)
        # joint_names=None -> RobotContext resolves them itself, inside the
        # same critical section as the command (see set_command's docstring:
        # reading robot_strategy.get_joint_name() here first would race a
        # concurrent strategy switch).
        robot_context.set_command(None, [[0.0] * n], [[0.0] * n], [start_position])
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

        # Atomic: no gap between load and send where a concurrent producer
        # (another call site touching the same RobotContext) could overwrite
        # the buffers first — see RobotContext.set_and_send_command.
        robot_context.set_and_send_command(None, velocity, acceleration, position)
