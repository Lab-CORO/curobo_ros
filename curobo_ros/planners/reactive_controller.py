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
        # Refresh the perception ESDF every N producer-loop iterations (0
        # disables, 1 = every iteration). Units: for LBFGSController this is
        # BATCHES (loop_iter, one producer iteration = batch_size=4 step()
        # calls), not individual steps -- see loop_iter's own docstring below
        # in _execute_paced for why tstep can't be used here.
        # At 20 (~3s at 7Hz) the MPC collision world lagged behind a moving
        # obstacle (a hand) -- the arm made contact before the update landed.
        # 2 is roughly camera rate (5Hz), affordable since voxelization moved to
        # the GPU (no more ~10s CPU fallback). See debug 2026-07-15.
        # Re-verified 2026-08-18 on the Jetson Orin AGX (LBFGSController):
        # median measured batch period is ~150ms (lbfgs_diag CSV, dt_step_ms
        # column), so period=2 -> ~300ms/3.3Hz, already slower than the 200ms/
        # 5Hz camera -- no GPU cycles wasted re-checking a stale frame. period=4
        # (~600ms/1.7Hz) was considered to cut refresh_perception_world()'s
        # blocking gpu_lock overhead further, but would triple the worst-case
        # staleness on a moving obstacle vs. the 2026-07-15 incident's own
        # trade-off -- kept at 2, since that incident is exactly the failure
        # mode this guards against.
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
        # Condition sharing _pending_lock: the consumer (_on_send_tick)
        # notifies it after every pop, so the producer (_execute_paced) can
        # block waiting for queue room instead of pacing itself to a
        # wall-clock deadline. A deadline-based scheme (tried first, removed
        # 2026-08-17) doesn't work here: the producer's batch time and the
        # consumer's fixed per-tick drain are two independent clocks, and any
        # wall-clock target that assumes a fixed relationship between them
        # either starves the consumer (padding too much) or silently drops
        # not-yet-sent actions (padding too little, requiring clear() to
        # refill) -- see _execute_paced's backpressure wait. cf. debug
        # 2026-08-17.
        self._pending_cv = threading.Condition(self._pending_lock)
        # Queue of not-yet-sent actions, one entry per _command_interval
        # tick. For _batch_size > 1 (LBFGSController) only ever grown by
        # extend() (never clear()): a not-yet-consumed leftover must stay
        # queued, not be silently discarded -- discarding it is
        # indistinguishable downstream from the robot never having received a
        # command for that tick. For _batch_size == 1 (MPCController) the
        # producer still clear()s before each extend(): a full-horizon
        # re-solve makes any unsent previous result stale, so "latest wins"
        # is intentional there, not a leftover of the old scheme.
        self._pending_queue = deque()
        self._paced_send_error = False
        # Backpressure diagnostics (reset per goal in _execute_paced):
        # minimum queue depth ever observed at a consumer pop (0 means a
        # tick found the queue empty -- see _on_send_tick) and how many pops
        # that happened on. Both should stay at/near their initial values in
        # normal operation; a climbing starvation count means the queue cap
        # is too tight for the measured solve-time distribution.
        self._min_queue_depth_seen = None
        self._starvation_ticks = 0

        # Per-producer-iteration timing breakdown, written in _execute_paced,
        # read by LBFGSController.step() -> MPCDiagnostics.record_tick (see
        # _execute_paced's own comments on each field). Defaults cover
        # MPCController and any pre-first-iteration read.
        self._diag_backpressure_wait_ms = 0.0
        self._diag_perception_ms = 0.0
        self._diag_live_goal_ms = 0.0
        self._diag_loop_iter = 0
        self._diag_cheap_ms_before_resolve = 0.0
        self._diag_batch_wall_ms = 0.0

        # Real-velocity finite-difference baseline for _debug_real_velocity_feedback
        # (see that method's docstring). Reset per-goal in _set_target so a stale
        # dt/velocity from a previous goal (or a long pause) never contaminates
        # the first real-acceleration estimate of a new one.
        self._prev_real_velocity = None
        self._prev_real_velocity_t = None

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
        # back-to-back whenever the queue has room (see _execute_paced) and
        # hands the whole batch to the consumer timer to drain one-per-tick,
        # so a slow GPU resolve only has to fit inside however long the
        # consumer takes to drain the existing buffer, not a single
        # _command_interval.
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

    def step_batch(self, current_state: JointState, n: int) -> list:
        """Produce ``n`` actions for one producer-loop iteration (default: call
        ``step()`` n times). Override only when the underlying solver already
        computes all ``n`` actions on a single ``step()`` call and exposes a
        cheaper way to read the rest back out (see LBFGSController's override,
        which skips n-1 redundant solver-side calls this way).

        Also tracks ``_diag_cheap_ms_before_resolve`` (read by
        MPCDiagnostics.record_tick): the wall time of any calls in this batch
        that land before whichever one turns out to be the real resolve. Only
        meaningful with this default, per-call implementation -- an override
        that never makes n-1 extra calls has nothing to attribute here (see
        LBFGSController.step_batch's docstring).
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

        Reactive solvers each hold their own collision checker, so the node
        delegates world updates here (instead of reaching into solver internals).
        Default is a no-op; concrete controllers override (e.g. MPC reloads its
        scene_collision_checker, retarget updates its IK solvers).
        """
        return None

    def has_solver(self) -> bool:
        """TrajectoryPlanner override: each reactive controller owns its own
        cuRobo solver exclusively (unlike SinglePlanner's shared
        MotionPlanner) -- the base class's default world_identity (id(self),
        i.e. never collides with another planner) is therefore correct as-is
        and isn't overridden here.
        """
        return self.solver is not None

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
        self._prev_real_velocity = None
        self._prev_real_velocity_t = None
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
                    self.node.refresh_perception_world(active_only=True)

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
        it can and deposits a batch of _batch_size action(s) under a lock; a
        per-goal timer (consumer) sends the next queued action at a fixed
        cadence, or warns and sends nothing if the queue is empty. For
        _batch_size == 1 (MPCController) the producer clears+replaces the
        queue every iteration — "latest wins", never blocks on room, no
        resending stale data. For _batch_size > 1 (LBFGSController) the
        producer instead appends and blocks (via _pending_cv) whenever the
        queue already holds queue_max_depth items, so a full batch is always
        delivered in order instead of being partially overwritten. Only this
        loop calls step() (CUDA) — the timer only pops from the queue and
        sends, so it never contends for the GPU with a slow/cold-start solve.
        cf. debug 2026-07-17.
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
            # One spare beyond a full batch: lets the producer stay a batch
            # ahead of the consumer without unbounded growth. This is now
            # the SOLE pacing mechanism (see the backpressure wait below) --
            # there is no wall-clock target to tune. Raising it trades more
            # starvation headroom for more in-queue actuation lag (each
            # extra slot delays an already-computed action by one
            # _command_interval before it's sent); it does NOT stale the
            # solver's own input state, since _close_state_loop always reads
            # the robot's real position fresh, right before the next batch
            # is computed, regardless of queue depth. cf. debug 2026-08-17.
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

                    # Backpressure: block until the consumer has drained
                    # enough room for a new batch, instead of pacing to a
                    # wall-clock deadline (see _pending_cv's docstring in
                    # __init__ for why a deadline can't work: production
                    # time and the consumer's fixed per-tick drain are two
                    # independent clocks). _on_send_tick notifies this after
                    # every pop, so this wakes promptly when there's room;
                    # the wait timeout is only a responsiveness safety net
                    # for cancellation, not part of the pacing. No-op
                    # (queue starts empty and drains faster than filled)
                    # for MPCController's batch_size==1.
                    t_wait0 = time.monotonic()
                    with self._pending_cv:
                        while (len(self._pending_queue) >= queue_max_depth
                               and self.is_goal_active and not self._paced_send_error):
                            if goal_handle is not None and goal_handle.is_cancel_requested:
                                break
                            self._pending_cv.wait(timeout=0.1)
                    # Diagnostics only (read by LBFGSController.step() -> record_tick,
                    # see that module for why these live as plain attrs rather than
                    # being threaded through step()'s signature: step() is the
                    # cuRobo-solver-agnostic hook every ReactiveController subclass
                    # implements, and only LBFGSController's CSV path cares about
                    # this loop's internal timing breakdown). cf. debug 2026-08-18.
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
                    # current_state is passed unchanged to every call in the
                    # burst. Safe because it's only CONSUMED on the call(s)
                    # that actually re-solve (verified in solver_mpc.py:
                    # optimize_next_action skips warm_start_solve, the only
                    # place current_state is read, whenever
                    # trajectory_execution_manager.has_valid_next_command()
                    # is True) -- the other calls in the batch would ignore
                    # a "fresher" current_state just the same.
                    with self._step_guard():
                        batch = self.step_batch(current_state, batch_size)
                    if tstep > 5:
                        self._step_times.append(time.time() - st_time)
                    self._diag_batch_wall_ms = (time.time() - st_time) * 1000.0

                    with self._pending_cv:
                        if batch_size == 1:
                            # MPCController: re-solves the full horizon every
                            # call, so an unsent previous result is stale, not
                            # useful backlog -- keep "latest wins" instead of
                            # accumulating (see docstring above).
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

                    # Real-position read stays here, right after queuing the
                    # batch and before the next one is computed — as late as
                    # possible for freshness, feeding the NEXT iteration's
                    # resolving call. Unaffected by queue depth: this always
                    # reads the robot's actual current position, regardless
                    # of how many already-computed actions are still queued
                    # ahead of it (cf. debug 2026-08-17 — see queue_max_depth
                    # above for why that decoupling matters).
                    predicted_state = self._state_from_action(last_action)
                    current_state = self._close_state_loop(robot_context, predicted_state)
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

        Never calls step()/CUDA — only pops from the pending queue and sends.
        Notifies _pending_cv after every pop so the producer (blocked in
        _execute_paced waiting for queue room) wakes promptly instead of
        polling. Any exception here is caught (never let it escape an rclpy
        timer callback) and signaled to the producer loop via
        _paced_send_error, which checks it every iteration and stops cleanly
        (mirrors the immediate loop's except-block behavior: stop_robot() +
        return False).
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

    def _debug_real_velocity_feedback(self) -> bool:
        """EXPERIMENT (2026-08-18, gated -- default False, does not change the
        validated default behaviour). Testing whether feeding REAL velocity/
        acceleration back (instead of the solver's own PREDICTED ones, see
        _close_state_loop's docstring) prevents the runaway-velocity
        oscillation observed when a goal is unreachable (e.g. placed in
        collision): fk_err_m stayed pinned near-constant for the whole run
        while v_max_dps climbed 0->39 deg/s and con_cspace_bound exploded
        (lbfgs_diag_20260818_074717.csv) -- consistent with predicted-
        velocity warm-start windup (the optimizer keeps carrying forward an
        increasingly energetic initial guess with no Cartesian progress to
        justify it). Real velocity was already tried and reverted for the
        NORMAL reachable-goal case (see below) because the hardware safety
        clamp throttles it below plan -- this flag re-tests the same idea
        specifically for the stuck-goal failure mode, where the windup this
        would suppress may outweigh that earlier problem. Do not flip this on
        for production without confirming it doesn't reintroduce the
        2026-0x-xx oscillation the docstring below describes.
        """
        if not self.node.has_parameter('use_real_velocity_feedback'):
            self.node.declare_parameter('use_real_velocity_feedback', False)
        return bool(self.node.get_parameter('use_real_velocity_feedback').value)

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

        See ``_debug_real_velocity_feedback`` for a gated, opt-in experiment
        re-testing real velocity/acceleration specifically for the stuck-goal
        oscillation failure mode — off by default, this docstring's reasoning
        still holds for the normal (reachable-goal) case.
        """
        real_pose = robot_context.get_joint_pose()
        pos = torch.tensor([real_pose], dtype=self._dtype, device=self._device)
        state = JointState.from_position(pos, joint_names=self.solver.joint_names)

        if self._debug_real_velocity_feedback():
            real_vel_list = robot_context.get_joint_velocity()
            real_vel = torch.tensor([real_vel_list], dtype=self._dtype, device=self._device)
            now = time.monotonic()
            if self._prev_real_velocity is not None and self._prev_real_velocity_t is not None:
                dt = now - self._prev_real_velocity_t
                real_acc = (
                    (real_vel - self._prev_real_velocity) / dt
                    if dt > 1e-6 else torch.zeros_like(real_vel)
                )
            else:
                real_acc = torch.zeros_like(real_vel)
            self._prev_real_velocity = real_vel
            self._prev_real_velocity_t = now
            state.velocity = real_vel
            state.acceleration = real_acc
        else:
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
