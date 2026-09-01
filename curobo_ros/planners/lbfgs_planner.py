#!/usr/bin/env python3
"""
LBFGS reactive controller -- the thinnest possible ROS wrapper around
cuRobo's stock ``ModelPredictiveControl``, kept as close as this framework
allows to cuRobo's own getting-started example
(``curobo.examples.getting_started.reactive_control``).

No custom optimizer YAML (``optimizer_configs`` keeps cuRobo's default
``mpc/lbfgs_mpc.yml``).

Replaces an earlier ``LBFGSController`` built on ``optimize_next_action()``
with an amortized-solve producer/consumer loop (``lbfgs_solve_mode``,
``MPCDiagnostics``, ``mpc_common.py``'s custom optimizer YAML loading). That
version is gone, not kept as an option: it does not share this file's
plan/execute-with-overlap scheme or its state-feedback fix (see below), and
carrying both would mean maintaining two different answers to the same
correctness bugs this docstring documents finding and fixing.

Execution: hand the robot a trajectory SEGMENT, not one point
--------------------------------------------------------------
The arm is driven through leeloo's ``execute_trajectory`` node, which pops one
point per ``command_period`` tick (0.08 s, control.launch.py:332) and forwards
ONLY ``point.velocities`` to the Doosan's ``speedj_rt``. ``point.positions`` is
never read (execute_trajectory.cpp:89-99): the arm integrates a velocity
stream and there is no position servo anywhere in the chain. Two more
behaviours of that node matter here:

- ``this->trajectory = *msg;`` (line 191) -- each publish REPLACES the queue.
- an empty queue commands ``vel[i] = 0.0`` (lines 62-83) -- an immediate stop.

So publishing a ONE-point trajectory per cycle (an earlier version of this
file) means any publish jitter leaves the queue empty for a tick and stops the
arm dead for 80 ms before it resumes. ``step()`` therefore returns a 3D
``JointState`` -- a whole segment -- which ``_send_command``
(reactive_controller.py:715) publishes as a multi-point trajectory, giving the
node a buffer to chew through. MPPIController already does this
(``action = seq.clone()``, mppi_planner.py:271).

Plan / execute, with an overlap
-------------------------------
``lbfgs_command_points`` (ROS param, default 4) points are published per solve,
but the next segment is published after only ``(n-1) * interpolation_dt`` --
one point EARLY. So point ``n-1`` is never played: it is the spare that keeps
the node's queue non-empty if a publish runs late, which is what prevents the
zero-velocity stop. The arm executes points ``0 .. n-2`` before the queue is
replaced.

Waiting for that execution is what makes the whole scheme work, and it is the
piece an earlier version was missing.

State feedback -- ONE source of truth, and wait for it to be true
------------------------------------------------------------------
cuRobo's example has NO robot: ``current_state`` is entirely one plan point
(position, velocity and acceleration all from ``action_sequence``), so it is
self-consistent by construction and a lag cannot exist. That is why the example
never exhibits the failures below -- one source of truth versus two, not
simulation versus hardware.

Three things were measured on this arm before arriving at the scheme above:

- Feeding the prediction back WITHOUT waiting for it to be executed is
  open-loop in position and drifts: joint 1 commanded 31.93 -> 50.34 deg while
  the arm reached 34.25, i.e. ~12% of the commanded motion, predicted FK error
  converging to 0.00000 m against a real 0.22370 m (base_diag_20260820_172914).
- Mixing sources -- measured position with the PLAN's velocity -- is worse:
  "you are at q_real, moving at v(q_pred)" is a state the arm was never in.
  Harmless while the arm tracks (lag ~1.8 deg), amplifying when it cannot. On
  an in-obstacle goal the lag grew 0 -> 9 -> 22 -> 54 -> 92 deg and commanded
  velocity 1.3 -> 73.6 deg/s in five seconds, with the PLAN's own FK error
  growing 0.56 -> 1.09 m (base_diag_20260820_175022).
- Feeding measured position with ZERO velocity is self-consistent and does stop
  the amplification (that same goal then oscillated in a bounded way, lag
  1.61 deg, v_exec capped at 8.4 deg/s) -- but it re-plans from rest every
  cycle, and since only the first points of an acceleration ramp are ever
  executed, the arm never leaves the start of the ramp: v_exec 0.10 deg/s and
  0.07 deg of motion in 3.8 s (base_diag_20260820_181919). Paralysis.

Both failures share one root cause: not waiting for the segment to be executed.
Wait for it, and plan point ``n-2`` IS the arm's state -- measurement and
prediction agree, so there is no inconsistency to exploit, and velocity carries
forward instead of restarting from rest. So ``_close_state_loop`` returns that
plan point whole (position, velocity, acceleration -- one source).

The cost is that this is open-loop in position between solves: nothing corrects
an execution failure. It rests on the arm actually executing what it is handed,
which measured true here (lag 0.02 deg reachable, 1.61 deg in-obstacle). The
CSV logs measured ``q_real_*`` against fed-back ``q_pred_*`` precisely so that
assumption stays falsifiable.

The frozen prefix, and why the feedback is a DELTA
---------------------------------------------------
Every plan begins with a prefix of ``command_start_idx + 1`` points (5 here)
that the optimizer never touches: it dead-reckons the state that was handed in,
integrating its velocity AND acceleration. Measured from a moving state, v in
dps at idx 0..5::

    16.1439 16.0658 15.9876 15.9095 15.8313 15.7290   <- coasting, decelerating

The acceleration term matters: it is what sizes the fictional coast the segment
is offset by, and therefore what the absolute feedback below double-counted.
From rest the prefix is flat, and identically so on every solve -- a structural
property, not a start-up artefact::

    solve 1: v at idx 0,1,2,3,4,5,6,8,12 = 0.000 0.000 0.000 0.000 0.000 0.155 0.618 2.473 6.547
    solve 5: v at idx 0,1,2,3,4,5,6,8,12 = 0.000 0.000 0.000 0.000 0.000 0.160 0.638 2.553 6.740

That is what ``command_start_idx`` (== ``interpolation_steps``) skips, and it
is why ``result.action_sequence`` equals
``robot_state_sequence[command_start_idx:command_end_idx]`` rather than
starting at ``current_state``. Two consequences, both measured:

- Publishing from index 0 publishes only the frozen prefix. From rest that is
  a perfect fixed point -- the state fed back is bit-identical to the state
  fed in, so every solve returns the same plan. Closed loop over 25 cycles:
  ``full[1..4]`` travel 0.0000 deg, ``full[5..8]`` travel ~83 deg. On hardware
  this was 17 s of frozen, bit-identical rows (base_diag_20260821_063052).
  The earlier "measured position + zero velocity" paralysis
  (base_diag_20260820_181919) is this same mechanism.
- Feeding back ``seq.position[m]`` ABSOLUTELY double-counts the lead: the
  plan advances ``j + m`` steps per cycle while the arm, replayed from where
  it actually is, advances only ``m``. That is the ~2x mismatch that read as a
  stalling execution ratio (base_diag_20260820_190138, ratio ~0.5).

So ``step()`` slices from ``command_start_idx`` and feeds back
``current_state.position + (seq.position[m] - seq.position[0])`` -- the arm's
own position plus the plan's delta over the points it will actually play,
which is exactly what integrating the commanded velocities produces. Simulated
over 60 cycles (probe8), absolute vs delta feedback::

    j=4 n=4 abs   -> arm travelled 35.653 deg | fk_err 0.24360 m | lag 47.5296 deg
    j=4 n=4 delta -> arm travelled 80.264 deg | fk_err 0.04770 m | lag  0.0314 deg

Velocity and acceleration still come from that same plan point, so the fed-back
state remains one self-consistent point (see above).

Checked in the regime every previous fix in this file broke in -- an
unreachable goal -- over 80 cycles (probe9). The solver's OWN error
(``result.position_error``) is the tell: in the amplifying failure
(base_diag_20260820_175022) it GREW 0.56 -> 1.09 m. Under delta feedback it
decreases in all three regimes (in-obstacle 0.0017 -> 0.0003, far 0.0028 ->
0.0018), per-cycle motion decays (in-obstacle dq 6.90 -> 0.70 deg) and the arm
settles at its best reachable approach instead of oscillating -- the sim
behaviour we were after. ``j = command_start_idx`` and ``j+1`` are
indistinguishable there (lag 0.038 vs 0.026 deg), so this keeps cuRobo's value.

One caveat when reading a fresh CSV: the FIRST solve after ``setup()`` returns
an entirely flat 81-point plan (the un-warmed seed), so row 1 legitimately
shows zero velocity and ``q_pred == q_real``. It escapes on solve 2. Judge the
execution ratio from row 2 onward.
"""

import math
import time
from typing import Any

import torch
from curobo.types import JointState, Pose, GoalToolPose
from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg

from .reactive_controller import ReactiveController
from .mpc_diagnostics import MPCDiagnostics
from curobo_ros.core.config_wrapper import resolve_interpolation_dt, resolve_use_cuda_graph
from curobo_ros.core.diagnostics import open_diag_csv


class LBFGSController(ReactiveController):
    """Closed-loop MPC on cuRobo's stock config + optimize_action_sequence()."""

    def get_planner_name(self) -> str:
        return "LBFGS Model Predictive Control"

    def get_config_parameters(self) -> list:
        return ['convergence_threshold', 'convergence_threshold_rad',
                'convergence_hold_steps', 'max_mpc_iterations']

    # ---- cuRobo-specific hooks ------------------------------------------------

    def build_solver(self):
        cw = self.config_wrapper
        node = self.node

        interpolation_dt = resolve_interpolation_dt(node)
        # optimization_dt IS the spacing of the points we publish -- measured,
        # not assumed: every point of action_sequence (and of
        # get_command_sequence() and robot_state_sequence) sits
        # optimization_dt apart, and the solver reports the same value as
        # result.action_dt. interpolation_steps does NOT subdivide time; there
        # is no finer-sampled sequence anywhere in the result.
        #
        # It must therefore equal execute_trajectory's command_period (0.08 s),
        # which is what interpolation_dt already is. lbfgs_planner.py's
        # `optimization_dt = 4.0 * interpolation_dt` is WRONG for this pipeline:
        # it plans points 0.32 s apart and then has them played at 0.08 s each,
        # so every velocity is applied for a quarter of its intended duration
        # and the arm achieves ~1/4 of the planned motion. Measured on hardware
        # (base_diag_20260820_185016): the plan advanced ~3.0 deg per cycle, the
        # arm 0.35 -- a ratio of 0.13 -- and the lag grew to 38 deg before both
        # stalled.
        optimization_dt = interpolation_dt

        config = ModelPredictiveControlCfg.create(
            robot=cw.robot_config_file,
            scene_model=cw.obstacle_manager.primitives_only_scene(),
            collision_cache=cw.collision_cache,
            use_cuda_graph=resolve_use_cuda_graph(node),
            optimization_dt=optimization_dt,
        )
        solver = ModelPredictiveControl(config)

        warmup_state = solver.default_joint_state.clone().unsqueeze(0)
        warmup_state.velocity = torch.zeros_like(warmup_state.position)
        warmup_state.acceleration = torch.zeros_like(warmup_state.position)
        solver.setup(warmup_state)

        warmup_kin = solver.compute_kinematics(warmup_state)
        warmup_goal = GoalToolPose.from_poses(
            {solver.tool_frames[0]: Pose(
                position=warmup_kin.tool_poses.position.reshape(-1, 3)[:1],
                quaternion=warmup_kin.tool_poses.quaternion.reshape(-1, 4)[:1],
            )},
            ordered_tool_frames=solver.tool_frames,
            num_goalset=1,
        )
        solver.update_goal_tool_poses(warmup_goal, run_ik=False)

        node.lbfgs = solver

        # RViz-facing publisher for the MPC's predicted end-effector path
        # (nav_msgs/Path on 'mpc_predicted_path', under this node's namespace).
        # Reuses MPPIController's diagnostics helper rather than duplicating
        # the FK-to-Path conversion; only publish_predicted_path() is used
        # here, csv_init() is never called so this does not open a second CSV
        # alongside LBFGSController's own (see _csv_init below).
        self._diag = MPCDiagnostics(
            node, solver, cw.base_link, interpolation_dt,
            self._fk_position_error, self._fk_orientation_error,
        )

        # Points published per solve; n-1 of them get executed, the last is the
        # anti-starvation spare (see the module docstring). Minimum 2: with a
        # single point there is no spare and no executed point to feed back.
        if not node.has_parameter('lbfgs_command_points'):
            node.declare_parameter('lbfgs_command_points', 6)
        self._command_points = max(2, int(node.get_parameter('lbfgs_command_points').value))
        self._interpolation_dt = interpolation_dt
        # Where the publishable part of robot_state_sequence starts. Read from
        # the TEM rather than hardcoded to 4: it is defined as
        # interpolation_steps, so it tracks the config.
        self._command_start_idx = int(
            getattr(solver.trajectory_execution_manager, 'command_start_idx', 4))
        # Publish-to-publish period: one point SHORT of the segment.
        self._publish_period = (self._command_points - 1) * interpolation_dt
        self._next_publish_t = None
        # Wall-clock timestamp when the previous step() call returned -- lets
        # _hold_publish()'s late-warning break "outside step()" (send_command,
        # _close_state_loop, perception refresh, live-goal check, loop
        # overhead -- all in reactive_controller.py's _execute_immediate, not
        # measured here individually) out from step()'s own phases. See
        # _last_cycle_breakdown.
        self._t_prev_step_end = None
        self._last_cycle_breakdown = (0.0, 0.0, 0.0, 0.0)  # (outside_step, predicted_path, controller_fk, step_diag) ms

        # NOT setting _command_interval: execute() then picks
        # _execute_immediate() (solve -> publish the whole segment -> re-solve)
        # rather than the paced producer/consumer loop, whose per-tick pop
        # would undo the multi-point publish.

        node.get_logger().info(
            f"LBFGS solver built: interpolation_dt={interpolation_dt}s, "
            f"optimization_dt={optimization_dt}s, "
            f"command_points={self._command_points} "
            f"({self._command_points * interpolation_dt:.3f}s segment, "
            f"republished every {self._publish_period:.3f}s), "
            f"robot={cw.robot_config_file}, collision_cache={cw.collision_cache}"
        )
        return solver

    def setup(self, start_state: JointState, goal_request: Any) -> bool:
        p = goal_request.target_pose
        raw = [
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z,
        ]
        self.node.get_logger().info(
            f"LBFGS: new goal - position=({raw[0]:.4f}, {raw[1]:.4f}, {raw[2]:.4f})m "
            f"orientation(wxyz)=({raw[3]:.4f}, {raw[4]:.4f}, {raw[5]:.4f}, {raw[6]:.4f})"
        )
        goal = self._set_target(raw)
        self.solver.setup(start_state)  # returns None -- MPCSolver.setup() has no return value
        self.solver.update_goal_tool_poses(goal, run_ik=False)
        self.goal = goal
        self._next_publish_t = None  # new goal = new publish clock
        self._t_prev_step_end = None  # no "outside step()" gap to measure yet
        self._csv_init()
        return True

    # ---- Control step -----------------------------------------------------

    def step(self, current_state: JointState) -> JointState:
        """One resolve -> one trajectory segment, as a 3D ``[1, n, dof]``
        JointState so ``_send_command`` publishes it as a multi-point
        trajectory (see the module docstring).

        Also stashes ``_exec_state`` -- the plan point the arm will have
        reached when this segment is replaced -- and holds the publish back
        until the previous segment is one point from running out.
        """
        t_step_start = time.monotonic()
        # Everything since the PREVIOUS step() call returned: _send_command,
        # _close_state_loop, perception refresh, live-goal check and plain
        # loop overhead, all in reactive_controller.py's _execute_immediate --
        # none of it measured individually here, it is whatever is left once
        # this step()'s own phases (below) are subtracted from the total
        # publish-to-publish cycle. See _last_cycle_breakdown / _hold_publish.
        outside_step_ms = (
            (t_step_start - self._t_prev_step_end) * 1000.0
            if self._t_prev_step_end is not None else 0.0
        )

        t_solve = time.monotonic()
        result = self.solver.optimize_action_sequence(current_state)
        self._last_solve_ms = (time.monotonic() - t_solve) * 1000.0

        # RViz feed of the full predicted horizon. Placed before
        # _hold_publish() so this FK+publish is absorbed by that call's
        # sleep-to-deadline rather than delaying the trajectory command it
        # gates (see module docstring on why that publish is time-critical).
        t0 = time.monotonic()
        self._diag.publish_predicted_path(result)
        predicted_path_ms = (time.monotonic() - t0) * 1000.0

        # Slice robot_state_sequence from command_start_idx -- NOT from 0, and
        # not action_sequence (which is exactly this slice, but capped at 4
        # points, so it cannot honour lbfgs_command_points > 4). The skipped
        # prefix is not optional; see the module docstring.
        rss = result.robot_state_sequence
        full = rss.joint_state if rss is not None else None
        j = self._command_start_idx
        if full is None or full.position.shape[1] <= j + 1:
            action = current_state.clone()
            action.velocity = torch.zeros_like(action.position)
            action.acceleration = torch.zeros_like(action.position)
            self._last_n_pts = 0
            self._exec_state = action
            self._last_controller_position_error = float('inf')
            self._last_controller_orientation_error = float('inf')
            self._t_prev_step_end = time.monotonic()
            return action

        seq = JointState(
            position=full.position[:, j:, :],
            velocity=full.velocity[:, j:, :] if full.velocity is not None else None,
            acceleration=full.acceleration[:, j:, :] if full.acceleration is not None else None,
            joint_names=full.joint_names,
        )
        n = min(self._command_points, seq.position.shape[1])
        self._last_n_pts = n

        # "Controller error" at THIS instant, not result.position_error --
        # measured (probe_controller_error.py) to sit near zero from the very
        # first solve even with the state frozen 0.5 m from the goal: it is
        # the optimizer's own horizon-convergence metric (how close its plan
        # gets to the goal BY THE END of its horizon), not a real-time
        # tracking error, so it stays small independent of how far the
        # current/real state actually is. FK-measuring seq's point 0 -- the
        # very first commanded point, i.e. the solver's own estimate of where
        # the arm will be right after this solve -- gives an instantaneous
        # number instead, directly comparable to (and, once execution tracks
        # the plan, converging toward) _last_position_error/_last_orientation_error.
        t0 = time.monotonic()
        first_point = self._point(seq, 0)
        self._last_controller_position_error = self._fk_position_error(first_point)
        self._last_controller_orientation_error = self._fk_orientation_error(first_point)
        controller_fk_ms = (time.monotonic() - t0) * 1000.0

        # Points 0..n-2 are executed before the next publish replaces the queue;
        # n-1 is the spare that is never played. So the arm plays m = n-1
        # velocities, which move it by the plan's DELTA over those points --
        # from wherever it actually is, NOT to the plan's absolute position.
        # The segment starts j steps ahead of current_state, so seq.position[m]
        # taken absolutely double-counts that lead: the plan would advance j+m
        # steps per cycle while the arm advances m. Anchor on current_state and
        # add only the delta, and the two are equal by construction for any j.
        m = n - 1
        self._exec_state = self._point(seq, m)
        self._exec_state.position = (
            current_state.position + (seq.position[:, m, :] - seq.position[:, 0, :])
        )

        # Diagnostics for the segment about to be published -- before
        # _hold_publish() so this work is absorbed by that call's
        # sleep-to-deadline rather than delaying the time-critical publish
        # (same reasoning as publish_predicted_path above). Budget is the
        # segment's own duration: the same one _hold_publish() itself warns
        # against when overrun.
        t0 = time.monotonic()
        self._diag.publish_step_diagnostics(
            solve_ms=self._last_solve_ms, budget_ms=self._publish_period * 1000.0,
            result=result, position=seq.position[:, :n, :],
            velocity=seq.velocity[:, :n, :] if seq.velocity is not None else None,
            acceleration=seq.acceleration[:, :n, :] if seq.acceleration is not None else None,
            joint_names=seq.joint_names, dt=self._interpolation_dt,
        )
        step_diag_ms = (time.monotonic() - t0) * 1000.0

        self._last_cycle_breakdown = (outside_step_ms, predicted_path_ms, controller_fk_ms, step_diag_ms)

        self._hold_publish()
        self._t_prev_step_end = time.monotonic()

        # Cloned for the same reason as _point(): _execute_immediate consumes
        # this before the next solve, but _execute_paced QUEUES actions, and a
        # queued view would be rewritten under it by the next solve.
        return JointState(
            position=seq.position[:, :n, :].clone(),
            velocity=seq.velocity[:, :n, :].clone() if seq.velocity is not None else None,
            acceleration=(seq.acceleration[:, :n, :].clone()
                          if seq.acceleration is not None else None),
            joint_names=seq.joint_names,
        )

    @staticmethod
    def _point(seq: JointState, i: int) -> JointState:
        """Point ``i`` of an action sequence as a standalone JointState.

        CLONED, not a view. With use_cuda_graph the solver reuses the same
        output tensors on every call, so a view into a result silently mutates
        when the next solve runs -- verified: the same data_ptr comes back and
        a held slice drifted 0.49 dps. Nothing currently reads _exec_state
        after the next solve, so this is latent rather than active, but it is
        invisible when it does bite.
        """
        return JointState(
            position=seq.position[:, i, :].clone(),
            velocity=seq.velocity[:, i, :].clone() if seq.velocity is not None else None,
            acceleration=(seq.acceleration[:, i, :].clone()
                          if seq.acceleration is not None else None),
            joint_names=seq.joint_names,
        )

    def _hold_publish(self):
        """Block until the previously published segment is one point from
        running out, then let the caller publish.

        This is what gives the arm time to execute points 0..n-2 before they
        are replaced -- ``_execute_immediate`` otherwise loops at solve rate
        (~150 ms) and would replace a 320 ms segment before it had played.
        Publishing one point EARLY (rather than exactly at the end) leaves a
        spare in the node's queue, so a late publish degrades into a slightly
        stale command instead of the zero-velocity stop an empty queue
        commands (execute_trajectory.cpp:62-83).

        The first call after a goal never sleeps (``_next_publish_t`` is None),
        which is also the call that may capture a CUDA graph under gpu_lock
        (``_step_guard``) -- so the sleep does not hold that lock.
        """
        if self._next_publish_t is not None:
            now = time.monotonic()
            wait = self._next_publish_t - now
            if wait > 0:
                time.sleep(wait)
            else:
                # The budget (_publish_period) is compared against the WHOLE
                # cycle since the last publish, not just the solve -- solve_ms
                # alone routinely under-explains a "late" warning (e.g.
                # 159ms solve vs a 400ms budget, still 1ms late). cycle_start
                # is exactly when the previous publish went out (_next_publish_t
                # was set to cycle_start + _publish_period then), so cycle_ms
                # is measured directly. _last_cycle_breakdown (filled in by
                # step(), see there) accounts for where the rest of it went:
                # outside_step (_send_command/_close_state_loop/perception
                # refresh/live-goal check/loop overhead, all in
                # reactive_controller.py, not measured individually) +
                # predicted_path (publish_predicted_path) + controller_fk
                # (the seq[0] FK used for _last_controller_position_error) +
                # step_diag (publish_step_diagnostics: FK + cost breakdown +
                # trajectory build/publish).
                cycle_start = self._next_publish_t - self._publish_period
                cycle_ms = (now - cycle_start) * 1000.0
                outside_step_ms, predicted_path_ms, controller_fk_ms, step_diag_ms = (
                    self._last_cycle_breakdown)
                self.node.get_logger().warn(
                    f"LBFGS: publish {-wait * 1000.0:.0f}ms late - cycle "
                    f"{cycle_ms:.0f}ms = outside_step {outside_step_ms:.0f}ms "
                    f"(send_command/close_state_loop/perception/loop) + solve "
                    f"{self._last_solve_ms:.0f}ms + predicted_path "
                    f"{predicted_path_ms:.0f}ms + controller_fk "
                    f"{controller_fk_ms:.0f}ms + step_diag {step_diag_ms:.0f}ms "
                    f"vs {self._publish_period * 1000.0:.0f}ms budget - raise "
                    f"lbfgs_command_points (or cut whichever term dominates)",
                    throttle_duration_sec=5.0,
                )
        self._next_publish_t = time.monotonic() + self._publish_period

    def apply_live_goal(self, raw_goal) -> bool:
        goal = self._set_target(raw_goal)
        self.solver.update_goal_tool_poses(goal, run_ik=False)
        self.goal = goal
        return True

    def update_world(self, scene) -> None:
        """Reload the shared Scene into this solver's collision checker.

        Without this override, obstacle updates never reach this solver:
        ReactiveController.update_world() is a no-op by default, so the
        collision model would stay frozen at whatever
        cw.obstacle_manager.primitives_only_scene() returned at
        build_solver() time. Mirrors LBFGSController.update_world().
        """
        self.solver.scene_collision_checker.load_collision_model(scene)

    # ---- State feedback ---------------------------------------------------

    def _close_state_loop(self, robot_context, predicted_state: JointState) -> JointState:
        """Feed back plan point ``n-2`` WHOLE -- position, velocity and
        acceleration from the same point, one source of truth. Legitimate only
        because ``_hold_publish()`` waited for the arm to execute it; see the
        module docstring.

        ``predicted_state`` (the segment's last point, built by
        ``_state_from_action``) is ignored: that point is the never-played
        spare.
        """
        state = self._exec_state

        # Convergence is measured on the REAL arm, never on the state we feed
        # back. The two are meant to agree here, but if execution ever fails
        # they must not agree silently: a metric taken from the plan would
        # converge by construction and report success from anywhere.
        real_state = self._read_state(robot_context)
        # xyz first, scalar derived from it -- one FK call instead of two
        # (_fk_position_error would otherwise redo the same FK pass).
        self._last_position_error_xyz = self._fk_position_error_xyz(real_state)
        self._last_position_error = (
            float(torch.linalg.norm(self._last_position_error_xyz).item())
            if self._last_position_error_xyz is not None else float('inf')
        )
        self._last_orientation_error = self._fk_orientation_error(real_state)
        self._update_hold()

        self._csv_write(real_state, state, robot_context)
        return state

    # ---- Minimal diagnostics (gated by the `mpc_debug` ROS param) ----------

    def _csv_init(self):
        """Open a fresh CSV per goal when `mpc_debug` is set, else stay off."""
        self._csv_close()
        if not self.node.has_parameter('mpc_debug'):
            self.node.declare_parameter('mpc_debug', False)
        if not bool(self.node.get_parameter('mpc_debug').value):
            return
        self._csv = open_diag_csv(self.node, "lbfgs_diag")
        self._csv_t0 = time.monotonic()

    def _csv_close(self):
        csv = getattr(self, '_csv', None)
        if csv is not None:
            csv.close()
        self._csv = None

    def _csv_write(self, real_state, predicted_state, robot_context):
        """One row per solve.

        ``q_pred_*`` is plan point ``n-2`` -- the state fed to the next solve,
        i.e. where the arm is ASSUMED to be. ``q_real_*`` is where it measurably
        IS. This controller is open-loop in position between solves, so these
        two columns are the assumption it rests on: they are expected to agree
        to a fraction of a degree, and a growing gap means the arm is not
        executing what it is handed -- read it before anything else.

        ``v_exec_max_dps`` is what the plan COMMANDS at that point;
        ``v_real_max_dps`` what the arm actually DOES.
        """
        csv = getattr(self, '_csv', None)
        if csv is None:
            return

        def q_deg(state):
            p = state.position
            return [math.degrees(v) for v in (p[0] if p.dim() > 1 else p).cpu().tolist()]

        q_real, q_pred = q_deg(real_state), q_deg(predicted_state)
        names = self.solver.joint_names

        # Commanded velocity at the fed-back point. cuRobo bounds plan velocity
        # with a COST/constraint, not a hard clamp, so a plan can come back over
        # the limit; JointSpeedStrategy clamps again on its side
        # (_clamp_velocities). Compare against the URDF limit (min 120 deg/s) to
        # tell "the plan asked for too much" from "the driver clamped it".
        v_exec = getattr(predicted_state, 'velocity', None)
        v_max = (max(abs(x) for x in v_exec.reshape(-1).cpu().tolist())
                 if v_exec is not None else float('nan'))

        # Measured arm velocity: a real driver reading, averaged at the source's
        # own ~100Hz rate (joint_speed_strategy.py:120-127) rather than sampled
        # per solve.
        try:
            v_real = max(abs(x) for x in robot_context.get_joint_velocity_filtered())
        except Exception:
            v_real = float('nan')

        csv.write_header_once(
            ["t_s", "solve_ms", "n_pts", "seg_ms", "fk_err_real_m", "fk_err_pred_m",
             "fk_rot_err_real_deg", "v_exec_max_dps", "v_real_max_dps",
             "hold_count", "on_target"]
            + [f"q_real_j{i + 1}_deg" for i in range(len(names))]
            + [f"q_pred_j{i + 1}_deg" for i in range(len(names))])
        n_pts = getattr(self, '_last_n_pts', 0)
        csv.writerow(
            [f"{time.monotonic() - self._csv_t0:.3f}",
             f"{getattr(self, '_last_solve_ms', float('nan')):.1f}",
             f"{n_pts}", f"{n_pts * self._interpolation_dt * 1000.0:.0f}",
             f"{self._last_position_error:.5f}",
             f"{self._fk_position_error(predicted_state):.5f}",
             f"{math.degrees(self._last_orientation_error):.3f}",
             f"{math.degrees(v_max):.2f}", f"{math.degrees(v_real):.2f}",
             f"{self._hold_count}", f"{int(self.is_on_target())}"]
            + [f"{v:.2f}" for v in q_real] + [f"{v:.2f}" for v in q_pred])

    def cancel(self):
        self._csv_close()
        super().cancel()
