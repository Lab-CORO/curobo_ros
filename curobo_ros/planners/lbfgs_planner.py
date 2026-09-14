#!/usr/bin/env python3
"""
LBFGS reactive controller -- the thinnest possible ROS wrapper around
cuRobo's stock ``ModelPredictiveControl``, kept as close as this framework
allows to cuRobo's own getting-started example
(``curobo.examples.getting_started.reactive_control``).

By default, ``optimizer_configs`` keeps cuRobo's stock ``mpc/lbfgs_mpc.yml``
-- unchanged from cuRobo's own shipped tuning. A parallel set of ROS params
(the ``lbfgs.*`` prefix, see ``_LBFGS_DEFAULTS`` below and
``config/mpc/lbfgs_params.yaml``) carries this repo's own tuned cost/optimizer
config, migrated from the file that same YAML used to be loaded from -- but
it is applied only when ``lbfgs_apply_custom_config`` is explicitly set
``true``. Off by default on purpose: those values were never validated
against this controller (see build_solver()'s docstring note on ``horizon``),
so flipping the switch is a deliberate opt-in, not a silent behavior change.

Replaces an earlier ``LBFGSController`` built on ``optimize_next_action()``
with an amortized-solve producer/consumer loop (``lbfgs_solve_mode``,
``MPCDiagnostics``, a hand-rolled custom-YAML loader in ``mpc_common.py``
this file no longer uses). That version is gone, not kept as an option: it
does not share this file's plan/execute-with-overlap scheme or its
state-feedback fix (see below), and carrying both would mean maintaining two
different answers to the same correctness bugs this docstring documents
finding and fixing.

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
``lbfgs_command_points`` (ROS param, default 4) points are CONSUMED per solve,
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
from .mpc_common import declare_from_defaults, read_nested
from .mpc_diagnostics import MPCDiagnostics
from curobo_ros.core.config_wrapper import resolve_interpolation_dt, resolve_use_cuda_graph
from curobo_ros.core.diagnostics import open_diag_csv


# Fallback defaults for every 'lbfgs.*' ROS param declared below, migrated
# from the now-deleted config/mpc/lbfgs_mpc.yaml (its 'rollout'/'optimizer'
# tree, proven-on-hardware for MPPI's own lbfgs-tuned run -- see
# config/mpc/lbfgs_params.yaml for the full dated engineering history behind
# each value). Applied only when lbfgs_apply_custom_config=true; see this
# module's docstring for why that is off by default. 'lbfgs_debug' has no
# YAML-file source anymore (removed from the file during this migration) --
# kept here as its own ROS param since it maps directly to create()'s
# store_debug kwarg, defaulting to cuRobo's own store_debug default (False).
# inner_iters of the L-BFGS optimizer, in cuRobo's stock
# content/configs/task/mpc/lbfgs_mpc.yml AND in _LBFGS_DEFAULTS below -- the
# same 25 either way, so the divisibility check in build_solver() holds
# whether or not lbfgs_apply_custom_config is set. Hardcoded rather than read
# back from the config because the check has to run BEFORE
# ModelPredictiveControlCfg.create() builds the optimizer.
_LBFGS_INNER_ITERS = 25

_LBFGS_DEFAULTS = {
    'lbfgs_debug': False,
    'warm_start_iters': 25,
    'cold_start_iters': 100,
    'horizon': 30,
    'rollout': {
        'cost_cfg': {
            'cspace_cfg': {
                'activation_distance': [0.01, 0.01, 0.01, 0.01, 0.01],
                'squared_l2_regularization_weight': [10.0, 100.0, 10.0, 0.0, 0.0],
                'weight': [1000.0, 1000.0, 1000.0, 100.0, 0.0],
                'cost_type': 'STATE',
                'retime_weights': False,
                'retime_regularization_weights': True,
                'cspace_target_weight': 0.0,
                'cspace_non_terminal_weight_factor': 0.05,
            },
            'tool_pose_cfg': {
                'use_lie_group': False,
                'weight': [5000.0, 1000.0],
                '_terminal_pose_convergence_tolerance': [0.005, 0.0001],
                '_non_terminal_pose_axes_weight_factor': [0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
            },
        },
        'constraint_cfg': {
            'scene_collision_cfg': {
                'activation_distance': 0.01,
                'use_speed_metric': True,
                'use_sweep': True,
                'use_sweep_kernel': True,
                'weight': 10000.0,
            },
            'self_collision_cfg': {'weight': 100000.0},
        },
    },
    'optimizer': {
        'solver_type': 'lbfgs',
        'solver_name': 'lbfgs',
        'cost_convergence': 1.0e-11,
        'cost_delta_threshold': 0.0,
        'cost_relative_threshold': 1.0,
        'epsilon': 0.01,
        'fixed_iters': True,
        'history': 27,
        'inner_iters': 25,
        'last_best': 10,
        'line_search_scale': [0.01, 0.1, 0.5, 1.0],
        'line_search_type': 'approx_wolfe',
        'num_iters': 50,
        'num_problems': 1,
        'stable_mode': True,
        'step_scale': 0.98,
        'store_debug': False,
        'sync_cuda_time': False,
        'use_coo_sparse': True,
        'use_cuda_kernel': True,
        'use_cuda_line_search_kernel': True,
        'use_cuda_update_best_kernel': True,
        'use_shared_buffers_kernel': True,
        'line_search_wolfe_c_1': 0.001,
        'line_search_wolfe_c_2': 0.98,
        'return_best_action': True,
    },
}


def _build_lbfgs_custom_config(node) -> dict:
    """Declare (if needed) and read back every 'lbfgs.*' ROS param (see
    _LBFGS_DEFAULTS above and config/mpc/lbfgs_params.yaml), then pop off the
    create()-level kwargs (warm_start_iters/cold_start_iters/horizon/
    lbfgs_debug) that are not optimizer_configs fields. debug_info is
    re-injected because it has no ROS-param equivalent (null field)."""
    declare_from_defaults(node, 'lbfgs', _LBFGS_DEFAULTS)
    cfg = read_nested(node, 'lbfgs')
    lbfgs_debug = cfg.pop('lbfgs_debug')
    warm_start_iters = cfg.pop('warm_start_iters')
    cold_start_iters = cfg.pop('cold_start_iters')
    horizon = cfg.pop('horizon')
    cfg['optimizer']['debug_info'] = {'visual_traj': None}
    return cfg, lbfgs_debug, warm_start_iters, cold_start_iters, horizon


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

        create_kwargs = dict(
            robot=cw.robot_config_file,
            scene_model=cw.obstacle_manager.primitives_only_scene(),
            collision_cache=cw.collision_cache,
            use_cuda_graph=resolve_use_cuda_graph(node),
            optimization_dt=optimization_dt,
        )

        # Off by default (see this module's docstring) -- opt-in application
        # of this repo's own tuned lbfgs cost/optimizer config, migrated from
        # the now-deleted config/mpc/lbfgs_mpc.yaml to real ROS params. Only
        # store_debug/warm_start/cold_start map onto confirmed create()
        # kwargs (same names MPPIController already uses); 'horizon' has NO
        # direct create() kwarg for the B-spline transition model this
        # controller uses by default (unlike MPPI's ACCELERATION transition
        # model, which takes horizon explicitly) -- num_control_points is the
        # closest create()-level lever, but the two are not proven equivalent
        # for this controller, so 'horizon' is declared as a ROS param
        # (settable, inspectable) but deliberately NOT threaded into create()
        # here. Verify that mapping on hardware before relying on it.
        if not node.has_parameter('lbfgs_apply_custom_config'):
            node.declare_parameter('lbfgs_apply_custom_config', False)
        if node.get_parameter('lbfgs_apply_custom_config').value:
            (optimizer_cfg, lbfgs_debug, warm_start_iters,
             cold_start_iters, _horizon) = _build_lbfgs_custom_config(node)
            create_kwargs.update(
                optimizer_configs=[optimizer_cfg],
                store_debug=lbfgs_debug,
                warm_start_optimization_num_iters=warm_start_iters,
                cold_start_optimization_num_iters=cold_start_iters,
            )

        # Iteration counts ALONE, independent of lbfgs_apply_custom_config.
        # Without this the only way to change them was to flip that flag,
        # which also swaps in the whole tuned rollout/optimizer tree of
        # _LBFGS_DEFAULTS (cost weights, history, step_scale, ...) -- so a
        # solve_ms change could never be attributed to the iteration count
        # rather than to the new cost function. These two knobs are the
        # single biggest lever on solve_ms (measured 202 ms mean at cuRobo's
        # default warm=200, ~1 ms per iteration) and deserve to be settable
        # on their own.
        #
        # 0 means "leave cuRobo's own default alone" (200 warm / 300 cold,
        # solver_mpc_cfg.py) -- NOT zero iterations. Set explicitly, they win
        # over the values the custom-config branch above may have just put in
        # create_kwargs, so a targeted override stays targeted.
        #
        # Must be a positive multiple of the optimizer's inner_iters (25 in
        # both cuRobo's stock lbfgs_mpc.yml and _LBFGS_DEFAULTS): LBFGSOptCfg
        # .update_niters() raises on anything else, and it does so at the
        # first solve -- long after launch, mid-motion. Rejected here instead,
        # at build time, with the arm still stopped.
        for name, kw, default in (
            ('lbfgs_warm_start_iters', 'warm_start_optimization_num_iters', 0),
            ('lbfgs_cold_start_iters', 'cold_start_optimization_num_iters', 0),
        ):
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            iters = int(node.get_parameter(name).value)
            if iters <= 0:
                continue
            inner = _LBFGS_INNER_ITERS
            if iters % inner != 0:
                node.get_logger().error(
                    f"{name}={iters} is not a multiple of inner_iters ({inner}) -- "
                    f"cuRobo's LBFGS would raise on the first solve. Ignored; "
                    f"using {create_kwargs.get(kw, 'cuRobo default')}.")
                continue
            create_kwargs[kw] = iters
            node.get_logger().info(f"LBFGS: {kw}={iters}")

        config = ModelPredictiveControlCfg.create(**create_kwargs)
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

        # Raising _command_points to buy that margin would ALSO stretch
        # _publish_period (it is (n-1)*dt below) and slow the control loop --
        # which is why the two are separated here instead. The extra points are
        # a tail that is normally DISCARDED: each publish replaces the queue
        # (execute_trajectory.cpp `this->trajectory = *msg;`), so they are only
        # ever played when a solve runs late. Reaction time is set by
        # _publish_period, not by queue depth, and is unchanged.
        #
        if not node.has_parameter('lbfgs_publish_points'):
            node.declare_parameter('lbfgs_publish_points', 11 )#/self._command_points)
        self._publish_points = max(
            self._command_points, int(node.get_parameter('lbfgs_publish_points').value))
        self._interpolation_dt = interpolation_dt
        # Where the publishable part of robot_state_sequence starts. Read from
        # the TEM rather than hardcoded to 4: it is defined as
        # interpolation_steps, so it tracks the config.
        self._command_start_idx = int(
            getattr(solver.trajectory_execution_manager, 'command_start_idx', 4))

        self._publish_period = (self._command_points - 1) * interpolation_dt
        self._next_publish_t = None

        # _exec_state's bookkeeping: how many points the arm is assumed to have
        # played when the next publish replaces the queue.
        #
        # False = the historical behaviour, m = _command_points - 1, i.e. the
        # NOMINAL _publish_period. That is only correct when the loop actually
        # holds its period. Measured 2026-09-07: nominal 400ms against a real
        # cycle of 482-563ms, so the arm played 6.0-7.0 points while _exec_state
        # counted 5. The 17-29% of un-counted motion COMPOUNDS, because
        # _close_state_loop feeds _exec_state back in as the next step's
        # current_state -- 2 to 15 deg of plan/real error by the end of a goal,
        # which is what made the execution watchdog cancel.
        #
        # True = derive m from the previous publish-to-publish cycle actually
        # measured in _hold_publish(). This is bookkeeping of commands this
        # planner ITSELF emitted -- no real-world feedback enters the solver,
        # and it does NOT change which points are published (that is
        # _publish_points, sliced independently at the top of step()) nor how
        # often (that is _publish_period). It only corrects what the planner
        # BELIEVES the arm did with them.
        if not node.has_parameter('lbfgs_dynamic_m'):
            node.declare_parameter('lbfgs_dynamic_m', True)
        self._dynamic_m = bool(node.get_parameter('lbfgs_dynamic_m').value)

        # Duration of the last COMPLETED publish-to-publish cycle, and the
        # timestamp of the last publish it is measured from. None until two
        # publishes have gone out (and after every setup(), which restarts the
        # publish clock) -- m then falls back to the nominal value.
        self._measured_cycle_s = None
        self._t_last_publish = None
        self._last_m = 0  # diagnostics only (CSV columns m_used / cycle_ms)
        self._m_cycle_s = None
        # Wall-clock timestamp when the previous step() call returned -- lets
        # _hold_publish()'s late-warning break "outside step()" (send_command,
        # _close_state_loop, perception refresh, live-goal check, loop
        # overhead -- all in reactive_controller.py's _execute_immediate, not
        # measured here individually) out from step()'s own phases. See
        # _last_cycle_breakdown.
        self._t_prev_step_end = None
        self._last_cycle_breakdown = (0.0, 0.0, 0.0, 0.0)  # (outside_step, predicted_path, controller_fk, step_diag) ms
        # Kept OUT of the tuple above on purpose: that tuple is consumed by
        # _hold_publish()'s late-publish warn(), which runs BEFORE _last_hold_ms
        # is known for the cycle it is describing. Separate attributes let
        # _csv_write (later still, from _close_state_loop) report both without
        # the warn() ever printing a stale or off-by-one hold figure.
        self._last_exec_state_ms = 0.0
        self._last_hold_ms = 0.0
        # Per-phase decomposition of that outside_step lump, snapshotted at the
        # instant the window closes -- see _snapshot_outside_breakdown.
        self._outside_breakdown = {}

        # Drain the CUDA queue at every phase boundary (see
        # ReactiveController._mark). DEBUG ONLY, off by default, for two
        # reasons worth stating explicitly: the drain costs real time on every
        # cycle, and it CHANGES the distribution it measures -- numbers from a
        # profile_sync run are not comparable to numbers from a normal one.
        # What it buys is correct attribution: without it the solve's GPU work
        # is billed to whichever later phase first blocks on .item().
        if not node.has_parameter('lbfgs_profile_sync'):
            node.declare_parameter('lbfgs_profile_sync', False)
        self._profile_sync = bool(node.get_parameter('lbfgs_profile_sync').value)

        # NOT setting _command_interval: execute() then picks
        # _execute_immediate() (solve -> publish the whole segment -> re-solve)
        # rather than the paced producer/consumer loop, whose per-tick pop
        # would undo the multi-point publish.

        node.get_logger().info(
            f"LBFGS solver built: interpolation_dt={interpolation_dt}s, "
            f"optimization_dt={optimization_dt}s, "
            f"command_points={self._command_points} "
            f"({self._command_points * interpolation_dt:.3f}s consumed, "
            f"republished every {self._publish_period:.3f}s), "
            f"publish_points={self._publish_points} "
            f"({(self._publish_points - self._command_points) * interpolation_dt:.3f}s "
            f"underrun tail), "
            f"robot={cw.robot_config_file}, collision_cache={cw.collision_cache}"
        )
        return solver

    def setup(self, start_state: JointState, goal_request: Any) -> bool:

        self._publish_points = max(
            self._command_points,
            int(self.node.get_parameter('lbfgs_publish_points').value))

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
        # Same reason: the previous goal's cycle says nothing about this one's
        # first segment (the first solve of a goal may capture a CUDA graph and
        # is not representative), so m falls back to nominal until two publishes
        # of THIS goal have been timed.
        self._measured_cycle_s = None
        self._t_last_publish = None
        # Same reason once more, and this one would otherwise go NEGATIVE: with
        # _t_prev_step_end cleared, the first step() of this goal measures
        # outside_step_ms = 0, while the phase counters still hold the previous
        # goal's last cycle -- loop_other_ms is computed as the residual, so it
        # would come out as minus that stale sum.
        self._reset_phase_counters()
        self._outside_breakdown = {}
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
        t_step_start = self._mark()
        # Everything since the PREVIOUS step() call returned: _send_command,
        # _close_state_loop, perception refresh, live-goal check and plain
        # loop overhead, all in reactive_controller.py's _execute_immediate.
        # This total is still measured by subtraction; what it is made of is
        # now itemised by _snapshot_outside_breakdown (the phase timers live
        # in _execute_immediate and _close_state_loop, since that is where
        # the work happens). See _last_cycle_breakdown / _hold_publish.
        outside_step_ms = (
            (t_step_start - self._t_prev_step_end) * 1000.0
            if self._t_prev_step_end is not None else 0.0
        )
        self._snapshot_outside_breakdown(outside_step_ms)

        t_solve = self._mark()
        result = self.solver.optimize_action_sequence(current_state)
        self._last_solve_ms = (self._mark() - t_solve) * 1000.0

        # RViz feed of the full predicted horizon. Placed before
        # _hold_publish() so this FK+publish is absorbed by that call's
        # sleep-to-deadline rather than delaying the trajectory command it
        # gates (see module docstring on why that publish is time-critical).
        t0 = self._mark()
        self._diag.publish_predicted_path(result)
        # Kept as a named instant rather than discarded: it is also where the
        # controller_fk phase starts, so reusing it saves one _mark() (a full
        # torch.cuda.synchronize() under profile_sync) per cycle. What it folds
        # into controller_fk is the `seq` slice below -- tensor VIEWS, no copy,
        # no kernel -- so the attribution stays honest.
        t_pp_end = self._mark()
        predicted_path_ms = (t_pp_end - t0) * 1000.0

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
            # Nothing was published, so no plan point was reached: the arm is
            # commanded to hold. Say so in the CSV rather than leaving the
            # previous segment's m standing.
            self._last_m = 0
            self._m_cycle_s = None
            self._last_controller_position_error = float('inf')
            self._last_controller_orientation_error = float('inf')
            # Neither phase ran on this path. Say 0 rather than leaving the
            # previous cycle's values standing, for the same reason as _last_m.
            self._last_exec_state_ms = 0.0
            self._last_hold_ms = 0.0
            # _mark(), like the t_step_start that closes this window: under
            # profile_sync both ends must drain, or GPU work still in flight
            # when step() returns is billed to the outside-step phases.
            self._t_prev_step_end = self._mark()
            return action

        seq = JointState(
            position=full.position[:, j:, :],
            velocity=full.velocity[:, j:, :] if full.velocity is not None else None,
            acceleration=full.acceleration[:, j:, :] if full.acceleration is not None else None,
            joint_names=full.joint_names,
        )

        n = min(self._publish_points, seq.position.shape[1])
        self._last_n_pts = n

        # "Controller error" at THIS instant, not result.position_error --
        # measured (probe_controller_error.py) to sit near zero from the very
        # first solve even with the state frozen 0.5 m from the goal: it is
        # the optimizer's own horizon-convergence metric (how close its plan
        # gets to the goal BY THE END of its horizon), not a real-time
        # tracking error, so it stays small independent of how far the
        # current/real state actually is.
        #
        # FK-measure current_state, NOT seq's point 0: seq[0] is
        # command_start_idx (4) steps into this segment, i.e. the plan's
        # estimate of where the arm will be ~320ms from now -- comparing that
        # to the real arm's error-right-now (position_error, in
        # _close_state_loop) mixes two different instants, so the difference
        # tracks EE speed over that 320ms rather than actual plan/real
        # divergence (grasp.py's watchdog was firing on fast-but-on-plan
        # motion because of exactly this). current_state is where the arm
        # will actually be when this solve's result starts playing, so
        # FK(current_state) is directly comparable to FK(real_state) at the
        # same instant.
        #
        # Both errors from ONE _fk_pose_errors pass: the previous
        # _fk_position_error + _fk_orientation_error pair ran the same FK
        # twice on the same state and blocked on two separate .item() calls.
        _, self._last_controller_position_error, self._last_controller_orientation_error = (
            self._fk_pose_errors(current_state))
        t_fk_end = self._mark()
        controller_fk_ms = (t_fk_end - t_pp_end) * 1000.0

        # The arm plays points 0..m-1 of this segment before the next publish
        # replaces the queue (points m..n-1 are the tail, normally discarded).
        # Those m velocities move it by the plan's DELTA over those points --
        # from wherever it actually is, NOT to the plan's absolute position.
        # The segment starts j steps ahead of current_state, so seq.position[m]
        # taken absolutely double-counts that lead: the plan would advance j+m
        # steps per cycle while the arm advances m. Anchor on current_state and
        # add only the delta, and the two are equal by construction for any j.
        #
        # m is how many points the arm actually gets through before the queue is
        # replaced, which is (cycle duration) / interpolation_dt -- NOT the
        # nominal _command_points - 1, unless the loop holds its period exactly.
        # See _dynamic_m in build_solver() for the measurements. The cycle that
        # matters here is the one this segment is about to be played over, which
        # has not happened yet, so the last completed one is used as the
        # estimate: that removes the SYSTEMATIC bias (the loop overruns in one
        # direction only, never under-runs -- _hold_publish sleeps to the
        # deadline) and leaves only zero-mean jitter.
        #
        # Clamped to [1, n-1]: at least one point is always played, and n-1 is
        # a real saturation, not just a guard -- once the queue is exhausted
        # execute_trajectory.cpp commands zero velocity and the arm holds at the
        # last point, so it cannot advance past it however late the publish is.
        m_nominal = min(self._command_points, n) - 1
        if self._dynamic_m and self._measured_cycle_s is not None:
            m = int(round(self._measured_cycle_s / self._interpolation_dt))
            m = max(1, min(m, n - 1))
        else:
            m = m_nominal
        self._last_m = m
        # Snapshot the estimate m was actually derived from: _hold_publish()
        # runs later in this same step() and overwrites _measured_cycle_s with
        # the cycle that has just ended, so reading it back in _csv_write (via
        # _close_state_loop, later still) would pair m with the WRONG cycle and
        # make the bookkeeping look off by one when it is not.
        self._m_cycle_s = self._measured_cycle_s
        self._exec_state = self._point(seq, m)
        self._exec_state.position = (
            current_state.position + (seq.position[:, m, :] - seq.position[:, 0, :])
        )

        # Closes the _exec_state phase: _point() plus the position arithmetic
        # just above are real GPU ops that used to sit in NO phase at all --
        # part of the 59.1ms (13.3% of the cycle) the five top-level terms
        # failed to account for on lbfgs_diag_20260909_150318.
        t_exec_end = self._mark()
        self._last_exec_state_ms = (t_exec_end - t_fk_end) * 1000.0

        # Diagnostics for the segment about to be published -- before
        # _hold_publish() so this work is absorbed by that call's
        # sleep-to-deadline rather than delaying the time-critical publish
        # (same reasoning as publish_predicted_path above). Budget is the
        # segment's own duration: the same one _hold_publish() itself warns
        # against when overrun.
        self._diag.publish_step_diagnostics(
            solve_ms=self._last_solve_ms, budget_ms=self._publish_period * 1000.0,
            result=result, position=seq.position[:, :n, :],
            velocity=seq.velocity[:, :n, :] if seq.velocity is not None else None,
            acceleration=seq.acceleration[:, :n, :] if seq.acceleration is not None else None,
            joint_names=seq.joint_names, dt=self._interpolation_dt,
        )
        t_diag_end = self._mark()
        step_diag_ms = (t_diag_end - t_exec_end) * 1000.0

        self._last_cycle_breakdown = (outside_step_ms, predicted_path_ms, controller_fk_ms, step_diag_ms)

        self._hold_publish()
        # Opens the "outside step()" window; t_step_start closes it. Both ends
        # go through _mark() so the boundary is drained symmetrically under
        # profile_sync -- otherwise step_diag's trailing GPU work lands in
        # whichever outside-step phase first blocks on it.
        self._t_prev_step_end = self._mark()
        # The last unaccounted region of step(): _hold_publish()'s sleep (0 on
        # an overrunning cycle, which is why the late-publish warn() does not
        # report this term -- there it is always ~0 and says nothing) plus the
        # closing _mark() itself. On a cycle that meets its budget this is the
        # slack, and the seven top-level terms should then sum to cycle_now_ms.
        self._last_hold_ms = (self._t_prev_step_end - t_diag_end) * 1000.0

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

    def _snapshot_outside_breakdown(self, outside_step_ms: float):
        """Freeze the per-phase split of the window that just closed.

        Called at the top of step(), which is exactly when the "outside
        step()" window ends -- so the phase counters hold the right values
        and pair with THIS cycle's ``outside_step_ms``. The window spans two
        loop iterations by construction: the tail of the previous one
        (send_command, close_state_loop, feedback) plus the head of this one
        (perception, live_goal). That is not an off-by-one, it is what the
        gap between two step() calls physically contains.

        ``loop_other`` is the residual -- iteration overhead, the throttled
        status log, and anything not itemised above. A large residual means a
        phase is missing from this list, not that the loop is slow.

        Note ``csv_write`` measures the diagnostics themselves: this CSV is
        not free, and a profile that hid its own cost would be misleading.
        """
        items = {
            'perception_ms': self._diag_perception_ms,
            'live_goal_ms': self._diag_live_goal_ms,
            'send_command_ms': self._diag_send_command_ms,
            'read_state_ms': self._diag_read_state_ms,
            'fk_err_ms': self._diag_fk_err_ms,
            'csv_write_ms': self._diag_csv_write_ms,
            'feedback_ms': self._diag_feedback_ms,
        }
        # Residual over the DISJOINT phases only -- computed before the
        # live_goal sub-terms are added below, since those are a breakdown OF
        # live_goal_ms, not siblings of it. Summing them in would double-count
        # and drive loop_other negative.
        items['loop_other_ms'] = outside_step_ms - sum(items.values())
        items['perception_ran'] = self._diag_perception_ran
        items['live_goal_wait_ms'] = self._diag_live_goal_wait_ms
        items['live_goal_apply_ms'] = self._diag_live_goal_apply_ms
        self._outside_breakdown = items

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
                # outside_step (itemised by _snapshot_outside_breakdown) +
                # predicted_path (publish_predicted_path) + controller_fk
                # (the current_state FK used for
                # _last_controller_position_error) +
                # step_diag (publish_step_diagnostics: FK + cost breakdown +
                # trajectory build/publish).
                #
                # This warning is throttled and only fires on cycles that
                # overran, so it is an alert, not a sample: for analysis read
                # the same terms from the CSV, which carries them every cycle.
                cycle_start = self._next_publish_t - self._publish_period
                cycle_ms = (now - cycle_start) * 1000.0
                outside_step_ms, predicted_path_ms, controller_fk_ms, step_diag_ms = (
                    self._last_cycle_breakdown)
                bd = self._outside_breakdown
                outside_detail = " ".join(
                    f"{k[:-3]} {bd[k]:.0f}ms" for k in
                    ('perception_ms', 'live_goal_ms', 'send_command_ms', 'read_state_ms',
                     'fk_err_ms', 'csv_write_ms', 'feedback_ms', 'loop_other_ms')
                    if k in bd) or "not yet sampled"
                self.node.get_logger().warn(
                    f"LBFGS: publish {-wait * 1000.0:.0f}ms late - cycle "
                    f"{cycle_ms:.0f}ms = outside_step {outside_step_ms:.0f}ms "
                    f"[{outside_detail}] + solve "
                    f"{self._last_solve_ms:.0f}ms + predicted_path "
                    f"{predicted_path_ms:.0f}ms + controller_fk "
                    f"{controller_fk_ms:.0f}ms + step_diag {step_diag_ms:.0f}ms "
                    f"vs {self._publish_period * 1000.0:.0f}ms budget - raise "
                    f"lbfgs_publish_points to absorb it (queue tail, does NOT "
                    f"slow the loop), or cut whichever term dominates",
                    throttle_duration_sec=5.0,
                )
        # This instant IS the publish: the caller publishes as soon as step()
        # returns. Measuring publish-to-publish here (rather than from
        # _next_publish_t, which is the nominal deadline) is what makes
        # _measured_cycle_s the REAL segment duration, overrun included -- the
        # quantity m must be derived from. See _dynamic_m in build_solver().
        t_publish = time.monotonic()
        if self._t_last_publish is not None:
            self._measured_cycle_s = t_publish - self._t_last_publish
        self._t_last_publish = t_publish
        self._next_publish_t = t_publish + self._publish_period

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
        t0 = self._mark()
        real_state = self._read_state(robot_context)
        self._diag_read_state_ms = (self._mark() - t0) * 1000.0

        # All three metrics from ONE FK pass and one device->host transfer.
        # The old xyz-then-orientation pair already avoided re-running FK for
        # the position scalar, but _fk_orientation_error still redid the whole
        # pass for the angle.
        t0 = self._mark()
        (self._last_position_error_xyz,
         self._last_position_error,
         self._last_orientation_error) = self._fk_pose_errors(real_state)
        self._diag_fk_err_ms = (self._mark() - t0) * 1000.0
        self._update_hold()

        t0 = self._mark()
        self._csv_write(real_state, state, robot_context)
        self._diag_csv_write_ms = (self._mark() - t0) * 1000.0
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

        ``q_pred_*`` is plan point ``m`` (column ``m_used``; with
        ``lbfgs_dynamic_m`` it tracks the measured cycle, column ``cycle_ms``,
        instead of the nominal ``_command_points - 1``) -- the state fed to the
        next solve,
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
            + [f"q_pred_j{i + 1}_deg" for i in range(len(names))]
            # Appended at the END so existing parsers reading by index still
            # line up. m_used is the point q_pred_* was taken at; cycle_ms is
            # the measured publish-to-publish duration m was derived from
            # (blank on the first segment of a goal, where m falls back to
            # nominal). m_used * interpolation_dt should track cycle_ms -- if
            # it does not, the bookkeeping is drifting again.
            + ["m_used", "cycle_ms"]
            # Per-phase split of the cycle, also appended at the END. The five
            # top-level terms (outside_step + solve + predicted_path +
            # controller_fk + step_diag) should sum to cycle_ms, and the
            # outside_step_* columns should in turn sum to outside_step_ms --
            # two independent closure checks on the profile.
            #
            # These used to exist only inside _hold_publish()'s late-publish
            # warning: throttled to 5s, and by construction only ever sampled
            # on cycles that OVERRAN. Written per row here so the sample is
            # every cycle and unbiased.
            #
            # perception_ran is 0/1 because refresh_perception_world only runs
            # every perception_refresh_period-th iteration: averaging
            # perception_ms over all rows without splitting on this column
            # halves the number and hides which cycles actually pay it.
            #
            # cycle_now_ms, NOT cycle_ms, is the cycle these terms decompose.
            # cycle_ms carries _m_cycle_s, which is deliberately the PREVIOUS
            # cycle (it must pair with m_used -- see the _m_cycle_s snapshot in
            # step()), whereas the breakdown describes the cycle that just
            # ended at this step()'s publish. Pairing the two is off by one:
            # measured on lbfgs_diag_20260909_110612, correlating the five
            # top-level terms against cycle_ms gave -0.30 and against the NEXT
            # row's cycle_ms +0.99. That offset also flips the sign of any
            # alternating effect -- it made perception look like it SAVED
            # 120ms/cycle when it costs that much.
            + ["outside_step_ms", "predicted_path_ms", "controller_fk_ms", "step_diag_ms",
               "cycle_now_ms",
               "perception_ms", "perception_ran",
               "live_goal_wait_ms", "live_goal_apply_ms", "live_goal_ms",
               "send_command_ms",
               "read_state_ms", "fk_err_ms", "csv_write_ms", "feedback_ms",
               "loop_other_ms", "profile_sync"]
            # Appended last, and they are top-level terms (siblings of
            # outside_step_ms / solve_ms / ...), NOT sub-terms of outside_step:
            # both measure regions INSIDE step(). With them the closure check
            # becomes a SEVEN-term sum against cycle_now_ms. They exist because
            # the five-term version left 59.1ms (13.3%) unexplained under
            # profile_sync=1 versus 3.3% without it -- exec_state_ms should
            # capture the _point()/position GPU work, hold_ms the sleep and the
            # sync overhead. If the gap is still large with both present, the
            # remainder is the _mark() calls themselves and will NOT exist in a
            # production run (profile_sync=0): it is an accounting artefact, not
            # a saving available in the budget.
            + ["exec_state_ms", "hold_ms"])
        n_pts = getattr(self, '_last_n_pts', 0)
        outside_step_ms, predicted_path_ms, controller_fk_ms, step_diag_ms = (
            self._last_cycle_breakdown)
        bd = self._outside_breakdown
        csv.writerow(
            [f"{time.monotonic() - self._csv_t0:.3f}",
             f"{getattr(self, '_last_solve_ms', float('nan')):.1f}",
             f"{n_pts}", f"{n_pts * self._interpolation_dt * 1000.0:.0f}",
             f"{self._last_position_error:.5f}",
             f"{self._fk_position_error(predicted_state):.5f}",
             f"{math.degrees(self._last_orientation_error):.3f}",
             f"{math.degrees(v_max):.2f}", f"{math.degrees(v_real):.2f}",
             f"{self._hold_count}", f"{int(self.is_on_target())}"]
            + [f"{v:.2f}" for v in q_real] + [f"{v:.2f}" for v in q_pred]
            + [f"{self._last_m}",
               "" if self._m_cycle_s is None
               else f"{self._m_cycle_s * 1000.0:.0f}"]
            + [f"{outside_step_ms:.1f}", f"{predicted_path_ms:.1f}",
               f"{controller_fk_ms:.1f}", f"{step_diag_ms:.1f}",
               "" if self._measured_cycle_s is None
               else f"{self._measured_cycle_s * 1000.0:.1f}"]
            + [f"{bd.get('perception_ms', 0.0):.1f}",
               f"{bd.get('perception_ran', 0)}",
               f"{bd.get('live_goal_wait_ms', 0.0):.1f}",
               f"{bd.get('live_goal_apply_ms', 0.0):.1f}",
               f"{bd.get('live_goal_ms', 0.0):.1f}",
               f"{bd.get('send_command_ms', 0.0):.1f}",
               f"{bd.get('read_state_ms', 0.0):.1f}",
               f"{bd.get('fk_err_ms', 0.0):.1f}",
               f"{bd.get('csv_write_ms', 0.0):.1f}",
               f"{bd.get('feedback_ms', 0.0):.1f}",
               f"{bd.get('loop_other_ms', 0.0):.1f}",
               f"{int(self._profile_sync)}"]
            + [f"{self._last_exec_state_ms:.1f}", f"{self._last_hold_ms:.1f}"])

    def cancel(self):
        self._csv_close()
        super().cancel()
