#!/usr/bin/env python3


import copy
import math
import time
from typing import Any

import torch
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from curobo_msgs.msg import MpcCosts
from curobo.content import get_task_configs_path
from curobo.types import JointState, GoalToolPose, Pose
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.model_predictive_control import (
    ModelPredictiveControl,
    ModelPredictiveControlCfg,
)
from curobo._src.util.config_io import resolve_config, join_path

from .reactive_controller import ReactiveController
from curobo_ros.core.config_wrapper import resolve_use_cuda_graph
from curobo_ros.core.diagnostics import open_diag_csv


# --- MPPI recipe + ACCELERATION control space ------
# Scope of mpc_m1013_speedj2.py (validated on real m1013 on 2026-07-14). The
# default MPC config (L-BFGS + B-spline) freezes on certain poorly-conditioned
# m1013 postures: L-BFGS line-search with fixed grid [0,0.1,0.5,1.0] finds no
# valid step when the direction is ill-conditioned (6-DOF non-redundant near-singular).
# MPPI (no line-search) is immune to this. L-BFGS + ACCELERATION is impossible here
# (backward of acceleration integration kernel not implemented), so MPPI is the only
# viable optimizer.
_MPPI_CSPACE_REGULARIZATION = [0.3, 1.0, 0.0, 0.0, 0.0]  # [vel, acc, jerk, torque, energy]

# Velocity boundary feedback cap (deg/s) — now a SAFETY NET, not a control knob.
#
# Was 5.0, chosen when the runaway below was thought to be inherent to feeding
# planned velocity back in ACCELERATION mode. Instrumented on hardware
# 2026-08-07 (mpc_diag CSV) and that diagnosis was wrong:
#   - vfirst_max_dps sat at exactly 5.00 on EVERY step of the moving phase, i.e.
#     the cap bound continuously, telling the MPC it was crawling at 5 deg/s
#     while the arm was really at ~11.3 deg/s;
#   - so each cycle replanned a fresh acceleration from a false state, and the
#     plan-to-plan velocity discontinuity (accel_boundary_dps2) reached
#     200-424 deg/s^2, which JointSpeedStrategy's clamp then had to absorb
#     (saturating at its 45 deg/s^2 setting on 18 of 31 sends);
#   - the moment the arm slowed below the cap, vfirst == vlast == vbc and the
#     discontinuity fell to exactly 0.0 — the cap was the whole effect.
# The 6->80 deg/s runaway of debug 2026-07-15 came from sampling the LAST plan
# point for the feedback instead of the point actually executed; that is fixed
# at the sampling site in step(). This value is now only a hardware backstop
# against a regression there, set to the LOWEST per-joint velocity limit in the
# URDF (joint1/joint2 = 2.094 rad/s = 120 deg/s; joints 3-6 allow 180-225), so
# it bounds every joint and cannot bind in normal operation — the arm's measured
# working range is ~30 deg/s.
#
# HOW TO TELL A REGRESSION FROM FAST MOTION: "vbc_max_dps sits at the cap" is
# NOT by itself a failure. At 30.0 it pinned on 7/128 steps while the arm really
# reached 31.6 deg/s — the cap was simply tracking a genuinely fast arm. The
# failure signature is vbc_max_dps pinned at the cap while real_vel_max_dps
# (speedj_publish CSV) stays well BELOW it: that is the cap feeding the MPC a
# velocity the arm has already exceeded, which is the pathology this whole fix
# removed. Compare the two columns, never the cap alone.
_VBC_CAP_DPS = 120.0


def _build_mppi_transition_model(step_dt: float, horizon: int, interpolation_steps: int = 4) -> dict:
    return {
        "transition_model_cfg": {
            "control_space": "ACCELERATION",
            "dt_traj_params": {"base_dt": step_dt, "base_ratio": 1.0, "max_dt": step_dt},
            "horizon": horizon,
            "interpolation_steps": interpolation_steps,
            "n_knots": 0,
            "state_filter_cfg": {
                "enable": True,
                "filter_coeff": {"position": 0.1, "velocity": 0.1, "acceleration": 0.0, "jerk": 0.0},
            },
            "teleport_mode": False,
            "vel_scale": 1.0,
            "return_full_act_buffer": True,
        }
    }


def _build_mppi_optimizer_config(num_iters: int, num_particles: int = 800) -> dict:
    return {
        "rollout": {
            "cost_cfg": {
                "cspace_cfg": {
                    "activation_distance": [0.01] * 5,
                    # NOTE the indexing differs from "weight" just below:
                    #   weight                            = [pos, vel, accel, jerk, torque]
                    #   squared_l2_regularization_weight  = [vel, accel, jerk,
                    #                                        torque_smooth, energy_smooth]
                    # So index 0 here is VELOCITY, not position.
                    #
                    # Velocity regularization, back to 0.0. It was briefly set to 0.01
                    # (cuRobo's shipped value in task/mpc/lbfgs_mpc.yml) to damp what
                    # looked like null-space thrash: the M1013 has a spherical wrist, so
                    # axes 4 and 6 go collinear as axis 5 nears 0 and any (j4, -j4) pair
                    # produces exactly zero end-effector motion. corr(j4, j6) = -0.99 with
                    # ~92% of j4's motion cancelled by j6 looked like pure waste.
                    #
                    # Two measurements on 2026-08-07 killed that reading:
                    #
                    # 1. It is not thrash. Over the 16 moving steps of
                    #    mpc_diag_20260807_130000.csv, j4 rises smoothly 6 -> 32 -> 12
                    #    deg/s with ZERO sign changes and j6 mirrors it exactly. A
                    #    sustained, smooth counter-rotation is the wrist RECONFIGURING
                    #    through the null space, not the optimizer chattering. Damping it
                    #    suppresses a manoeuvre the arm may actually need to reach the
                    #    goal orientation.
                    #
                    # 2. Its authority is inverted. Ratio cost_cspace/cost_tool_pose_pos
                    #    over the same run: 0.001-0.035 during the fast phase (i.e. no
                    #    authority exactly when the counter-rotation happens), rising to
                    #    1.79-2.15 as the pose cost collapses near the goal -- so the one
                    #    place it dominates is the endgame, where it biases the optimizer
                    #    toward stopping rather than correcting. The pose cost varies over
                    #    four decades across a run; a constant regularization weight
                    #    cannot be right at both ends of that.
                    #
                    # If a genuine chattering signature ever shows up (sign flips, not a
                    # smooth ramp), the acceleration term (index 1, cuRobo ships 10000) is
                    # the one to reach for -- it penalizes the oscillation rather than the
                    # steady approach velocity. Note retime_regularization_weights=True
                    # rescales these by dt, so shipped magnitudes are not directly
                    # comparable. Add one at a time.
                    "squared_l2_regularization_weight": [0.0, 0.0, 0.0, 0.0, 0.0],
                    "weight": [10.0, 0.0, 0.0, 0.0, 0.0],
                    "cost_type": "STATE",
                    "retime_weights": False,
                    "retime_regularization_weights": True,
                    # Weight of joint IK anchor. At 0.0, enable_joint_position_tracking
                    # activated cspace_target at zero weight AND disabled target_cspace_dist
                    # → no force → complete freeze from start on hardware (cf. debug 2026-07-15).
                    # 50 = compromise validated in sandbox (converges; 200 throttles).
                    "cspace_target_weight": 0.0,
                    "cspace_non_terminal_weight_factor": 0.05,
                },
                "tool_pose_cfg": {
                    "use_lie_group": True,
                    "weight": [5000.0, 2000.0],
                    "_terminal_pose_convergence_tolerance": [0.0, 0.0],
                    # MUST be set explicitly. ToolPoseCriteria defaults it to
                    # [0,0,0,0,0,0] (tool_pose_criteria.py), i.e. the pose cost
                    # scores ONLY the terminal state of the horizon and weighs
                    # every preceding step at exactly zero. That is fatal here,
                    # because of how little of the horizon is ever executed:
                    #     optimized horizon   30 steps = 0.90s  <- only step 30 scored
                    #     action_sequence     16 steps = 0.48s  <- trimmed to
                    #                                             interpolation_steps*2
                    #     actually executed    8 steps = 0.24s  <- mpc_command_interval
                    # so the ONLY part of the plan the robot ever performs is the
                    # part the cost function ignores. Any trajectory that lands on
                    # the goal at step 30 scores identically whether it starts
                    # moving now or dawdles for half a second first, and nothing
                    # breaks the tie (squared_l2_regularization_weight is all
                    # zeros above, where cuRobo ships [0.01, 10000, 10, 0, 0]).
                    # Measured 2026-08-07: from t=2.7s to t=20s the plan reported
                    # a terminal error of 0.00002 m while FK on the arm's MEASURED
                    # joints stayed at 0.021 m, with no constraint active and
                    # commanded joint velocities of 0.2-3 deg/s whose signs flipped
                    # every window -- the executed prefix was unscored noise that
                    # averaged to no net motion. Not a tracking problem: the same
                    # run's speedj_publish CSV shows the arm following commands
                    # faithfully down to 0.6 deg/s (real/commanded 0.9-1.35), so
                    # there is no low-speed deadband to blame.
                    # 0.05 mirrors cspace_non_terminal_weight_factor above. With
                    # gamma=0.98 the discounted horizon sum is ~22.8, so the running
                    # term totals ~1.1x the terminal one -- enough to force early
                    # progress without flattening the terminal precision. Raise
                    # toward 0.1 if the approach is still lazy; obstacle clearance
                    # stays protected by scene_collision_cfg (constraint, weight
                    # 10000), not by this cost.
                    "_non_terminal_pose_axes_weight_factor": [0.05] * 6,
                },
            },
            "constraint_cfg": {
                "scene_collision_cfg": {
                    "activation_distance": 0.12,
                    "use_speed_metric": True,
                    "use_sweep": True,
                    "use_sweep_kernel": True,
                    "weight": 10000.0,
                },
                "self_collision_cfg": {"weight": 100000.0},
                # Joint POSITION limits as a hard constraint. Until 2026-08-07 this
                # entry did not exist, so the only trace of the joint limits in the
                # optimization rollout was the soft cost_cfg/cspace_cfg above:
                # weight 10 against tool_pose's 5000/2000, activating just 0.01 rad
                # from the bound. It lost every time, and the arm drove joint 4 to
                # 362.5 deg against its 360 deg limit (controller alarm 9008), while
                # the metrics rollout — which DOES carry this constraint, from
                # metrics_base.yml — reported con_cspace_bound non-zero on 80 of 81
                # steps. cuRobo knew, nothing acted on it. A joint limit is not
                # negotiable against pose error, hence a constraint, not a cost.
                #
                # POSITION ONLY on purpose. The weight vector is
                # [position, velocity, acceleration, jerk, torque] and the other four
                # stay at 0:
                #   - acceleration IS the MPPI action in ACCELERATION space and
                #     already saturates exactly at its bound (measured: 28.65 deg/s^2
                #     = max_acceleration 0.5 rad/s^2), so constraining it again would
                #     only fight the action clamp;
                #   - jerk is currently violated by roughly 3x on every step
                #     (+-0.5 rad/s^2 can flip sign in one 0.03s step -> ~33 rad/s^3
                #     against leeloo_curobo.yaml's max_jerk 10.0), so switching it on
                #     at weight 5000 could leave the optimizer with no feasible action
                #     at all. Raise max_jerk to a physically achievable value FIRST,
                #     verify con_cspace_bound drops, and only then consider enabling
                #     it here.
                "cspace_cfg": {
                    "weight": [5000.0, 0.0, 0.0, 0.0, 0.0],
                    # 0.20 rad (11.5 deg), widened from 0.05 on 2026-08-07.
                    #
                    # 0.05 was sized against a 2.5 deg overshoot, i.e. to catch a
                    # joint that had already essentially arrived at its bound. It
                    # is a stopping distance, and 2.9 deg is not one: at the
                    # 40-67 deg/s this arm reaches, 2.9 deg is ~50 ms of travel,
                    # well inside a single 240 ms command window, so the
                    # constraint could not act before the next window was already
                    # committed. Joint 5 (limit +-2.356 rad) reached its stop
                    # during a pose-matrix campaign while chasing a goal whose own
                    # |j5| was only 1.53 rad -- the bound was hit in transit, not
                    # demanded by the target.
                    #
                    # Cost: the outer ~11 deg of each bounded joint's range now
                    # carries a penalty, so poses very near a limit become harder
                    # to hold. That is the intended trade -- run_pose_matrix's
                    # --limit-margin (0.20 rad, same value) keeps the test matrix
                    # out of that band for the same reason.
                    #
                    # Note this only bites once a joint is within the band. It is
                    # not a substitute for damping null-space drift, which is what
                    # walks a joint into the band in the first place; see
                    # squared_l2_regularization_weight above, currently 0.
                    "activation_distance": [0.20, 0.0, 0.0, 0.0, 0.0],
                    "cost_type": "STATE",
                },
            },
        },
        "optimizer": {
            "solver_type": "mppi",
            "solver_name": "mppi",
            "num_iters": num_iters,
            "inner_iters": 1,
            "num_particles": num_particles,
            "null_act_frac": 0.05,
            "beta": 0.1,
            "gamma": 0.98,
            "kappa": 0.0001,
            "init_cov": 0.05,
            "update_cov": True,
            "cov_type": "DIAG_A",
            "step_size_mean": 0.9,
            # 0.1 (cuRobo's own default), NOT 0.01. Two effects, both fatal at 0.01:
            #   - time constant 1/step_size_cov = 100 iterations. At num_iters=5 per
            #     solve that is ~20 solves (~5 s) for ONE time constant, so the
            #     covariance never moves off init_cov during a manoeuvre;
            #   - noise floor: jit_blend_cov is
            #         cov <- (1 - step_size_cov)*cov + step_size_cov*cov_update + kappa
            #     whose fixed point for cov_update -> 0 is kappa/step_size_cov, i.e.
            #     0.0001/0.01 = 0.01. Exploration can never shrink below that.
            # Frozen at init_cov=0.05, the sampling std is sqrt(0.05)=0.224 rad/s^2
            # against a 0.5 rad/s^2 action bound - 45% of full scale, forever. The
            # bound then sits at 2.23 sigma, so over the 16x6 values of the window
            # ~2.5 samples hit the CLAMP every step by pure chance: that is the
            # measured accel_win_max_dps2 pinned at 28.60 (= 0.5 rad/s^2) on 95% of
            # the 168 steps of the 2026-08-07 run, while the arm only moved 2-4 deg/s.
            # The terminal limit cycle (error bouncing 1-6 cm for 20 s after reaching
            # 4.8 cm in 2.6 s) was this noise, not the optimizer asking for more
            # acceleration. At 0.1 the floor drops to 0.001 (std 1.8 deg/s^2) and the
            # covariance can actually contract as the cost landscape flattens.
            "step_size_cov": 0.1,
            # BEST, and NOT "MEAN": _get_action_seq returns self._dist.mean for MEAN
            # but the dedicated self._dist.best_traj for BEST, and _opt_iters builds
            # its OptimizationIterationState with action=/best_action=<that tensor>
            # WITHOUT cloning (it clones `cost` right next to it - that asymmetry is
            # the tell). ParticleOptCore.finish_init creates the _opt_iters graph
            # executor without clone_outputs, so it defaults to False and the graph
            # returns its outputs BY REFERENCE; optimize() then feeds that same
            # tensor back in as the input of the next of its outer_iters replays.
            # In MEAN that output IS _dist.mean, which the graph itself rewrites
            # every replay (_update_distribution: c._dist.mean.copy_(new_mean)) --
            # a read/write alias on a static captured-graph buffer. On Tegra it
            # surfaces as cudaErrorIllegalInstruction at the first sync point, i.e.
            # CudaEventTimer.stop() (sync_cuda_time is True), which is exactly where
            # the 2026-08-07 crash landed. cuRobo's dataclass default is MEAN because
            # that default targets the non-cuda-graph path. Do not switch this back.
            "sample_mode": "BEST",
            "squash_fn": "CLAMP",
            "base_action": "REPEAT",
            "random_mean": False,
            "seed": 0,
            "sample_per_problem": True,
            "sample_params": {
                "fixed_samples": True,
                "n_knots": 5,
                "filter_coeffs": [0.3, 0.3, 0.4],
                "sample_ratio": {
                    "halton": 0.3,
                    "halton-knot": 0.7,
                    "random": 0.0,
                    "random-knot": 0.0,
                    "stomp": 0.0,
                },
                "seed": 0,
            },
            "store_debug": False,
            "sync_cuda_time": True,
            "use_coo_sparse": True,
        },
    }


def _build_metrics_rollout_cfg(cost_cfg_source: dict) -> dict:
    """metrics_base.yml (the default metrics_rollout) has NO cost_cfg — only
    constraint_cfg + convergence_cfg — so get_current_metrics() never exposes
    weighted COST magnitudes, only constraint violations. Mirror the ACTIVE
    branch's tool_pose_cfg/cspace_cfg into a copy of metrics_base.yml's own
    cost_cfg so the metrics rollout (fixed batch size, no cuda-graph rebatch)
    computes them too, safe to read via get_current_metrics() every solve.

    CRASH-SAFETY (cf. debug 2026-07-20): a prior version instead called
    compute_metrics_from_action() on the OPTIMIZATION rollout (use_cuda_graph=True,
    shared with the optimizer) to get these same magnitudes — its rebatch
    (num_particles -> 1) under a captured graph triggered a device-side assert
    that corrupted the whole CUDA context. This metrics-rollout approach avoids
    that entirely: validated in sandbox with use_cuda_graph=True, identical cost
    values to the removed dangerous path, zero CUDA errors."""
    metrics_cfg = copy.deepcopy(
        resolve_config(join_path(get_task_configs_path(), "metrics_base.yml"))
    )
    metrics_cfg["rollout"]["cost_cfg"] = {
        "tool_pose_cfg": copy.deepcopy(cost_cfg_source["tool_pose_cfg"]),
        "cspace_cfg": copy.deepcopy(cost_cfg_source["cspace_cfg"]),
    }
    return metrics_cfg


class MPCController(ReactiveController):
    """Closed-loop MPC built on cuRobo ``ModelPredictiveControl`` (v2)."""

    def get_planner_name(self) -> str:
        return "Model Predictive Control (MPC)"

    def get_config_parameters(self) -> list:
        return ['convergence_threshold', 'convergence_threshold_rad',
                'convergence_hold_steps', 'max_mpc_iterations']

    # ---- cuRobo-specific hooks ------------------------------------------------

    def build_solver(self):
        cw = self.config_wrapper
        node = self.node
        step_dt = node.get_parameter('mpc_step_dt').get_parameter_value().double_value
        horizon = node.get_parameter('mpc_horizon_steps').get_parameter_value().integer_value
        warm_iters = node.get_parameter('mpc_warm_start_iters').get_parameter_value().integer_value
        cold_iters = node.get_parameter('mpc_cold_start_iters').get_parameter_value().integer_value
        solver_type = node.get_parameter('mpc_solver_type').get_parameter_value().string_value

        # Common kwargs for both branches. The REAL production collision scene
        # (obstacle_manager) must be preserved — do not copy the scene_model=None
        # from the standalone script.
        #
        # Built WITHOUT the perception voxel layer on purpose: handing a live
        # ESDF layer to the constructor makes cuRobo alias the solver's collision
        # buffer onto our tensor, and the first update_world then clears it to
        # "solid everywhere". collision_cache pre-allocates the voxel storage
        # instead; update_world fills it by copy. See primitives_only_scene().
        base_kwargs = dict(
            robot=cw.robot_config_file,
            scene_model=cw.obstacle_manager.primitives_only_scene(),
            optimization_dt=step_dt,
            use_cuda_graph=resolve_use_cuda_graph(node),
            self_collision_check=True,
            collision_cache=cw.collision_cache,
            store_debug=False,
            warm_start_optimization_num_iters=warm_iters,
            cold_start_optimization_num_iters=cold_iters,
        )

        self._use_mppi_acceleration = (solver_type == 'mppi_acceleration')
        self._vel_feedback_alpha = node.get_parameter('mpc_vel_feedback_alpha').get_parameter_value().double_value
        self._step_dt = step_dt  # For debug CSV (point-to-point interval in the loop)
        # Fixed-interval command pacing (s); 0.0 = off. Read by the servo loop
        # (ReactiveController.execute) to hold each command window for its full
        # execution duration before re-solving. See mpc_command_interval param.
        self._command_interval = node.get_parameter('mpc_command_interval').get_parameter_value().double_value
        if self._use_mppi_acceleration:
            num_particles = node.get_parameter('mpc_mppi_num_particles').get_parameter_value().integer_value
            mppi_optimizer_cfg = _build_mppi_optimizer_config(warm_iters, num_particles)
            # NO num_control_points here: it writes n_knots (B-spline concept).
            # The horizon lives in the transition_model dict we provide.
            cfg = ModelPredictiveControlCfg.create(
                optimizer_configs=[mppi_optimizer_cfg],
                transition_model=_build_mppi_transition_model(step_dt, horizon),
                squared_l2_regularization_weight=_MPPI_CSPACE_REGULARIZATION,
                interpolation_steps=8,
                metrics_rollout=_build_metrics_rollout_cfg(mppi_optimizer_cfg["rollout"]["cost_cfg"]),
                **base_kwargs,
            )
        else:
            lbfgs_cost_cfg = resolve_config(
                join_path(get_task_configs_path(), "mpc/lbfgs_mpc.yml")
            )["rollout"]["cost_cfg"]
            cfg = ModelPredictiveControlCfg.create(
                num_control_points=horizon,
                metrics_rollout=_build_metrics_rollout_cfg(lbfgs_cost_cfg),
                **base_kwargs,
            )
        solver = ModelPredictiveControl(cfg)

        node.mpc = solver
        # Predicted end-effector path (current MPC horizon), for RViz (nav_msgs/Path
        # renders natively, no custom plugin needed).
        self._path_pub = node.create_publisher(Path, 'mpc_predicted_path', 10)
        # Same data, guaranteed un-trimmed: _publish_predicted_path falls back to
        # the 8-point action_sequence whenever robot_state_sequence.tool_poses is
        # unavailable (cf. _publish_full_predicted_path). This topic always FKs
        # the full mpc_horizon_steps-length robot_state_sequence itself.
        self._full_path_pub = node.create_publisher(Path, 'mpc_predicted_path_full', 10)
        self._goal_marker_pub = node.create_publisher(Marker, 'mpc_goal_marker', 10)
        # Cost/constraint breakdown, for live inspection via rqt_plot (each
        # named field is individually plottable). See _cost_breakdown().
        self._cost_pub = node.create_publisher(MpcCosts, 'mpc_costs', 10)
        self._path_frame = cw.base_link
        node.get_logger().info(
            f"MPC solver built: solver_type={solver_type}, optimization_dt={step_dt}s, "
            f"horizon={horizon}, warm_start_iters={warm_iters}, cold_start_iters={cold_iters}, "
            f"robot={cw.robot_config_file}, collision_cache={cw.collision_cache}"
        )
        return solver

    def setup(self, start_state: JointState, goal_request: Any) -> bool:
        self._v_bc = None  # New goal = new velocity continuity baseline
        # Re-read per goal: build_solver() reads this once, and PlannerManager
        # caches the planner instance, so without this the send pacing could
        # only be changed by relaunching the whole stack. It is the main knob
        # on the batch-boundary discontinuity (the bridge plays
        # mpc_command_interval / interpolation_dt points before the next batch
        # overwrites the queue at index 0), so it needs to be sweepable with
        # `ros2 param set` between runs.
        self._command_interval = self.node.get_parameter(
            'mpc_command_interval').get_parameter_value().double_value
        self._csv_init()
        p = goal_request.target_pose
        raw = [
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z,
        ]
        goal = self._set_target(raw)
        self.solver.setup(start_state)
        applied = self._apply_goal(goal, raw, start_state)
        self.goal = goal
        self._publish_goal_marker(raw, applied)

        if self._debug_enabled():
            ee_pos, ee_quat = None, None
            try:
                kin = self.solver.compute_kinematics(start_state)
                ee_pos = [round(v, 4) for v in kin.tool_poses.position.reshape(-1, 3)[0].cpu().tolist()]
                ee_quat = [round(v, 4) for v in kin.tool_poses.quaternion.reshape(-1, 4)[0].cpu().tolist()]
            except Exception as e:
                self.node.get_logger().warn(f"[MPC DIAG] EE pose log failed: {e}")
            self.node.get_logger().debug(
                f"[MPC DIAG] setup: goal_xyz={[round(v, 4) for v in raw[0:3]]} "
                f"goal_quat(wxyz)={[round(v, 4) for v in raw[3:7]]} "
                f"tool_frame={self.solver.tool_frames[0]} "
                f"start_EE_pos={ee_pos} start_EE_quat(wxyz)={ee_quat} "
                f"fk_err={self._fk_position_error(start_state):.4f}m goal_applied={applied}"
            )
        return True

    # ---- Debug CSV ----

    def _csv_init(self):
        """Opens a diagnostic CSV (one line per step). Gated by mpc_debug.

        Closes any CSV left open by a previous goal first (setup() is called
        once per goal — without this, each new goal leaked the previous
        file's descriptor).
        """
        self._csv_close()
        if not self._debug_enabled():
            return
        self._csv = open_diag_csv(self.node, "mpc_diag")
        if self._csv is not None:
            self._csv_t0 = time.monotonic()
            self._csv_t_prev = self._csv_t0
            self._csv_last_vexec = None

    def _csv_close(self):
        if getattr(self, '_csv', None) is not None:
            self._csv.close()
        self._csv = None

    def _csv_write(self, result, solve_ms, breakdown: dict):
        if getattr(self, '_csv', None) is None:
            return
        seq = result.action_sequence
        now = time.monotonic()
        dt_step_ms = (now - self._csv_t_prev) * 1000.0  # Loop period: captures 248ms AND pauses ~10s
        self._csv_t_prev = now
        dt = self._step_dt
        deg = math.degrees
        vel = seq.velocity[0]  # [npts, dof]
        npts, dof = vel.shape
        vfirst = vel[0].cpu().tolist()
        vlast = vel[-1].cpu().tolist()
        # Intra-window accel: max over (i,j) of |v[i+1]-v[i]|/dt (MPPI plan noise)
        accel_win = deg((vel[1:] - vel[:-1]).abs().max().item() / dt) if npts > 1 else 0.0
        # Boundary accel, measured against the point the previous window actually
        # EXECUTED (_executed_idx, the same one _v_bc is sampled from), not its
        # last point. Comparing against vel[-1] measured a transition the robot
        # never makes — the unexecuted tail of the previous plan — and so just
        # reported the plan's own ramp across that tail: on 2026-08-07 the ratio
        # accel_boundary/accel_win pinned at exactly 8.0, i.e. the 8 unexecuted
        # steps, which is an artifact and not a discontinuity.
        kexec = getattr(self, '_executed_idx', npts - 1)
        kexec = max(0, min(kexec, npts - 1))
        vexec = vel[kexec].cpu().tolist()
        if self._csv_last_vexec is not None:
            accel_bd = deg(max(abs(a - b) for a, b in zip(vfirst, self._csv_last_vexec)) / dt)
        else:
            accel_bd = 0.0
        self._csv_last_vexec = vexec
        vbc = self._v_bc[0].cpu().tolist() if getattr(self, '_v_bc', None) is not None else [0.0] * dof
        pos_err = getattr(result, 'position_error', None)
        rot_err = getattr(result, 'rotation_error', None)
        pose_pos_err = float(pos_err.reshape(-1)[0].item()) if pos_err is not None else float('nan')
        pose_rot_err = float(rot_err.reshape(-1)[0].item()) if rot_err is not None else float('nan')
        self._csv.write_header_once(
            # cmd_interval_ms is constant per run but recorded per row so the CSV
            # is self-describing: the real-time budget for solve_ms is the SEND
            # interval, not dt_step_ms (in paced mode the producer loop period is
            # structurally ~solve_ms, so comparing the two always looks tight).
            ["t_s", "dt_step_ms", "solve_ms", "cmd_interval_ms",
             "fk_err_m", "fk_rot_err_deg", "vfirst_max_dps", "vlast_max_dps",
             "vexec_max_dps",
             "accel_win_max_dps2", "accel_boundary_dps2", "vbc_max_dps",
             "pose_pos_err_m", "pose_rot_err_rad", "cost_tool_pose_pos", "cost_tool_pose_orient",
             "cost_cspace", "con_self_collision", "con_scene_collision", "con_cspace_bound"]
            + [f"vfirst_j{i+1}_dps" for i in range(dof)]
            + [f"vlast_j{i+1}_dps" for i in range(dof)]
            + [f"q_j{i+1}_deg" for i in range(dof)])
        self._csv.writerow(
            [f"{now - self._csv_t0:.3f}", f"{dt_step_ms:.1f}", f"{solve_ms:.1f}",
             f"{self._command_interval * 1000.0:.1f}",
             f"{self._last_position_error:.5f}",
             # Degrees, 3 decimals: the old pose_rot_err_rad column plateaued at
             # 1.6e-4 with 5-decimal formatting, i.e. ~16 distinct values over a
             # whole run — it could neither confirm nor refute an orientation
             # complaint. This one resolves 0.001 deg.
             f"{deg(self._last_orientation_error):.3f}",
             f"{deg(max(abs(v) for v in vfirst)):.2f}",
             f"{deg(max(abs(v) for v in vlast)):.2f}",
             f"{deg(max(abs(v) for v in vexec)):.2f}",
             f"{accel_win:.1f}", f"{accel_bd:.1f}",
             f"{deg(max(abs(v) for v in vbc)):.2f}",
             f"{pose_pos_err:.5f}", f"{pose_rot_err:.5f}",
             f"{breakdown.get('cost_tool_pose_pos', float('nan')):.4f}",
             f"{breakdown.get('cost_tool_pose_orient', float('nan')):.4f}",
             f"{breakdown.get('cost_cspace', float('nan')):.4f}",
             f"{breakdown.get('con_self_collision', float('nan')):.4f}",
             f"{breakdown.get('con_scene_collision', float('nan')):.4f}",
             f"{breakdown.get('con_cspace_bound', float('nan')):.4f}"]
            + [f"{deg(v):.2f}" for v in vfirst] + [f"{deg(v):.2f}" for v in vlast]
            + [f"{deg(v):.2f}" for v in (getattr(self, '_last_q', None) or [float('nan')] * dof)])

    def _publish_predicted_path(self, result):
        """Publish the MPC's full predicted end-effector path for RViz.

        ``result.action_sequence`` (used elsewhere to command the robot) is
        NOT the full optimized horizon: cuRobo's TrajectoryExecutionManager
        trims it to only ``interpolation_steps * 2`` points — the near-term
        slice meant to be executed before the next re-plan (see
        ``get_command_sequence()``). The full horizon the optimizer actually
        reasoned over (collision costs, goal convergence, etc.) is
        ``result.robot_state_sequence`` instead, already FK'd (untrimmed).
        """
        if getattr(self, '_path_pub', None) is None:
            return
        try:
            state_seq = getattr(result, 'robot_state_sequence', None)
            if state_seq is not None and state_seq.tool_poses is not None:
                ee_pos = state_seq.tool_poses.position.reshape(-1, 3).cpu().tolist()
            else:
                # Fallback: FK on the trimmed action_sequence (partial horizon).
                seq = result.action_sequence
                js = JointState.from_position(seq.position[0], joint_names=self.solver.joint_names)
                fk = getattr(self.solver, 'compute_kinematics', None) or self.solver.kinematics.compute_kinematics
                ee_pos = fk(js).tool_poses.position.reshape(-1, 3).cpu().tolist()
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] predicted path FK failed: {e}", throttle_duration_sec=5.0)
            return
        self._path_pub.publish(self._ee_positions_to_path(ee_pos))

    def _ee_positions_to_path(self, ee_pos) -> Path:
        path = Path()
        path.header.frame_id = self._path_frame
        path.header.stamp = self.node.get_clock().now().to_msg()
        for x, y, z in ee_pos:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _publish_full_predicted_path(self, result):
        """Publish the MPC's full ``mpc_horizon_steps``-length predicted path.

        ``_publish_predicted_path`` above silently degrades to an 8-point path
        (``interpolation_steps * 2``) whenever ``robot_state_sequence.tool_poses``
        isn't populated by the metrics rollout. ``robot_state_sequence.joint_state
        .position`` itself has no such gap — the transition model always fills it
        for the whole horizon — so FK it directly here instead of trusting
        ``tool_poses`` to have been computed.
        """
        if getattr(self, '_full_path_pub', None) is None:
            return
        try:
            state_seq = getattr(result, 'robot_state_sequence', None)
            if state_seq is None:
                return
            if state_seq.tool_poses is not None:
                ee_pos = state_seq.tool_poses.position.reshape(-1, 3).cpu().tolist()
            else:
                js = JointState.from_position(
                    state_seq.joint_state.position[0], joint_names=self.solver.joint_names
                )
                fk = getattr(self.solver, 'compute_kinematics', None) or self.solver.kinematics.compute_kinematics
                ee_pos = fk(js).tool_poses.position.reshape(-1, 3).cpu().tolist()
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] full predicted path FK failed: {e}", throttle_duration_sec=5.0)
            return
        self._full_path_pub.publish(self._ee_positions_to_path(ee_pos))

    def _cost_breakdown(self, result) -> dict:
        """Per-term COST and CONSTRAINT values (horizon-summed) for this solve.

        CRASH-SAFETY NOTE (cf. debug 2026-07-20): an earlier version read the
        weighted COST magnitudes (tool_pose, cspace/anchor) via
        `compute_metrics_from_action(result.action_buffer)` on the
        OPTIMIZATION rollout (the only one with cost_cfg set by default). That
        rollout runs with use_cuda_graph=True and is shared with the
        optimizer; the extra call rebatches it (num_particles -> 1) while a
        CUDA graph is captured for a different batch size, which triggered a
        device-side assert (`index out of bounds`) on real hardware. A
        device-side assert corrupts the WHOLE CUDA context for the process —
        unrecoverable via try/except, and it took down the next solve,
        _send_command, and perception/voxelization with it. That call has
        been REMOVED. Do not reintroduce compute_metrics_from_action on a
        use_cuda_graph=True rollout shared with the optimizer.

        Costs are recovered safely instead: build_solver() injects a cost_cfg
        (mirroring the active branch's tool_pose_cfg/cspace_cfg) into the
        METRICS rollout's config (see _build_metrics_rollout_cfg) — a
        fixed-batch-size rollout, never rebatched, so it computes these costs
        as a side effect of the normal solve. get_current_metrics() just
        returns an attribute (`_current_metrics`) already populated during
        that solve — no rebatch, no graph, no extra GPU call. Validated in
        sandbox: identical cost values to the removed dangerous path, zero
        CUDA errors across use_cuda_graph=True runs."""
        try:
            m = self.solver.trajectory_execution_manager.get_current_metrics()
            if m is None:
                return {}
            cc = m.costs_and_constraints
            out = {}
            for name, val in zip(cc.costs.names, cc.costs.values):
                if name == "tool_pose":
                    out["cost_tool_pose_pos"] = float(val[0, :, 0].sum().item())
                    out["cost_tool_pose_orient"] = float(val[0, :, 1].sum().item())
                else:
                    out[f"cost_{name}"] = float(val.sum().item())
            for name, val in zip(cc.constraints.names, cc.constraints.values):
                if name == "cspace":
                    out["con_cspace_bound"] = float(val.sum().item())
                else:
                    out[f"con_{name}"] = float(val.sum().item())
            return out
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] cost breakdown failed: {e}", throttle_duration_sec=5.0)
            return {}

    def _publish_costs(self, result, breakdown: dict):
        if getattr(self, '_cost_pub', None) is None:
            return
        msg = MpcCosts()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self._path_frame
        msg.fk_err_m = float(self._last_position_error)
        msg.fk_rot_err_deg = math.degrees(float(self._last_orientation_error))
        # Deprecated pair, still published so old bags stay parseable -- see the
        # note in MpcCosts.msg. Read fk_err_m / fk_rot_err_deg instead.
        pos_err = getattr(result, 'position_error', None)
        rot_err = getattr(result, 'rotation_error', None)
        msg.pose_pos_err_m = float(pos_err.reshape(-1)[0].item()) if pos_err is not None else 0.0
        msg.pose_rot_err_rad = float(rot_err.reshape(-1)[0].item()) if rot_err is not None else 0.0
        msg.cost_tool_pose_pos = breakdown.get("cost_tool_pose_pos", 0.0)
        msg.cost_tool_pose_orient = breakdown.get("cost_tool_pose_orient", 0.0)
        msg.cost_cspace = breakdown.get("cost_cspace", 0.0)
        msg.con_self_collision = breakdown.get("con_self_collision", 0.0)
        msg.con_scene_collision = breakdown.get("con_scene_collision", 0.0)
        msg.con_cspace_bound = breakdown.get("con_cspace_bound", 0.0)
        self._cost_pub.publish(msg)

    def step(self, current_state: JointState) -> JointState:

        if getattr(self, '_use_mppi_acceleration', False):
            # Velocity continuity: the next solve starts from the velocity this
            # window will actually have reached (see the sampling index below),
            # so the MPC's assumed state matches the arm's real one.
            if self._v_bc is None:
                self._v_bc = torch.zeros_like(current_state.position)
            current_state = current_state.clone()
            current_state.velocity = self._v_bc

        # Snapshot of the state we are about to hand to the solver. Everything
        # the diagnostics and the convergence criterion report below must
        # describe THIS state, not the object as it looks once
        # optimize_action_sequence has had it.
        #
        # Measured 2026-08-07 over five hardware runs: current_state.position
        # occasionally reads back, AFTER the solve, as a small vector bounded by
        # max_acceleration (0.5 rad, leeloo_curobo.yaml) — a 125-200 deg step in
        # ~120 ms, with the following step resuming the true trajectory exactly
        # where the previous one left it. It produced phantom 1.3-1.9 m / 96-163
        # deg FK errors in the CSV while the commanded per-joint velocities ran
        # straight through them without a discontinuity, which is not how MPPI
        # would react to a genuinely 125-deg-wrong initial state. A plausibility
        # gate on the joint_states feedback (JointSpeedStrategy.
        # _feedback_is_plausible) did not remove them, and that topic has a
        # single publisher with the six expected names — so the reading, not the
        # feedback, is the prime suspect. The mismatch check after the solve
        # settles it: if it fires, cuRobo is writing into the argument.
        state_in = current_state.clone()

        t_solve = time.monotonic()
        result = self.solver.optimize_action_sequence(current_state)
        solve_ms = (time.monotonic() - t_solve) * 1000.0

        try:
            if not torch.equal(state_in.position, current_state.position):
                self.node.get_logger().warn(
                    "MPC: the solver mutated the state it was given "
                    f"(in={[round(math.degrees(v), 2) for v in state_in.position.reshape(-1).tolist()]} "
                    f"out={[round(math.degrees(v), 2) for v in current_state.position.reshape(-1).tolist()]}) "
                    "- diagnostics use the input snapshot",
                    throttle_duration_sec=1.0)
        except Exception:
            pass
        seq = result.action_sequence
        if seq is not None and seq.position.shape[1] > 0:
            action = seq.clone()
            if getattr(self, '_use_mppi_acceleration', False):
                # Sample the plan at the point the arm will actually have
                # REACHED when this window is replaced, not at the plan's last
                # point. The bridge pops one point per interpolation_dt tick and
                # holds a window for _command_interval, so only the first
                # (_command_interval / step_dt) points are ever executed — at the
                # production pacing that is 8 of 16, and the unexecuted half is
                # pure overshoot. Feeding back seq.velocity[:, -1, :] therefore
                # reinjected a velocity the arm never reached, compounding cycle
                # over cycle: that, not ACCELERATION mode itself, is the
                # 6->80 deg/s runaway of debug 2026-07-15. Measured 2026-08-07:
                # plan point 15 = 17.7 deg/s vs the arm's real peak 11.3, while
                # plan point 7 = 11.0 — the executed point matches reality.
                npts = seq.velocity.shape[1]
                if self._command_interval > 0.0 and self._step_dt > 0.0:
                    k = int(round(self._command_interval / self._step_dt)) - 1
                    k = max(0, min(k, npts - 1))
                else:
                    # Unpaced: the window is replaced as soon as the next solve
                    # returns, so there is no fixed executed prefix to sample.
                    k = npts - 1
                self._executed_idx = k  # _csv_write measures continuity at this point
                a = self._vel_feedback_alpha
                cap = math.radians(_VBC_CAP_DPS)
                self._v_bc = ((1.0 - a) * self._v_bc + a * seq.velocity[:, k, :]).clamp(-cap, cap)
        else:

            action = current_state.clone()
            action.velocity = torch.zeros_like(action.position)
            action.acceleration = torch.zeros_like(action.position)

        # state_in, not current_state: a post-solve corruption would otherwise
        # feed a phantom metre-scale error into _update_hold() and silently
        # reset a hold counter that had legitimately been accumulating.
        self._last_position_error = self._fk_position_error(state_in)
        self._last_orientation_error = self._fk_orientation_error(state_in)
        self._update_hold()
        # Measured joint POSITIONS, for the CSV. Without them a run that ends
        # against a joint limit is undiagnosable offline: con_cspace_bound is a
        # composite of all five cspace terms and, per the constraint_cfg note
        # above, is dominated by the jerk violation on every step -- it cannot
        # isolate a position bound. Logged raw so limits can be checked against
        # the URDF afterwards rather than baked in here.
        try:
            q = state_in.position
            self._last_q = (q[0] if q.dim() > 1 else q).detach().cpu().tolist()
        except Exception:
            self._last_q = None
        if seq is not None and seq.position.shape[1] > 0:
            if self._debug_enabled():
                breakdown = self._cost_breakdown(result)
                self._csv_write(result, solve_ms, breakdown)
                self._publish_costs(result, breakdown)
            self._publish_predicted_path(result)
            self._publish_full_predicted_path(result)

        if self._debug_enabled():
            try:
                vel = action.velocity
                is_horizon = vel.dim() == 3
                v_first = float(vel[:, 0, :].abs().max()) if is_horizon else float(vel.abs().max())
                v_last = float(vel[:, -1, :].abs().max()) if is_horizon else v_first
                n_pts = action.position.shape[1] if action.position.dim() == 3 else 1
                self.node.get_logger().debug(
                    f"[MPC DIAG] step: fk_err={self._last_position_error:.4f}m "
                    f"horizon_points={n_pts} |v|first={v_first:.3e} |v|last={v_last:.3e} rad/s",
                    throttle_duration_sec=1.0,
                )
            except Exception as e:
                self.node.get_logger().warn(f"[MPC DIAG] step log failed: {e}", throttle_duration_sec=5.0)

        return action

    def apply_live_goal(self, raw_goal) -> bool:
        goal = self._set_target(raw_goal)
        applied = self._apply_goal(goal, raw_goal)
        self.goal = goal
        self._publish_goal_marker(raw_goal, applied)
        return applied

    def _publish_goal_marker(self, raw, applied: bool = True):
        """Publish a sphere Marker at the current Cartesian goal, for RViz.

        Color reflects whether the solver actually accepted the goal: red when
        applied (tracking normally), orange when IK failed and the arm is only
        pose-tracking (may not move) — see _apply_goal's warn path. Without
        this, the marker showed a goal as "set" even when the controller
        wasn't really tracking it, which is a bad debugging trap. cf. debug
        2026-07-28.
        """
        if getattr(self, '_goal_marker_pub', None) is None:
            return
        marker = Marker()
        marker.header.frame_id = self._path_frame
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "mpc_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(raw[0])
        marker.pose.position.y = float(raw[1])
        marker.pose.position.z = float(raw[2])
        marker.pose.orientation.w = float(raw[3])
        marker.pose.orientation.x = float(raw[4])
        marker.pose.orientation.y = float(raw[5])
        marker.pose.orientation.z = float(raw[6])
        marker.scale.x = marker.scale.y = marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0 if applied else 0.5  # red = tracking, orange = IK failed
        marker.color.b = 0.0
        self._goal_marker_pub.publish(marker)

    def update_world(self, scene) -> None:
        """Reload the shared Scene into the MPC's collision checker.

        MPCSolver has no ``update_world``; the collision scene is owned by its
        ``scene_collision_checker`` (a SceneCollision).
        """
        self.solver.scene_collision_checker.load_collision_model(scene)

    # ---- helpers --------------------------------------------------------------

    def _apply_goal(self, goal: GoalToolPose, raw, current_js=None) -> bool:

        if self.solver.update_goal_tool_poses(goal, run_ik=False):
            return True

        goal_js = self._solve_goal_state(raw, current_js)
        if goal_js is not None:
            self.solver.update_goal_tool_poses(goal, run_ik=False)  # Cartesian goal (disables joint tracking)
            self.solver.update_goal_state(goal_js)                  # joint goal
            self.solver.enable_joint_position_tracking()            # re-enable joint tracking
            self.node.get_logger().info("MPC: goal_state set via multi-seed IK (joint tracking on)")
            return True

        self.solver.update_goal_tool_poses(goal, run_ik=False)
        self.node.get_logger().warn("MPC: IK failed for goal pose - pose-only tracking (arm may not move)")
        return False



    def _ensure_ik_solver(self):
        """Lazily build a multi-seed IK solver to convert the Cartesian goal
        into a joint-space ANCHOR (cspace_target) for MPPI. Without it, MPC
        tracks pose only and, being deterministic (fixed_samples), gets stuck in
        a local minimum ~9cm from the goal (cf. debug 2026-07-15).

        NO scene / live ESDF here (self-collision only): the anchor just needs to
        MATCH the pose; MPPI's own live collision world handles obstacle avoidance
        during motion. Including the live ESDF caused IK to fail on transient
        perception phantoms → no anchor → the plateau we were chasing."""
        if getattr(self, '_ik_solver', None) is None:
            cw = self.config_wrapper
            cfg = InverseKinematicsCfg.create(
                robot=cw.robot_config_file,
                scene_model=None,
                num_seeds=20,
                position_tolerance=0.005,
                orientation_tolerance=0.05,
                self_collision_check=True,
                use_cuda_graph=False,
            )
            self._ik_solver = InverseKinematics(cfg)
            self.node.get_logger().info("MPC: IK solver for goal-state built (self-collision only)")
        return self._ik_solver

    def _solve_goal_state(self, raw, current_js=None):
        """Multi-seed IK for the target pose -> a feasible JointState, or None.

        Seeded from the robot's CURRENT pose (current_js) rather than random seeds:
        otherwise IK returns an arbitrary joint branch, far from the current config
        (measured: up to ~268°), and MPPI then attempts a huge joint jump (wild motions)
        — cf. debug 2026-07-15."""
        try:
            ik = self._ensure_ik_solver()
            pose = Pose(
                position=torch.tensor([raw[0:3]], dtype=self._dtype, device=self._device),
                quaternion=torch.tensor([raw[3:7]], dtype=self._dtype, device=self._device),
            )
            goal = GoalToolPose.from_poses({ik.kinematics.tool_frames[0]: pose})
            seed_state = None
            if current_js is not None:
                seed_state = JointState.from_position(
                    current_js.position[:, :self.solver.action_dim].clone(),
                    joint_names=self.solver.joint_names,
                )
            result = ik.solve_pose(goal_tool_poses=goal, current_state=seed_state)
            torch.cuda.synchronize()
            if not bool(result.success.reshape(-1)[0].item()):
                self.node.get_logger().warn("MPC: IK goal-state: no successful solution")
                return None
            sol = result.solution.reshape(-1, self.solver.action_dim)[0:1].clone()  # best seed [1, dof]
            # Log the found solution + joint offset from current pose.
            sol_deg = [round(math.degrees(v), 1) for v in sol[0].cpu().tolist()]
            if current_js is not None:
                cur = current_js.position[0, :self.solver.action_dim].cpu().tolist()
                dmax = max(abs(math.degrees(s - c)) for s, c in zip(sol[0].cpu().tolist(), cur))
                seed_info = f"seed=current(deg={[round(math.degrees(v),1) for v in cur]}) offset_max={dmax:.0f}deg"
            else:
                seed_info = "seed=random"
            self.node.get_logger().info(f"MPC: IK goal-state(deg)={sol_deg}  {seed_info}")
            return JointState.from_position(sol, joint_names=self.solver.joint_names)
        except Exception as e:
            self.node.get_logger().error(f"MPC goal IK failed: {e}")
            return None

    def _debug_enabled(self) -> bool:
        # Off by default: gates both the per-step diagnostic CSV (see
        # diagnostics.open_diag_csv) and the [MPC DIAG] logging — neither is
        # meant to run in production. Enable explicitly for tuning/debugging.
        if not self.node.has_parameter('mpc_debug'):
            self.node.declare_parameter('mpc_debug', False)
        return bool(self.node.get_parameter('mpc_debug').value)

    def cancel(self):
        self._csv_close()
        super().cancel()


# Backwards-compatible alias (old name still used by some imports / docs).
MPCPlanner = MPCController
