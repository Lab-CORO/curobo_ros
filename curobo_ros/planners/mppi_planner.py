#!/usr/bin/env python3
"""
MPPI reactive controller — cuRobo v2's ``ModelPredictiveControl`` driven via
``optimize_action_sequence()``, which re-solves on EVERY call (unlike
LBFGSController's ``optimize_next_action()``, which amortizes the solve over
``interpolation_steps`` calls — see lbfgs_planner.py's module docstring and
solver_mpc.py). MPPI's particle-sampling solve is cheap enough per call to
afford resolving every tick; LBFGS's iterative gradient solve is not.

Because every call is a fresh solve from a full window (``command_start_idx``
.. ``command_end_idx``) rather than a single popped point, the state fed back
each tick needs its own velocity-continuity bookkeeping (``_v_bc``,
``_executed_idx`` below) instead of relying on cuRobo's internal
TrajectoryExecutionManager pointer the way LBFGS does.
"""

import math
import time
from typing import Any

import torch

from curobo.types import JointState, GoalToolPose, Pose
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg

from .reactive_controller import ReactiveController
from .mpc_common import _load_mpc_config, _build_metrics_rollout_cfg, _extract_cspace_reg_weights
from .mpc_diagnostics import MPCDiagnostics
from curobo_ros.core.config_wrapper import resolve_use_cuda_graph


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


def _build_mppi_optimizer_config(config_path: str, num_iters: int, num_particles: int = 800) -> dict:
    """Load the MPPI cost/optimizer config from YAML (see config/mpc/mppi_mpc.yaml
    and the mpc_mppi_config_file ROS param) and apply the two fields that stay
    under separate, already-existing ROS params rather than living in the file:
    optimizer.num_iters (mpc_warm_start_iters) and optimizer.num_particles
    (mpc_mppi_num_particles)."""
    cfg = _load_mpc_config(config_path)
    cfg["optimizer"]["num_iters"] = num_iters
    cfg["optimizer"]["num_particles"] = num_particles
    return cfg


class MPPIController(ReactiveController):
    """Closed-loop MPPI built on cuRobo ``ModelPredictiveControl`` (v2)."""

    def get_planner_name(self) -> str:
        return "MPPI Model Predictive Control"

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

        self._vel_feedback_alpha = node.get_parameter('mpc_vel_feedback_alpha').get_parameter_value().double_value
        self._step_dt = step_dt  # For debug CSV (point-to-point interval in the loop)
        # Fixed-interval command pacing (s); 0.0 = off. Read by the servo loop
        # (ReactiveController.execute) to hold each command window for its full
        # execution duration before re-solving. See mpc_command_interval param.
        self._command_interval = node.get_parameter('mpc_command_interval').get_parameter_value().double_value

        num_particles = node.get_parameter('mpc_mppi_num_particles').get_parameter_value().integer_value
        mppi_config_path = node.get_parameter('mpc_mppi_config_file').get_parameter_value().string_value
        mppi_optimizer_cfg = _build_mppi_optimizer_config(mppi_config_path, warm_iters, num_particles)
        # cspace_regularization_weight lives at the top level of the YAML
        # (see its comment there) but is a create()-level kwarg, not part of
        # an optimizer_configs entry -- pop it off before handing the rest
        # of the dict (rollout/optimizer only) to optimizer_configs.
        cspace_regularization_weight = mppi_optimizer_cfg.pop("cspace_regularization_weight")
        # NOT the same field as cspace_regularization_weight just popped above
        # (that one is the transition-model-level term, a create()-level
        # kwarg) -- this is the PER-ROLLOUT cost_cfg.cspace_cfg field, the one
        # actually mirrored into the metrics rollout by _build_metrics_rollout_cfg
        # below, so it's what get_current_metrics()/cost_breakdown() sees.
        cspace_reg_weights = _extract_cspace_reg_weights(mppi_optimizer_cfg["rollout"]["cost_cfg"])
        # NO num_control_points here: it writes n_knots (B-spline concept).
        # The horizon lives in the transition_model dict we provide.
        cfg = ModelPredictiveControlCfg.create(
            optimizer_configs=[mppi_optimizer_cfg],
            transition_model=_build_mppi_transition_model(step_dt, horizon),
            squared_l2_regularization_weight=cspace_regularization_weight,
            metrics_rollout=_build_metrics_rollout_cfg(mppi_optimizer_cfg["rollout"]["cost_cfg"]),
            **base_kwargs,
        )
        solver = ModelPredictiveControl(cfg)

        node.mpc = solver
        # Diagnostics/RViz publishing lives alongside the solver it observes
        # (mpc_diagnostics.py) — a rebuild gets a fresh MPCDiagnostics too.
        self._diag = MPCDiagnostics(
            node, solver, cw.base_link, step_dt,
            self._fk_position_error, self._fk_orientation_error,
            cspace_reg_weights=cspace_reg_weights,
        )
        node.get_logger().info(
            f"MPPI solver built: optimization_dt={step_dt}s, horizon={horizon}, "
            f"warm_start_iters={warm_iters}, cold_start_iters={cold_iters}, "
            f"num_particles={num_particles}, robot={cw.robot_config_file}, "
            f"collision_cache={cw.collision_cache}"
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
        self._diag.csv_init(self._debug_enabled())
        p = goal_request.target_pose
        raw = [
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z,
        ]
        self.node.get_logger().info(
            f"MPPI: new goal received - position=({raw[0]:.4f}, {raw[1]:.4f}, {raw[2]:.4f})m "
            f"orientation(wxyz)=({raw[3]:.4f}, {raw[4]:.4f}, {raw[5]:.4f}, {raw[6]:.4f})"
        )
        goal = self._set_target(raw)
        self.solver.setup(start_state)
        applied = self._apply_goal(goal, raw, start_state)
        self.goal = goal
        self._diag.publish_goal_marker(raw, applied)

        if self._debug_enabled():
            self._diag.log_setup_summary(start_state, raw, applied)
        return True

    # ---- Control step ----

    def step(self, current_state: JointState) -> JointState:

        # Velocity continuity: the next solve starts from the velocity this
        # window will actually have reached (see the sampling index below), so
        # the MPC's assumed state matches the arm's real one. solver_mpc's
        # optimize_action_sequence -> _solve_impl -> update_current_state feeds
        # current_state.velocity into the shared goal_registry_manager/rollout
        # params. Without it, MPPI was warm-started every cycle from
        # _state_from_action's seq.position[:, -1, :] (reactive_controller.py)
        # -- the plan's LAST horizon point -- while paced sending only ever
        # executes the first k = round(command_interval / step_dt) - 1 points,
        # so the solver's assumed starting velocity never matched what the arm
        # actually reached. That produced large plan-to-plan boundary jumps
        # (accel_boundary_dps2) even though each individual plan window stayed
        # smooth (accel_win_max_dps2 clean), and vbc_max_dps read 0.0 on every
        # row because _v_bc was never assigned.
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
                    "MPPI: the solver mutated the state it was given "
                    f"(in={[round(math.degrees(v), 2) for v in state_in.position.reshape(-1).tolist()]} "
                    f"out={[round(math.degrees(v), 2) for v in current_state.position.reshape(-1).tolist()]}) "
                    "- diagnostics use the input snapshot",
                    throttle_duration_sec=1.0)
        except Exception:
            pass
        seq = result.action_sequence
        if seq is not None and seq.position.shape[1] > 0:
            action = seq.clone()
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
        # xyz first, scalar derived from it -- one FK call instead of two
        # (_fk_position_error would otherwise redo the same FK pass).
        self._last_position_error_xyz = self._fk_position_error_xyz(state_in)
        self._last_position_error = (
            float(torch.linalg.norm(self._last_position_error_xyz).item())
            if self._last_position_error_xyz is not None else float('inf')
        )
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
                breakdown = self._diag.cost_breakdown(result)
                horizon_diag = self._diag.horizon_diag(result, seq, self._executed_idx)
                self._diag.csv_write(
                    result, solve_ms, breakdown, horizon_diag,
                    command_interval=self._command_interval,
                    last_position_error=self._last_position_error,
                    last_orientation_error=self._last_orientation_error,
                    v_bc=self._v_bc, executed_idx=self._executed_idx, last_q=self._last_q,
                )
                self._diag.publish_costs(
                    result, breakdown, self._last_position_error, self._last_orientation_error)
            self._diag.publish_predicted_path(result)
            self._diag.publish_full_predicted_path(result)
            # Published every cycle, not gated by mpc_debug like the CSV/
            # mpc_costs above: budget is mpc_command_interval (0.0 = unpaced,
            # i.e. no fixed budget -- see publish_step_diagnostics).
            self._diag.publish_step_diagnostics(
                solve_ms=solve_ms, budget_ms=self._command_interval * 1000.0,
                result=result, position=action.position, velocity=action.velocity,
                acceleration=action.acceleration, joint_names=action.joint_names,
                dt=self._step_dt,
            )

        if self._debug_enabled():
            self._diag.log_step_summary(action, self._last_position_error)

        return action

    def apply_live_goal(self, raw_goal) -> bool:
        self.node.get_logger().info(
            f"MPPI: live goal received - position=({raw_goal[0]:.4f}, {raw_goal[1]:.4f}, "
            f"{raw_goal[2]:.4f})m orientation(wxyz)=({raw_goal[3]:.4f}, {raw_goal[4]:.4f}, "
            f"{raw_goal[5]:.4f}, {raw_goal[6]:.4f})"
        )
        goal = self._set_target(raw_goal)
        applied = self._apply_goal(goal, raw_goal)
        self.goal = goal
        self._diag.publish_goal_marker(raw_goal, applied)
        return applied

    def update_world(self, scene) -> None:
        """Reload the shared Scene into the MPPI's collision checker.

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
            self.node.get_logger().info("MPPI: goal_state set via multi-seed IK (joint tracking on)")
            return True

        self.solver.update_goal_tool_poses(goal, run_ik=False)
        self.node.get_logger().warn("MPPI: IK failed for goal pose - pose-only tracking (arm may not move)")
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
            self.node.get_logger().info("MPPI: IK solver for goal-state built (self-collision only)")
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
                self.node.get_logger().warn("MPPI: IK goal-state: no successful solution")
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
            self.node.get_logger().info(f"MPPI: IK goal-state(deg)={sol_deg}  {seed_info}")
            return JointState.from_position(sol, joint_names=self.solver.joint_names)
        except Exception as e:
            self.node.get_logger().error(f"MPPI goal IK failed: {e}")
            return None

    def _debug_enabled(self) -> bool:
        # Off by default: gates both the per-step diagnostic CSV (see
        # diagnostics.open_diag_csv) and the [MPC DIAG] logging — neither is
        # meant to run in production. Enable explicitly for tuning/debugging.
        if not self.node.has_parameter('mpc_debug'):
            self.node.declare_parameter('mpc_debug', False)
        return bool(self.node.get_parameter('mpc_debug').value)

    def cancel(self):
        if getattr(self, '_diag', None) is not None:
            self._diag.csv_close()
        super().cancel()
