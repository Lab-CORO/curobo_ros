#!/usr/bin/env python3


import csv
import math
import os
import time
from typing import Any

import torch
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from curobo.types import JointState, GoalToolPose, Pose
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.model_predictive_control import (
    ModelPredictiveControl,
    ModelPredictiveControlCfg,
)

from .reactive_controller import ReactiveController


# --- MPPI recipe + ACCELERATION control space ------
# Scope of mpc_m1013_speedj2.py (validated on real m1013 on 2026-07-14). The
# default MPC config (L-BFGS + B-spline) freezes on certain poorly-conditioned
# m1013 postures: L-BFGS line-search with fixed grid [0,0.1,0.5,1.0] finds no
# valid step when the direction is ill-conditioned (6-DOF non-redundant near-singular).
# MPPI (no line-search) is immune to this. L-BFGS + ACCELERATION is impossible here
# (backward of acceleration integration kernel not implemented), so MPPI is the only
# viable optimizer.
_MPPI_CSPACE_REGULARIZATION = [0.3, 1.0, 0.0, 0.0, 0.0]  # [vel, acc, jerk, torque, energy]

# Velocity boundary feedback cap (continuity → smoother motion).
# REQUIRED: in ACCELERATION mode the plan integrates beyond the boundary, so
# reinjecting planned velocity causes ratcheting (runaway 6→80°/s without cap, cf.
# debug 2026-07-15). In sandbox, a LOW cap (~5°/s) converges, a high cap (>=8)
# overshoots and diverges. Intentionally low value = safe (slow motion, can be
# canceled even if deviating) + smoother than zero boundary.
#   _VBC_CAP_DPS = 0.0  -> zero boundary (current robust behavior, jerky)
#   higher            -> more continuity (smooth) BUT risk of overshoot
_VBC_CAP_DPS = 5.0


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
                    "squared_l2_regularization_weight": [0.0] * 5,
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
                    "use_lie_group": False,
                    "weight": [5000.0, 2000.0],
                    "_terminal_pose_convergence_tolerance": [0.0, 0.0],
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
            "step_size_cov": 0.01,
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


class MPCController(ReactiveController):
    """Closed-loop MPC built on cuRobo ``ModelPredictiveControl`` (v2)."""

    def get_planner_name(self) -> str:
        return "Model Predictive Control (MPC)"

    def get_config_parameters(self) -> list:
        return ['convergence_threshold', 'max_mpc_iterations']

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
        base_kwargs = dict(
            robot=cw.robot_config_file,
            scene_model=cw.obstacle_manager.get_scene(),
            optimization_dt=step_dt,
            use_cuda_graph=True,
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
            # NO num_control_points here: it writes n_knots (B-spline concept).
            # The horizon lives in the transition_model dict we provide.
            cfg = ModelPredictiveControlCfg.create(
                optimizer_configs=[_build_mppi_optimizer_config(warm_iters, num_particles)],
                transition_model=_build_mppi_transition_model(step_dt, horizon),
                squared_l2_regularization_weight=_MPPI_CSPACE_REGULARIZATION,
                interpolation_steps=8,
                **base_kwargs,
            )
        else:
            cfg = ModelPredictiveControlCfg.create(
                num_control_points=horizon,
                **base_kwargs,
            )
        solver = ModelPredictiveControl(cfg)

        node.mpc = solver
        # Predicted end-effector path (current MPC horizon), for RViz (nav_msgs/Path
        # renders natively, no custom plugin needed).
        self._path_pub = node.create_publisher(Path, 'mpc_predicted_path', 10)
        self._goal_marker_pub = node.create_publisher(Marker, 'mpc_goal_marker', 10)
        self._path_frame = cw.base_link
        node.get_logger().info(
            f"MPC solver built: solver_type={solver_type}, optimization_dt={step_dt}s, "
            f"horizon={horizon}, warm_start_iters={warm_iters}, cold_start_iters={cold_iters}, "
            f"robot={cw.robot_config_file}, collision_cache={cw.collision_cache}"
        )
        return solver

    def setup(self, start_state: JointState, goal_request: Any) -> bool:
        self._v_bc = None  # New goal = new velocity continuity baseline
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
        self._publish_goal_marker(raw)

        if self._debug_enabled():
            ee_pos, ee_quat = None, None
            try:
                kin = self.solver.compute_kinematics(start_state)
                ee_pos = [round(v, 4) for v in kin.tool_poses.position.reshape(-1, 3)[0].cpu().tolist()]
                ee_quat = [round(v, 4) for v in kin.tool_poses.quaternion.reshape(-1, 4)[0].cpu().tolist()]
            except Exception as e:
                self.node.get_logger().warn(f"[MPC DIAG] EE pose log failed: {e}")
            self.node.get_logger().info(
                f"[MPC DIAG] setup: goal_xyz={[round(v, 4) for v in raw[0:3]]} "
                f"goal_quat(wxyz)={[round(v, 4) for v in raw[3:7]]} "
                f"tool_frame={self.solver.tool_frames[0]} "
                f"start_EE_pos={ee_pos} start_EE_quat(wxyz)={ee_quat} "
                f"fk_err={self._fk_position_error(start_state):.4f}m goal_applied={applied}"
            )
        return True

    # ---- Debug CSV ----

    def _csv_init(self):
        """Opens a diagnostic CSV (one line per step). Gated by mpc_debug."""
        self._csv = None
        if not self._debug_enabled():
            return
        try:
            d = "/home/ros2_ws/src/curobo_ros/config"
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, f"mpc_diag_{time.strftime('%Y%m%d_%H%M%S')}.csv")
            self._csv_file = open(path, "w", newline="")
            self._csv = csv.writer(self._csv_file)
            self._csv_header_written = False
            self._csv_t0 = time.monotonic()
            self._csv_t_prev = self._csv_t0
            self._csv_last_vlast = None
            self.node.get_logger().info(f"[MPC DIAG] CSV -> {path}")
        except Exception as e:
            self._csv = None
            self.node.get_logger().warn(f"[MPC DIAG] CSV init failed: {e}")

    def _csv_write(self, seq, solve_ms):
        if getattr(self, '_csv', None) is None:
            return
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
        # Batch boundary accel: |vfirst - vlast_prev|/dt (stop-start discontinuity)
        if self._csv_last_vlast is not None:
            accel_bd = deg(max(abs(a - b) for a, b in zip(vfirst, self._csv_last_vlast)) / dt)
        else:
            accel_bd = 0.0
        self._csv_last_vlast = vlast
        vbc = self._v_bc[0].cpu().tolist() if getattr(self, '_v_bc', None) is not None else [0.0] * dof
        if not self._csv_header_written:
            self._csv.writerow(
                ["t_s", "dt_step_ms", "solve_ms", "fk_err_m", "vfirst_max_dps", "vlast_max_dps",
                 "accel_win_max_dps2", "accel_boundary_dps2", "vbc_max_dps"]
                + [f"vfirst_j{i+1}_dps" for i in range(dof)]
                + [f"vlast_j{i+1}_dps" for i in range(dof)])
            self._csv_header_written = True
        self._csv.writerow(
            [f"{now - self._csv_t0:.3f}", f"{dt_step_ms:.1f}", f"{solve_ms:.1f}",
             f"{self._last_position_error:.5f}", f"{deg(max(abs(v) for v in vfirst)):.2f}",
             f"{deg(max(abs(v) for v in vlast)):.2f}", f"{accel_win:.1f}", f"{accel_bd:.1f}",
             f"{deg(max(abs(v) for v in vbc)):.2f}"]
            + [f"{deg(v):.2f}" for v in vfirst] + [f"{deg(v):.2f}" for v in vlast])
        self._csv_file.flush()

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
        self._path_pub.publish(path)

    def step(self, current_state: JointState) -> JointState:

        if getattr(self, '_use_mppi_acceleration', False):
            # Velocity continuity (fewer jerks): reinjecting planned velocity from last point,
            # filtered with EMA and CAPPED. The cap is required — without it, in ACCELERATION
            # mode the plan integrates beyond the boundary → ratchet/runaway (cf. _VBC_CAP_DPS, debug 2026-07-15).
            if self._v_bc is None:
                self._v_bc = torch.zeros_like(current_state.position)
            current_state = current_state.clone()
            current_state.velocity = self._v_bc

        t_solve = time.monotonic()
        result = self.solver.optimize_action_sequence(current_state)
        solve_ms = (time.monotonic() - t_solve) * 1000.0
        seq = result.action_sequence
        if seq is not None and seq.position.shape[1] > 0:
            action = seq.clone()
            if getattr(self, '_use_mppi_acceleration', False):
                a = self._vel_feedback_alpha
                cap = math.radians(_VBC_CAP_DPS)
                self._v_bc = ((1.0 - a) * self._v_bc + a * seq.velocity[:, -1, :]).clamp(-cap, cap)
        else:

            action = current_state.clone()
            action.velocity = torch.zeros_like(action.position)
            action.acceleration = torch.zeros_like(action.position)

        self._last_position_error = self._fk_position_error(current_state)
        if seq is not None and seq.position.shape[1] > 0:
            self._csv_write(seq, solve_ms)
            self._publish_predicted_path(result)

        if self._debug_enabled():
            try:
                vel = action.velocity
                is_horizon = vel.dim() == 3
                v_first = float(vel[:, 0, :].abs().max()) if is_horizon else float(vel.abs().max())
                v_last = float(vel[:, -1, :].abs().max()) if is_horizon else v_first
                n_pts = action.position.shape[1] if action.position.dim() == 3 else 1
                self.node.get_logger().info(
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
        self._publish_goal_marker(raw_goal)
        return applied

    def _publish_goal_marker(self, raw):
        """Publish a sphere Marker at the current Cartesian goal, for RViz."""
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
        marker.color.g = 0.0
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

        if self.solver.update_goal_tool_poses(goal, run_ik=True):
            return True

        goal_js = self._solve_goal_state(raw, current_js)
        if goal_js is not None:
            self.solver.update_goal_tool_poses(goal, run_ik=False)  # Cartesian goal (disables joint tracking)
            self.solver.update_goal_state(goal_js)                  # joint goal
            self.solver.enable_joint_position_tracking()            # re-enable joint tracking
            self.node.get_logger().info("MPC: goal_state set via multi-seed IK (joint tracking on)")
            return True

        self.solver.update_goal_tool_poses(goal, run_ik=False)
        self.node.get_logger().warn("MPC: IK failed for goal pose — pose-only tracking (arm may not move)")
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
        if not self.node.has_parameter('mpc_debug'):
            self.node.declare_parameter('mpc_debug', True)
        return bool(self.node.get_parameter('mpc_debug').value)


# Backwards-compatible alias (old name still used by some imports / docs).
MPCPlanner = MPCController
