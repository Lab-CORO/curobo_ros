#!/usr/bin/env python3
"""
LBFGS reactive controller — cuRobo v2's own ``optimize_next_action()`` API,
followed as documented rather than reproduced by hand.
"""

import time
from typing import Any

import torch
from curobo.types import JointState, Pose, GoalToolPose
from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg

from .reactive_controller import ReactiveController
from .mpc_planner import _build_lbfgs_optimizer_config, _build_metrics_rollout_cfg
from .mpc_diagnostics import MPCDiagnostics
from curobo_ros.core.config_wrapper import resolve_interpolation_dt, resolve_use_cuda_graph


class LBFGSController(ReactiveController):
    """Closed-loop LBFGS+B-spline MPC built directly on cuRobo's ``optimize_next_action()``."""

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
        optimization_dt = 4.0 * interpolation_dt

        config_path = node.get_parameter('lbfgs_config_file').get_parameter_value().string_value
        lbfgs_cfg = _build_lbfgs_optimizer_config(config_path)

        warm_iters = lbfgs_cfg.pop('warm_start_iters')
        cold_iters = lbfgs_cfg.pop('cold_start_iters')
        horizon = lbfgs_cfg.pop('horizon')


        base_kwargs = dict(
            robot=cw.robot_config_file,
            scene_model=cw.obstacle_manager.primitives_only_scene(),
            optimization_dt=optimization_dt,
            use_cuda_graph=resolve_use_cuda_graph(node),
            self_collision_check=True,
            collision_cache=cw.collision_cache,
            store_debug=False,
            warm_start_optimization_num_iters=warm_iters,
            cold_start_optimization_num_iters=cold_iters,
        )
        cfg = ModelPredictiveControlCfg.create(
            optimizer_configs=[lbfgs_cfg],
            num_control_points=horizon,
            metrics_rollout=_build_metrics_rollout_cfg(lbfgs_cfg["rollout"]["cost_cfg"]),
            **base_kwargs,
        )
        solver = ModelPredictiveControl(cfg)


        warmup_state = solver.default_joint_state.clone().unsqueeze(0)
        warmup_state.velocity = torch.zeros_like(warmup_state.position)
        warmup_state.acceleration = torch.zeros_like(warmup_state.position)
        t_warm = time.monotonic()
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
        node.get_logger().info(
            f"LBFGS: solver + IK warmed up in {(time.monotonic() - t_warm) * 1000:.0f}ms"
        )

        node.lbfgs = solver
        self._diag = MPCDiagnostics(
            node, solver, cw.base_link, interpolation_dt,
            self._fk_position_error, self._fk_orientation_error,
            csv_prefix="lbfgs_diag",
        )
        node.get_logger().info(
            f"LBFGS solver built: interpolation_dt={interpolation_dt}s, "
            f"optimization_dt={optimization_dt}s, horizon={horizon}, "
            f"warm_start_iters={warm_iters}, cold_start_iters={cold_iters}, "
            f"robot={cw.robot_config_file}, collision_cache={cw.collision_cache}"
        )

        self._command_interval = interpolation_dt

        self._batch_size = solver.trajectory_execution_manager.interpolation_steps
        return solver

    def setup(self, start_state: JointState, goal_request: Any) -> bool:
        self._diag.csv_init(self._debug_enabled())
        p = goal_request.target_pose
        raw = [
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z,
        ]
        node = self.node
        node.get_logger().info(
            f"LBFGS: new goal received - position=({raw[0]:.4f}, {raw[1]:.4f}, {raw[2]:.4f})m "
            f"orientation(wxyz)=({raw[3]:.4f}, {raw[4]:.4f}, {raw[5]:.4f}, {raw[6]:.4f})"
        )
        goal = self._set_target(raw)
        self.solver.setup(start_state)  # returns None -- MPCSolver.setup() has no return value
        applied = self.solver.update_goal_tool_poses(goal, run_ik=False)
        self.goal = goal
        self._diag.publish_goal_marker(raw, applied)

        node.get_logger().info(f"LBFGS: goal IK anchor applied={applied}")

        if self._debug_enabled():
            self._diag.log_setup_summary(start_state, raw, applied)
        return True

    # ---- Control step ----

    def step(self, current_state: JointState) -> JointState:

        is_resolving = not self.solver.trajectory_execution_manager.has_valid_next_command()

        t_solve = time.monotonic()
        result = self.solver.optimize_next_action(current_state)
        solve_ms = (time.monotonic() - t_solve) * 1000.0


        action = result.next_action
        if getattr(action, 'velocity', None) is None:

            self.node.get_logger().warn(
                "LBFGS: next_action.velocity is None (send path will zero-fill)",
                throttle_duration_sec=2.0,
            )

        if is_resolving:
            self._last_position_error = self._fk_position_error(action)
            self._last_orientation_error = self._fk_orientation_error(action)
            self._update_hold()

            self._diag.publish_predicted_path(result)
            self._diag.publish_full_predicted_path(result)

            if self._debug_enabled():
                breakdown = self._diag.cost_breakdown(result)
                self._diag.publish_costs(
                    result, breakdown, self._last_position_error, self._last_orientation_error)
                with self._pending_cv:
                    queue_depth = len(self._pending_queue)
                self._diag.record_tick(
                    result, solve_ms=solve_ms,
                    last_position_error=self._last_position_error,
                    last_orientation_error=self._last_orientation_error,
                    queue_depth=queue_depth,
                    min_queue_depth_seen=(
                        self._min_queue_depth_seen
                        if self._min_queue_depth_seen is not None else -1
                    ),
                    starvation_ticks=self._starvation_ticks,
                    backpressure_wait_ms=self._diag_backpressure_wait_ms,
                    perception_ms=self._diag_perception_ms,
                    live_goal_ms=self._diag_live_goal_ms,
                    cheap_ms_before_resolve=self._diag_cheap_ms_before_resolve,
                    batch_wall_ms=self._diag_batch_wall_ms,
                    loop_iter=self._diag_loop_iter,
                    vel_input_err_max=self._diag_vel_input_err_max,
                    accel_input_err_max=self._diag_accel_input_err_max,
                    pred_vel=self._diag_pred_vel,
                    real_vel=self._diag_real_vel,
                    pred_acc=self._diag_pred_acc,
                    real_acc=self._diag_real_acc,
                    vel_input_err_max_lagged=self._diag_vel_input_err_max_lagged,
                    accel_input_err_max_lagged=self._diag_accel_input_err_max_lagged,
                    lag_steps=self._diag_lag_steps,
                    extrap_vel=self._diag_extrap_vel,
                    extrap_acc=self._diag_extrap_acc,
                    vel_extrap_err_max=self._diag_vel_extrap_err_max,
                    accel_extrap_err_max=self._diag_accel_extrap_err_max,
                    extrap_tau=self._diag_extrap_tau,
                    input_vel=self._diag_input_vel,
                    input_acc=self._diag_input_acc,
                    output_vel=self._diag_output_vel,
                    output_acc=self._diag_output_acc,
                    output_t=self._diag_output_t,
                    windup_active=self._diag_windup_active,
                    windup_clamp_ratio=self._diag_windup_clamp_ratio,
                )

        return action

    def step_batch(self, current_state: JointState, n: int) -> list:

        first_action = self.step(current_state)
        tem = self.solver.trajectory_execution_manager
        try:
            seq = tem.get_command_sequence()  # [.., n, dof] horizon window
            tem._current_command_idx = tem.interpolation_steps  # mark fully drained
        except AttributeError:
            self.node.get_logger().error(
                "LBFGS: get_command_sequence()/_current_command_idx missing on "
                "this cuRobo version -- falling back to per-call step()",
                throttle_duration_sec=5.0,
            )
            return [first_action] + [self.step(current_state) for _ in range(n - 1)]

        batch = [first_action]
        for i in range(1, n):
            batch.append(JointState(
                position=seq.position[..., i, :],
                velocity=seq.velocity[..., i, :] if seq.velocity is not None else None,
                acceleration=seq.acceleration[..., i, :] if seq.acceleration is not None else None,
                joint_names=seq.joint_names,
            ))
        return batch

    def apply_live_goal(self, raw_goal) -> bool:
        self.node.get_logger().info(
            f"LBFGS: live goal received - position=({raw_goal[0]:.4f}, {raw_goal[1]:.4f}, "
            f"{raw_goal[2]:.4f})m orientation(wxyz)=({raw_goal[3]:.4f}, {raw_goal[4]:.4f}, "
            f"{raw_goal[5]:.4f}, {raw_goal[6]:.4f})"
        )
        goal = self._set_target(raw_goal)

        applied = self.solver.update_goal_tool_poses(goal, run_ik=False)
        self.goal = goal
        self._diag.publish_goal_marker(raw_goal, applied)
        return applied

    def update_world(self, scene) -> None:
        """Reload the shared Scene into the LBFGS solver's collision checker.
        """
        self.solver.scene_collision_checker.load_collision_model(scene)

    # ---- helpers --------------------------------------------------------------

    def _debug_enabled(self) -> bool:
        if not self.node.has_parameter('lbfgs_debug'):
            self.node.declare_parameter('lbfgs_debug', False)
        return bool(self.node.get_parameter('lbfgs_debug').value)

    def cancel(self):
        if getattr(self, '_diag', None) is not None:
            self._diag.csv_close()
        super().cancel()
