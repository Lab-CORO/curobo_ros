#!/usr/bin/env python3
"""
LBFGS reactive controller — cuRobo v2's own ``optimize_next_action()`` API,
followed as documented rather than reproduced by hand.

``lbfgs_solve_mode`` (ROS param, read once in ``build_solver()``) switches the
solver call between:

- ``'next_action'`` (default): ``optimize_next_action()``, amortized over
  ``interpolation_steps`` calls -- see the module docstring's discussion
  above and mpc_diagnostics.py's record_tick docstring.
- ``'action_sequence'``: ``optimize_action_sequence()``, which re-solves on
  EVERY call and returns the whole near-term window instead of one popped
  point -- the same API MPPIController uses (mppi_planner.py). Provided
  for A/B comparison against the 'next_action' path; deliberately does NOT
  reproduce MPPIController's ``_v_bc``/``executed_idx`` velocity-continuity
  feedback (current_state is passed through as-is from
  ReactiveController._close_state_loop), so it may be less stable under
  sustained motion -- see mppi_planner.py's module docstring for why that
  feedback exists there. Not meant to switch mid-goal: the two modes leave
  cuRobo's TrajectoryExecutionManager in incompatible internal states
  (a dense pre-sampled pointer buffer vs. nothing to resume from), so
  picking a new mode requires a fresh build_solver() (rebuild_solver() /
  new planner selection), not just a live ros2 param set.
"""

import time
from typing import Any

import torch
from curobo.types import JointState, Pose, GoalToolPose
from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg

from .reactive_controller import ReactiveController
from .mpc_common import _load_mpc_config, _build_metrics_rollout_cfg, _extract_cspace_reg_weights
from .mpc_diagnostics import MPCDiagnostics
from curobo_ros.core.config_wrapper import resolve_interpolation_dt, resolve_use_cuda_graph


def _build_lbfgs_optimizer_config(config_path: str) -> dict:
    """Load the LBFGS cost/optimizer config from YAML (see config/mpc/lbfgs_mpc.yaml
    and the lbfgs_config_file ROS param). Nothing is overridden after loading --
    num_iters/inner_iters and every cost/constraint weight come straight from
    the file."""
    return _load_mpc_config(config_path)


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

        if not node.has_parameter('lbfgs_solve_mode'):
            node.declare_parameter('lbfgs_solve_mode', 'action_sequence')
        solve_mode = node.get_parameter('lbfgs_solve_mode').get_parameter_value().string_value
        if solve_mode not in ('next_action', 'action_sequence'):
            node.get_logger().warn(
                f"LBFGS: unknown lbfgs_solve_mode '{solve_mode}', defaulting to 'next_action'"
            )
            solve_mode = 'next_action'
        self._solve_mode = solve_mode

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
        cspace_reg_weights = _extract_cspace_reg_weights(lbfgs_cfg["rollout"]["cost_cfg"])
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
            cspace_reg_weights=cspace_reg_weights,
        )
        node.get_logger().info(
            f"LBFGS solver built: solve_mode={self._solve_mode}, "
            f"interpolation_dt={interpolation_dt}s, "
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
        if self._solve_mode == 'action_sequence':
            action, _result, _seq, _solve_ms = self._solve_action_sequence(current_state)
            return action
        return self._step_next_action(current_state)

    def _step_next_action(self, current_state: JointState) -> JointState:

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
                )

        return action

    def step_batch(self, current_state: JointState, n: int) -> list:
        if self._solve_mode == 'action_sequence':
            return self._step_batch_action_sequence(current_state, n)

        first_action = self._step_next_action(current_state)
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
            return [first_action] + [self._step_next_action(current_state) for _ in range(n - 1)]

        batch = [first_action]
        for i in range(1, n):
            batch.append(JointState(
                position=seq.position[..., i, :],
                velocity=seq.velocity[..., i, :] if seq.velocity is not None else None,
                acceleration=seq.acceleration[..., i, :] if seq.acceleration is not None else None,
                joint_names=seq.joint_names,
            ))
        return batch

    # ---- 'action_sequence' mode ------------------------------------------------

    def _solve_action_sequence(self, current_state: JointState):
        """One ``optimize_action_sequence()`` resolve (always re-solves, unlike
        ``optimize_next_action()``). Returns ``(first_point_action, result, seq,
        solve_ms)`` so both ``step()`` (which only needs the first point) and
        ``_step_batch_action_sequence()`` (which slices more points out of the
        same window) share exactly one resolve per producer-loop iteration.

        Deliberately NOT MPPIController's ``_v_bc``/``executed_idx`` velocity-
        continuity feedback -- see the module docstring. ``current_state`` is
        used as handed in by ReactiveController._close_state_loop.
        """
        state_in = current_state.clone()

        t_solve = time.monotonic()
        result = self.solver.optimize_action_sequence(current_state)
        solve_ms = (time.monotonic() - t_solve) * 1000.0

        try:
            if not torch.equal(state_in.position, current_state.position):
                self.node.get_logger().warn(
                    "LBFGS (action_sequence): the solver mutated the state it was given "
                    "- diagnostics use the input snapshot",
                    throttle_duration_sec=1.0)
        except Exception:
            pass

        seq = result.action_sequence
        if seq is None or seq.position.shape[1] == 0:
            action = current_state.clone()
            action.velocity = torch.zeros_like(action.position)
            action.acceleration = torch.zeros_like(action.position)
            self._last_position_error = self._fk_position_error(state_in)
            self._last_orientation_error = self._fk_orientation_error(state_in)
            self._update_hold()
            return action, result, seq, solve_ms

        first_action = JointState(
            position=seq.position[:, -1, :],
            velocity=seq.velocity[:, -1, :] if seq.velocity is not None else None,
            acceleration=seq.acceleration[:, -1, :] if seq.acceleration is not None else None,
            joint_names=seq.joint_names,
        )

        self._last_position_error = self._fk_position_error(state_in)
        self._last_orientation_error = self._fk_orientation_error(state_in)
        self._update_hold()
        try:
            q = state_in.position
            self._last_q = (q[0] if q.dim() > 1 else q).detach().cpu().tolist()
        except Exception:
            self._last_q = None

        # Diagnostic-only approximation: the point that will actually have
        # been sent before the NEXT resolve is the batch_size-th one in a
        # paced producer loop (step_batch), but 0 in an unpaced step()-only
        # loop. step() doesn't know which caller it has, so this always uses
        # the paced assumption -- harmless since it only affects CSV columns,
        # not control.
        npts = seq.position.shape[1]
        executed_idx = min(max(0, self._batch_size - 1), npts - 1)

        if self._debug_enabled():
            breakdown = self._diag.cost_breakdown(result)
            horizon_diag = self._diag.horizon_diag(result, seq, executed_idx)
            self._diag.csv_write(
                result, solve_ms, breakdown, horizon_diag,
                command_interval=self._command_interval,
                last_position_error=self._last_position_error,
                last_orientation_error=self._last_orientation_error,
                v_bc=None, executed_idx=executed_idx, last_q=self._last_q,
            )
            self._diag.publish_costs(
                result, breakdown, self._last_position_error, self._last_orientation_error)
        self._diag.publish_predicted_path(result)
        self._diag.publish_full_predicted_path(result)

        return first_action, result, seq, solve_ms

    def _step_batch_action_sequence(self, current_state: JointState, n: int) -> list:
        first_action, _result, seq, _solve_ms = self._solve_action_sequence(current_state)
        if seq is None or seq.position.shape[1] == 0:
            return [first_action] * n

        npts = seq.position.shape[1]
        batch = [first_action]
        for i in range(1, min(n, npts)):
            batch.append(JointState(
                position=seq.position[..., i, :],
                velocity=seq.velocity[..., i, :] if seq.velocity is not None else None,
                acceleration=seq.acceleration[..., i, :] if seq.acceleration is not None else None,
                joint_names=seq.joint_names,
            ))
        # Window shorter than the requested batch (shouldn't happen with the
        # default command_end_idx=interpolation_steps*2 >= batch_size, but
        # guard anyway): pad by holding the last available point.
        while len(batch) < n:
            batch.append(batch[-1])
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
        if not self.node.has_parameter('mpc_debug'):
            self.node.declare_parameter('mpc_debug', False)
        return bool(self.node.get_parameter('mpc_debug').value)

    def cancel(self):
        if getattr(self, '_diag', None) is not None:
            self._diag.csv_close()
        super().cancel()
