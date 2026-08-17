#!/usr/bin/env python3
"""
LBFGS reactive controller — cuRobo v2's own ``optimize_next_action()`` API,
followed as documented rather than reproduced by hand.

Rewritten from scratch (2026-08-17) as a deliberate sibling to
``MPCController`` (``mpc_planner.py``), not a replacement: that controller's
MPPI branch, and its hand-rolled horizon-consumption/velocity-continuity
workaround (``_v_bc``, the ``k = round(command_interval/step_dt) - 1``
sampling index), stay untouched and validated on hardware exactly as they
are. This controller exists because that workaround was built to paper over
a structural mismatch specific to ``optimize_action_sequence()``: cuRobo's
own ``TrajectoryExecutionManager`` assumes "one time step at a time, with
re-optimization after each action" (its own docstring) and provides
``optimize_next_action()`` precisely for that pattern -- shift/buffer/pacing
handled internally, one command per call, a real resolve every
``interpolation_steps`` (4, hard-locked by ``MPCSolverCfg.create()``) calls.
Verified directly in cuRobo source (``solver_mpc.py``,
``trajectory_execution_manager.py``) and confirmed empirically
(``/home/coro/.claude/plans/streamed-inventing-eagle.md``, Vérification étape
0, 2026-08-17): a resolve buffer holds exactly 4 points for LBFGS
(BSPLINE_3), spaced ``command_dt = optimization_dt/interpolation_steps``
apart -- so pacing this controller's send cadence to ``interpolation_dt`` and
deriving ``optimization_dt = 4 * interpolation_dt`` makes one
``optimize_next_action()`` call, one real hardware tick, and one internal
fine command coincide exactly, with no hand-rolled index into a partially-
consumed horizon.

Also uses ``update_goal_tool_poses(goal, run_ik=True)`` directly --
``MPCController._apply_goal()`` always calls ``run_ik=False`` first, and that
branch unconditionally returns ``True`` (``solver_mpc.py:432-437``), so its
multi-seed IK anchor fallback (``_solve_goal_state``) is dead code as
currently wired. This controller's IK anchor is cuRobo's own single-seed
``IKSolver`` built inside ``MPCSolver.__init__`` -- a known, called-out
limitation (see the plan's Risques section), not silently inherited.
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

        # Single source of truth for the real hardware tick, shared with
        # JointSpeedStrategy (see resolve_interpolation_dt's own docstring) --
        # not a separately-tunable pacing knob like MPCController's
        # mpc_command_interval. optimization_dt = 4 * interpolation_dt is
        # imposed by cuRobo (interpolation_steps locked to 4 by
        # MPCSolverCfg.create()) and confirmed empirically (see module
        # docstring) -- never decouple this relation.
        interpolation_dt = resolve_interpolation_dt(node) # 17_08_2026 : 0.03s
        optimization_dt = 4.0 * interpolation_dt # 17_08_2026 : 0.12 s

        config_path = node.get_parameter('lbfgs_config_file').get_parameter_value().string_value
        lbfgs_cfg = _build_lbfgs_optimizer_config(config_path)
        # warm_start_iters/cold_start_iters/horizon are ModelPredictiveControlCfg
        # .create() kwargs, not optimizer_configs fields -- pop them off before
        # handing the rest to optimizer_configs, same pattern MPCController
        # uses for cspace_regularization_weight (mpc_planner.py:200-204). This
        # file (lbfgs_reactive.yaml) is NOT shared with MPCController's
        # lbfgs_mpc.yaml precisely so these top-level keys can't leak into
        # that controller's untouched optimizer_configs entry.
        warm_iters = lbfgs_cfg.pop('warm_start_iters')
        cold_iters = lbfgs_cfg.pop('cold_start_iters')
        horizon = lbfgs_cfg.pop('horizon')

        # Common kwargs, mirroring MPCController.build_solver() (mpc_planner.py:177-187):
        # the REAL production collision scene must be preserved (not scene_model=None),
        # and built WITHOUT the perception voxel layer -- collision_cache
        # pre-allocates the voxel storage instead; update_world fills it by copy.
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

        # Warm up NOW, during this build/warmup call, the two CUDA-graph
        # captures that `setup()`/`update_goal_tool_poses(run_ik=True)`
        # would otherwise pay on the FIRST client goal: the main optimizer's
        # (via `solver.setup()` -> `cold_start_solve()`) and the single-seed
        # IKSolver's (via `solve_pose()`, first triggered by run_ik=True).
        # Mirrors this class's own `setup()` call order exactly (solver.setup()
        # then update_goal_tool_poses(run_ik=True)) -- update_goal_tool_poses
        # requires the goal buffer solver.setup() allocates (raises "Goal
        # buffer has not been initialized" via manager_goal.py otherwise, hit
        # during development). Measured cost of skipping this warmup
        # (2026-08-17 integration test, m1013, use_cuda_graph=True): ~13s
        # blocking under gpu_lock on the first real goal -- enough to blow
        # through the test's 6s cancel_after window and leave the goal
        # cancelled before a single optimize_next_action() tick, let alone
        # any feedback, ran. Target: the solver's OWN current (default) EE
        # pose, so this warmup trivially succeeds regardless of robot/scene
        # state -- only the CUDA graph capture cost matters here, not
        # reaching any particular goal.
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

        # Fixed-interval send pacing (ReactiveController._execute_paced): one
        # optimize_next_action() call, one command, one real tick.
        self._command_interval = interpolation_dt

        # Pace the producer loop itself, not just the consumer's send timer:
        # optimize_next_action() advances a fixed 4-point internal buffer one
        # index per call with no awareness of real elapsed time, re-anchoring
        # to reality only once every interpolation_steps calls -- an
        # unthrottled producer burns through that buffer far faster than the
        # real robot executes commands. See ReactiveController.__init__'s
        # docstring on _producer_min_interval.
        self._producer_min_interval = interpolation_dt
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
        # INFO (not gated behind lbfgs_debug): the single-seed IK anchor
        # (Risque #1, plan streamed-inventing-eagle.md) failing silently is
        # indistinguishable, from the outside, from a slow/starved send path
        # -- both look like "the arm doesn't move". This was previously only
        # visible via log_setup_summary (debug level) or goal-marker color.
        node.get_logger().info(f"LBFGS: goal IK anchor applied={applied}")

        if self._debug_enabled():
            self._diag.log_setup_summary(start_state, raw, applied)
        return True

    # ---- Control step ----

    def step(self, current_state: JointState) -> JointState:
        t_solve = time.monotonic()
        result = self.solver.optimize_next_action(current_state)
        solve_ms = (time.monotonic() - t_solve) * 1000.0

        # [1, dof], not a horizon -- optimize_next_action's whole point. No
        # shape dispatch needed: _send_command/_state_from_action already
        # handle this shape identically to RetargetController's next_action.
        action = result.next_action
        if getattr(action, 'velocity', None) is None:
            # _send_command (reactive_controller.py) zero-fills a None velocity,
            # which is indistinguishable in the CSV from "the solver commanded
            # zero velocity on purpose" -- surface it explicitly instead.
            self.node.get_logger().warn(
                "LBFGS: next_action.velocity is None (send path will zero-fill)",
                throttle_duration_sec=2.0,
            )
        self._last_position_error = self._fk_position_error(action)
        self._last_orientation_error = self._fk_orientation_error(action)
        self._update_hold()

        self._diag.publish_predicted_path(result)
        self._diag.publish_full_predicted_path(result)

        if self._debug_enabled():
            breakdown = self._diag.cost_breakdown(result)
            self._diag.publish_costs(
                result, breakdown, self._last_position_error, self._last_orientation_error)
            self._diag.record_tick(
                result, solve_ms=solve_ms,
                last_position_error=self._last_position_error,
                last_orientation_error=self._last_orientation_error,
            )

        return action

    def apply_live_goal(self, raw_goal) -> bool:
        self.node.get_logger().info(
            f"LBFGS: live goal received - position=({raw_goal[0]:.4f}, {raw_goal[1]:.4f}, "
            f"{raw_goal[2]:.4f})m orientation(wxyz)=({raw_goal[3]:.4f}, {raw_goal[4]:.4f}, "
            f"{raw_goal[5]:.4f}, {raw_goal[6]:.4f})"
        )
        goal = self._set_target(raw_goal)
        # Direct run_ik=True -- see module docstring on why this differs from
        # MPCController._apply_goal(). Single-seed IK is a known limitation
        # (see the plan's Risques section), not silently swallowed here: a
        # failed anchor is visible via `applied=False` on the goal marker.
        applied = self.solver.update_goal_tool_poses(goal, run_ik=False)
        self.goal = goal
        self._diag.publish_goal_marker(raw_goal, applied)
        return applied

    def update_world(self, scene) -> None:
        """Reload the shared Scene into the LBFGS solver's collision checker.

        Same solver type as MPCController's (ModelPredictiveControl); see
        that controller's update_world for why this reaches into
        scene_collision_checker directly (MPCSolver has no update_world).
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
