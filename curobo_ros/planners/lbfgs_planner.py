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
``optimize_next_action()`` precisely for that pattern -- a real resolve
every ``interpolation_steps`` (4, hard-locked by ``MPCSolverCfg.create()``)
calls. Verified directly in cuRobo source (``solver_mpc.py``,
``trajectory_execution_manager.py``) and confirmed empirically
(``/home/coro/.claude/plans/streamed-inventing-eagle.md``, Vérification étape
0, 2026-08-17): a resolve buffer holds exactly 4 points for LBFGS
(BSPLINE_3), spaced ``command_dt = optimization_dt/interpolation_steps``
apart, giving ``optimization_dt = 4 * interpolation_dt``.

Calling ``optimize_next_action()`` once per ``interpolation_dt`` (one real
hardware tick per call) was the original design here, on the assumption that
cuRobo would handle buffer pacing "internally" for free. Measured on hardware
it wasn't free: ``optimize_next_action()``'s ONE real resolve per 4 calls
(up to ~60ms observed) then has to fit inside a single ``interpolation_dt``
(30ms) instead of the full ``optimization_dt`` (120ms) it actually has before
that command window is due -- a structural, not incidental, source of
"command tick out of time" drops (debug session 2026-08-17). Confirmed
directly in ``solver_mpc.py``: the non-resolving calls skip
``warm_start_solve`` (the only place ``current_state`` is read) entirely
whenever ``trajectory_execution_manager.has_valid_next_command()`` is True --
a pure call-counter with no wall-clock awareness -- so those calls gain
nothing from being spaced one per real tick. ``ReactiveController`` now
batches all ``interpolation_steps`` calls into one back-to-back producer
burst instead (``_batch_size``, see its docstring in
``ReactiveController.__init__``), so the one real resolve gets the full
``optimization_dt`` to fit in, and hands the resulting queue to the
consumer's existing per-tick send timer.

Also uses ``update_goal_tool_poses(goal, run_ik=False)`` -- ``run_ik=True``
(cuRobo's single-seed internal ``IKSolver`` anchor) was tried first and
failed silently (``applied=False``) on this 6DOF non-redundant arm, freezing
the solver's own commanded trajectory at the start pose for the whole goal
(debug session 2026-08-17): the anchor exists to resolve redundancy on arms
with more DOF than task-space constraints, which a 6DOF arm chasing a single
6D pose doesn't have. ``run_ik=False`` disables that anchor and only updates
the rollout's Cartesian target -- confirmed converging cleanly (<1mm) on this
robot without it.
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

        # Fixed-interval SEND pacing (ReactiveController._on_send_tick): one
        # queued action, one command, one real tick.
        self._command_interval = interpolation_dt

        # optimize_next_action() only does real GPU work on 1 call out of
        # every interpolation_steps -- confirmed directly in solver_mpc.py:
        # it calls warm_start_solve(current_state) (the only place
        # current_state is read) only when
        # trajectory_execution_manager.has_valid_next_command() is False, a
        # pure call-counter with no wall-clock awareness. The other calls
        # just index into the already-computed buffer (get_next_command()),
        # ignoring current_state entirely. So there is nothing to gain from
        # spacing those calls 1 per real hardware tick. Batch all
        # interpolation_steps calls into one back-to-back burst per producer
        # iteration instead (see ReactiveController._batch_size /
        # _execute_paced) and let the consumer's existing per-tick timer
        # drain the resulting queue -- backpressure (queue depth, not a
        # fixed optimization_dt window) is what now gives the one resolving
        # call room to run long without starving the consumer. cf. debug
        # 2026-08-17.
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
        # optimize_next_action only does real GPU work (warm_start_solve,
        # which reads current_state) when the internal command buffer is
        # exhausted -- checking this BEFORE the call tells us whether THIS
        # call is that one real resolve, or one of the interpolation_steps-1
        # cheap buffer-index calls in the same batch (see ReactiveController
        # _batch_size docstring). Measured 2026-08-17: those "cheap" calls
        # still cost real time even so -- optimize_next_action()
        # unconditionally calls get_current_metrics()+_get_result(metrics),
        # and _get_result reprocesses convergence/feasibility tensors
        # (torch.cat, comparisons, torch.all) on the SAME cached metrics
        # object every call, resolving or not (verified in solver_mpc.py).
        #
        # A same-day attempt to dodge that by calling
        # trajectory_execution_manager.get_next_command() directly on
        # non-resolving calls (skipping optimize_next_action() and its
        # _get_result() entirely) crashed in practice: has_valid_next_command()
        # only checks _current_action_trajectory, but get_next_command() reads
        # the SEPARATE _current_joint_state_trajectory buffer, and a private
        # flag (_mpc_warm_start_available, not visible from outside the
        # solver) can require cold_start_solve() to run -- and repopulate
        # that joint-state buffer -- before has_valid_next_command() is even
        # meaningful, specifically on the first call after a new goal
        # (setup()). optimize_next_action() checks that private flag first,
        # internally; a caller outside the solver cannot replicate the same
        # ordering without seeing it, so always go through
        # optimize_next_action() and eat the _get_result() cost -- reported
        # upstream (see the cuRobo GitHub issue) rather than worked around
        # here. cf. debug 2026-08-17 (AttributeError: 'NoneType' object has
        # no attribute 'position').
        is_resolving = not self.solver.trajectory_execution_manager.has_valid_next_command()

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
        """One real resolve (via ``step()``, which drives the FULL
        ``optimize_next_action()`` -- FK error, hold-count, RViz publish, CSV
        row, all unchanged) plus ``n - 1`` more actions read directly out of
        the B-spline buffer THAT SAME resolve already filled, via
        ``TrajectoryExecutionManager.get_command_sequence()`` -- one tensor
        slice instead of ``n - 1`` more ``optimize_next_action()`` calls.

        Why this is safe (unlike the get_next_command()-instead-of-
        optimize_next_action() bypass tried and reverted 2026-08-17, see
        step()'s own docstring above): step() still goes through
        optimize_next_action() exactly once here, so cuRobo's own cold/warm-
        start triggering logic (_mpc_warm_start_available,
        has_valid_next_command()) runs completely unmodified for the call
        that matters. Only the interpolation_steps-1 calls that would have
        re-read the SAME already-computed buffer are replaced.

        Verified against curobo 0.8.0.post1.dev42 (solver_mpc.py:45-54): for
        a B-spline control space (this controller's BSPLINE_3),
        command_start_idx == interpolation_steps, and
        get_command_sequence()'s default command_end_idx ==
        interpolation_steps * 2 -- i.e. it returns EXACTLY the same `n`
        horizon points that `n` sequential get_next_command() calls would
        each return one at a time (same index window). Measured cost of the
        skipped calls (2026-08-18, lbfgs_diag_20260818_072102.csv): ~17ms
        each on this Jetson (~51ms/batch, ~28% of the real-time budget) --
        entirely get_current_metrics()+_get_result() redundantly
        reprocessing a metrics object that does not change between them
        (reported upstream, see curobo_github_issue.md). sync_cuda_time had
        no effect on this cost (confirmed empirically the same day) because
        it's real kernel-launch/indexing work, not sync overhead -- this is
        the actual fix, not a profiling-flag workaround.

        _current_command_idx is advanced manually afterward (a private
        cuRobo attribute -- there is no public "mark consumed" API) so the
        manager's own bookkeeping matches what we actually took. Without
        this, the next batch's first call would still see
        has_valid_next_command() == True and silently re-serve the SAME
        stale points instead of resolving -- this is the one fragile part of
        this override: a cuRobo internal refactor could rename or repurpose
        that attribute without warning. Guarded by an AttributeError
        fallback to the safe (slower) per-call behavior.
        """
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
