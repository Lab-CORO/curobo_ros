#!/usr/bin/env python3
"""
Model Predictive Control (MPC) — reactive controller (cuRobo v2).

Thin wrapper over cuRobo's ``ModelPredictiveControl``. All the ROS / robot /
perception control loop lives in :class:`ReactiveController`; this class only
implements the cuRobo-specific steps (build, goal setup, step, retarget).

v2 reactive lifecycle (mirrored here):
    ModelPredictiveControlCfg.create(robot=, scene_model=, collision_cache=…)
    -> mpc.setup(state)
    -> mpc.update_goal_tool_poses({ee: pose}, run_ik=False)  # pure pose tracking
    -> loop: result = mpc.optimize_next_action(state)        # .next_action
    -> mpc.update_world(scene)  # dynamic obstacles, driven by the node

Two cuRobo quirks shaped this design:
- The MPC's internal goal IK is single-seed (hard-coded, not exposed) and fails
  on perfectly reachable poses. We therefore track the Cartesian pose directly
  (``run_ik=False``) — the standard MPC mode — instead of relying on that IK.
- ``optimize_next_action().position_error`` is a per-step solver metric (≈0 even
  far from the goal), so the on_target signal is computed from real forward
  kinematics (current EE vs target) instead.

The solver is built from the node's single shared context (the motion config
wrapper): robot YAML, the shared Scene (obstacle_manager) and the shared
collision cache.
"""

from typing import Any

import torch
from curobo.types import JointState, Pose, GoalToolPose
from curobo.model_predictive_control import (
    ModelPredictiveControl,
    ModelPredictiveControlCfg,
)

from .reactive_controller import ReactiveController


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

        cfg = ModelPredictiveControlCfg.create(
            robot=cw.robot_config_file,
            scene_model=cw.obstacle_manager.get_scene(),
            optimization_dt=step_dt,
            num_control_points=horizon,
            use_cuda_graph=True,
            self_collision_check=True,
            collision_cache=cw.collision_cache,
            store_debug=False,  # debug storage adds per-step overhead (slows the loop)
        )
        solver = ModelPredictiveControl(cfg)

        # Publish where the node expects it so update_world / collision-distance
        # queries reach this solver (single shared reference).
        node.mpc = solver
        node.get_logger().info(
            f"MPC solver built: optimization_dt={step_dt}s, "
            f"num_control_points={horizon}, collision_cache={cw.collision_cache}"
        )
        return solver

    def setup(self, start_state: JointState, goal_request: Any) -> bool:
        p = goal_request.target_pose
        raw = [
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z,
        ]
        goal = self._set_target(raw)
        self.solver.setup(start_state)
        self._update_goal(goal)  # run_ik=False -> always succeeds (no IK)
        self.goal = goal
        return True

    def step(self, current_state: JointState) -> JointState:
        result = self.solver.optimize_next_action(current_state)
        # The solver's position_error is a per-step metric (≈0 even far from the
        # goal); use real FK (current EE vs target) for the on_target signal.
        self._last_position_error = self._fk_position_error(current_state)
        return result.next_action

    def apply_live_goal(self, raw_goal) -> bool:
        goal = self._set_target(raw_goal)
        self._update_goal(goal)
        self.goal = goal
        return True

    # ---- helpers --------------------------------------------------------------

    def _set_target(self, raw) -> GoalToolPose:
        """Store the target position (for FK error) and build the tool-pose goal."""
        self._target_position = torch.tensor(
            raw[0:3], dtype=self._dtype, device=self._device
        )
        return GoalToolPose.from_poses(
            {self.solver.tool_frames[0]: Pose.from_list(list(raw))}
        )

    def _fk_position_error(self, current_state: JointState) -> float:
        """Real Cartesian distance (m) between the current EE and the target."""
        target = getattr(self, '_target_position', None)
        if target is None:
            return float('inf')
        try:
            kin = self.solver.compute_kinematics(current_state)
            ee = kin.tool_poses.position.reshape(-1, 3)[0]  # [B,H,L,3] -> first link
            return float(torch.linalg.norm(ee - target).item())
        except Exception:
            return float('inf')

    def _update_goal(self, goal: GoalToolPose) -> bool:
        """Set the tracked Cartesian goal — pure pose tracking, NO IK.

        cuRobo's MPC tracks the tool pose via its cost (the standard MPC mode);
        a per-update IK is not required. We deliberately use run_ik=False: the
        MPC's internal retarget IK is single-seed (no num_seeds knob exposed) and
        was failing on perfectly reachable poses. With continuous servoing (no
        step-0 convergence stop), pose-only tracking drives the arm to the goal
        without any IK dependency. Returns True (no IK to fail).
        """
        return bool(self.solver.update_goal_tool_poses(goal, run_ik=False))


# Backwards-compatible alias (old name still used by some imports / docs).
MPCPlanner = MPCController
