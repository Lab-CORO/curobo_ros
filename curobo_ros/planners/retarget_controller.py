#!/usr/bin/env python3
"""
Motion Retargeting — reactive teleop controller (cuRobo v2).

Thin wrapper over cuRobo's ``MotionRetargeter``. Like the MPC controller it is a
closed-loop sibling that follows a live stream of tool poses, but it solves with
**inverse kinematics** rather than a receding-horizon optimisation:

    solve_frame(goal):
        first frame  -> GLOBAL IK with num_seeds_global=64  (robust: finds a
                        solution on reachable poses where the MPC's hard-coded
                        single-seed retarget IK fails — see MPCController)
        next frames  -> LOCAL IK warm-started from the previous solution
                        (fast & smooth for a teleop pose stream)

This makes it the clean alternative to MPC for live tracking, free of the MPC
1-seed IK issue (see [[controller-interface-contract]]). All the ROS / robot /
perception control loop is inherited from :class:`ReactiveController`; this class
only implements the cuRobo-specific steps.

The solver is built from the node's single shared context (robot YAML + shared
Scene). ``solve_frame`` ignores the fed-back joint state (it warm-starts
internally), so unlike the MPC we don't need velocity/acceleration continuity.
"""

from typing import Any

from curobo.config_io import load_yaml
from curobo.types import JointState
from curobo._src.cost.tool_pose_criteria import ToolPoseCriteria
from curobo.motion_retargeter import MotionRetargeter, MotionRetargeterCfg

from .reactive_controller import ReactiveController


class RetargetController(ReactiveController):
    """Closed-loop teleop controller built on cuRobo ``MotionRetargeter`` (v2)."""

    SOLVER_KEY = 'retarget'

    def get_planner_name(self) -> str:
        return "Motion Retargeting (Teleop)"

    def get_config_parameters(self) -> list:
        return ['convergence_threshold', 'convergence_threshold_rad',
                'convergence_hold_steps', 'retarget_position_weight',
                'retarget_orientation_weight']

    # ---- cuRobo-specific hooks ------------------------------------------------

    def build_solver(self):
        cw = self.config_wrapper
        node = self.node
        w_pos = node.get_parameter('retarget_position_weight').get_parameter_value().double_value
        w_rot = node.get_parameter('retarget_orientation_weight').get_parameter_value().double_value
        use_mpc = node.get_parameter('retarget_use_mpc').get_parameter_value().bool_value

        tool = self._tool_frame()
        cfg = MotionRetargeterCfg.create(
            robot=cw.robot_config_file,
            tool_pose_criteria={
                tool: ToolPoseCriteria.track_position_and_orientation(
                    xyz=[w_pos, w_pos, w_pos], rpy=[w_rot, w_rot, w_rot],
                )
            },
            # No collision_cache in MotionRetargeterCfg: the voxel storage
            # comes from a private copy of the ESDF layer (never the live one).
            scene_model=cw.obstacle_manager.scene_with_private_voxel_copy(),
            use_mpc=use_mpc,
            num_seeds_global=64,          # robust first-frame IK
            self_collision_check=True,
        )
        solver = MotionRetargeter(cfg)

        node.get_logger().info(
            f"Retargeter built: tool={tool}, global IK 64 seeds, use_mpc={use_mpc}"
        )
        return solver

    def setup(self, start_state: JointState, goal_request: Any) -> bool:
        p = goal_request.target_pose
        raw = [
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z,
        ]
        # reset() clears the warm-start so the first solve_frame uses global IK.
        self.solver.reset()
        self.goal = self._set_target(raw)
        return True

    def step(self, current_state: JointState) -> JointState:
        # solve_frame tracks self.goal and warm-starts internally; it ignores the
        # fed-back current_state. on_target is the FK distance of the FRESH
        # solution (not the stale previous action).
        result = self.solver.solve_frame(self.goal)
        # Both errors, then the hold counter -- is_on_target() gates on all
        # three. Leaving the orientation error at its `inf` init would silently
        # pin on_target to False for this controller.
        self._last_position_error = self._fk_position_error(result.joint_state)
        self._last_orientation_error = self._fk_orientation_error(result.joint_state)
        self._update_hold()
        return result.joint_state

    def apply_live_goal(self, raw_goal) -> bool:
        # Replace the tracked goal; the next solve_frame warm-starts from the
        # current solution (local IK) for a smooth transition.
        self.goal = self._set_target(raw_goal)
        return True

    def update_world(self, scene) -> None:
        """Reload the shared Scene into every sub-solver of the retargeter."""
        for sub in self.sub_solvers(self.solver):
            # IKSolver.update_world is exactly this; MPCSolver lacks the wrapper.
            sub.scene_collision_checker.load_collision_model(scene)

    def attachment_managers(self) -> list:
        """Out of scope for object-attach: MotionRetargeter has no .core, so
        the ReactiveController default would raise on it. Known full traversal
        for a future extension: solver._global_ik_solver, plus either
        solver._local_ik_solver or solver._mpc_solver depending on
        retarget_use_mpc -- each exposing its own .core.attachment_manager.
        """
        return []

    # ---- helpers --------------------------------------------------------------

    @staticmethod
    def sub_solvers(retargeter) -> list:
        """The cuRobo solvers inside a MotionRetargeter: global IK, plus local
        IK or MPC (retarget_use_mpc). They hold the CUDA graphs and the
        collision world; the retargeter itself has neither."""
        if retargeter is None:
            return []
        return [s for s in (getattr(retargeter, '_global_ik_solver', None),
                            getattr(retargeter, '_local_ik_solver', None),
                            getattr(retargeter, '_mpc_solver', None))
                if s is not None]

    def graph_parts(self, retargeter):
        return self.sub_solvers(retargeter)

    def _captures_graphs(self) -> bool:
        # IKSolverCfg / MPCSolverCfg default use_cuda_graph=True and
        # MotionRetargeterCfg forwards no flag: always captures.
        return True

    def _prime_solve(self) -> bool:
        """One frame on the pose the default state already reaches: frame 0
        is global IK, later ones local IK or MPC (whose second solve
        dispatches more graphs)."""
        if self._warm_state is None:
            self._warm_state, self._warm_goal = self._make_warm_state_and_goal(self.solver)
        self.solver.solve_frame(self._warm_goal)
        return True

    def _prime_begin(self) -> None:
        # A previous goal leaves a warm start: without a reset the first prime
        # solve takes the local path and global IK (whose graphs activate()
        # released) would capture at the next prime, at goal start.
        self.solver.reset()

    def _prime_done(self) -> None:
        # A goal must start from global IK, not from the prime frames.
        if self.solver is not None:
            self.solver.reset()

    def _tool_frame(self) -> str:
        """Read the tracked tool frame from the robot YAML (needed pre-build)."""
        cfg = load_yaml(self.config_wrapper.robot_config_file)
        kin = cfg.get('robot_cfg', cfg).get('kinematics', {})
        frames = kin.get('tool_frames')
        return frames[0] if frames else 'link6'
