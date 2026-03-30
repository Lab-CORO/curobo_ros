#!/usr/bin/env python3
"""
IK services for the unified planner node.

Provides a lazy-initialized IK solver that shares the obstacle manager
and robot config of the trajectory planner.

Services exposed (prefixed with the node name):
  /<node>/warmup_ik  (WarmupIK) - init IK solver with given batch size (default 1)
  /<node>/ik         (Ik)       - single pose → joint state
  /<node>/ik_batch   (IkBatch)  - N poses → joint states
"""

import torch
import std_msgs.msg
from sensor_msgs.msg import JointState

from curobo.types.math import Pose as CuroboPose
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from curobo.geom.sdf.world import CollisionCheckerType

from curobo_msgs.srv import Ik, IkBatch, WarmupIK


class IKServices:
    """
    Manages the IK solver and its ROS services on an existing node.

    Depends on:
    - config_wrapper_motion.robot_cfg       (robot kinematics)
    - config_wrapper_motion.obstacle_manager (collision world, shared with MotionGen)
    - node.tensor_args                       (CUDA device)

    The solver is created only when warmup_ik is called.
    Obstacle updates are propagated via update_world().
    """

    def __init__(self, node, config_wrapper_motion):
        self._node = node
        self._config = config_wrapper_motion

        self._ik_solver: IKSolver | None = None
        self._ik_batch_size: int = 0  # 0 = not yet warmed up

        name = node.get_name()
        node.create_service(WarmupIK, f'{name}/warmup_ik', self._warmup_ik_callback)
        node.create_service(Ik,       f'{name}/ik',        self._ik_callback)
        node.create_service(IkBatch,  f'{name}/ik_batch',  self._ik_batch_callback)

        node.get_logger().info("IKServices registered (not yet initialized — call warmup_ik)")

    # ------------------------------------------------------------------
    # Warmup
    # ------------------------------------------------------------------

    def _warmup_ik_callback(self, request: WarmupIK.Request, response: WarmupIK.Response):
        batch_size = max(1, request.batch_size)
        try:
            self._init(batch_size)
            response.success = True
            response.message = f"IK solver ready (batch_size={batch_size})"
        except Exception as e:
            self._node.get_logger().error(f"IK warmup failed: {e}")
            response.success = False
            response.message = str(e)
        return response

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _ik_callback(self, request: Ik.Request, response: Ik.Response):
        if self._ik_solver is None:
            response.success = False
            response.error_msg.data = "IK not initialized. Call warmup_ik first."
            return response

        ok, result = self._solve([request.pose])
        if not ok:
            response.success = False
            response.error_msg.data = "IK solve failed"
            return response

        js = JointState()
        js.position = result.solution.cpu().numpy()[0][0].tolist()
        valid = std_msgs.msg.Bool()
        valid.data = bool(result.success.cpu().numpy()[0][0])
        response.joint_states = js
        response.joint_states_valid = valid
        response.success = True
        return response

    def _ik_batch_callback(self, request: IkBatch.Request, response: IkBatch.Response):
        if self._ik_solver is None:
            response.success = False
            response.error_msg.data = "IK not initialized. Call warmup_ik first."
            return response

        ok, result = self._solve(request.poses)
        if not ok:
            response.success = False
            response.error_msg.data = "IK batch solve failed"
            return response

        for i, j in enumerate(result.solution.cpu().numpy()):
            js = JointState()
            js.position = j[0].tolist()
            valid = std_msgs.msg.Bool()
            valid.data = bool(result.success.cpu().numpy()[i][0])
            response.joint_states.append(js)
            response.joint_states_valid.append(valid)
        response.success = True
        return response

    # ------------------------------------------------------------------
    # World update (called by the node when obstacles change)
    # ------------------------------------------------------------------

    def update_world(self):
        """Propagate obstacle changes to the IK solver. No-op if not initialized."""
        if self._ik_solver is None:
            return
        world_cfg = self._config.obstacle_manager.get_world_cfg()
        self._ik_solver.world_coll_checker.clear_cache()
        self._ik_solver.update_world(world_cfg)
        self._node.get_logger().info("IKServices: world updated")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _init(self, batch_size: int):
        """Create (or recreate) the IK solver for the given batch size."""
        world_cfg = self._config.obstacle_manager.get_world_cfg()
        robot_cfg = self._config.robot_cfg

        self._node.get_logger().info(f"Initializing IK solver (batch_size={batch_size})...")

        ik_config = IKSolverConfig.load_from_robot_config(
            robot_cfg,
            world_cfg,
            rotation_threshold=0.05,
            position_threshold=0.005,
            num_seeds=20,
            self_collision_check=True,
            self_collision_opt=True,
            collision_checker_type=CollisionCheckerType.BLOX,
            collision_cache={"obb": 100},
            tensor_args=self._node.tensor_args,
            use_cuda_graph=False,
        )
        self._ik_solver = IKSolver(ik_config)
        self._ik_batch_size = batch_size

        # Warmup: solve a batch of random configs to prime CUDA kernels
        q_sample = self._ik_solver.sample_configs(batch_size)
        kin_state = self._ik_solver.fk(q_sample)
        goal = CuroboPose(kin_state.ee_position, kin_state.ee_quaternion)
        self._ik_solver.solve_batch(goal)
        torch.cuda.synchronize()

        self._node.get_logger().info("IK solver ready")

    def _solve(self, poses):
        """
        Solve IK for a list of geometry_msgs/Pose.
        Reinitializes the solver if the batch size has changed.
        Returns (success: bool, result).
        """
        if not poses:
            self._node.get_logger().error("IK: empty pose list")
            return False, None

        n = len(poses)
        if n != self._ik_batch_size:
            try:
                self._init(n)
            except Exception as e:
                self._node.get_logger().error(f"IK reinit for batch_size={n} failed: {e}")
                self._ik_batch_size = 0
                return False, None

        positions    = [[p.position.x, p.position.y, p.position.z] for p in poses]
        orientations = [[p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
                        for p in poses]

        goal = CuroboPose(torch.tensor(positions), torch.tensor(orientations))

        try:
            result = self._ik_solver.solve_batch(goal)
        except Exception:
            try:
                self._init(n)
                result = self._ik_solver.solve_batch(goal)
            except Exception as e:
                self._node.get_logger().error(f"IK solve failed: {e}")
                self._ik_batch_size = 0
                return False, None

        torch.cuda.synchronize()
        return True, result
