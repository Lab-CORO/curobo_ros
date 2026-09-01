#!/usr/bin/env python3
"""
FK services for the unified planner node (v2).

Provides a lazy-initialized FK model that depends only on the robot's
kinematic model — no world/obstacle dependency for the poses themselves.

Services exposed (prefixed with the node name):
  /<node>/warmup_fk  (WarmupFK)  - init FK model with given batch size (default 1)
  /<node>/fk         (Fk)        - N joint states → poses (+ per-pose collision validity)
  /<node>/fk_batch   (FkBatch)   - same as /fk, with success/error_msg like IkBatch

v2 notes:
- CudaRobotModel → Kinematics (curobo.kinematics).
- RobotConfig no longer exists; KinematicsCfg.create accepts the YAML path
  (or dict) via `robot=`.

Collision validity (``poses_valid``)
-------------------------------------
FK itself is purely geometric — ``compute_kinematics`` has no notion of
collision. Per-pose validity is answered SEPARATELY by
``curobo.collision_checking.RobotCollisionChecker`` (a differentiable,
non-CUDA-graph collision checker meant for exactly this kind of custom
pipeline — see its module docstring), which owns its own internal kinematics
model and is built alongside ``_fk_model`` in ``_init()``. Its
``validate(q)`` folds self-collision, scene/obstacle collision AND joint
bound checks into a single boolean per configuration; unlike the IK solver,
it does not need reinitializing when the request's batch size changes.
"""

import torch
import std_msgs.msg
from geometry_msgs.msg import Pose

from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import JointState as CuRoboJS, DeviceCfg
from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg

from curobo_msgs.srv import Fk, FkBatch, WarmupFK


class FKServices:
    """
    Manages the FK model and its ROS services on an existing node.

    Depends on:
    - config_wrapper.robot_config_file   (YAML path — v2 factories accept it directly)
    - config_wrapper.obstacle_manager    (Scene, shared with MotionPlanner/IK — for
      the collision checker's scene half only; FK poses themselves never see it)
    - config_wrapper.collision_cache     (single v2 cache integer)

    The model is created only when warmup_fk is called.
    Obstacle updates are propagated via update_world() (collision checker only —
    the FK poses are purely geometric and unaffected).
    """

    def __init__(self, node, config_wrapper):
        """
        Args:
            node: ROS2 node.
            config_wrapper: shared config (robot YAML, obstacle manager, collision
                cache) — same object handed to IKServices.
        """
        self._node = node
        self._config = config_wrapper
        self._robot_config_file = config_wrapper.robot_config_file

        self._fk_model: Kinematics | None = None
        self._collision_checker: RobotCollisionChecker | None = None
        self._fk_batch_size: int = 0  # 0 = not yet warmed up

        # Resolve device/dtype from the node's tensor_args if present, else default.
        tensor_args = getattr(node, 'tensor_args', None)
        if tensor_args is not None and hasattr(tensor_args, 'device'):
            self._device = torch.device(tensor_args.device)
            self._dtype = getattr(tensor_args, 'dtype', torch.float32)
        else:
            self._device = torch.device('cuda')
            self._dtype = torch.float32

        name = node.get_name()
        node.create_service(WarmupFK, f'{name}/warmup_fk', self._warmup_fk_callback)
        node.create_service(Fk,       f'{name}/fk',        self._fk_callback)
        node.create_service(FkBatch,  f'{name}/fk_batch',  self._fk_batch_callback)

        node.get_logger().info("FKServices registered (not yet initialized - call warmup_fk)")

    # ------------------------------------------------------------------
    # Warmup
    # ------------------------------------------------------------------

    def _warmup_fk_callback(self, request: WarmupFK.Request, response: WarmupFK.Response):
        batch_size = max(1, request.batch_size)
        try:
            self._init(batch_size)
            response.success = True
            response.message = f"FK model ready (batch_size={batch_size})"
        except Exception as e:
            self._node.get_logger().error(f"FK warmup failed: {e}")
            response.success = False
            response.message = str(e)
        return response

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _fk_callback(self, request: Fk.Request, response: Fk.Response):
        if self._fk_model is None:
            self._node.get_logger().error("FK not initialized. Call warmup_fk first.")
            response.error_msg.data = "FK not initialized. Call warmup_fk first."
            return response

        if not request.joint_states:
            self._node.get_logger().error("FK: no joint states provided")
            response.error_msg.data = "FK: no joint states provided"
            return response

        try:
            q = self._positions_tensor(request.joint_states)
        except Exception as e:
            self._node.get_logger().error(f"FK: malformed joint_states: {e}")
            response.error_msg.data = f"malformed joint_states: {e}"
            return response

        response.poses, response.poses_valid = self._compute(q)
        return response

    def _fk_batch_callback(self, request: FkBatch.Request, response: FkBatch.Response):
        if self._fk_model is None:
            response.success = False
            response.error_msg.data = "FK not initialized. Call warmup_fk first."
            return response

        if not request.joint_states:
            response.success = False
            response.error_msg.data = "FK batch: empty joint_states list"
            return response

        try:
            q = self._positions_tensor(request.joint_states)
            response.poses, response.poses_valid = self._compute(q)
        except Exception as e:
            self._node.get_logger().error(f"FK batch failed: {e}")
            response.success = False
            response.error_msg.data = str(e)
            return response

        response.success = True
        return response

    # ------------------------------------------------------------------
    # World update (called by the node when obstacles change)
    # ------------------------------------------------------------------

    def update_world(self):
        """Propagate obstacle changes to the collision checker. No-op if not
        initialized. The FK poses themselves are unaffected — purely geometric."""
        if self._collision_checker is None:
            return
        scene = self._config.obstacle_manager.get_scene()
        self._collision_checker.update_world(scene)
        self._node.get_logger().info("FKServices: world updated")

    def rebuild(self):
        """Recreate the FK model + collision checker after a collision-cache
        change (reusing the last warmup batch size). No-op if never initialized."""
        if self._fk_model is None:
            return
        self._init(max(1, self._fk_batch_size))
        self._node.get_logger().info("FKServices: solver rebuilt after cache change")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _init(self, batch_size: int):
        """Create the FK model + collision checker, and run a warmup batch."""
        self._node.get_logger().info(f"Initializing FK model (batch_size={batch_size})...")
        self._fk_model = Kinematics(KinematicsCfg.from_robot_yaml_file(self._robot_config_file))
        self._fk_batch_size = batch_size

        q = torch.rand(
            (batch_size, self._fk_model.get_dof()),
            dtype=self._dtype,
            device=self._device,
        )
        js = CuRoboJS.from_position(q, joint_names=self._fk_model.joint_names)
        self._fk_model.compute_kinematics(js)

        # Primitives only at construction, same as IKServices/MotionPlanner;
        # update_world() pushes the perception layer by copy afterwards.
        scene = self._config.obstacle_manager.primitives_only_scene()
        # warp's device_from_torch indexes its CUDA device list with
        # torch_device.index, which is None for a bare torch.device('cuda')
        # (exactly what tensor_args.device resolves to -- see
        # config_manager.py's DeviceCfg(device='cuda', ...)) -- crashes with
        # "list indices must be integers ... not NoneType" unless normalized.
        collision_device = self._device
        if collision_device.type == 'cuda' and collision_device.index is None:
            collision_device = torch.device('cuda', torch.cuda.current_device())
        collision_cfg = RobotCollisionCheckerCfg.load_from_config(
            robot_config=self._robot_config_file,
            scene_model=scene,
            device_cfg=DeviceCfg(device=collision_device, dtype=self._dtype),
            n_meshes=self._config.collision_cache,
            n_cuboids=self._config.collision_cache,
        )
        self._collision_checker = RobotCollisionChecker(collision_cfg)

        self._node.get_logger().info("FK model ready")

    def _positions_tensor(self, joint_states) -> torch.Tensor:
        qs = [list(js.position) for js in joint_states]
        return torch.tensor(qs, dtype=self._dtype, device=self._device)

    def _compute(self, q: torch.Tensor):
        """FK poses + per-pose collision validity for joint positions ``q``
        (``[batch, dof]``). Returns ``(list[Pose], list[std_msgs.msg.Bool])``.
        """
        js = CuRoboJS.from_position(q, joint_names=self._fk_model.joint_names)
        kin_state = self._fk_model.compute_kinematics(js)

        # ToolPose.position/quaternion: [B, H=1, L, 3/4]; take first tool frame.
        # v2 quaternion is wxyz; ROS geometry_msgs.Pose.orientation is xyzw.
        positions = kin_state.tool_poses.position[:, 0, 0, :].cpu().numpy()
        quaternions = kin_state.tool_poses.quaternion[:, 0, 0, :].cpu().numpy()

        poses = []
        for pos, ori in zip(positions, quaternions):
            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            pose.orientation.w = float(ori[0])
            pose.orientation.x = float(ori[1])
            pose.orientation.y = float(ori[2])
            pose.orientation.z = float(ori[3])
            poses.append(pose)

        valid_mask = self._collision_checker.validate(q.unsqueeze(1))  # [batch, horizon=1]
        poses_valid = []
        for v in valid_mask.reshape(-1).cpu().tolist():
            b = std_msgs.msg.Bool()
            b.data = bool(v)
            poses_valid.append(b)

        return poses, poses_valid
