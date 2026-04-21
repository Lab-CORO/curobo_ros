#!/usr/bin/env python3
"""
FK services for the unified planner node (v2).

Provides a lazy-initialized FK model that depends only on the robot's
kinematic model — no world/obstacle dependency.

Services exposed (prefixed with the node name):
  /<node>/warmup_fk  (WarmupFK) - init FK model with given batch size (default 1)
  /<node>/fk         (Fk)       - joint states → poses

v2 notes:
- CudaRobotModel → Kinematics (curobo.kinematics).
- RobotConfig no longer exists; KinematicsCfg.create accepts the YAML path
  (or dict) via `robot=`.
"""

import torch
from geometry_msgs.msg import Pose

from curobo.kinematics import Kinematics, KinematicsCfg

from curobo_msgs.srv import Fk, WarmupFK


class FKServices:
    """
    Manages the FK model and its ROS services on an existing node.

    Depends only on:
    - robot_config_file (YAML path — no world/obstacle needed)
    - node's CUDA device/dtype (via tensor_args or config_wrapper)

    The model is created only when warmup_fk is called.
    No update_world() needed: FK is purely geometric.
    """

    def __init__(self, node, robot_config_file: str):
        """
        Args:
            node: ROS2 node.
            robot_config_file: Path to the robot YAML config (v2 accepts this
                directly via KinematicsCfg.create(robot=...)).
        """
        self._node = node
        self._robot_config_file = robot_config_file

        self._fk_model: Kinematics | None = None

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

        node.get_logger().info("FKServices registered (not yet initialized — call warmup_fk)")

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
    # Service
    # ------------------------------------------------------------------

    def _fk_callback(self, request: Fk.Request, response: Fk.Response):
        if self._fk_model is None:
            self._node.get_logger().error("FK not initialized. Call warmup_fk first.")
            return response

        if not request.joint_states:
            self._node.get_logger().error("FK: no joint states provided")
            return response

        qs = [list(js.position) for js in request.joint_states]
        q = torch.tensor(qs, dtype=self._dtype, device=self._device)
        result = self._fk_model.get_state(q)

        for pos, ori in zip(
            result.ee_position.cpu().numpy(),
            result.ee_quaternion.cpu().numpy(),
        ):
            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            pose.orientation.x = float(ori[0])
            pose.orientation.y = float(ori[1])
            pose.orientation.z = float(ori[2])
            pose.orientation.w = float(ori[3])
            response.poses.append(pose)

        return response

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _init(self, batch_size: int):
        """Create the FK model and run a warmup batch of the given size."""
        self._node.get_logger().info(f"Initializing FK model (batch_size={batch_size})...")
        self._fk_model = Kinematics(KinematicsCfg.create(robot=self._robot_config_file))

        q = torch.rand(
            (batch_size, self._fk_model.get_dof()),
            dtype=self._dtype,
            device=self._device,
        )
        self._fk_model.get_state(q)

        self._node.get_logger().info("FK model ready")
