import torch
from typing import List, Tuple

from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import JointState
from curobo_msgs.srv import SetLinkCollision


class RobotModelManager:
    """
    Manages the robot kinematics and collision geometry.

    v2 notes:
    - `CudaRobotModel` → `Kinematics` (curobo.kinematics)
    - `RobotConfig` no longer exists — kinematics is built from a YAML path
      or dict via `KinematicsCfg.create(robot=…)`.
    - KinematicsTensorConfig still lives on the kinematics instance; the
      `enable_link_spheres` / `disable_link_spheres` API is unchanged.
    """

    def __init__(self, robot_config_file: str, robot, base_link: str, node=None):
        """
        Args:
            robot_config_file: Path to the robot YAML config (v2 accepts this directly).
            robot: Robot interface object for accessing joint states.
            base_link: Robot base frame name.
            node: ROS2 node (for logging and service registration).
        """
        self.robot_config_file = robot_config_file
        self.robot = robot
        self.base_link = base_link
        self.node = node

        self.kin_model = Kinematics(KinematicsCfg.create(robot=robot_config_file))

        self._ops_dtype = torch.float32
        self._device = torch.device('cuda')

    def get_kinematics_state(self, joint_positions):
        return self.kin_model.get_state(joint_positions)

    def get_collision_spheres(self) -> List[List[float]]:
        q_js = JointState(
            position=torch.tensor(
                self.robot.get_joint_pose(),
                dtype=self._ops_dtype,
                device=self._device,
            ),
            joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
        )

        kinematics_state = self.kin_model.get_state(q_js.position)
        robot_spheres = kinematics_state.link_spheres_tensor.view(-1, 4)
        return robot_spheres.cpu().numpy().tolist()

    def set_link_collision(
        self, link_names: List[str], enabled: bool
    ) -> Tuple[List[str], List[str]]:
        kc = self.kin_model.kinematics_config
        applied, unknown = [], []
        for link in link_names:
            if link not in kc.link_name_to_idx_map:
                unknown.append(link)
            else:
                if enabled:
                    kc.enable_link_spheres(link)
                else:
                    kc.disable_link_spheres(link)
                applied.append(link)
        return applied, unknown

    def set_link_collision_callback(
        self,
        request: SetLinkCollision.Request,
        response: SetLinkCollision.Response,
    ) -> SetLinkCollision.Response:
        applied, unknown = self.set_link_collision(list(request.link_names), request.enabled)

        response.applied_links = applied
        response.unknown_links = unknown

        if unknown:
            kc = self.kin_model.kinematics_config
            if self.node:
                self.node.get_logger().warn(
                    f"set_link_collision: unknown links {unknown}. "
                    f"Available: {list(kc.link_name_to_idx_map.keys())}"
                )

        if applied:
            state = "enabled" if request.enabled else "disabled"
            if self.node:
                self.node.get_logger().info(f"Collision spheres {state} for: {applied}")
            response.success = True
            response.message = f"Collision {state} for {applied}"
        else:
            response.success = False
            response.message = f"No valid links found in: {list(request.link_names)}"

        return response

    def get_joint_state(self) -> JointState:
        return JointState(
            position=torch.tensor(
                self.robot.get_joint_pose(),
                dtype=self._ops_dtype,
                device=self._device,
            ),
            joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
        )
