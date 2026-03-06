import torch
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo_msgs.srv import SetLinkCollision
from typing import List, Tuple


class RobotModelManager:
    """
    Manages the robot model, kinematics, and collision geometry.
    Responsible for:
    - Managing CudaRobotModel for kinematics computations
    - Computing robot collision spheres for visualization and masking
    - Providing joint state management
    - Enabling/disabling link collision spheres (shared with all solvers)
    """

    def __init__(self, robot_cfg: RobotConfig, robot, base_link: str, node=None):
        """
        Initialize the robot model manager.

        Args:
            robot_cfg: Robot configuration from cuRobo
            robot: Robot interface object for accessing joint states
            base_link: Robot base frame name
            node: ROS2 node (for logging and service registration)
        """
        self.robot_cfg = robot_cfg
        self.robot = robot
        self.base_link = base_link
        self.node = node

        # Create CUDA robot model for kinematics
        self.kin_model = CudaRobotModel(robot_cfg.kinematics)

        # Device configuration for operations
        self._ops_dtype = torch.float32
        self._device = torch.device('cuda')

    def get_kinematics_state(self, joint_positions):
        """
        Compute the kinematics state from joint positions.

        Args:
            joint_positions: Tensor of joint positions

        Returns:
            Kinematics state containing link poses, spheres, etc.
        """
        return self.kin_model.get_state(joint_positions)

    def get_collision_spheres(self) -> List[List[float]]:
        """
        Get the robot's collision spheres in current configuration.
        Useful for visualization and point cloud masking.

        Returns:
            List of spheres, each as [x, y, z, radius]
        """
        q_js = JointState(
            position=torch.tensor(
                self.robot.get_joint_pose(),
                dtype=self._ops_dtype,
                device=self._device
            ),
            joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        )

        kinematics_state = self.kin_model.get_state(q_js.position)

        robot_spheres = kinematics_state.link_spheres_tensor.view(-1, 4)
        robot_spheres = robot_spheres.cpu().numpy().tolist()

        return robot_spheres

    def set_link_collision(
        self, link_names: List[str], enabled: bool
    ) -> Tuple[List[str], List[str]]:
        """
        Enable or disable collision spheres for a list of links.

        Modifies link_spheres in-place on the shared KinematicsTensorConfig,
        so the change is immediately visible to all solvers (MotionGen, MPC).

        Returns:
            Tuple[List[str], List[str]]: (applied_links, unknown_links)
        """
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
        """
        ROS service callback — enable or disable collision spheres for a list of links.

        All solvers (MotionGen, MPC) share the same KinematicsTensorConfig, so a
        single in-place modification affects every active solver simultaneously.
        The state persists until an explicit call with the opposite value.
        """
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
        """
        Get current joint state from robot interface.

        Returns:
            JointState object with current joint positions
        """
        return JointState(
            position=torch.tensor(
                self.robot.get_joint_pose(),
                dtype=self._ops_dtype,
                device=self._device
            ),
            joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        )
