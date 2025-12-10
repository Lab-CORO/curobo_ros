import torch
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from typing import List


class RobotModelManager:
    """
    Manages the robot model, kinematics, and collision geometry.
    Responsible for:
    - Managing CudaRobotModel for kinematics computations
    - Computing robot collision spheres for visualization and masking
    - Providing joint state management
    """

    def __init__(self, robot_cfg: RobotConfig, robot, base_link: str):
        """
        Initialize the robot model manager.

        Args:
            robot_cfg: Robot configuration from cuRobo
            robot: Robot interface object for accessing joint states
            base_link: Robot base frame name
        """
        self.robot_cfg = robot_cfg
        self.robot = robot
        self.base_link = base_link

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
        # Get current joint pose from robot
        q_js = JointState(
            position=torch.tensor(
                self.robot.get_joint_pose(),
                dtype=self._ops_dtype,
                device=self._device
            ),
            joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        )

        # Compute kinematics state
        kinematics_state = self.kin_model.get_state(q_js.position)

        # Extract collision spheres (x, y, z, radius)
        robot_spheres = kinematics_state.link_spheres_tensor.view(-1, 4)
        robot_spheres = robot_spheres.cpu().numpy().tolist()

        return robot_spheres

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
