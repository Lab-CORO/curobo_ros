"""
Standalone FK node (v2).

v2 notes:
- CudaRobotModel → Kinematics (curobo.kinematics)
- RobotConfig.from_basic(...) is gone. KinematicsCfg.create accepts the
  robot YAML path (or dict) directly via `robot=`.
- TensorDeviceType → DeviceCfg (curobo.types).
"""

import os

import rclpy
import torch
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from curobo_msgs.srv import Fk
from geometry_msgs.msg import Pose

from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import DeviceCfg


class FK(Node):
    def __init__(self):
        super().__init__('FK')
        self.robot_name = "ur10e"

        self.srv_fk = self.create_service(Fk, '/curobo/fk_poses', self.fk_callback)

        self.device_cfg = DeviceCfg(device='cuda', dtype=torch.float32)
        self._device = torch.device(self.device_cfg.device)
        self._dtype = self.device_cfg.dtype

        robot_yml = os.path.join(
            get_package_share_directory('curobo_ros'),
            'curobo_doosan', 'src', 'm1013', 'm1013.yml',
        )
        self.kin_model = Kinematics(KinematicsCfg.create(robot=robot_yml))

        self.fk_init()
        self.get_logger().info("FK service up !")

    def fk_callback(self, request, response):
        if len(request.joint_states) > 1000:
            self.get_logger().error('Max batch size is 1000')
            return response
        if len(request.joint_states) <= 0:
            self.get_logger().error('No positions to compute')
            return response

        qs = [list(joint.position) for joint in request.joint_states]
        q = torch.tensor(qs, dtype=self._dtype, device=self._device)
        result = self.kin_model.get_state(q)

        for position, orientation in zip(
            result.ee_position.cpu().numpy(),
            result.ee_quaternion.cpu().numpy(),
        ):
            pose = Pose()
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
            pose.orientation.x = float(orientation[0])
            pose.orientation.y = float(orientation[1])
            pose.orientation.z = float(orientation[2])
            pose.orientation.w = float(orientation[3])
            response.poses.append(pose)
        return response

    def fk_init(self):
        """Warmup the kinematics model (first call is slow)."""
        dof = self.kin_model.get_dof()
        q = torch.rand((10, dof), dtype=self._dtype, device=self._device)
        self.kin_model.get_state(q)


def main(args=None):
    rclpy.init(args=args)
    fk = FK()
    rclpy.spin(fk)
    fk.destroy_node()
    rclpy.shutdown()
