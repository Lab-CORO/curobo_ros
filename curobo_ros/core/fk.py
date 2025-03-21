import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory

# Third Party
import torch

# msg ik
from curobo_msgs.srv import Fk
from geometry_msgs.msg import Pose

# cuRobo
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import get_robot_configs_path, join_path, load_yaml


class FK(Node):
    def __init__(self):
        '''
        This class is working but needs to be rework to add the config wrapper.
        '''
        super().__init__('IK')
        # sub with the name string of the robot
        self.robot_name = "ur10e"

        # service for list of poses to calculate the inverse kinematics
        self.srv_ik = self.create_service(
            Fk, '/curobo/fk_poses', self.fk_callback)

        # curobo args
        self.tensor_args = TensorDeviceType()
        config_file_path = os.path.join(get_package_share_directory(
            "curobo_ros"), 'curobo_doosan/src/m1013/m1013.yml')

        config_file = load_yaml(
            join_path(get_robot_configs_path(), config_file_path))
        urdf_file = config_file["robot_cfg"]["kinematics"][
            "urdf_path"
        ]  # Send global path starting with "/"
        base_link = config_file["robot_cfg"]["kinematics"]["base_link"]
        ee_link = config_file["robot_cfg"]["kinematics"]["ee_link"]
        # Generate robot configuration from  urdf path, base frame, end effector frame
        robot_cfg = RobotConfig.from_basic(
            urdf_file, base_link, ee_link, self.tensor_args)

        self.kin_model = CudaRobotModel(robot_cfg.kinematics)

        self.fk_init()
        self.get_logger().info("FK service up !")

    def fk_callback(self, request, response):
        # check max batch size 1000
        if len(request.joint_states) > 1000:
            self.get_logger().error('Max batch size is 1000')
            return response
        if len(request.joint_states) <= 0:
            self.get_logger().error('No positions to compute')
            return response
        qs = []
        for joint in request.joint_states:
            qs.append(list(joint.position))
        #     create tensor
        q = torch.tensor(qs, **(self.tensor_args.as_torch_dict()))
        result = self.kin_model.get_state(q)

        for index, (position, orientation) in enumerate(zip(result.ee_position.cpu().numpy(), result.ee_quaternion.cpu().numpy())):
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
        # init the kin model, the firt time it takes a while
        q = torch.rand((10, self.kin_model.get_dof()), **
                       (self.tensor_args.as_torch_dict()))
        out = self.kin_model.get_state(q)


def main(args=None):
    rclpy.init(args=args)

    fk = FK()

    rclpy.spin(fk)

    fk.destroy_node()
    rclpy.shutdown()
