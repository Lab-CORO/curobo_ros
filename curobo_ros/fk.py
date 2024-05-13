import rclpy
import std_msgs.msg
from sensor_msgs.msg import JointState
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import numpy as np

# Third Party
import torch

# msg ik
from data_generation.srv import Fk
from geometry_msgs.msg import Pose

# cuRobo
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import get_robot_configs_path, join_path, load_yaml

class FK(Node):
    def __init__(self):
        super().__init__('IK')
        # sub with the name string of the robot
        # self.subscription = self.create_subscription( String, '/curobo/robot_name', self.robot_name_callback, 10)
        # self.subscription
        self.robot_name = "ur10e"

        # service for list of poses to calculate the inverse kinematics
        self.srv_ik = self.create_service(Fk, '/curobo/fk_poses', self.fk_callback)

        # curobo args
        self.tensor_args = TensorDeviceType()

        config_file = load_yaml(join_path(get_robot_configs_path(), "ur10e.yml"))
        urdf_file = config_file["robot_cfg"]["kinematics"][
            "urdf_path"
        ]  # Send global path starting with "/"
        base_link = config_file["robot_cfg"]["kinematics"]["base_link"]
        ee_link = config_file["robot_cfg"]["kinematics"]["ee_link"]
        # Generate robot configuration from  urdf path, base frame, end effector frame
        robot_cfg = RobotConfig.from_basic(urdf_file, base_link, ee_link, self.tensor_args)

        self.kin_model = CudaRobotModel(robot_cfg.kinematics)

        self.fk_init()

    def fk_callback(self, request, response):
        # check max batch size 1000
        if len(request.joint_states) > 1000:
            self.get_logger().error('Max batch size is 1000')
            return response
        qs = []
        print(request.joint_states)
        for joint in request.joint_states:
            qs.append(list(joint.position))
        print(qs)
        #     create tensor
        q = torch.tensor(qs, **(self.tensor_args.as_torch_dict()))
        print(q)
        result = self.kin_model.get_state(q)

        for index, (position, orientation) in enumerate(zip(result.ee_position.cpu().numpy(), result.ee_quaternion.cpu().numpy())):
            pose = Pose()
            print(position)
            print(orientation)
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
        q = torch.rand((10, self.kin_model.get_dof()), **(self.tensor_args.as_torch_dict()))
        out = self.kin_model.get_state(q)
        print(out)



def main(args=None):
    rclpy.init(args=args)

    fk = FK()

    rclpy.spin(fk)

    fk.destroy_node()
    rclpy.shutdown()


