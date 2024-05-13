import rclpy
import std_msgs.msg
from sensor_msgs.msg import JointState
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import numpy as np

# Third Party
import torch

# cuRobo
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

# msg ik
from data_generation.srv import Ik


# This class use curobo to generate the inverse kinematics of the robot

class IK(Node):
    
    def __init__(self):
        super().__init__('IK')
        # sub with the name string of the robot
        # self.subscription = self.create_subscription( String, '/curobo/robot_name', self.robot_name_callback, 10)
        # self.subscription
        self.robot_name = "ur10e"

        # service for list of poses to calculate the inverse kinematics
        self.srv_ik = self.create_service( Ik, '/curobo/ik_poses', self.ik_callback)

        # curobo args
        self.tensor_args = TensorDeviceType()

        config_file = load_yaml(join_path(get_robot_configs_path(), "ur10e.yml"))
        urdf_file = config_file["robot_cfg"]["kinematics"][
            "urdf_path"
        ]  # Send global path starting with "/"
        base_link = config_file["robot_cfg"]["kinematics"]["base_link"]
        ee_link = config_file["robot_cfg"]["kinematics"]["ee_link"]
        robot_cfg = RobotConfig.from_basic(urdf_file, base_link, ee_link, self.tensor_args)

        ik_config = IKSolverConfig.load_from_robot_config(
            robot_cfg,
            None,
            rotation_threshold=0.05,
            position_threshold=0.005,
            num_seeds=20,
            self_collision_check=False,
            self_collision_opt=False,
            tensor_args=self.tensor_args,
            use_cuda_graph=True,
        )
        self.ik_solver = IKSolver(ik_config)

        self.ik_init()


    # def robot_name_callback(self, msg):
    #     self.robot_name = msg.data

    def ik_callback(self, request, response):

        # check limit of poses 1000 poses
        if len(request.poses) > 1000:
            return response
        # get the poses
        poses = request.poses
        # print(poses[0].position.x)
        positions = []
        orientations = []
        for pose in poses:
            positions.append([pose.position.x, pose.position.y, pose.position.z])
            orientations.append([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        # Convert lists to tensors
        position_tensor = torch.tensor(positions)
        orientation_tensor = torch.tensor(orientations)

        # # poses.position to tensor
        # positions = torch.tensor([pose.position for pose in poses], **self.tensor_args)
        # # poses.orientation to tensor
        # orientations = torch.tensor([pose.orientation for pose in poses], **self.tensor_args)

        goal = Pose(position_tensor, orientation_tensor)

        result = self.ik_solver.solve_batch(goal)
        torch.cuda.synchronize()
        # convert result joins angles to sensor_msgs/JointState
        joints_results = []
        for index, j in enumerate(result.solution.cpu().numpy()):
            joint = JointState()
            joint.position = j[0].tolist()
            # joints_results.append(joint)

            # joint state valide to Bool list
            res = std_msgs.msg.Bool()
            res.data = bool(result.success.cpu().numpy()[index][0])
            # print(res)

            response.joint_states_valid.append(res)
            response.joint_states.append(joint)
        # response.joint_states_valid = result.success.cpu().numpy()[:,0]

        # response.joint_angles = result.joint_angles.cpu().numpy().tolist()
        return response
        
    def ik_init(self):
        q_sample = self.ik_solver.sample_configs(50)
        # print(q_sample)
        kin_state = self.ik_solver.fk(q_sample)
        goal = Pose(kin_state.ee_position, kin_state.ee_quaternion)

        # st_time = time.time()
        result = self.ik_solver.solve_batch(goal)
        torch.cuda.synchronize()
        # print sulutions in a numpy array
        join_results = []
        for j in result.solution.cpu().numpy():
            joint = JointState()
            joint.position = j[0].tolist()
            join_results.append(joint)
        # print the nb of solutions
        # print(join_results)
        print(joint)
        # print(result.solution.cpu().numpy().tolist())
        # print the nb of solutions
        # print(len(result.solution.cpu().numpy()))
        # print(result.success.cpu().numpy().tolist())

    def add_collisions(self):
        return True


def main(args=None):
    rclpy.init(args=args)

    ik = IK()

    rclpy.spin(ik)

    ik.destroy_node()
    rclpy.shutdown()