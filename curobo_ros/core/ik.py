import time

import rclpy
import std_msgs.msg
from sensor_msgs.msg import JointState
from rclpy.node import Node

# Third Party
import torch

# cuRobo
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose

from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig


# msg ik
from curobo_msgs.srv import Ik
from .config_wrapper_motion import ConfigWrapperIK



# This class use curobo to generate the inverse kinematics of the robot

class IK(Node):

    def __init__(self):
        node_name = 'curobo_ik'
        super().__init__(node_name)

        self.declare_parameter('voxel_size', 0.5)
 
        # curobo args
        self.tensor_args = TensorDeviceType()

        self.size_init = 50
        self.config_wrapper = ConfigWrapperIK(self)
        # self.config_wrapper.set_ik_gen_config(self, None, None)
        # self.ik_init()

        # service for list of poses to calculate the inverse kinematics
        

    def ik_callback(self, request, response):

        # check limit of poses 1000 poses
        if len(request.poses) != self.size_init:
            # print("new size")
            self.tensor_args = TensorDeviceType()
            self.config_wrapper.set_ik_gen_config(self, None, None)
            # ik_config = IKSolverConfig.load_from_robot_config(
            #     self.robot_cfg,
            #     self.world_cfg,
            #     rotation_threshold=0.05,
            #     position_threshold=0.005,
            #     num_seeds=20,
            #     self_collision_check=True,
            #     self_collision_opt=True,
            #     tensor_args=self.tensor_args,
            #     use_cuda_graph=True,
            # )
            # self.ik_solver = IKSolver(ik_config)
            self.size_init = len(request.poses)
            self.ik_init()
        # get the poses
        poses = request.poses
        # print(poses[0])

        positions = []
        orientations = []
        for pose in poses:
            positions.append(
                [pose.position.x, pose.position.y, pose.position.z])
            orientations.append(
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        # Convert lists to tensors
        position_tensor = torch.tensor(positions)
        # print(position_tensor)
        orientation_tensor = torch.tensor(orientations)

        goal = Pose(position_tensor, orientation_tensor)
        # print(goal.position)

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
        q_sample = self.ik_solver.sample_configs(self.size_init)
        # print(q_sample)
        kin_state = self.ik_solver.fk(q_sample)
        goal = Pose(kin_state.ee_position, kin_state.ee_quaternion)

        result = self.ik_solver.solve_batch(goal)
        torch.cuda.synchronize()

        self.get_logger().info("Init done")

    def add_collisions(self):
        return True


def main(args=None):
    rclpy.init(args=args)

    ik = IK()

    rclpy.spin(ik)

    ik.destroy_node()
    rclpy.shutdown()
