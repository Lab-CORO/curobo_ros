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



# msg ik
from .config_wrapper_motion import ConfigWrapperIK
from curobo_ros.robot.robot_context import RobotContext




# This class use curobo to generate the inverse kinematics of the robot

class IK(Node):
    '''
    
    '''
    def __init__(self):
        node_name = 'curobo_ik'
        super().__init__(node_name)

        
        self.declare_parameter('voxel_size', 0.5)
        voxel_size = self.get_parameter(
                'voxel_size').get_parameter_value().double_value
        self.declare_parameter('init_btch_size', 1000)
        voxel_size = self.get_parameter(
                'init_btch_size').get_parameter_value().double_value
        
        # curobo args
        self.tensor_args = TensorDeviceType()

        self.size_init = 5500
        self.robot_context = RobotContext(self, 0.03)
        self.config_wrapper = ConfigWrapperIK(self, self.robot_context)

    def ik_callback(self, request, response):

        res, ik_result = self.get_ik([request.pose])
        if (res == False):
            response.success = res 
            return response

        for index, j in enumerate(ik_result.solution.cpu().numpy()):
            joint = JointState()
            joint.position = j[0].tolist()

            # joint state valide to Bool list
            res = std_msgs.msg.Bool()
            res.data = bool(ik_result.success.cpu().numpy()[index][0])

            response.joint_states_valid =res
            response.joint_states = joint
            response.success = True
        return response

    def get_ik(self, poses):
        '''
        Get a list of poses (geometry...) 
        return a tuple (result(bool), [joint_states])
        '''
        if len(poses) == 0:
            self.get_logger().info("0 pose requested")
            # self.get_logger().info(len(request.poses))
            return False, []
        # check limit of poses 1000 poses
        try:
            if len(poses) != self.size_init:
                # print("new size")
                self.tensor_args = TensorDeviceType()
                self.config_wrapper.set_ik_gen_config(self, None, None)

                self.size_init = len(poses)
            
            self.ik_init()
        except:
            # response.error_msg = "May be in collision"
            self.size_init = 0 # Init could not be done also the size is set to 0.
            return False, []
        # get the poses

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

        try:
            result = self.ik_solver.solve_batch(goal)
        except:
            try:
                self.ik_init()
                result = self.ik_solver.solve_batch(goal)
            except:
                self.size_init = 0 # Init could not be done also the size is set to 0.
                return False, []
        torch.cuda.synchronize()


        return True, result


    def ik_batch_callback(self, request, response):


        res, ik_result = self.get_ik(request.poses)

        if (res == False):
            response.success = res 
            return response

        for index, j in enumerate(ik_result.solution.cpu().numpy()):
            joint = JointState()
            joint.position = j[0].tolist()

            # joint state valide to Bool list
            res = std_msgs.msg.Bool()
            res.data = bool(ik_result.success.cpu().numpy()[index][0])

            response.joint_states_valid.append(res)
            response.joint_states.append(joint)
            response.success = True
        return response


    def ik_init(self):
        q_sample = self.ik_solver.sample_configs(self.size_init)
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
