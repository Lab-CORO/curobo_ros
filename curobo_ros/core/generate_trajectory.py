#!/usr/bin/env python3

import torch
import numpy as np

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_srvs.srv import Trigger

from curobo_msgs.srv import TrajectoryGeneration
from curobo_msgs.action import SendTrajectory

from .config_wrapper_motion import ConfigWrapperMotion

from curobo_ros.robot.robot_context import RobotContext

from curobo_ros.cameras.camera_context import CameraContext
from curobo_ros.cameras.realsense_strategy import RealsenseStrategy

from curobo.geom.types import Cuboid
from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
from .marker_publisher import MarkerPublisher



class CuRoboTrajectoryMaker(Node):
    '''
    This class is a node that generates a trajectory for the robot to reach a given goal.
    It uses the motion generation module to generate the trajectory and the robot context to send the commands to the robot.
    It also uses the camera context to get the camera observations and the robot context to send the commands to the robot.
    The classic workflow is to set a goal from the ros2 service. The trajectory generated is show to rviz with a ghost robot. 
    To send the commands to an exeterne robot, an action is available. The feedback is percentage of advevncament of the trajectory.
    This class use a config wrapper so it has many other sesrvice available (add object, get voxel map....)
    '''
    def __init__(self):
        node_name = 'curobo_gen_traj'
        super().__init__(node_name)
        
        self.config_wrapper = ConfigWrapperMotion(self)

        # Trajectory generation parameters
        self.declare_parameter('max_attempts', 1)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('time_dilation_factor', 0.5)

        # World configuration parameters
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('collision_activation_distance', 0.025)

        # Publishers and subscribers
        self.marker_publisher = MarkerPublisher()

        self.tensor_args = TensorDeviceType()

        self.config_wrapper.set_motion_gen_config(self, None, None)

        self.camera_pose = Pose.from_list(
            [0.5, 0, 0.5, 0.5, -0.5, 0.5, -0.5])

        # get control dt
        time_dilation_factor = self.get_parameter(
                'time_dilation_factor').get_parameter_value().double_value

        # Create the robot strategy
        self.robot_context = RobotContext(self, time_dilation_factor)
        
        # create camera strategy 
        self.camera_context = CameraContext()
        self.camera_strategy = RealsenseStrategy(self)
        self.camera_context.set_camera_strategy(self.camera_strategy)
        
        # update the voxel obstacle at 30hz
        camera_update_hz = 30
        self.timer_update_camera = self.create_timer(1/camera_update_hz, self.update_camera)

        # create an action server to send the trajectory to the robot.
        self._action_server = ActionServer(
            self,
            SendTrajectory,                   # The action type
            self.get_name() + "/send_trajectrory",     # The action name
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        #  create a service server to generate the trajectory
        self.send_trajectory_srv = self.create_service(
            TrajectoryGeneration, self.get_name() + '/generate_trajectory', self.generate_trajectrory_callback, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info("Ready to generate trajectories")

    def generate_trajectrory_callback(self,  request: TrajectoryGeneration, response):
        '''
        This method generate a trajectory to the goal pose.
        '''
        # Get Robot pose
        self.start_state = JointState.from_position(torch.Tensor([self.robot_context.get_joint_pose(self)]).to(device=self.tensor_args.device))

        # Get goal pose
        self.goal_pose = Pose.from_list([
            request.target_pose.position.x, request.target_pose.position.y, request.target_pose.position.z,
            request.target_pose.orientation.x, request.target_pose.orientation.y,
            request.target_pose.orientation.z, request.target_pose.orientation.w
        ])

        # get goal pose and generate traj
        try:
            max_attempts = self.get_parameter(
                'max_attempts').get_parameter_value().integer_value
            timeout = self.get_parameter(
                'timeout').get_parameter_value().double_value
            time_dilation_factor = self.get_parameter(
                'time_dilation_factor').get_parameter_value().double_value

            result = self.motion_gen.plan_single(
                self.start_state,
                self.goal_pose,
                MotionGenPlanConfig(
                    max_attempts=max_attempts,
                    timeout=timeout,
                    time_dilation_factor=time_dilation_factor,
                ),
            )
            # new_result = result.clone()
            # new_result.retime_trajectory(
            #     time_dilation_factor, create_interpolation_buffer=True)
            traj = result.get_interpolated_plan()

            # send command to strategies:
            self.robot_context.set_command(traj.joint_names, traj.velocity.tolist(),traj.acceleration.tolist(), traj.position.tolist())

            response.success = True
            response.message = "Trajectory generated"

        except Exception as e:
            response.success = False
            response.message = "Error The trajectory could not be generated"
            self.get_logger().error(
                f"An error occurred during trajectory generation: {e}")
        return response
        
    def execute_callback(self, goal_handle):
        '''
        This method sends the trajectory to the robot (currently with a twist message). Currently there is not verifications (TODO). 
        '''
        self.robot_context.send_trajectrory(True)

        # Here we wait the message from leeloo to 
        progression = self.robot_context.get_progression()
        while progression < 1.0 and self.is_goal_up is True:
            feedback_msg = SendTrajectory.Feedback()
            feedback_msg.step_progression = self.robot_context.get_progression()
            goal_handle.publish_feedback(feedback_msg)
            progression = self.robot_context.get_progression()

        # Stop the robot and clean the command list at the end.
        self.robot_context.stop_robot()
        result_msg = SendTrajectory.Result()
        result_msg.success = True
        result_msg.message = "Goal reached"
        goal_handle.succeed()
        return result_msg

    def goal_callback(self, goal):
        self.get_logger().info("Received goal request")
        self.is_goal_up = True
        return rclpy.action.GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
        self.robot_context.stop_robot()
        self.is_goal_up = False
        self.robot_context.set_send_to_robot(False)
        self.get_logger().info("Canceling goal")
        return True

    def update_camera(self):
        '''
        This method get the camera depth map and add it in the collision map.
        TODO using a point cloud for robot segmentation with multi-cameras
        '''
        if (self.camera_context.get_depth_map() is None):
            return
        # get depth map from camera strategy
        self.world_model.decay_layer("world")
        data_camera = CameraObservation(
            depth_image=self.camera_context.get_depth_map()/1000, 
            intrinsics=self.camera_context.get_camera_intrinsic(), 
            pose=self.camera_pose)

        data_camera = data_camera.to(device=self.tensor_args.device)
        self.world_model.add_camera_frame(data_camera, "world")
        self.world_model.process_camera_frames("world", False)
        torch.cuda.synchronize()
        self.world_model.update_blox_hashes()
        self.debug_voxel()


    def debug_voxel(self):
        '''
        This method show voxels with rviz. 
        TODO change it and using voxelmap method
        '''
        # Voxel debug
        bounding = Cuboid("t", dims=[2.0, 2.0, 2.0], pose=[
                          0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0])

        voxel_size = self.get_parameter(
            'voxel_size').get_parameter_value().double_value

        voxels = self.world_model.get_voxels_in_bounding_box(
            bounding, voxel_size)

        if voxels.shape[0] > 0:
            voxels = voxels.cpu().numpy()
        self.marker_publisher.publish_markers_voxel(voxels, voxel_size)


def main(args=None):
    rclpy.init(args=args)
    node_curobo = CuRoboTrajectoryMaker()

    executor = MultiThreadedExecutor()
    executor.add_node(node_curobo)

    try:
        node_curobo.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node_curobo.get_logger().info('Keyboard interrupt, shutting down.\n')
    node_curobo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
