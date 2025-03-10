#!/usr/bin/env python3

import torch
import numpy as np

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse

from curobo_msgs.action import TrajectoryGeneration

from .config_wrapper_motion import ConfigWrapperMotion

from curobo_ros.robot.robot_context import RobotContext
from curobo_ros.robot.doosan_strategy import DoosanControl

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
        self.robot_context = RobotContext()
        self.robot_strategy = DoosanControl(self, time_dilation_factor)
        self.robot_context.set_robot_strategy(self.robot_strategy, self, time_dilation_factor)
        
        # create camera strategy 
        self.camera_context = CameraContext()
        self.camera_strategy = RealsenseStrategy(self)
        self.camera_context.set_camera_strategy(self.camera_strategy)
        
        # update the voxel obstacle at 30hz
        camera_update_hz = 30
        self.timer_update_camera = self.create_timer(1/camera_update_hz, self.update_camera)

        # create an action server
        self._action_server = ActionServer(
            self,
            TrajectoryGeneration,                   # The action type
            self.get_name() + "/generate_trajectrory",     # The action name
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("Ready to generate trajectories")

    def execute_callback(self, goal_handle):
        # if not self.marker_received:
        #     return

        # Get Robot pose
        self.start_state = JointState.from_position(torch.Tensor([self.robot_context.get_joint_pose(self)]).to(device=self.tensor_args.device))

        # Get goal pose
        self.goal_pose = Pose.from_list([
            goal_handle.request.target_pose.position.x, goal_handle.request.target_pose.position.y, goal_handle.request.target_pose.position.z,
            goal_handle.request.target_pose.orientation.x, goal_handle.request.target_pose.orientation.y,
            goal_handle.request.target_pose.orientation.z, goal_handle.request.target_pose.orientation.w
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
            new_result = result.clone()
            new_result.retime_trajectory(
                time_dilation_factor, create_interpolation_buffer=True)
            traj = result.get_interpolated_plan()

            # send command to strategies:
            self.robot_context.set_command(traj.joint_names, traj.velocity.tolist(),traj.acceleration.tolist(), traj.position.tolist())

        except Exception as e:
            self.get_logger().error(
                f"An error occurred during trajectory generation: {e}")

        # self.marker_received = False
        start_time = time.time()
        # loop with dt timer

        # while self.robot_context.get_progression != 1.0 and self.is_goal_up is True:
        #     # sleep as dt
        #     # time.sleep(time_dilation_factor)
        #     if (time.time() - start_time) > time_dilation_factor:

        #         feedback_msg = TrajectoryGeneration.Feedback()
        #         feedback_msg.step_progression = self.robot_context.get_progression()
        #         goal_handle.publish_feedback(feedback_msg)
        #         self.get_logger().info(f"Progression: {self.robot_context.get_progression()}")
        #         start_time = time.time()
        result_msg = TrajectoryGeneration.Result()
        result_msg.success = True
        result_msg.message = "Aucun scan n'a été acquis."
        goal_handle.succeed()
        return result_msg

    def goal_callback(self, goal):
        self.get_logger().info("Received goal request")
        self.is_goal_up = True
        return rclpy.action.GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
        self.robot_context.stop_robot()
        self.is_goal_up = False
        self.get_logger().info("Canceling goal")
        return True

    def update_camera(self):
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
    curobo_test = CuRoboTrajectoryMaker()
    rclpy.spin(curobo_test)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
