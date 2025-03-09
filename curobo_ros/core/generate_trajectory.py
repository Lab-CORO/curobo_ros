#!/usr/bin/env python3

import torch
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


import rclpy
from rclpy.node import Node
from .wait_for_message import wait_for_message

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState as SensorJointState
from sensor_msgs.msg import Image, CameraInfo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


from curobo_msgs.srv import Fk
from .config_wrapper_motion import ConfigWrapperMotion

from curobo_ros.robot.joint_control_strategy import JointCommandStrategy
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

        self.marker_sub = self.create_subscription(
            PoseStamped, 'marker_pose', self.callback_marker, 1)

        # Markers
        self.marker_data = None
        self.marker_received = False

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

        # CUROBO FK
        self.client = self.create_client(Fk, '/curobo/fk_poses')

        # create time for traj gen
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.trajectory_generator)

        self.get_logger().info("Ready to generate trajectories")

    def callback_marker(self, msg):
        # self.get_logger().info(f"Message reçu sur marker_pose: {msg}")
        self.marker_data = msg
        self.marker_received = True  # Met le booléen à True lorsque le message est reçu
        self.goal_pose = Pose.from_list([
            self.marker_data.pose.position.x, self.marker_data.pose.position.y, self.marker_data.pose.position.z,
            self.marker_data.pose.orientation.x, self.marker_data.pose.orientation.y,
            self.marker_data.pose.orientation.z, self.marker_data.pose.orientation.w
        ])

    def callback(self, future):
        try:
            response = future.result()
            # make sure we have an answer from the service
            assert len(response.poses) > 0
            self.marker_publisher.publish_markers_trajectory(
                response.poses)  # publish the markers to visualize them
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

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

    def get_actual_pose(self):
        # get actual pose
        retract_cfg = self.motion_gen.get_retract_config()
        self.start_state = JointState.from_position(retract_cfg.view(1, -1))

        self.get_logger().warn("Waiting for a pose to be published")

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

    def trajectory_generator(self):
        if not self.marker_received:
            return
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service is currently unavailable, waiting...')

        self.req = Fk.Request()
        self.req.joint_states = []

        self.get_actual_pose()

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
            print(result)
            new_result = result.clone()
            new_result.retime_trajectory(
                time_dilation_factor, create_interpolation_buffer=True)
            traj = result.get_interpolated_plan()

            # send command to strategies:
            self.robot_context.set_command(traj.joint_names, traj.velocity.tolist(),traj.acceleration.tolist(), traj.position.tolist())
        
            positions = traj.position.cpu().tolist()

        except Exception as e:
            self.get_logger().error(
                f"An error occurred during trajectory generation: {e}")
            positions = []

        for i, element in enumerate(positions):
            self.req.joint_states.append(SensorJointState())
            self.req.joint_states[i].position = element
            self.req.joint_states[i].name = [
                'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.callback)
        self.marker_received = False


def main(args=None):
    rclpy.init(args=args)
    curobo_test = CuRoboTrajectoryMaker()
    rclpy.spin(curobo_test)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
