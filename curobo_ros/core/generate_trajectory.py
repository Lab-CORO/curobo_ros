#!/usr/bin/env python3

import torch
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from functools import partial

import rclpy
from rclpy.node import Node
from .wait_for_message import wait_for_message

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState as SensorJointState
from sensor_msgs.msg import Image, CameraInfo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_srvs.srv import Trigger
from curobo_msgs.srv import AddObject, Fk, RemoveObject, GetVoxelGrid
from .config_wrapper import ConfigWrapper
from .config_wrapper_motion import ConfigWrapperMotion

from curobo.geom.types import Cuboid
from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
from .marker_publisher import MarkerPublisher

DEPTH_CAMERA_NAMESPACE = '/camera/camera/depth'


class CuRoboTrajectoryMaker(Node):
    def __init__(self):
        node_name = 'curobo_gen_traj'
        super().__init__(node_name)

        self.config_wrapper = ConfigWrapperMotion()

        # Trajectory generation parameters
        self.declare_parameter('max_attempts', 1)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('time_dilation_factor', 0.5)

        # World configuration parameters
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('collision_activation_distance', 0.025)

        # Publishers and subscribers
        self.marker_publisher = MarkerPublisher()
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, 'trajectory', 10)

        self.marker_sub = self.create_subscription(
            PoseStamped, 'marker_pose', self.callback_marker, 1)

        # Markers
        self.marker_data = None
        self.marker_received = False

        # Image processing
        self.depth_image = None
        self.bridge = CvBridge()
        self.tensor_args = TensorDeviceType()

        self.config_wrapper.set_motion_gen_config(self, None, None)

        # Camera info
        self.success, camera_info_sub = wait_for_message(
            CameraInfo, self, DEPTH_CAMERA_NAMESPACE + '/camera_info', 1)

        self.intrinsics = None
        if self.success:
            self.intrinsics = torch.tensor(
                camera_info_sub.k).view(3, 3).float()
        else:
            self.get_logger().error("Warning: no camera info received")

        self.camera_pose = Pose.from_list(
            [0.5, 0, 0.5, 0.5, -0.5, 0.5, -0.5])

        self.sub_depth = self.create_subscription(
            Image, DEPTH_CAMERA_NAMESPACE + 'image_rect_raw', self.callback_depth, 1)

        # CUROBO FK
        self.client = self.create_client(Fk, '/curobo/fk_poses')

        self.get_logger().info("Ready to generate trajectories")

        # create time for traj gen
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.trajectory_generator)

    def callback_depth(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            depth_img_float = (depth_img.astype(np.float32))
            self.depth_image = torch.from_numpy(depth_img_float).float()

            self.update_camera()

        except CvBridgeError as e:
            self.get_logger().error(f"An error has occurred: {e}")

    def callback_joint_trajectory(self, traj: JointState, time_step: float):
        """
        Convert CuRobo JointState to ROS2 JointTrajectory message with multiple points.

        Args:
            joint_state (JointState): CuRobo JointState object that may contain multiple time steps.
            time_step (float): Time between each trajectory point in seconds.

        Returns:
            JointTrajectory: A ROS2 JointTrajectory message.
        """
        # Initialize JointTrajectory message
        joint_trajectory_msg = JointTrajectory()

        # Set joint names
        joint_trajectory_msg.joint_names = traj.joint_names

        # Create a list of JointTrajectoryPoints for every position in the JointState
        for i in range(len(traj.position)):
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract the i-th positions, velocities, and accelerations
            joint_trajectory_point.positions = traj.position[i].tolist()
            joint_trajectory_point.velocities = traj.velocity[i].tolist()
            joint_trajectory_point.accelerations = traj.acceleration[i].tolist(
            )

            # Set efforts to an empty list (can be customized later)
            joint_trajectory_point.effort = []

            # Set the time_from_start for this point (incremented by time_step for each point)
            joint_trajectory_point.time_from_start = Duration(sec=int(time_step * i),
                                                              nanosec=int((time_step * i % 1) * 1e9))

            # Add the point to the trajectory message
            joint_trajectory_msg.points.append(joint_trajectory_point)

        self.trajectory_publisher.publish(joint_trajectory_msg)

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
        self.world_model.decay_layer("world")
        data_camera = CameraObservation(
            depth_image=self.depth_image/1000, intrinsics=self.intrinsics, pose=self.camera_pose)
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
            new_result = result.clone()
            new_result.retime_trajectory(
                time_dilation_factor, create_interpolation_buffer=True)
            traj = result.get_interpolated_plan()
            self.callback_joint_trajectory(traj, result.interpolation_dt)
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
