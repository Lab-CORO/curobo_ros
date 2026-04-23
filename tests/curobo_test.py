#!/usr/bin/env python3
"""
Integration smoke test for curobo_ros — v2.

Rewritten for cuRobo v0.8.0:
- `WorldConfig` -> `Scene`
- `TensorDeviceType` -> `DeviceCfg`
- `MotionGen` / `MotionGenConfig` / `MotionGenPlanConfig` -> `MotionPlanner` /
  `MotionPlannerCfg` (+ per-call kwargs on `plan_pose`)
- `CollisionCheckerType` / `RobotWorld` -> removed (v2 Scene handles collision
  backends natively)
- Blox `add_camera_frame` / `process_camera_frames` / `update_blox_hashes`
  -> `mapper.integrate(CameraObservation)` + `mapper.compute_esdf()`
- `Pose` (goal) -> `ToolPose` wrapped in `GoalToolPose`
- `plan_single(..., MotionGenPlanConfig(...))` -> `plan_pose(start, goal, **kwargs)`
"""

import os

import numpy as np
import torch
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose as PoseGeo, PoseStamped
from sensor_msgs.msg import Image, CameraInfo, JointState as SensorJointState
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, Header

from curobo.scene import Scene, Cuboid
from curobo.types import CameraObservation, DeviceCfg, GoalToolPose, JointState, Pose, ToolPose
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg

from curobo_msgs.srv import Fk
from ..curobo_ros.interfaces.wait_for_message import wait_for_message
from ..curobo_ros.interfaces.marker_publisher import MarkerPublisher


class CuRoboTrajectoryMaker(Node):
    def __init__(self):
        super().__init__('curobo_test')

        # State
        self.j_names = None
        self.intrinsics = None
        self.depth_image = None
        self.marker_data = None

        self.camera_info_received = False
        self.depth_image_received = False
        self.marker_received = False

        self.bridge = CvBridge()
        self.tensor_args = DeviceCfg(device='cuda', dtype=torch.float32)

        # v2: Scene starts empty; runtime camera data is integrated through Mapper.
        self.scene = Scene()

        # A couple of static walls for a minimal collision scene.
        self.scene.add_obstacle(Cuboid(name='floor',
                                       dims=[2.0, 2.0, 0.02],
                                       pose=[0, 0, -0.01, 1, 0, 0, 0]))
        self.scene.add_obstacle(Cuboid(name='back_wall',
                                       dims=[2.0, 0.02, 2.0],
                                       pose=[0, 1.0, 1.0, 1, 0, 0, 0]))

        robot_yaml = os.path.abspath('/home/ros2_ws/src/curobo_ros/m1013/m1013.yml')

        # v2: single factory call replaces MotionGenConfig.load_from_robot_config(...).
        # Per-call tuning (attempts, timeout, etc.) moves out of the config.
        cfg = MotionPlannerCfg.create(
            robot=robot_yaml,
            scene_model=self.scene,
            num_trajopt_seeds=12,
            num_graph_seeds=12,
            interpolation_dt=0.03,
            collision_activation_distance=0.025,
            self_collision_check=True,
            use_cuda_graph=True,
        )
        self.motion_planner = MotionPlanner(cfg)

        self.get_logger().info('warming up...')
        self.motion_planner.warmup()

        # v2: the planner owns the scene model; camera frames flow through the node
        # mapper (see `mapper.integrate` below). No direct `world_collision` handle.
        self.mapper = getattr(self.motion_planner, 'mapper', None)

        # Joint names / retract config — read directly from the YAML for the test.
        self.j_names = cfg.robot_cfg.kinematics.joint_names
        self.default_config = cfg.robot_cfg.kinematics.cspace.retract_config.tolist()

        self.marker_sub = self.create_subscription(
            PoseStamped, 'marker_pose', self.callback_marker, 10)

    def callback_marker(self, msg):
        self.get_logger().info(f'marker_pose received: {msg}')
        self.marker_data = msg
        self.marker_received = True

        # Camera info
        ok, info_msg = wait_for_message(CameraInfo, self, '/camera/camera/color/camera_info')
        if ok:
            self.callback_camera_info(info_msg)

        # Depth image
        ok, depth_msg = wait_for_message(Image, self, '/camera/camera/depth/image_rect_raw')
        if ok:
            self.callback_depth(depth_msg)

        # Forward to /curobo/fk_poses service
        self.client = self.create_client(Fk, '/curobo/fk_poses')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('fk_poses service unavailable, waiting...')

        req = Fk.Request()
        req.joint_states = []

        if self.camera_info_received and self.depth_image_received:
            self.get_logger().info('Camera and depth ready, planning trajectory...')
            positions = self.trajectory_generator()

            for i, element in enumerate(positions):
                js = SensorJointState()
                js.position = element
                js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
                req.joint_states.append(js)

            self.future = self.client.call_async(req)
            self.future.add_done_callback(self.callback)
            self.marker_publisher = MarkerPublisher()

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received {len(response.poses)} FK poses')
            assert len(response.poses) > 0
            self.marker_publisher.publish_markers(response.poses)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def callback_camera_info(self, msg):
        if not self.camera_info_received:
            self.intrinsics = torch.tensor(msg.k).view(3, 3).float()
            self.camera_info_received = True

    def callback_depth(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.depth_image = torch.from_numpy(depth_img.astype(np.float32)).float()
            self.depth_image_received = True
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge: {e}')

    def trajectory_generator(self):
        # Camera pose in robot base frame (hand-tuned for this test rig).
        camera_pose = Pose.from_list(
            [0.5, 1.0, 0.5, -0.422551, 0.878444, 0.138406, 0.175016])

        obs = CameraObservation(
            depth_image=self.depth_image,
            intrinsics=self.intrinsics,
            pose=camera_pose,
        ).to(device=self.tensor_args.device)

        # v2: single call replaces add_camera_frame/process_camera_frames/update_blox_hashes.
        if self.mapper is not None:
            self.mapper.integrate(obs)
            torch.cuda.synchronize()

        # Build start + goal.
        start_state = JointState.from_position(
            torch.tensor([self.default_config],
                         dtype=self.tensor_args.dtype,
                         device=self.tensor_args.device))

        pose_msg = self.marker_data
        goal = GoalToolPose(
            tool_pose=ToolPose.from_list([
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
                pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y, pose_msg.pose.orientation.z,
            ])
        )
        self.get_logger().info(f'Goal ToolPose: {goal}')

        try:
            # v2: per-call params are kwargs on plan_pose.
            result = self.motion_planner.plan_pose(
                start_state,
                goal,
                max_attempts=1,
                timeout=5.0,
                time_dilation_factor=0.5,
            )
            traj = result.get_interpolated_plan()
            return traj.position.cpu().tolist()
        except Exception as e:
            self.get_logger().error(f'plan_pose failed: {e}')
            return []


def main(args=None):
    rclpy.init(args=args)
    node = CuRoboTrajectoryMaker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
