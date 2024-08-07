#!/usr/bin/env python3

import os
import torch
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from .wait_for_message import wait_for_message
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState as SensorJointState
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.robot import JointState, RobotConfig
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from nvblox_torch.datasets.realsense_dataset import RealsenseDataloader
from .marker_publisher import MarkerPublisher
from geometry_msgs.msg import Pose as PoseGeo
from capacinet_msg.srv import Fk


class CuRoboTrajectoryMaker(Node):
    def __init__(self):
        super().__init__('curobo_test')
        self.default_config = None
        self.j_names = None
        self.robot_cfg = None
        self.intrinsics = None
        self.depth_sub = None
        self.image_sub = None
        self.cv_image = None
        self.depth_image = None
        self.depth = 0
        self.camera_info_received = False

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/camera/color/image_rect_raw", self.callback_image, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/camera/color/image_rect_raw", self.callback_depth, 10)

        success, camera_info_sub = wait_for_message(CameraInfo, self, '/camera/camera/color/camera_info')
        if success:
            self.callback_camera_info(camera_info_sub)
            # self.get_logger().info(f'Result received {self.camera_info_received}')

        self.get_logger().info(f"Camera intrinsics : \n{self.intrinsics}")

        self.client = self.create_client(Fk, '/curobo/fk_poses')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service is currently unavailable, waiting...')

        self.req = Fk.Request()
        self.req.joint_states = []

        positions = self.trajectory_generator()

        for i, element in enumerate(positions):
            self.req.joint_states.append(SensorJointState())
            self.req.joint_states[i].position = element
            self.req.joint_states[i].name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.callback)
        self.marker_publisher = MarkerPublisher()


    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result received with {len(response.poses)} poses')

            assert len(response.poses) > 0   # make sure we have an answer from the service
            self.marker_publisher.publish_markers(response.poses)  # publish the markers to visualize them
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


    def callback_camera_info(self, msg):
        if not self.camera_info_received:
            self.intrinsics = np.reshape(msg.k, (3, 3))
            # self.get_logger().info(f"Camera intrinsics are: \n{self.intrinsics}")
            self.camera_info_received = True


    def callback_image(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f" An error has occurred : {e}")


    def callback_depth(self, msg):
        self.get_logger().error(f"Depth image is @@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@   {msg.data}")
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.get_logger().info(f"Depth image is @@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@#@{depth_img}")
            self.depth_image = torch.from_numpy(depth_img).float()
        except CvBridgeError as e:
            self.get_logger().erro(f" An error has occurred : {e}")


    def trajectory_generator(self):
        radius = 0.05
        act_distance = 0.4
        voxel_size = 0.05
        render_voxel_size = 0.02
        clipping_distance = 1.0  # meter
        tensor_args = TensorDeviceType()

        collision_checker_type = CollisionCheckerType.BLOX

        world_cfg = WorldConfig.from_dict(

            {
                "blox": {
                    "world": {
                        "pose": [0, 0, 0, 1, 0, 0, 0],
                        "integrator_type": "occupancy",
                        "voxel_size": 0.02,
                    },
                },
            }
        )

        tensor_args = TensorDeviceType()

        config_file_path = os.path.abspath(os.path.join(f"/home/ros2_ws/src/curobo_ros/m1013/m1013.yml"))
        self.robot_cfg = load_yaml(config_file_path)["robot_cfg"]
        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]

        world_cfg_table = WorldConfig.from_dict(load_yaml(join_path(get_world_configs_path(), "collision_wall.yml")))

        world_cfg_table.cuboid[0].pose[2] -= 0.01
        world_cfg.add_obstacle(world_cfg_table.cuboid[0])
        world_cfg.add_obstacle(world_cfg_table.cuboid[1])

        # robot_file = "franka.yml"
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            world_cfg,
            tensor_args,
            trajopt_tsteps=128,
            collision_checker_type=CollisionCheckerType.BLOX,
            use_cuda_graph=True,
            num_trajopt_seeds=12,
            num_graph_seeds=12,
            interpolation_dt=0.03,
            collision_activation_distance=0.025,
            acceleration_scale=1.0,
            self_collision_check=True,
            maximum_trajectory_dt=0.25,
            finetune_dt_scale=1.05,
            fixed_iters_trajopt=True,
            finetune_trajopt_iters=300,
            minimize_jerk=True,
        )

        motion_gen = MotionGen(motion_gen_config)

        print("warming up..")

        motion_gen.warmup()

        world_model = motion_gen.world_collision

        ##################################################################################
        ##################################################################################
        ##################################################################################
        ##################################################################################

        camera_pose = Pose.from_list([0, 0, 0.5, 1.0, 0, 0, 0])

        if self.depth_image is None:
            self.get_logger().error('No depth image received, trajectory generation aborted')

        data_camera = CameraObservation(depth_image=self.depth_image, intrinsics=self.intrinsics, pose=camera_pose)

        print(f" Intrinsics are : {data_camera.intrinsics}")
        print(f" Depth image is : {data_camera.depth_image}")

        data_camera = data_camera.to(device=tensor_args.device)
        world_model.add_camera_frame(data_camera, "world")
        world_model.process_camera_frames("world", False)

        ##################################################################################
        ##################################################################################
        ##################################################################################
        ##################################################################################

        torch.cuda.synchronize()
        world_model.update_blox_hashes()

        bounding = Cuboid("t", dims=[1, 1, 1.0], pose=[0, 0, 0, 1, 0, 0, 0])
        voxels = world_model.get_voxels_in_bounding_box(bounding, voxel_size)

        if voxels.shape[0] > 0:
            voxels = voxels[voxels[:, 2] > voxel_size]
            voxels = voxels[voxels[:, 0] > 0.0]

            voxels = voxels.cpu().numpy()

        # motion_gen.warmup(enable_graph=True, warmup_js_trajopt=js, parallel_finetune=True)
        # robot_cfg = load_yaml(join_path(get_robot_configs_path(), robot_file))["robot_cfg"]
        # robot_cfg = RobotConfig.from_dict(robot_cfg, tensor_args)

        retract_cfg = motion_gen.get_retract_config()
        state = motion_gen.rollout_fn.compute_kinematics(
            JointState.from_position(retract_cfg.view(1, -1))
        )

        retract_pose = Pose(state.ee_pos_seq.squeeze(), quaternion=state.ee_quat_seq.squeeze())
        start_state = JointState.from_position(retract_cfg.view(1, -1))
        goal_pose = Pose.from_list([0.5, 0.5, 0.5, 1.0, 0.0, 0.0, 0.0])

        start_state.position[0, 0] += 0.25

        result = motion_gen.plan_single(
            start_state,
            goal_pose,
            MotionGenPlanConfig(
                max_attempts=1,
                timeout=5,
                time_dilation_factor=0.5,
            ),
        )

        new_result = result.clone()
        new_result.retime_trajectory(0.5, create_interpolation_buffer=True)

        traj = result.get_interpolated_plan()

        position = traj.position.cpu().tolist()

        return position


def main(args=None):
    rclpy.init(args=args)
    curobo_test = CuRoboTrajectoryMaker()
    rclpy.spin(curobo_test)
    curobo_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
