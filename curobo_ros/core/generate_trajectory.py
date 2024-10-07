#!/usr/bin/env python3

import os
import torch
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from .wait_for_message import wait_for_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState as SensorJointState
from sensor_msgs.msg import Image, CameraInfo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.util_file import load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from .marker_publisher import MarkerPublisher
from capacinet_msg.srv import Fk

from ament_index_python.packages import get_package_share_directory


class CuRoboTrajectoryMaker(Node):
    def __init__(self):
        super().__init__('curobo_test')
        self.default_config = None
        self.j_names = None
        self.robot_cfg = None
        self.intrinsics = None
        self.cv_image = None
        self.depth_image = None
        self.marker_data = None
        self.depth = 0
        self.voxel_size = 0.05
        self.marker_publisher = MarkerPublisher()
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, 'joint_trajectory', 10)
        # checker
        self.camera_info_received = False
        self.depth_image_received = False
        self.marker_received = False
        self.camera_available = True

        self.bridge = CvBridge()

        self.world_cfg = WorldConfig.from_dict(

            {
                "blox": {
                    "world": {
                        "pose": [0, 0, 0, 1, 0, 0, 0],
                        "integrator_type": "occupancy",
                        "voxel_size":  self.voxel_size,
                    },
                },
            }
        )

        self.tensor_args = TensorDeviceType()

        config_file_path = os.path.join(get_package_share_directory(
            "curobo_ros"), 'curobo_doosan/src/m1013/m1013.yml')
        self.robot_cfg = load_yaml(config_file_path)["robot_cfg"]
        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]

        # world_cfg_table = WorldConfig.from_dict(load_yaml(join_path(get_world_configs_path(), "collision_wall.yml")))

        # world_cfg_table.cuboid[0].pose[2] -= 0.01
        # self.world_cfg.add_obstacle(world_cfg_table.cuboid[0])
        # self.world_cfg.add_obstacle(world_cfg_table.cuboid[1])

        # robot_file = "franka.yml"
        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            self.tensor_args,
            trajopt_tsteps=32,
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

        self.motion_gen = MotionGen(motion_gen_config)

        self.get_logger().info("warming up..")

        self.motion_gen.warmup()

        self.get_logger().info("Ready to generate trajectories")

        self.world_model = self.motion_gen.world_collision

        self.marker_sub = self.create_subscription(
            PoseStamped, 'marker_pose', self.callback_marker, 1)

    def callback_marker(self, msg):
        self.get_logger().info(f"Message reçu sur marker_pose: {msg}")
        self.marker_data = msg
        self.marker_received = True  # Met le booléen à True lorsque le message est reçu

        if self.camera_available:
            #### CAMERA INFO ####
            self.success, camera_info_sub = wait_for_message(
                CameraInfo, self, '/camera/camera/depth/camera_info')
            if self.success:
                self.callback_camera_info(camera_info_sub)
            #### DEPTH MAP ####
            self.success1, camera_depth_sub = wait_for_message(
                Image, self, '/camera/camera/depth/image_rect_raw')
            if self.success1:
                self.callback_depth(camera_depth_sub)
            else:
                self.get_logger().error("Warning: No depth map received !!")

        # CUROBO FK
        self.client = self.create_client(Fk, '/curobo/fk_poses')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service is currently unavailable, waiting...')

        self.req = Fk.Request()
        self.req.joint_states = []

        if (self.camera_info_received == True and self.depth_image_received == True) or not self.camera_available:
            self.get_logger().info('Camera info and depth image received, generating trajectory...')
            positions = self.trajectory_generator()

            for i, element in enumerate(positions):
                self.req.joint_states.append(SensorJointState())
                self.req.joint_states[i].position = element
                self.req.joint_states[i].name = [
                    'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

            self.future = self.client.call_async(self.req)
            self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'Result received with {len(response.poses)} poses')

            # make sure we have an answer from the service
            assert len(response.poses) > 0
            self.marker_publisher.publish_markers_trajectory(
                response.poses)  # publish the markers to visualize them
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def callback_camera_info(self, msg):
        if not self.camera_info_received:
            # self.intrinsics = np.reshape(msg.k, (3, 3))
            # self.get_logger().info(f"Camera intrinsics are: \n{self.intrinsics}")
            self.intrinsics = torch.tensor(msg.k).view(3, 3).float()
            print(self.intrinsics)
            self.camera_info_received = True

    def callback_depth(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.get_logger().info(
                f"Depth image casted in CV2 format looks like: {depth_img.shape}")
            # 0.8m = 255 and 0 = 0m
            depth_img_float = (depth_img.astype(np.float32))
            print("depth image")
            print(depth_img_float)
            self.depth_image = torch.from_numpy(depth_img_float).float()
            self.get_logger().info(
                f"Depth image converted to tensor: {self.depth_image.shape}")
            self.depth_image_received = True
            # self.check_and_generate_trajectory()
        except CvBridgeError as e:
            self.get_logger().error(f"An error has occurred: {e}")

    def callback_joint_trajectory(self, traj: JointState):
        """
        Convert CuRobo JointState to ROS2 JointTrajectory message with multiple points.

        Args:
            joint_state (JointState): CuRobo JointState object that may contain multiple time steps.
            time_step_sec (float): Time between each trajectory point in seconds.

        Returns:
            JointTrajectory: A ROS2 JointTrajectory message.
        """
        # Initialize JointTrajectory message
        joint_trajectory_msg = JointTrajectory()

        # Set joint names
        joint_trajectory_msg.joint_names = traj.joint_names

        # Determine the number of points (assuming position list defines this)
        num_points = len(traj.position)

        # Create a list of JointTrajectoryPoints
        for i in range(num_points):
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract the i-th positions, velocities, and accelerations
            joint_trajectory_point.positions = traj.position[i].tolist()
            joint_trajectory_point.velocities = traj.velocity[i].tolist()
            joint_trajectory_point.accelerations = traj.acceleration[i].tolist(
            )

            # Set efforts to an empty list (can be customized later)
            joint_trajectory_point.effort = []

            # # Set the time_from_start for this point (incremented by time_step_sec for each point)
            # joint_trajectory_point.time_from_start = Duration(sec=int(time_step_sec * i),
            #                                                   nanosec=int((time_step_sec * i % 1) * 1e9))

            # Add the point to the trajectory message
            joint_trajectory_msg.points.append(joint_trajectory_point)

        self.trajectory_publisher.publish(joint_trajectory_msg)

    def trajectory_generator(self):

        ##################################################################################
        ##################################################################################
        ##################################################################################
        ##################################################################################
        # -0.4999998, 0.4996018, 0.4999998, 0.5003982
        if self.camera_available:
            camera_pose = Pose.from_list([0.5, 0, 0.5, 0.5, -0.5, 0.5, -0.5])

            self.get_logger().warning(f'depth image is {self.depth_image}')

            data_camera = CameraObservation(
                depth_image=self.depth_image/1000, intrinsics=self.intrinsics, pose=camera_pose)

            self.get_logger().warning(
                f" Intrinsics are : {data_camera.intrinsics}")
            self.get_logger().warning(
                f" Depth image is : {data_camera.depth_image}")

            data_camera = data_camera.to(device=self.tensor_args.device)
            self.world_model.add_camera_frame(data_camera, "world")
            self.world_model.process_camera_frames("world", False)
            # a faire des qu un message est pub sur le topic du goal pose

        ##################################################################################
        ##################################################################################
        ##################################################################################
        ##################################################################################

        torch.cuda.synchronize()
        self.world_model.update_blox_hashes()

        bounding = Cuboid("t", dims=[10, 10, 10.0], pose=[0, 0, 0, 1, 0, 0, 0])

        voxels = self.world_model.get_voxels_in_bounding_box(
            bounding, self.voxel_size)
        print(voxels)
        if voxels.shape[0] > 0:
            # voxels = voxels[voxels[:, 2] > self.voxel_size]
            # voxels = voxels[voxels[:, 0] > 0.0]

            voxels = voxels.cpu().numpy()

        # Debug voxel map
        # ici on prend la liste des voxels et on creer des cubes de la dimension de voxel size
        self.marker_publisher.publish_markers_voxel(voxels, self.voxel_size)

        retract_cfg = self.motion_gen.get_retract_config()
        state = self.motion_gen.rollout_fn.compute_kinematics(
            JointState.from_position(retract_cfg.view(1, -1))
        )

        retract_pose = Pose(state.ee_pos_seq.squeeze(),
                            quaternion=state.ee_quat_seq.squeeze())
        start_state = JointState.from_position(retract_cfg.view(1, -1))

        self.get_logger().warn(f"Waiting for a pose to be published")
        pose_msg = self.marker_data

        goal_pose = Pose.from_list([
            pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z, pose_msg.pose.orientation.w
        ])

        self.get_logger().warn(f"Goal pose is {goal_pose}")

        start_state.position[0, 0] += 0.25
        try:
            result = self.motion_gen.plan_single(
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

            self.callback_joint_trajectory(traj)

            position = traj.position.cpu().tolist()

            self.get_logger().warning(f'Positions are {position}')
        except Exception as e:
            self.get_logger().error(
                f"An error occurred during trajectory generation: {e}")
            position = []

        return position


def main(args=None):
    rclpy.init(args=args)
    curobo_test = CuRoboTrajectoryMaker()
    rclpy.spin(curobo_test)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
