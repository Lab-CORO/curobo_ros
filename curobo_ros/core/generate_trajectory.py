#!/usr/bin/env python3

import os
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
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.util_file import load_yaml, join_path, get_world_configs_path
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from .marker_publisher import MarkerPublisher
from capacinet_msg.srv import Fk

from ament_index_python.packages import get_package_share_directory


class CuRoboTrajectoryMaker(Node):
    def __init__(self):
        node_name = 'curobo_gen_traj'
        super().__init__(node_name)

        # Trajectory generation parameters
        self.declare_parameter('max_attempts', 1)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('time_dilation_factor', 0.5)

        # World configuration parameters
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('collision_activation_distance', 0.025)

        self.default_config = None
        self.j_names = None
        self.robot_cfg = None
        self.intrinsics = None
        self.cv_image = None
        self.depth_image = None
        self.marker_data = None
        self.depth = 0

        self.marker_publisher = MarkerPublisher()
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, 'trajectory', 10)

        self.motion_gen_srv = self.create_service(
            Trigger, node_name + '/update_motion_gen_config', self.set_motion_gen_config)

        # checker
        self.depth_image_received = False
        self.marker_received = False

        self.bridge = CvBridge()

        # World config parameters
        self.world_pose = [0, 0, 0, 1, 0, 0, 0]
        self.world_integrator_type = "occupancy"

        self.tensor_args = TensorDeviceType()

        # Motion generation parameters
        self.trajopt_tsteps = 32
        self.collision_checker_type = CollisionCheckerType.BLOX
        self.use_cuda_graph = True
        self.num_trajopt_seeds = 12
        self.num_graph_seeds = 12
        self.interpolation_dt = 0.03
        self.acceleration_scale = 1.0
        self.self_collision_check = True
        self.maximum_trajectory_dt = 0.25
        self.finetune_dt_scale = 1.05
        self.fixed_iters_trajopt = True
        self.finetune_trajopt_iters = 300
        self.minimize_jerk = True

        self.set_motion_gen_config(None, None)

        self.marker_sub = self.create_subscription(
            PoseStamped, 'marker_pose', self.callback_marker, 1)

        #### CAMERA INFO ####
        self.success, camera_info_sub = wait_for_message(
            CameraInfo, self, '/camera/camera/depth/camera_info', 1)
        if self.success:
            self.intrinsics = torch.tensor(
                camera_info_sub.k).view(3, 3).float()
        else:
            self.get_logger().error("Warning: no camera info received")

        self.camera_pose = Pose.from_list(
            [0.5, 0, 0.5, 0.5, -0.5, 0.5, -0.5])

        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.callback_depth, 1)

        # CUROBO FK
        self.client = self.create_client(Fk, '/curobo/fk_poses')

        self.get_logger().info("Ready to generate trajectories")

        # create time for traj gen
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.trajectory_generator)

    def set_motion_gen_config(self, request, response):
        self.world_cfg = WorldConfig.from_dict(
            {
                "blox": {
                    "world": {
                        "pose": self.world_pose,
                        "integrator_type": self.world_integrator_type,
                        "voxel_size":  self.get_parameter('voxel_size').get_parameter_value().double_value,
                    },
                },
            }
        )

        config_file_path = os.path.join(get_package_share_directory(
            "curobo_ros"), 'curobo_doosan/src/m1013/m1013.yml')
        self.robot_cfg = load_yaml(config_file_path)["robot_cfg"]
        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]

        self.world_cfg_table = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_wall.yml")))

        self.world_cfg_table.cuboid[0].pose[2] -= 0.01
        self.world_cfg.add_obstacle(self.world_cfg_table.cuboid[0])
        self.world_cfg.add_obstacle(self.world_cfg_table.cuboid[1])

        self.motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            self.world_cfg,
            self.tensor_args,
            trajopt_tsteps=self.trajopt_tsteps,
            collision_checker_type=self.collision_checker_type,
            use_cuda_graph=self.use_cuda_graph,
            num_trajopt_seeds=self.num_trajopt_seeds,
            num_graph_seeds=self.num_graph_seeds,
            interpolation_dt=self.interpolation_dt,
            collision_activation_distance=self.get_parameter(
                'collision_activation_distance').get_parameter_value().double_value,
            acceleration_scale=self.acceleration_scale,
            self_collision_check=self.self_collision_check,
            maximum_trajectory_dt=self.maximum_trajectory_dt,
            finetune_dt_scale=self.finetune_dt_scale,
            fixed_iters_trajopt=self.fixed_iters_trajopt,
            finetune_trajopt_iters=self.finetune_trajopt_iters,
            minimize_jerk=self.minimize_jerk,
        )

        self.motion_gen = MotionGen(self.motion_gen_config)

        self.get_logger().info("warming up..")

        self.motion_gen.warmup()

        self.world_model = self.motion_gen.world_collision

        self.get_logger().info("Motion generation config set")

        if response is not None:
            response.success = True
            response.message = "Motion generation config set"

        return response

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
        state = self.motion_gen.rollout_fn.compute_kinematics(
            JointState.from_position(retract_cfg.view(1, -1))
        )
        retract_pose = Pose(state.ee_pos_seq.squeeze(),
                            quaternion=state.ee_quat_seq.squeeze())
        self.start_state = JointState.from_position(retract_cfg.view(1, -1))
        self.get_logger().warn("Waiting for a pose to be published")

    def debug_voxel(self):
        # Voxel debug
        bounding = Cuboid("t", dims=[1, 1, 1.0], pose=[1, 0, 0.5, 1, 0, 0, 0])

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

        # self.update_camera()
        # end debug
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
