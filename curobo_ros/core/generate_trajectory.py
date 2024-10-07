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
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import JointState as SensorJointState
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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

from ament_index_python.packages import get_package_share_directory


class CuRoboTrajectoryMaker(Node):
    def __init__(self):
        super().__init__('curobo_gen_traj')
        self.default_config = None
        self.j_names = None
        self.robot_cfg = None
        self.intrinsics = None
        self.cv_image = None
        self.depth_image = None
        self.marker_data = None
        self.depth = 0
        self.voxel_size = 0.02
        self.marker_publisher = MarkerPublisher()
        #checker
        self.depth_image_received = False
        self.marker_received = False

        # self.goal_pose = None

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

        config_file_path = os.path.join(get_package_share_directory("curobo_ros"), 'curobo_doosan/src/m1013/m1013.yml')
        self.robot_cfg = load_yaml(config_file_path)["robot_cfg"]
        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]

        world_cfg_table = WorldConfig.from_dict(load_yaml(join_path(get_world_configs_path(), "collision_wall.yml")))

        world_cfg_table.cuboid[0].pose[2] -= 0.01
        self.world_cfg.add_obstacle(world_cfg_table.cuboid[0])
        self.world_cfg.add_obstacle(world_cfg_table.cuboid[1])

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
        self.tensor_args = TensorDeviceType()

        self.motion_gen = MotionGen(motion_gen_config)

        self.get_logger().info("warming up..")

        self.motion_gen.warmup()

        self.get_logger().info("Ready to generate trajectories")

        self.world_model = self.motion_gen.world_collision

        self.marker_sub = self.create_subscription(PoseStamped, 'marker_pose', self.callback_marker, 1)

        #### CAMERA INFO ####
        self.success, camera_info_sub = wait_for_message(CameraInfo, self, '/camera/camera/depth/camera_info')
        if self.success:
            self.intrinsics = torch.tensor(camera_info_sub.k).view(3, 3).float()
        else:
            self.get_logger().error("Warning: camera info recieved !!")
        
        self.camera_pose = Pose.from_list([0.5, 0, 0.5, 0.5, -0.5, 0.5, -0.5])

        self.sub_depth = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.callback_depth, 1)

        #### CUROBO FK
        self.client = self.create_client(Fk, '/curobo/fk_poses')

        #create time for traj gen
        timer_period =0.1 # seconds
        self.timer = self.create_timer(timer_period, self.trajectory_generator)


    def callback_depth(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            depth_img_float = (depth_img.astype(np.float32))
            self.depth_image = torch.from_numpy(depth_img_float).float()

            self.update_camera()

        except CvBridgeError as e:
            self.get_logger().error(f"An error has occurred: {e}")
        


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
            assert len(response.poses) > 0   # make sure we have an answer from the service
            self.marker_publisher.publish_markers_trajectory(response.poses)  # publish the markers to visualize them
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


    def update_camera(self):
        self.world_model.decay_layer("world")
        data_camera = CameraObservation(depth_image=self.depth_image/1000, intrinsics=self.intrinsics, pose=self.camera_pose)
        data_camera = data_camera.to(device=self.tensor_args.device)
        self.world_model.add_camera_frame(data_camera, "world")
        self.world_model.process_camera_frames("world", False)
        torch.cuda.synchronize()
        self.world_model.update_blox_hashes()
        self.debug_voxel()

    def get_actual_pose(self):
        #### get actual pose
        retract_cfg = self.motion_gen.get_retract_config()
        state = self.motion_gen.rollout_fn.compute_kinematics(
            JointState.from_position(retract_cfg.view(1, -1))
        )
        retract_pose = Pose(state.ee_pos_seq.squeeze(), quaternion=state.ee_quat_seq.squeeze())
        self.start_state = JointState.from_position(retract_cfg.view(1, -1))
        self.get_logger().warn(f"Waiting for a pose to be published")

    def update_goal_pose(self):
        pose_msg = self.marker_data
        

    def debug_voxel(self):
        #### Voxel debug
        bounding = Cuboid("t", dims=[1, 1, 1.0], pose=[1, 0, 0.5, 1, 0, 0, 0])
        voxels = self.world_model.get_voxels_in_bounding_box(bounding, self.voxel_size)
        # print(len(voxels))
        if voxels.shape[0] > 0:
            voxels = voxels.cpu().numpy()
        self.marker_publisher.publish_markers_voxel(voxels, self.voxel_size)
        

    def trajectory_generator(self):
        if not self.marker_received:
            return
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service is currently unavailable, waiting...')

        self.req = Fk.Request()
        self.req.joint_states = []

        # self.update_camera()
        #end debug
        self.get_actual_pose()
        
        #### get goal pose and genereate traj
        
        # self.start_state.position[0, 0] += 0.25  ## why ??
        try:
            result = self.motion_gen.plan_single(
                self.start_state,
                self.goal_pose,
                MotionGenPlanConfig(
                    max_attempts=1,
                    timeout=5,
                    time_dilation_factor=0.5,
                ),
            )
            new_result = result.clone()
            new_result.retime_trajectory(0.5, create_interpolation_buffer=True)
            traj = result.get_interpolated_plan()
            positions = traj.position.cpu().tolist()
        # self.get_logger().warning(f'Positions are {position}')
        except:
            positions = []
        
        for i, element in enumerate(positions):
            self.req.joint_states.append(SensorJointState())
            self.req.joint_states[i].position = element
            self.req.joint_states[i].name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

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