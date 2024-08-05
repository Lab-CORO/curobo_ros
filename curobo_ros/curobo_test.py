#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState as SensorJointState
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, Float64MultiArray
import torch
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


class CuRoboTestFk(Node):
    def __init__(self):
        super().__init__('curobo_test')
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


            self.get_logger().info("#################################")
            self.get_logger().info("The initial position is {}".format(positions[0]))
            self.get_logger().info("#################################")

            print(len(positions))
            self.get_logger().info("#################################")
            self.get_logger().info("The final position is {}".format(positions[len(positions)-1]))
            self.get_logger().info("#################################")

    
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.callback)


        self.marker_publisher = MarkerPublisher()

    def trajectory_generator(self):
        # Standard Library
        # PLOT = True
        radius = 0.05
        act_distance = 0.4
        voxel_size = 0.05
        render_voxel_size = 0.02
        clipping_distance = 0.7
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

        self.robot_cfg = load_yaml(config_file_path)[
            "robot_cfg"]

        self.j_names = self.robot_cfg["kinematics"]["cspace"]["joint_names"]

        # print(j_names)
        self.default_config = self.robot_cfg["kinematics"]["cspace"]["retract_config"]

        print(type(self.default_config))
        # print(default_config)

        world_cfg_table = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_wall.yml"))
        )

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
        # realsense_data = RealsenseDataloader(clipping_distance_m=clipping_distance)
        # data = realsense_data.get_data()

        # camera_pose = Pose.from_list([0, 0, 0, 0.707, 0.707, 0, 0])

        # data_camera = CameraObservation(  # rgb_image = data["rgba_nvblox"],
        #     depth_image=data["depth"], intrinsics=data["intrinsics"], pose=camera_pose
        # )

        # data_camera = data_camera.to(device=tensor_args.device)
        # world_model.add_camera_frame(data_camera, "world")
        # world_model.process_camera_frames("world", False)
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



    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result received with {len(response.poses)} poses')
            for i, pose in enumerate(response.poses):

                # self.get_logger().info(f'Pose {i}: position={pose.position}, orientation={pose.orientation}')

                if i == 0:
                    self.get_logger().info("#################################")
                    self.get_logger().info("The initial position is {}".format(pose.position))
                    self.get_logger().info("#################################")
                if i == len(response.poses) - 1:
                    self.get_logger().info("#################################")
                    self.get_logger().info("The final position is {}".format(pose.position))
                    self.get_logger().info("#################################")


            assert len(response.poses) > 0
            self.marker_publisher.publish_markers(response.poses)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    curobo_test = CuRoboTestFk()
    rclpy.spin(curobo_test)
    curobo_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
