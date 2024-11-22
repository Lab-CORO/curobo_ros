#ROS2
import rclpy
import sensor_msgs
from rclpy.node import Node
from std_msgs.msg import String
# Standard Library
import time

# Third Party
import imageio
import numpy as np
import torch
import torch.autograd.profiler as profiler
from nvblox_torch.datasets.mesh_dataset import MeshDataset
from torch.profiler import ProfilerActivity, profile, record_function
import cv2
from cv_bridge import CvBridge

# CuRobo
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.geom.types import PointCloud, WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState
from curobo.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml
from curobo.wrap.model.robot_segmenter import RobotSegmenter


class RobotSegmentation(Node):
    '''
    node of the segmentation of the robot which have for objectives to
    subscribe to fhe fuse pointcloud of the two camera and mask the image
    of the robot so that we can see the working environnement without 
    the robot.

    '''
    def __init__(self):
        super().__init__('robot_segmentation')
        #create publisher of the robot segmentation
        self.publisher_ = self.create_publisher(String, 'mask_pointcloud', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        #create the subcriber to the fused pointcloud
        self.subscription = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            '/fused_pointcloud',        #topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def timer_callback(self):
        msg = sensor_msgs.PointCloud2 #mettre le pointcloud apres maskage
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def mask_image(robot_file="ur5e.yml"):
    save_debug_data = False
    write_pointcloud = False
    # create robot segmenter:
    tensor_args = TensorDeviceType()

    curobo_segmenter = RobotSegmenter.from_robot_file(
        robot_file, collision_sphere_buffer=0.01, distance_threshold=0.05, use_cuda_graph=True
    )

    mesh_dataset, q_js = create_render_dataset(robot_file, write_pointcloud, n_frames=20)

    if save_debug_data:
        visualize_scale = 10.0
        data = mesh_dataset[0]
        cam_obs = CameraObservation(
            depth_image=tensor_args.to_device(data["depth"]).unsqueeze(0) * 1000,
            intrinsics=data["intrinsics"],
            pose=Pose.from_matrix(data["pose"].to(device=tensor_args.device)),
        )
        # save depth image
        imageio.imwrite(
            "camera_depth.png",
            (cam_obs.depth_image * visualize_scale)
            .squeeze()
            .detach()
            .cpu()
            .numpy()
            .astype(np.uint16),
        )

        # save robot spheres in current joint configuration
        robot_kinematics = curobo_segmenter._robot_world.kinematics
        if write_pointcloud:
            sph = robot_kinematics.get_robot_as_spheres(q_js.position)
            WorldConfig(sphere=sph[0]).save_world_as_mesh("robot_spheres.stl")

            # save world pointcloud in robot origin

            pc = cam_obs.get_pointcloud()
            pc_obs = PointCloud("world", pose=cam_obs.pose.to_list(), points=pc)
            pc_obs.save_as_mesh("camera_pointcloud.stl", transform_with_pose=True)

        # run segmentation:
        depth_mask, filtered_image = curobo_segmenter.get_robot_mask_from_active_js(
            cam_obs,
            q_js,
        )
        # save robot points as mesh

        robot_mask = cam_obs.clone()
        robot_mask.depth_image[~depth_mask] = 0.0

        if write_pointcloud:
            robot_mesh = PointCloud(
                "world", pose=robot_mask.pose.to_list(), points=robot_mask.get_pointcloud()
            )
            robot_mesh.save_as_mesh("robot_segmented.stl", transform_with_pose=True)
        # save depth image
        imageio.imwrite(
            "robot_depth.png",
            (robot_mask.depth_image * visualize_scale)
            .detach()
            .squeeze()
            .cpu()
            .numpy()
            .astype(np.uint16),
        )

        # save world points as mesh

        world_mask = cam_obs.clone()
        world_mask.depth_image[depth_mask] = 0.0
        if write_pointcloud:
            world_mesh = PointCloud(
                "world", pose=world_mask.pose.to_list(), points=world_mask.get_pointcloud()
            )
            world_mesh.save_as_mesh("world_segmented.stl", transform_with_pose=True)

        imageio.imwrite(
            "world_depth.png",
            (world_mask.depth_image * visualize_scale)
            .detach()
            .squeeze()
            .cpu()
            .numpy()
            .astype(np.uint16),
        )

    dt_list = []

    for i in range(len(mesh_dataset)):

        data = mesh_dataset[i]
        cam_obs = CameraObservation(
            depth_image=tensor_args.to_device(data["depth"]).unsqueeze(0) * 1000,
            intrinsics=data["intrinsics"],
            pose=Pose.from_matrix(data["pose"].to(device=tensor_args.device)),
        )
        if not curobo_segmenter.ready:
            curobo_segmenter.update_camera_projection(cam_obs)
        st_time = time.time()

        depth_mask, filtered_image = curobo_segmenter.get_robot_mask_from_active_js(
            cam_obs,
            q_js,
        )

        torch.cuda.synchronize()
        dt_list.append(time.time() - st_time)

    print("Segmentation Time (ms), (hz)", np.mean(dt_list[5:]) * 1000.0, 1.0 / np.mean(dt_list[5:]))


def main(args=None):
    rclpy.init(args=args)

    robot_segmentation = RobotSegmentation()

    rclpy.spin(robot_segmentation)

    robot_segmentation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()