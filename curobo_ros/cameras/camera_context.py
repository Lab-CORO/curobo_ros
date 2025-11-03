#!/usr/bin/env python3

from typing import List, Dict
import torch
from curobo.types.camera import CameraObservation
from curobo_ros.cameras.camera_strategy import CameraStrategy
from curobo_ros.cameras.pointcloud_camera_strategy import PointCloudCameraStrategy
from curobo_ros.cameras.camera_depth_map_strategy import DepthMapCameraStrategy


class CameraContext:
    """
    Context class for managing multiple camera strategies.

    This class follows the strategy pattern to support different types of cameras
    (real depth cameras, point cloud sources, simulations, etc.) and provides
    a unified interface for obtaining camera observations for cuRobo collision checking.
    """

    def __init__(self, node):
        """
        Initialize the camera context.

        Args:
            node: ROS2 node for logging
        """
        self.node = node
        self.cameras: Dict[str, CameraStrategy] = {}
        self._device = torch.device('cuda')

    def add_camera(self, camera_name, camera_type, topic, camera_info, frame_id, **kwargs):
        """
        Add a camera strategy to the context.

        Args:
            camera_name: Unique identifier for this camera
            camera_type: Type of camera ('point_cloud' or 'depth_camera')
            topic: ROS topic for the camera data
            camera_info: Camera intrinsics (for depth cameras)
            frame_id: Frame ID for the camera
            **kwargs: Additional parameters for specific camera strategies
        """
        # Create the appropriate camera strategy based on type
        if camera_type == 'point_cloud':
            # For point cloud cameras
            pixel_size = kwargs.get('pixel_size', 0.01)
            bounds = kwargs.get('bounds', None)
            camera_strategy = PointCloudCameraStrategy(
                node=self.node,
                name=camera_name,
                topic=topic,
                camera_info=camera_info,
                frame_id=frame_id,
                pixel_size=pixel_size,
                bounds=bounds
            )

        elif camera_type == 'depth_camera':
            # For depth cameras (RealSense, etc.)
            camera_info_topic = kwargs.get('camera_info_topic', topic.replace('/image', '/camera_info'))
            camera_pose = kwargs.get('camera_pose', None)
            camera_strategy = DepthMapCameraStrategy(
                node=self.node,
                name=camera_name,
                depth_topic=topic,
                camera_info_topic=camera_info_topic,
                camera_pose=camera_pose
            )

        else:
            self.node.get_logger().error(f"Unknown camera type: {camera_type}")
            return

        self.cameras[camera_name] = camera_strategy
        self.node.get_logger().info(f"Added camera strategy '{camera_name}' of type '{camera_type}'")

    def set_camera_update_callback(self, callback):
        """
        Set a callback to be called when any camera receives new data.

        Args:
            callback: Function to call when camera data is updated
        """
        for camera in self.cameras.values():
            camera.set_update_callback(callback)

    def remove_camera(self, name: str):
        """
        Remove a camera strategy from the context.

        Args:
            name: Identifier of the camera to remove
        """
        if name in self.cameras:
            del self.cameras[name]
            self.node.get_logger().info(f"Removed camera strategy '{name}'")
        else:
            self.node.get_logger().warn(f"Camera '{name}' not found")

    def get_camera_observation(self, name: str) -> CameraObservation:
        """
        Get camera observation from a specific camera.

        Args:
            name: Identifier of the camera

        Returns:
            CameraObservation or None if camera not ready
        """
        if name not in self.cameras:
            self.node.get_logger().error(f"Camera '{name}' not found")
            return None

        camera = self.cameras[name]
        if not camera.is_ready():
            self.node.get_logger().warn(f"Camera '{name}' is not ready")
            return None

        return camera.get_camera_observation()

    def get_all_camera_observations(self) -> List[CameraObservation]:
        """
        Get camera observations from all ready cameras.

        Returns:
            List of CameraObservation objects from all ready cameras
        """
        observations = []

        for name, camera in self.cameras.items():
            if camera.is_ready():
                try:
                    obs = camera.get_camera_observation()
                    if obs is not None:
                        observations.append(obs)
                except Exception as e:
                    self.node.get_logger().error(
                        f"Error getting observation from camera '{name}': {e}"
                    )

        return observations

    def is_camera_ready(self, name: str) -> bool:
        """
        Check if a specific camera is ready.

        Args:
            name: Identifier of the camera

        Returns:
            bool: True if camera exists and is ready
        """
        if name not in self.cameras:
            return False
        return self.cameras[name].is_ready()

    def are_any_cameras_ready(self) -> bool:
        """
        Check if any cameras are ready.

        Returns:
            bool: True if at least one camera is ready
        """
        return any(camera.is_ready() for camera in self.cameras.values())

    def get_camera_names(self) -> List[str]:
        """
        Get list of all camera names.

        Returns:
            List of camera identifiers
        """
        return list(self.cameras.keys())
