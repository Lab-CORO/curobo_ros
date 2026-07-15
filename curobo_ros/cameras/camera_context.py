#!/usr/bin/env python3

from typing import List, Dict
import torch
from curobo_ros.cameras.camera_strategy import CameraStrategy
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

    def add_camera(self, camera_name, camera_type, topic, camera_info, frame_id, intrinsics=None, extrinsics=None, **kwargs):
        """
        Add a camera strategy to the context.

        Args:
            camera_name: Unique identifier for this camera
            camera_type: Type of camera ('depth_camera')
            topic: ROS topic for the camera data
            camera_info: Camera intrinsics (for depth cameras)
            frame_id: Frame ID for the camera
            intrinsics: Optional camera intrinsics from config (list or dict)
            extrinsics: Optional camera extrinsics from config (list)
            **kwargs: Additional parameters for specific camera strategies
        """
        # v2 perception ingests depth+rgb images through the Mapper TSDF; raw
        # point clouds are not supported, so only depth cameras are available.
        if camera_type == 'depth_camera':
            # For depth cameras (RealSense, etc.)
            # Use camera_info parameter from config or fallback to topic-based guess
            camera_info_topic = camera_info if camera_info else topic.replace('/image', '/camera_info')
            camera_strategy = DepthMapCameraStrategy(
                node=self.node,
                camera_name=camera_name,
                topic=topic,
                camera_info_topic=camera_info_topic,
                frame_id=frame_id,
                intrinsics=intrinsics,
                extrinsics=extrinsics
            )

        else:
            self.node.get_logger().error(
                f"Unknown/unsupported camera type: {camera_type} "
                f"(v2 supports 'depth_camera' only)"
            )
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

    # NOTE: the pull-based observation API (get_camera_observation / is_ready /
    # ...) was removed — it was dead AND broken (no strategy implements is_ready()
    # or get_camera_observation()). In v2 perception is PUSH-based:
    # DepthMapCameraStrategy feeds frames directly into the Mapper TSDF.

    def get_camera_names(self) -> List[str]:
        """
        Get list of all camera names.

        Returns:
            List of camera identifiers
        """
        return list(self.cameras.keys())
