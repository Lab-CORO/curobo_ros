#!/usr/bin/env python3
"""
Point-cloud camera strategy (v2).

v2 notes:
- `WorldConfig` / `BloxMap` / `VoxelGrid` mutations of `node.world_model` are
  gone. v2 ingests camera data through the `Mapper` (TSDF block-sparse + ESDF),
  which is owned by the planner node and exposed as `node.mapper`.
- A PointCloud2 is wrapped into a v2 `CameraObservation` (point-cloud variant)
  and forwarded to `mapper.integrate(obs)`. The planner queries ESDF at plan
  time via `mapper.compute_esdf()`; no manual voxel grid needs to be kept
  here anymore.
- The old custom `FastVoxelGridBuilder(CPU|GPU)` pipeline (≈250 lines that
  emulated distance-transform with max-pool) is dropped: `Mapper` performs
  the equivalent ESDF directly on GPU.
"""

import time

import numpy as np
import ros2_numpy as rnp
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2

from curobo.types import CameraObservation, Pose

from curobo_ros.cameras.camera_strategy import CameraStrategy


class PointCloudCameraStrategy(CameraStrategy):
    """
    Point-cloud strategy for cuRobo v2.

    Subscribes to a PointCloud2 topic (typically the masked cloud from
    robot_segmentation) and forwards each message to the node's `Mapper`
    as a `CameraObservation`. The planner then queries ESDF / signed
    distance through the Scene model — no manual voxel grid is written
    from this strategy.
    """

    def __init__(
        self,
        node,
        name,
        topic='',
        camera_info=None,
        frame_id='',
        pixel_size=0.01,
        bounds=None,
        intrinsics=None,
        extrinsics=None,
    ):
        super().__init__(node, name, topic, camera_info or [], frame_id, intrinsics, extrinsics)

        # Kept as ROS parameters for UI compatibility even though the Mapper
        # decides its own resolution internally in v2.
        self.node.declare_parameter('grid_size', [102, 102, 102])
        self.node.declare_parameter('origin', [-1.0, -1.0, -1.0])

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.node.create_subscription(
            PointCloud2, topic, self.pointcloud_callback, qos
        )

        # Camera pose in the robot base frame: config > TF > identity.
        self.camera_pose_static = self._parse_extrinsics(extrinsics)
        if self.camera_pose_static is not None:
            self.camera_pose = self.camera_pose_static
            node.get_logger().info("Point cloud: using extrinsics from config")
        else:
            self.camera_pose = self._lookup_pose_from_tf()

        node.get_logger().info(f"PointCloudCameraStrategy ready (topic={topic})")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def pointcloud_callback(self, msg: PointCloud2):
        """Convert PointCloud2 → CameraObservation → mapper.integrate(...)."""
        t_start = time.perf_counter()
        try:
            cloud_array = rnp.numpify(msg)
            points = np.stack(
                [cloud_array['x'], cloud_array['y'], cloud_array['z']],
                axis=1,
            ).astype(np.float32)

            valid_mask = np.all(np.isfinite(points), axis=1)
            points_clean = points[valid_mask]
            if len(points_clean) == 0:
                self.node.get_logger().warn('Empty point cloud')
                return

            mapper = getattr(self.node, 'mapper', None)
            if mapper is None:
                self.node.get_logger().warn(
                    "No mapper on node — point cloud ignored. "
                    "Unified planner should expose `node.mapper` (curobo.perception.Mapper).",
                    throttle_duration_sec=5.0,
                )
                return

            obs = CameraObservation(
                pointcloud=points_clean,
                pose=self.camera_pose,
            )
            mapper.integrate(obs)

            elapsed_ms = (time.perf_counter() - t_start) * 1000
            self.node.get_logger().info(
                f'PointCloud integrated: {len(points_clean)} pts, {elapsed_ms:.1f}ms',
                throttle_duration_sec=2.0,
            )
        except Exception as e:
            self.node.get_logger().error(f'PointCloud ingest failed: {e}')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _lookup_pose_from_tf(self) -> Pose:
        """Resolve the camera pose from TF, falling back to identity."""
        try:
            t = self.tf_buffer.lookup_transform("base_link", self._frame_id, rclpy.time.Time())
            position = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            quat_ros = [t.transform.rotation.x, t.transform.rotation.y,
                        t.transform.rotation.z, t.transform.rotation.w]
            rot = Rotation.from_quat(quat_ros)
            q = rot.as_quat()  # [x, y, z, w]
            pose_list = position + [q[3], q[0], q[1], q[2]]  # cuRobo order [x,y,z,qw,qx,qy,qz]
            self.node.get_logger().info("Point cloud: using extrinsics from TF")
            return Pose.from_list(pose_list, device_cfg=self.tensor_args)
        except Exception as ex:
            self.node.get_logger().warn(
                f"TF lookup for {self._frame_id} failed: {ex}. Using identity."
            )
            return Pose(
                position=self.tensor_args.to_device([0, 0, 0]),
                quaternion=self.tensor_args.to_device([1, 0, 0, 0]),
            )
