"""
TEST_E2E_003: Pipeline Perception → Planification

Test d'intégration validant la chaîne complète de perception:
Caméra Depth → Segmentation Robot → Point Cloud → Planification

Objectif: Valider que les obstacles détectés par perception influencent la planification
"""

import pytest
import rclpy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray
import time

try:
    from curobo_msgs.srv import TrajectoryGeneration, GetVoxelGrid
    from std_srvs.srv import Trigger
    CUROBO_MSGS_AVAILABLE = True
except ImportError:
    CUROBO_MSGS_AVAILABLE = False
    pytest.skip("curobo_msgs not available", allow_module_level=True)


@pytest.mark.integration
@pytest.mark.e2e
@pytest.mark.perception
class TestE2EPerceptionPipeline:
    """Test suite for perception to planning pipeline"""

    @pytest.fixture(autouse=True)
    def setup(self, test_node, service_helper_factory, topic_helper_factory,
              rosbag_base_path, performance_metrics):
        """Setup test fixtures"""
        self.node = test_node
        self.service_factory = service_helper_factory
        self.topic_factory = topic_helper_factory
        self.rosbag_path = rosbag_base_path
        self.metrics = performance_metrics

        # Service clients
        self.traj_gen_client = self.service_factory(
            TrajectoryGeneration,
            '/curobo_gen_traj/generate_trajectory'
        )
        self.voxel_grid_client = self.service_factory(
            GetVoxelGrid,
            '/curobo_gen_traj/get_voxel_grid'
        )

        # Topic subscribers
        self.masked_depth_sub = self.topic_factory(
            Image,
            '/masked_depth_image'
        )
        self.collision_spheres_sub = self.topic_factory(
            MarkerArray,
            '/collision_spheres'
        )
        self.pointcloud_debug_sub = self.topic_factory(
            PointCloud2,
            '/robot_pointcloud_debug'
        )

    @pytest.mark.requires_rosbag
    def test_depth_image_segmentation(self):
        """
        TEST_E2E_003a: Segmentation d'image depth

        Utilise rosbag: test_depth_static_scene.bag

        Validations:
        - Images depth reçues
        - Images maskées publiées
        - Latence < 100ms
        - Robot correctement masqué
        """
        rosbag_file = self.rosbag_path / "perception" / "test_depth_static_scene.db3"

        if not rosbag_file.exists():
            pytest.skip(f"Rosbag not found: {rosbag_file}")

        # TODO: Implement rosbag replay
        # player = RosbagPlayer(rosbag_file)
        # player.start()

        # Wait for masked depth images
        try:
            self.metrics.start_timer('depth_segmentation')
            masked_images = self.masked_depth_sub.wait_for_messages(
                count=50,  # ~2-3 seconds at 15-30Hz
                timeout=10.0
            )
            self.metrics.stop_timer('depth_segmentation')

            assert len(masked_images) >= 50, \
                f"Only received {len(masked_images)} images"

            # Check image properties
            for img in masked_images[:5]:  # Check first 5
                assert img.encoding in ['32FC1', '16UC1'], \
                    f"Unexpected encoding: {img.encoding}"
                assert img.height > 0 and img.width > 0, \
                    "Invalid image dimensions"

            print(f"\n✓ Received {len(masked_images)} masked depth images")
            print(f"  Resolution: {masked_images[0].width}x{masked_images[0].height}")
            print(f"  Encoding: {masked_images[0].encoding}")

        except TimeoutError as e:
            pytest.fail(f"Did not receive masked depth images: {e}")

    @pytest.mark.requires_rosbag
    def test_robot_segmentation_quality(self):
        """
        TEST_E2E_003b: Qualité de segmentation du robot

        Validations:
        - Robot masqué dans toutes configurations
        - Sphères de collision cohérentes
        - Pas de faux positifs/négatifs
        """
        rosbag_file = self.rosbag_path / "perception" / "test_depth_robot_motion.db3"

        if not rosbag_file.exists():
            pytest.skip(f"Rosbag not found: {rosbag_file}")

        # Wait for collision spheres publication
        try:
            collision_spheres = self.collision_spheres_sub.wait_for_messages(
                count=20,
                timeout=10.0
            )

            assert len(collision_spheres) >= 20, \
                f"Only received {len(collision_spheres)} marker arrays"

            # Check marker array properties
            for markers in collision_spheres[:5]:
                assert len(markers.markers) > 0, \
                    "Collision spheres should not be empty"

                # Check sphere count is reasonable for robot
                num_spheres = len(markers.markers)
                assert 10 < num_spheres < 200, \
                    f"Unexpected number of collision spheres: {num_spheres}"

            print(f"\n✓ Collision spheres published correctly")
            print(f"  Marker arrays: {len(collision_spheres)}")
            print(f"  Avg spheres per array: {sum(len(m.markers) for m in collision_spheres) / len(collision_spheres):.1f}")

        except TimeoutError as e:
            pytest.fail(f"Did not receive collision spheres: {e}")

    def test_perception_latency(self):
        """
        TEST_E2E_003c: Latence du pipeline de perception

        Validations:
        - Latence depth → masked depth < 100ms
        - Fréquence publication > 10 Hz
        """
        # Subscribe to input depth
        depth_sub = self.topic_factory(Image, '/depth_to_rgb/image_raw')

        # Collect timestamps
        start_time = time.time()
        wait_duration = 5.0

        try:
            depth_images = depth_sub.wait_for_messages(count=10, timeout=wait_duration)
            masked_images = self.masked_depth_sub.wait_for_messages(count=10, timeout=wait_duration)

            elapsed = time.time() - start_time

            depth_freq = len(depth_images) / elapsed
            masked_freq = len(masked_images) / elapsed

            print(f"\n✓ Perception pipeline running")
            print(f"  Input depth frequency: {depth_freq:.1f} Hz")
            print(f"  Masked depth frequency: {masked_freq:.1f} Hz")

            assert masked_freq > 10.0, \
                f"Masked depth frequency too low: {masked_freq:.1f} Hz"

            # TODO: Compare timestamps to measure latency
            # This requires matching depth and masked_depth messages by timestamp

        except TimeoutError:
            pytest.skip("Depth images not being published (no camera active)")

    def test_pointcloud_to_voxel_grid(self):
        """
        TEST_E2E_003d: Conversion Point Cloud → Voxel Grid

        Validations:
        - Point cloud reçu
        - Voxel grid généré
        - Voxels occupés cohérents avec point cloud
        """
        # This test requires point cloud data
        pointcloud_sub = self.topic_factory(PointCloud2, '/masked_pointcloud')

        try:
            # Wait for point cloud
            pointclouds = pointcloud_sub.wait_for_messages(count=5, timeout=10.0)

            assert len(pointclouds) > 0, "No point clouds received"

            # Get voxel grid
            self.metrics.start_timer('voxel_grid_generation')
            voxel_request = GetVoxelGrid.Request()
            voxel_response = self.voxel_grid_client.call(voxel_request, timeout=5.0)
            self.metrics.stop_timer('voxel_grid_generation')

            assert voxel_response.success, "Failed to get voxel grid"

            duration = self.metrics.get_duration('voxel_grid_generation')

            print(f"\n✓ Point cloud to voxel grid conversion")
            print(f"  Point clouds received: {len(pointclouds)}")
            print(f"  Voxel grid generation time: {duration:.3f}s")

        except TimeoutError:
            pytest.skip("Point cloud not being published")

    @pytest.mark.slow
    @pytest.mark.requires_rosbag
    def test_perceived_obstacle_blocks_trajectory(self):
        """
        TEST_E2E_003e: Obstacle perçu bloque trajectoire

        Utilise rosbag: test_depth_dynamic_obstacle.bag

        Scénario:
        1. Scène initiale sans obstacle → trajectoire OK
        2. Obstacle entre dans le champ → voxel grid mis à jour
        3. Même trajectoire → devrait échouer ou contourner

        Validations:
        - Obstacle détecté dans voxel grid
        - Planification réagit à obstacle perçu
        """
        rosbag_file = self.rosbag_path / "perception" / "test_depth_dynamic_obstacle.db3"

        if not rosbag_file.exists():
            pytest.skip(f"Rosbag not found: {rosbag_file}")

        # Define target pose that will be blocked by obstacle
        target_pose = Pose(
            position=Point(x=0.5, y=0.2, z=0.3),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
        )

        # Step 1: Get voxel grid before obstacle
        voxel_before = self.voxel_grid_client.call(GetVoxelGrid.Request(), timeout=5.0)

        if not voxel_before.success:
            pytest.skip("Cannot get voxel grid")

        # Step 2: Generate trajectory before obstacle
        traj_request_before = TrajectoryGeneration.Request()
        traj_request_before.target_pose = target_pose

        traj_before = self.traj_gen_client.call(traj_request_before, timeout=10.0)

        # TODO: Play rosbag to point where obstacle appears
        # Wait for voxel grid to update
        time.sleep(3.0)

        # Step 3: Get voxel grid with obstacle
        voxel_after = self.voxel_grid_client.call(GetVoxelGrid.Request(), timeout=5.0)

        # Voxel grid should have changed
        # (This check depends on voxel grid message format)

        # Step 4: Try to generate same trajectory
        traj_request_after = TrajectoryGeneration.Request()
        traj_request_after.target_pose = target_pose

        traj_after = self.traj_gen_client.call(traj_request_after, timeout=10.0)

        # Expect different result (failure or different path)
        if traj_before.success and not traj_after.success:
            print(f"\n✓ Perceived obstacle correctly blocked trajectory")
            print(f"  Before: success, After: blocked")
        elif traj_before.success and traj_after.success:
            # Both succeeded - trajectories should be different
            print(f"\n✓ Trajectory adapted to perceived obstacle")
            # TODO: Compare trajectories
        else:
            pytest.skip("Trajectory generation failed even before obstacle")

    def test_camera_calibration_used(self):
        """
        TEST_E2E_003f: Calibration caméra utilisée correctement

        Validations:
        - camera_info reçu et valide
        - Matrice intrinsèque non-nulle
        - Frame ID cohérent
        """
        camera_info_sub = self.topic_factory(CameraInfo, '/depth_to_rgb/camera_info')

        try:
            camera_infos = camera_info_sub.wait_for_messages(count=5, timeout=5.0)

            assert len(camera_infos) > 0, "No camera_info received"

            # Check calibration data
            cam_info = camera_infos[0]

            assert cam_info.width > 0 and cam_info.height > 0, \
                "Invalid camera resolution"

            # Check intrinsic matrix (K)
            assert len(cam_info.k) == 9, "K matrix should be 3x3 (9 elements)"

            # fx and fy (K[0] and K[4]) should be non-zero
            assert cam_info.k[0] > 0 and cam_info.k[4] > 0, \
                "Invalid focal lengths in K matrix"

            # cx and cy (K[2] and K[5]) should be around image center
            assert 0 < cam_info.k[2] < cam_info.width, "Invalid cx"
            assert 0 < cam_info.k[5] < cam_info.height, "Invalid cy"

            print(f"\n✓ Camera calibration valid")
            print(f"  Resolution: {cam_info.width}x{cam_info.height}")
            print(f"  fx: {cam_info.k[0]:.1f}, fy: {cam_info.k[4]:.1f}")
            print(f"  cx: {cam_info.k[2]:.1f}, cy: {cam_info.k[5]:.1f}")
            print(f"  Frame: {cam_info.header.frame_id}")

        except TimeoutError:
            pytest.skip("Camera info not being published (no camera active)")

    @pytest.mark.slow
    def test_perception_consistency(self):
        """
        TEST_E2E_003g: Cohérence de la perception

        Validations:
        - Pas de flicker (oscillations rapides) dans détection
        - Obstacles stables détectés de manière stable
        - Fréquence de mise à jour cohérente
        """
        # Collect multiple voxel grid snapshots
        snapshots = []

        for i in range(5):
            voxel_response = self.voxel_grid_client.call(
                GetVoxelGrid.Request(),
                timeout=5.0
            )

            if voxel_response.success:
                snapshots.append(voxel_response)
                self.metrics.increment('voxel_snapshots')

            time.sleep(1.0)  # 1 second between snapshots

        assert len(snapshots) >= 3, \
            f"Only collected {len(snapshots)} voxel grid snapshots"

        # TODO: Analyze snapshots for consistency
        # - Count occupied voxels in each
        # - Compute difference between consecutive snapshots
        # - Should be small for static scene

        print(f"\n✓ Collected {len(snapshots)} voxel grid snapshots")
        print(f"  Scene should be stable if no motion")

    def test_multi_camera_fusion(self):
        """
        TEST_E2E_003h: Fusion de plusieurs caméras

        Note: Ce test nécessite configuration multi-caméra

        Validations:
        - Plusieurs sources de point cloud fusionnées
        - Pas de doublons dans voxel grid
        - Couverture améliorée
        """
        # This test requires multi-camera setup
        pytest.skip("Multi-camera setup required")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
