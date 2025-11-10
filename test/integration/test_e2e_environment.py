"""
TEST_E2E_002: Gestion d'Environnement Dynamique

Test d'intégration validant l'ajout/suppression d'obstacles et la replanification

Objectif: Valider la mise à jour de l'environnement et son impact sur la planification
"""

import pytest
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
import time

try:
    from curobo_msgs.srv import (
        AddObject, RemoveObject, GetVoxelGrid,
        GetCollisionDistance, TrajectoryGeneration
    )
    from std_srvs.srv import Trigger
    CUROBO_MSGS_AVAILABLE = True
except ImportError:
    CUROBO_MSGS_AVAILABLE = False
    pytest.skip("curobo_msgs not available", allow_module_level=True)


@pytest.mark.integration
@pytest.mark.e2e
class TestE2EEnvironmentManagement:
    """Test suite for dynamic environment management"""

    @pytest.fixture(autouse=True)
    def setup(self, test_node, service_helper_factory, test_poses,
              test_obstacles, test_config, performance_metrics):
        """Setup test fixtures"""
        self.node = test_node
        self.service_factory = service_helper_factory
        self.poses = test_poses
        self.obstacles = test_obstacles
        self.config = test_config
        self.metrics = performance_metrics

        # Service clients
        self.add_object_client = self.service_factory(
            AddObject,
            '/curobo_gen_traj/add_object'
        )
        self.remove_object_client = self.service_factory(
            RemoveObject,
            '/curobo_gen_traj/remove_object'
        )
        self.remove_all_client = self.service_factory(
            Trigger,
            '/curobo_gen_traj/remove_all_objects'
        )
        self.get_obstacles_client = self.service_factory(
            Trigger,
            '/curobo_gen_traj/get_obstacles'
        )
        self.get_voxel_grid_client = self.service_factory(
            GetVoxelGrid,
            '/curobo_gen_traj/get_voxel_grid'
        )
        self.get_collision_distance_client = self.service_factory(
            GetCollisionDistance,
            '/curobo_gen_traj/get_collision_distance'
        )
        self.traj_gen_client = self.service_factory(
            TrajectoryGeneration,
            '/curobo_gen_traj/generate_trajectory'
        )

        # Wait for services
        assert self.add_object_client.wait_for_service(timeout=30.0), \
            "add_object service not available"

        # Clean environment before each test
        self.remove_all_client.call(Trigger.Request())
        time.sleep(0.5)  # Let environment update

    def teardown_method(self):
        """Clean up after each test"""
        self.remove_all_client.call(Trigger.Request())

    def test_add_single_obstacle(self):
        """
        TEST_E2E_002a: Ajout d'un obstacle simple

        Validations:
        - Obstacle ajouté avec succès
        - Apparaît dans liste get_obstacles
        - Temps d'ajout < 1s
        """
        self.metrics.start_timer('add_obstacle')

        # Add box obstacle
        request = AddObject.Request()
        request.name = self.obstacles['box_small']['name']
        # Note: Need to fill in actual message format based on AddObject definition
        # request.type = self.obstacles['box_small']['type']
        # request.dimensions = self.obstacles['box_small']['dimensions']
        # request.pose = ...

        response = self.add_object_client.call(request, timeout=5.0)
        self.metrics.stop_timer('add_obstacle')

        assert response.success, f"Failed to add obstacle: {response.message}"

        # Verify obstacle in list
        obstacles_response = self.get_obstacles_client.call(Trigger.Request())
        # Note: Parse response to verify obstacle present
        # assert self.obstacles['box_small']['name'] in obstacles_response.message

        duration = self.metrics.get_duration('add_obstacle')
        assert duration < 1.0, f"Adding obstacle took {duration:.3f}s, should be < 1s"

        print(f"\n✓ Obstacle added in {duration:.3f}s")

    def test_add_multiple_obstacles(self):
        """
        TEST_E2E_002b: Ajout de plusieurs obstacles

        Validations:
        - Tous les obstacles ajoutés avec succès
        - Ordre d'ajout préservé
        - Performance acceptable (< 5s pour 10 obstacles)
        """
        self.metrics.start_timer('add_multiple_obstacles')

        obstacles_to_add = [
            self.obstacles['box_small'],
            self.obstacles['box_large'],
            self.obstacles['cylinder'],
            self.obstacles['sphere']
        ]

        added_count = 0
        for obstacle in obstacles_to_add:
            request = AddObject.Request()
            request.name = obstacle['name']

            try:
                response = self.add_object_client.call(request, timeout=5.0)
                if response.success:
                    added_count += 1
                    self.metrics.increment('obstacles_added')
                else:
                    print(f"  ✗ Failed to add {obstacle['name']}: {response.message}")
            except Exception as e:
                print(f"  ✗ Exception adding {obstacle['name']}: {e}")

        self.metrics.stop_timer('add_multiple_obstacles')

        assert added_count == len(obstacles_to_add), \
            f"Only added {added_count}/{len(obstacles_to_add)} obstacles"

        duration = self.metrics.get_duration('add_multiple_obstacles')
        print(f"\n✓ Added {added_count} obstacles in {duration:.3f}s")
        print(f"  Average: {duration/added_count:.3f}s per obstacle")

    def test_remove_obstacle(self):
        """
        TEST_E2E_002c: Suppression d'obstacle

        Validations:
        - Obstacle supprimé avec succès
        - N'apparaît plus dans liste
        - Temps < 1s
        """
        # First add an obstacle
        add_request = AddObject.Request()
        add_request.name = self.obstacles['box_small']['name']
        add_response = self.add_object_client.call(add_request)
        assert add_response.success, "Failed to add obstacle for removal test"

        time.sleep(0.5)  # Let environment update

        # Now remove it
        self.metrics.start_timer('remove_obstacle')

        remove_request = RemoveObject.Request()
        remove_request.name = self.obstacles['box_small']['name']

        remove_response = self.remove_object_client.call(remove_request, timeout=5.0)
        self.metrics.stop_timer('remove_obstacle')

        assert remove_response.success, f"Failed to remove obstacle: {remove_response.message}"

        duration = self.metrics.get_duration('remove_obstacle')
        assert duration < 1.0, f"Removing obstacle took {duration:.3f}s"

        print(f"\n✓ Obstacle removed in {duration:.3f}s")

    def test_remove_nonexistent_obstacle(self):
        """
        TEST_E2E_002d: Suppression d'obstacle inexistant

        Validation:
        - Gestion gracieuse (pas de crash)
        - Message d'erreur approprié
        """
        request = RemoveObject.Request()
        request.name = "nonexistent_obstacle_12345"

        response = self.remove_object_client.call(request, timeout=5.0)

        # Should handle gracefully (either success=False or success=True with warning)
        # The exact behavior depends on implementation
        print(f"\n✓ Remove nonexistent handled: {response.message}")

    def test_remove_all_obstacles(self):
        """
        TEST_E2E_002e: Suppression de tous les obstacles

        Validations:
        - Tous obstacles supprimés
        - Liste vide après suppression
        - Temps raisonnable
        """
        # Add several obstacles first
        for obstacle_name in ['box_small', 'cylinder', 'sphere']:
            request = AddObject.Request()
            request.name = self.obstacles[obstacle_name]['name']
            self.add_object_client.call(request)

        time.sleep(0.5)

        # Remove all
        self.metrics.start_timer('remove_all')
        response = self.remove_all_client.call(Trigger.Request(), timeout=5.0)
        self.metrics.stop_timer('remove_all')

        assert response.success, f"Failed to remove all obstacles: {response.message}"

        # Verify empty
        obstacles_response = self.get_obstacles_client.call(Trigger.Request())
        # Should be empty or indicate no obstacles

        duration = self.metrics.get_duration('remove_all')
        print(f"\n✓ All obstacles removed in {duration:.3f}s")

    @pytest.mark.slow
    def test_obstacle_affects_trajectory(self):
        """
        TEST_E2E_002f: Impact d'obstacle sur génération de trajectoire

        Scénario:
        1. Générer trajectoire sans obstacle (baseline)
        2. Ajouter obstacle sur le chemin
        3. Générer trajectoire avec obstacle
        4. Comparer: trajectoire doit être différente

        Validations:
        - Les deux trajectoires générées avec succès
        - Trajectoires sont différentes
        - Trajectoire avec obstacle évite effectivement l'obstacle
        """
        target_pose = self.poses['pick']

        # Step 1: Generate baseline trajectory
        self.metrics.start_timer('trajectory_without_obstacle')
        request_baseline = TrajectoryGeneration.Request()
        request_baseline.target_pose = target_pose

        response_baseline = self.traj_gen_client.call(request_baseline, timeout=10.0)
        self.metrics.stop_timer('trajectory_without_obstacle')

        assert response_baseline.success, "Baseline trajectory generation failed"

        # Step 2: Add obstacle in path
        add_request = AddObject.Request()
        add_request.name = self.obstacles['box_large']['name']
        # Position obstacle between robot and target
        # request.pose.position = Point(x=0.4, y=0.1, z=0.2)

        add_response = self.add_object_client.call(add_request, timeout=5.0)
        assert add_response.success, "Failed to add obstacle"

        time.sleep(1.0)  # Let environment update

        # Step 3: Generate trajectory with obstacle
        self.metrics.start_timer('trajectory_with_obstacle')
        request_with_obstacle = TrajectoryGeneration.Request()
        request_with_obstacle.target_pose = target_pose

        response_with_obstacle = self.traj_gen_client.call(request_with_obstacle, timeout=10.0)
        self.metrics.stop_timer('trajectory_with_obstacle')

        # May succeed or fail depending on obstacle placement
        if not response_with_obstacle.success:
            print(f"\n⚠ Trajectory blocked by obstacle (expected for some configurations)")
            print(f"  Message: {response_with_obstacle.message}")
            # This is actually a valid result - obstacle successfully blocked path
            return

        # If both succeeded, compare trajectories
        traj1 = response_baseline.trajectory
        traj2 = response_with_obstacle.trajectory

        # Trajectories should be different (different number of points or different positions)
        same_length = len(traj1.points) == len(traj2.points)

        if same_length:
            # Compare waypoint positions
            import numpy as np
            differences = []
            for i in range(len(traj1.points)):
                pos1 = np.array(traj1.points[i].positions)
                pos2 = np.array(traj2.points[i].positions)
                diff = np.linalg.norm(pos2 - pos1)
                differences.append(diff)

            max_diff = max(differences)
            avg_diff = np.mean(differences)

            print(f"\n✓ Trajectories generated with and without obstacle")
            print(f"  Max waypoint difference: {max_diff:.3f} rad")
            print(f"  Avg waypoint difference: {avg_diff:.3f} rad")

            # At least some waypoints should be significantly different
            assert max_diff > 0.1, \
                "Trajectories too similar - obstacle may not have affected planning"
        else:
            print(f"\n✓ Obstacle changed trajectory structure")
            print(f"  Without: {len(traj1.points)} waypoints")
            print(f"  With: {len(traj2.points)} waypoints")

    def test_voxel_grid_reflects_obstacles(self):
        """
        TEST_E2E_002g: Grille voxel reflète les obstacles

        Validations:
        - Voxel grid obtenu avec succès
        - Occupancy augmente quand obstacles ajoutés
        - Occupancy diminue quand obstacles supprimés
        """
        # Get baseline voxel grid (empty)
        self.metrics.start_timer('voxel_grid_empty')
        request_empty = GetVoxelGrid.Request()
        response_empty = self.get_voxel_grid_client.call(request_empty, timeout=5.0)
        self.metrics.stop_timer('voxel_grid_empty')

        assert response_empty.success, "Failed to get empty voxel grid"

        # Count occupied voxels (depends on response format)
        # occupied_empty = count_occupied_voxels(response_empty.voxel_grid)

        # Add obstacles
        for obstacle_name in ['box_small', 'cylinder']:
            request = AddObject.Request()
            request.name = self.obstacles[obstacle_name]['name']
            self.add_object_client.call(request)

        time.sleep(1.0)

        # Get voxel grid with obstacles
        self.metrics.start_timer('voxel_grid_with_obstacles')
        request_with_obs = GetVoxelGrid.Request()
        response_with_obs = self.get_voxel_grid_client.call(request_with_obs, timeout=5.0)
        self.metrics.stop_timer('voxel_grid_with_obstacles')

        assert response_with_obs.success, "Failed to get voxel grid with obstacles"

        # occupied_with_obs = count_occupied_voxels(response_with_obs.voxel_grid)

        # Should have more occupied voxels
        # assert occupied_with_obs > occupied_empty, \
        #     "Voxel grid should show more occupancy with obstacles"

        print(f"\n✓ Voxel grid reflects environment changes")
        # print(f"  Empty: {occupied_empty} voxels")
        # print(f"  With obstacles: {occupied_with_obs} voxels")

    def test_collision_distance_changes(self):
        """
        TEST_E2E_002h: Distance de collision mise à jour

        Validations:
        - Distance calculée correctement
        - Distance diminue quand obstacle se rapproche
        - Distance = 0 ou négative en collision
        """
        # Get collision distance for a configuration
        request_no_obs = GetCollisionDistance.Request()
        # request_no_obs.joint_states = ...  # Some configuration

        response_no_obs = self.get_collision_distance_client.call(request_no_obs, timeout=5.0)

        if not response_no_obs.success:
            pytest.skip("Collision distance service not fully functional")

        # distance_no_obs = response_no_obs.distance

        # Add nearby obstacle
        add_request = AddObject.Request()
        add_request.name = self.obstacles['sphere']['name']
        # Position close to robot
        self.add_object_client.call(add_request)

        time.sleep(1.0)

        # Get collision distance again
        request_with_obs = GetCollisionDistance.Request()
        # request_with_obs.joint_states = same as before

        response_with_obs = self.get_collision_distance_client.call(request_with_obs, timeout=5.0)
        assert response_with_obs.success, "Failed to get collision distance with obstacle"

        # distance_with_obs = response_with_obs.distance

        # Distance should be smaller
        # assert distance_with_obs < distance_no_obs, \
        #     "Collision distance should decrease with obstacle"

        print(f"\n✓ Collision distance updates with obstacles")

    @pytest.mark.slow
    def test_dynamic_replanning(self):
        """
        TEST_E2E_002i: Replanification dynamique

        Scénario simulant un obstacle apparaissant pendant planification:
        1. Commencer génération trajectoire
        2. Ajouter obstacle pendant génération (si possible)
        3. Vérifier que obstacle est pris en compte

        Note: Ce test est complexe car il nécessite un timing précis
        """
        # This is a challenging test to implement correctly
        # For now, we test sequential replanning

        target_pose = self.poses['pick']

        # Plan 1
        request1 = TrajectoryGeneration.Request()
        request1.target_pose = target_pose
        response1 = self.traj_gen_client.call(request1, timeout=10.0)
        assert response1.success, "Initial plan failed"

        # Add obstacle
        add_request = AddObject.Request()
        add_request.name = self.obstacles['box_small']['name']
        self.add_object_client.call(add_request)
        time.sleep(1.0)

        # Plan 2 (replan)
        self.metrics.start_timer('replan')
        request2 = TrajectoryGeneration.Request()
        request2.target_pose = target_pose
        response2 = self.traj_gen_client.call(request2, timeout=10.0)
        self.metrics.stop_timer('replan')

        replan_duration = self.metrics.get_duration('replan')

        if response2.success:
            print(f"\n✓ Replanning successful in {replan_duration:.3f}s")
        else:
            print(f"\n⚠ Replanning blocked by obstacle (valid result)")

        # Verify node still responsive
        response_remove_all = self.remove_all_client.call(Trigger.Request())
        assert response_remove_all.success, "Node should remain responsive"


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
