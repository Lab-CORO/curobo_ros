"""
TEST_E2E_001: Pipeline Complet de Planification et Exécution

Test d'intégration bout-en-bout validant la chaîne complète:
IK → Génération Trajectoire → Exécution

Objectif: Valider qu'un scénario pick-and-place complet fonctionne de bout en bout
"""

import pytest
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32
import time
import numpy as np

# Import custom message types (assuming they exist in curobo_msgs)
try:
    from curobo_msgs.srv import Ik, TrajectoryGeneration
    from curobo_msgs.action import SendTrajectory
    CUROBO_MSGS_AVAILABLE = True
except ImportError:
    CUROBO_MSGS_AVAILABLE = False
    pytest.skip("curobo_msgs not available", allow_module_level=True)

from std_srvs.srv import Trigger


class TestE2EPlanning:
    """Test suite for end-to-end planning pipeline"""

    @pytest.fixture(autouse=True)
    def setup(self, test_node, service_helper_factory, topic_helper_factory,
              test_poses, test_config, performance_metrics):
        """Setup test fixtures"""
        self.node = test_node
        self.service_factory = service_helper_factory
        self.topic_factory = topic_helper_factory
        self.poses = test_poses
        self.config = test_config
        self.metrics = performance_metrics

        # Service clients
        self.ik_client = self.service_factory(Ik, '/curobo_ik/ik_pose')
        self.traj_gen_client = self.service_factory(
            TrajectoryGeneration,
            '/curobo_gen_traj/generate_trajectory'
        )
        self.is_available_client = self.service_factory(
            Trigger,
            '/curobo_gen_traj/is_available'
        )

        # Wait for services to be available
        assert self.is_available_client.wait_for_service(timeout=30.0), \
            "curobo_gen_traj service not available"
        assert self.ik_client.wait_for_service(timeout=30.0), \
            "curobo_ik service not available"
        assert self.traj_gen_client.wait_for_service(timeout=30.0), \
            "generate_trajectory service not available"

        # Check node is ready
        response = self.is_available_client.call(Trigger.Request())
        assert response.success, f"Node not available: {response.message}"

    def test_simple_trajectory_generation(self):
        """
        TEST_E2E_001a: Génération simple de trajectoire vers pose atteignable

        Validations:
        - Service retourne success=True
        - Trajectoire contient des waypoints
        - Durée de génération < timeout configuré
        """
        self.metrics.start_timer('trajectory_generation')

        # Request trajectory to pick pose
        request = TrajectoryGeneration.Request()
        request.target_pose = self.poses['pick']

        response = self.traj_gen_client.call(request, timeout=10.0)

        self.metrics.stop_timer('trajectory_generation')

        # Validations
        assert response.success, f"Trajectory generation failed: {response.message}"
        assert response.trajectory is not None, "No trajectory returned"
        assert len(response.trajectory.points) > 0, "Trajectory has no waypoints"

        # Check timing
        duration = self.metrics.get_duration('trajectory_generation')
        assert duration < self.config['timeout'], \
            f"Trajectory generation took {duration:.3f}s, exceeds timeout {self.config['timeout']}s"

        print(f"\n✓ Generated trajectory with {len(response.trajectory.points)} waypoints in {duration:.3f}s")

    def test_ik_to_trajectory_pipeline(self):
        """
        TEST_E2E_001b: Pipeline IK → Génération Trajectoire

        Étapes:
        1. Calculer IK pour pose target
        2. Utiliser solution IK comme seed pour génération trajectoire
        3. Générer trajectoire

        Validations:
        - IK trouve solution
        - Trajectoire générée avec succès
        - Pipeline complet < 2s
        """
        self.metrics.start_timer('ik_to_trajectory')

        # Step 1: Solve IK
        self.metrics.start_timer('ik_solve')
        ik_request = Ik.Request()
        ik_request.pose = self.poses['place']

        ik_response = self.ik_client.call(ik_request, timeout=5.0)
        self.metrics.stop_timer('ik_solve')

        assert ik_response.success, f"IK failed: {ik_response.message}"
        assert ik_response.joint_states is not None, "No joint solution returned"

        # Step 2: Generate trajectory using IK solution
        self.metrics.start_timer('trajectory_with_seed')
        traj_request = TrajectoryGeneration.Request()
        traj_request.target_pose = self.poses['place']
        # Note: If service supports seed, add it here
        # traj_request.seed_joint_states = ik_response.joint_states

        traj_response = self.traj_gen_client.call(traj_request, timeout=10.0)
        self.metrics.stop_timer('trajectory_with_seed')

        self.metrics.stop_timer('ik_to_trajectory')

        # Validations
        assert traj_response.success, f"Trajectory generation failed: {traj_response.message}"

        total_duration = self.metrics.get_duration('ik_to_trajectory')
        assert total_duration < 2.0, \
            f"IK→Trajectory pipeline took {total_duration:.3f}s, should be < 2s"

        print(f"\n✓ IK→Trajectory pipeline completed in {total_duration:.3f}s")
        print(f"  - IK: {self.metrics.get_duration('ik_solve'):.3f}s")
        print(f"  - Trajectory: {self.metrics.get_duration('trajectory_with_seed'):.3f}s")

    @pytest.mark.slow
    def test_pick_and_place_sequence(self, topic_helper_factory):
        """
        TEST_E2E_001c: Séquence complète Pick-and-Place

        Séquence:
        1. Home → Approach Pick (au-dessus de l'objet)
        2. Approach Pick → Pick (descente)
        3. Pick → Approach Place (montée + déplacement)
        4. Approach Place → Place (descente)
        5. Place → Home (retour)

        Validations:
        - Chaque trajectoire générée avec succès
        - Temps total raisonnable (< 30s pour génération)
        - Pas d'erreurs de planification
        """
        self.metrics.start_timer('pick_place_sequence')

        # Define sequence of poses
        sequence = [
            ('home', self.poses['home']),
            ('approach_pick', Pose(
                position=Point(
                    x=self.poses['pick'].position.x,
                    y=self.poses['pick'].position.y,
                    z=self.poses['pick'].position.z + 0.1  # 10cm above
                ),
                orientation=self.poses['pick'].orientation
            )),
            ('pick', self.poses['pick']),
            ('approach_place', Pose(
                position=Point(
                    x=self.poses['place'].position.x,
                    y=self.poses['place'].position.y,
                    z=self.poses['place'].position.z + 0.1
                ),
                orientation=self.poses['place'].orientation
            )),
            ('place', self.poses['place']),
            ('home_return', self.poses['home'])
        ]

        trajectories = []
        failed_steps = []

        # Generate trajectory for each step
        for step_name, target_pose in sequence:
            self.metrics.start_timer(f'step_{step_name}')

            request = TrajectoryGeneration.Request()
            request.target_pose = target_pose

            try:
                response = self.traj_gen_client.call(request, timeout=15.0)

                if response.success:
                    trajectories.append((step_name, response.trajectory))
                    self.metrics.increment('successful_steps')
                else:
                    failed_steps.append(step_name)
                    self.metrics.increment('failed_steps')
                    print(f"  ✗ Step '{step_name}' failed: {response.message}")

            except Exception as e:
                failed_steps.append(step_name)
                self.metrics.increment('failed_steps')
                print(f"  ✗ Step '{step_name}' exception: {e}")

            self.metrics.stop_timer(f'step_{step_name}')

        self.metrics.stop_timer('pick_place_sequence')

        # Validations
        assert len(failed_steps) == 0, \
            f"Failed steps: {failed_steps}. Only {len(trajectories)}/{len(sequence)} succeeded"

        total_duration = self.metrics.get_duration('pick_place_sequence')
        assert total_duration < 30.0, \
            f"Pick-place sequence took {total_duration:.3f}s, should be < 30s"

        # Report
        print(f"\n✓ Pick-and-place sequence completed successfully")
        print(f"  Total planning time: {total_duration:.3f}s")
        print(f"  Steps completed: {len(trajectories)}/{len(sequence)}")

        for step_name, _ in sequence:
            step_duration = self.metrics.get_duration(f'step_{step_name}')
            if step_duration:
                print(f"    - {step_name}: {step_duration:.3f}s")

    def test_trajectory_quality(self):
        """
        TEST_E2E_001d: Validation de la qualité de trajectoire

        Validations:
        - Trajectoire est lisse (pas de sauts brusques)
        - Vitesses et accélérations raisonnables
        - Waypoints espacés uniformément dans le temps
        """
        # Generate trajectory
        request = TrajectoryGeneration.Request()
        request.target_pose = self.poses['pick']

        response = self.traj_gen_client.call(request, timeout=10.0)
        assert response.success, "Trajectory generation failed"

        trajectory = response.trajectory
        assert len(trajectory.points) >= 2, "Trajectory too short for quality check"

        # Check 1: Time stamps are monotonically increasing
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                 for point in trajectory.points]

        assert all(times[i] < times[i+1] for i in range(len(times)-1)), \
            "Trajectory timestamps not monotonically increasing"

        # Check 2: No extreme velocity jumps
        for i, point in enumerate(trajectory.points):
            if hasattr(point, 'velocities') and len(point.velocities) > 0:
                max_velocity = max(abs(v) for v in point.velocities)
                assert max_velocity < 3.0, \
                    f"Waypoint {i} has extreme velocity: {max_velocity} rad/s"

        # Check 3: Position continuity (if multiple waypoints)
        if len(trajectory.points) >= 3:
            for i in range(1, len(trajectory.points) - 1):
                prev_pos = np.array(trajectory.points[i-1].positions)
                curr_pos = np.array(trajectory.points[i].positions)
                next_pos = np.array(trajectory.points[i+1].positions)

                # Check for discontinuities
                delta1 = np.linalg.norm(curr_pos - prev_pos)
                delta2 = np.linalg.norm(next_pos - curr_pos)

                # Deltas shouldn't differ by more than 5x
                if delta1 > 0:
                    ratio = delta2 / delta1
                    assert 0.2 < ratio < 5.0, \
                        f"Position discontinuity at waypoint {i}: ratio {ratio:.2f}"

        print(f"\n✓ Trajectory quality validated")
        print(f"  Waypoints: {len(trajectory.points)}")
        print(f"  Duration: {times[-1]:.3f}s")
        print(f"  Average waypoint spacing: {(times[-1] / len(times)):.3f}s")

    def test_unreachable_pose_handling(self):
        """
        TEST_E2E_001e: Gestion de pose inatteignable

        Validation:
        - Service retourne success=False pour pose hors workspace
        - Message d'erreur explicite
        - Pas de crash du nœud
        """
        request = TrajectoryGeneration.Request()
        request.target_pose = self.poses['unreachable']

        response = self.traj_gen_client.call(request, timeout=10.0)

        # Should fail gracefully
        assert not response.success, \
            "Service should return success=False for unreachable pose"
        assert len(response.message) > 0, \
            "Should provide error message for unreachable pose"

        # Verify node is still responsive
        available_response = self.is_available_client.call(Trigger.Request())
        assert available_response.success, \
            "Node should remain available after failed trajectory generation"

        print(f"\n✓ Unreachable pose handled gracefully")
        print(f"  Error message: {response.message}")

    @pytest.mark.parametrize("max_attempts", [1, 3, 5])
    def test_retry_mechanism(self, max_attempts, test_node):
        """
        TEST_E2E_001f: Mécanisme de retry avec max_attempts

        Test avec différentes valeurs de max_attempts pour voir l'impact
        sur le taux de succès avec une pose difficile
        """
        # Note: This would require launching node with specific max_attempts parameter
        # For now, just test with current configuration

        request = TrajectoryGeneration.Request()
        # Use a challenging but reachable pose
        request.target_pose = Pose(
            position=Point(x=0.6, y=0.35, z=0.15),  # Near edge of workspace
            orientation=Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
        )

        self.metrics.start_timer(f'retry_test_attempts_{max_attempts}')
        response = self.traj_gen_client.call(request, timeout=15.0)
        self.metrics.stop_timer(f'retry_test_attempts_{max_attempts}')

        duration = self.metrics.get_duration(f'retry_test_attempts_{max_attempts}')

        # Log result
        if response.success:
            self.metrics.increment('retry_successes')
            print(f"\n✓ Challenging pose solved in {duration:.3f}s")
        else:
            self.metrics.increment('retry_failures')
            print(f"\n✗ Challenging pose failed after {duration:.3f}s")

        # Just verify no crash
        available_response = self.is_available_client.call(Trigger.Request())
        assert available_response.success, "Node should remain available"


@pytest.mark.integration
@pytest.mark.e2e
class TestE2EExecutionPipeline:
    """
    Test suite for trajectory execution using action server

    Note: Ces tests nécessitent que l'action server SendTrajectory soit disponible
    """

    @pytest.fixture(autouse=True)
    def setup(self, test_node, service_helper_factory, topic_helper_factory, test_poses):
        """Setup for execution tests"""
        self.node = test_node
        self.service_factory = service_helper_factory
        self.topic_factory = topic_helper_factory
        self.poses = test_poses

        # Create trajectory generation client
        self.traj_gen_client = self.service_factory(
            TrajectoryGeneration,
            '/curobo_gen_traj/generate_trajectory'
        )

    @pytest.mark.skip(reason="Action client implementation needed")
    def test_trajectory_execution_with_feedback(self):
        """
        TEST_E2E_001g: Exécution de trajectoire avec feedback

        Validations:
        - Action démarre
        - Feedback reçu régulièrement (step_progression 0→1)
        - Result indique succès
        - Durée d'exécution cohérente avec trajectoire
        """
        # Generate trajectory first
        request = TrajectoryGeneration.Request()
        request.target_pose = self.poses['pick']

        traj_response = self.traj_gen_client.call(request)
        assert traj_response.success, "Trajectory generation failed"

        # TODO: Implement action client for SendTrajectory
        # goal = SendTrajectory.Goal()
        # goal.trajectory = traj_response.trajectory
        #
        # feedbacks = []
        # def feedback_callback(feedback):
        #     feedbacks.append(feedback.feedback.step_progression)
        #
        # # Send goal and wait
        # action_client.send_goal_async(goal, feedback_callback=feedback_callback)
        # ...

        pytest.skip("Action client implementation TODO")

    @pytest.mark.skip(reason="Action client implementation needed")
    def test_trajectory_cancellation(self):
        """
        TEST_E2E_001h: Annulation de trajectoire en cours

        Validations:
        - Action peut être annulée pendant exécution
        - Robot s'arrête proprement
        - Result indique cancellation
        """
        pytest.skip("Action client implementation TODO")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
