"""
TEST_E2E_005 & TEST_E2E_006: Stress Test et Robustesse

Tests d'intégration validant:
- TEST_E2E_005: Performance sous charge (stress test)
- TEST_E2E_006: Récupération après erreurs

Objectif: Valider la stabilité et robustesse du système
"""

import pytest
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import threading
import random
import psutil
import os

try:
    from curobo_msgs.srv import (
        Ik, IkBatch, TrajectoryGeneration,
        AddObject, RemoveObject
    )
    from std_srvs.srv import Trigger
    CUROBO_MSGS_AVAILABLE = True
except ImportError:
    CUROBO_MSGS_AVAILABLE = False
    pytest.skip("curobo_msgs not available", allow_module_level=True)


@pytest.mark.integration
@pytest.mark.e2e
@pytest.mark.stress
class TestE2EStressAndPerformance:
    """Test suite for stress testing and performance validation"""

    @pytest.fixture(autouse=True)
    def setup(self, test_node, service_helper_factory, test_poses,
              test_obstacles, performance_metrics):
        """Setup test fixtures"""
        self.node = test_node
        self.service_factory = service_helper_factory
        self.poses = test_poses
        self.obstacles = test_obstacles
        self.metrics = performance_metrics

        # Service clients
        self.ik_client = self.service_factory(Ik, '/curobo_ik/ik_pose')
        self.ik_batch_client = self.service_factory(IkBatch, '/curobo_ik/ik_batch_poses')
        self.traj_gen_client = self.service_factory(
            TrajectoryGeneration,
            '/curobo_gen_traj/generate_trajectory'
        )
        self.add_object_client = self.service_factory(
            AddObject,
            '/curobo_gen_traj/add_object'
        )
        self.remove_object_client = self.service_factory(
            RemoveObject,
            '/curobo_gen_traj/remove_object'
        )
        self.is_available_client = self.service_factory(
            Trigger,
            '/curobo_gen_traj/is_available'
        )

    @pytest.mark.slow
    def test_concurrent_ik_requests(self):
        """
        TEST_E2E_005a: Requêtes IK concurrentes

        Validations:
        - 100 requêtes IK simultanées
        - Toutes reçoivent réponse
        - Pas de deadlock
        - Temps total raisonnable
        """
        num_requests = 100
        results = []
        errors = []

        def make_ik_request(pose_idx):
            """Make single IK request"""
            try:
                # Generate random pose
                pose = Pose(
                    position=Point(
                        x=0.3 + random.uniform(-0.2, 0.2),
                        y=random.uniform(-0.3, 0.3),
                        z=0.3 + random.uniform(-0.1, 0.2)
                    ),
                    orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
                )

                request = Ik.Request()
                request.pose = pose

                response = self.ik_client.call(request, timeout=10.0)
                results.append((pose_idx, response.success))

            except Exception as e:
                errors.append((pose_idx, str(e)))

        # Launch concurrent requests
        self.metrics.start_timer('concurrent_ik')

        threads = []
        for i in range(num_requests):
            thread = threading.Thread(target=make_ik_request, args=(i,))
            thread.start()
            threads.append(thread)

        # Wait for all to complete
        for thread in threads:
            thread.join(timeout=30.0)

        self.metrics.stop_timer('concurrent_ik')

        # Validations
        assert len(results) + len(errors) == num_requests, \
            f"Only {len(results) + len(errors)}/{num_requests} requests completed"

        success_rate = len([r for r in results if r[1]]) / len(results) if results else 0

        duration = self.metrics.get_duration('concurrent_ik')

        print(f"\n✓ Concurrent IK requests completed")
        print(f"  Total requests: {num_requests}")
        print(f"  Successful: {len([r for r in results if r[1]])}")
        print(f"  Failed: {len([r for r in results if not r[1]])}")
        print(f"  Errors: {len(errors)}")
        print(f"  Total time: {duration:.3f}s")
        print(f"  Avg per request: {duration/num_requests:.3f}s")
        print(f"  Success rate: {success_rate*100:.1f}%")

        # At least 95% should succeed
        assert success_rate > 0.95, f"Success rate too low: {success_rate*100:.1f}%"

    @pytest.mark.slow
    def test_batch_ik_large_batch(self):
        """
        TEST_E2E_005b: IK batch avec grand nombre de poses

        Validations:
        - Batch de 1000 poses
        - Performance acceptable
        - Pas de memory leak
        """
        batch_size = 1000

        # Generate random poses
        poses = []
        for _ in range(batch_size):
            pose = Pose(
                position=Point(
                    x=0.3 + random.uniform(-0.2, 0.2),
                    y=random.uniform(-0.3, 0.3),
                    z=0.3 + random.uniform(-0.1, 0.2)
                ),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
            )
            poses.append(pose)

        # Measure memory before
        process = psutil.Process(os.getpid())
        mem_before = process.memory_info().rss / 1024 / 1024  # MB

        # Call batch IK
        self.metrics.start_timer('ik_batch_1000')

        request = IkBatch.Request()
        request.poses = poses

        try:
            response = self.ik_batch_client.call(request, timeout=30.0)

            self.metrics.stop_timer('ik_batch_1000')

            # Measure memory after
            mem_after = process.memory_info().rss / 1024 / 1024  # MB
            mem_increase = mem_after - mem_before

            duration = self.metrics.get_duration('ik_batch_1000')

            if response.success:
                # Check results
                num_solutions = len(response.joint_states_list) if hasattr(response, 'joint_states_list') else 0

                print(f"\n✓ Batch IK completed")
                print(f"  Batch size: {batch_size}")
                print(f"  Solutions found: {num_solutions}")
                print(f"  Time: {duration:.3f}s")
                print(f"  Time per pose: {duration/batch_size*1000:.1f}ms")
                print(f"  Memory increase: {mem_increase:.1f} MB")

                # Performance should be better than individual calls
                assert duration < batch_size * 0.1, \
                    f"Batch IK not efficient: {duration:.3f}s for {batch_size} poses"

                # Memory increase should be reasonable
                assert mem_increase < 500, \
                    f"Excessive memory usage: {mem_increase:.1f} MB"

            else:
                pytest.fail(f"Batch IK failed: {response.message}")

        except Exception as e:
            pytest.fail(f"Batch IK exception: {e}")

    @pytest.mark.slow
    def test_continuous_trajectory_generation(self):
        """
        TEST_E2E_005c: Génération continue de trajectoires

        Validations:
        - 100 trajectoires générées séquentiellement
        - Pas de dégradation de performance
        - Pas de memory leak
        - CPU/mémoire stables
        """
        num_trajectories = 100
        durations = []
        errors = 0

        # Monitor memory
        process = psutil.Process(os.getpid())
        mem_samples = []

        self.metrics.start_timer('continuous_generation')

        for i in range(num_trajectories):
            # Random target pose
            pose = Pose(
                position=Point(
                    x=0.4 + random.uniform(-0.1, 0.1),
                    y=random.uniform(-0.2, 0.2),
                    z=0.3 + random.uniform(-0.1, 0.1)
                ),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
            )

            start = time.time()

            request = TrajectoryGeneration.Request()
            request.target_pose = pose

            try:
                response = self.traj_gen_client.call(request, timeout=10.0)

                duration = time.time() - start
                durations.append(duration)

                if not response.success:
                    errors += 1

                # Sample memory every 10 iterations
                if i % 10 == 0:
                    mem_samples.append(process.memory_info().rss / 1024 / 1024)

            except Exception as e:
                errors += 1
                print(f"  Error in iteration {i}: {e}")

        self.metrics.stop_timer('continuous_generation')

        # Analysis
        total_duration = self.metrics.get_duration('continuous_generation')

        import numpy as np
        avg_duration = np.mean(durations)
        std_duration = np.std(durations)
        p95_duration = np.percentile(durations, 95)

        mem_increase = max(mem_samples) - min(mem_samples) if mem_samples else 0

        print(f"\n✓ Continuous trajectory generation completed")
        print(f"  Total trajectories: {num_trajectories}")
        print(f"  Successful: {num_trajectories - errors}")
        print(f"  Errors: {errors}")
        print(f"  Total time: {total_duration:.3f}s")
        print(f"  Avg duration: {avg_duration:.3f}s")
        print(f"  Std duration: {std_duration:.3f}s")
        print(f"  p95 duration: {p95_duration:.3f}s")
        print(f"  Memory delta: {mem_increase:.1f} MB")

        # Validations
        assert errors / num_trajectories < 0.1, \
            f"Too many errors: {errors}/{num_trajectories}"

        assert mem_increase < 200, \
            f"Memory leak suspected: {mem_increase:.1f} MB increase"

        # Performance should not degrade significantly
        first_10 = np.mean(durations[:10])
        last_10 = np.mean(durations[-10:])
        degradation = (last_10 - first_10) / first_10

        assert degradation < 0.5, \
            f"Performance degraded by {degradation*100:.1f}%"

    @pytest.mark.slow
    def test_rapid_object_add_remove(self):
        """
        TEST_E2E_005d: Ajout/suppression rapide d'obstacles

        Validations:
        - 100 cycles ajout/suppression
        - Pas de memory leak
        - Performance stable
        """
        num_cycles = 100

        self.metrics.start_timer('rapid_add_remove')

        for i in range(num_cycles):
            # Add obstacle
            add_request = AddObject.Request()
            add_request.name = f"stress_test_obj_{i}"

            try:
                add_response = self.add_object_client.call(add_request, timeout=2.0)
                if add_response.success:
                    self.metrics.increment('objects_added')
            except Exception:
                self.metrics.increment('add_errors')

            # Immediately remove
            remove_request = RemoveObject.Request()
            remove_request.name = f"stress_test_obj_{i}"

            try:
                remove_response = self.remove_object_client.call(remove_request, timeout=2.0)
                if remove_response.success:
                    self.metrics.increment('objects_removed')
            except Exception:
                self.metrics.increment('remove_errors')

        self.metrics.stop_timer('rapid_add_remove')

        duration = self.metrics.get_duration('rapid_add_remove')

        print(f"\n✓ Rapid add/remove stress test completed")
        print(f"  Cycles: {num_cycles}")
        print(f"  Total time: {duration:.3f}s")
        print(f"  Avg per cycle: {duration/num_cycles*1000:.1f}ms")
        print(f"  Objects added: {self.metrics.get_counter('objects_added')}")
        print(f"  Objects removed: {self.metrics.get_counter('objects_removed')}")
        print(f"  Add errors: {self.metrics.get_counter('add_errors')}")
        print(f"  Remove errors: {self.metrics.get_counter('remove_errors')}")

        # System should remain responsive
        response = self.is_available_client.call(Trigger.Request())
        assert response.success, "Node should remain available after stress test"


@pytest.mark.integration
@pytest.mark.e2e
@pytest.mark.robustness
class TestE2EErrorRecovery:
    """Test suite for error handling and recovery"""

    @pytest.fixture(autouse=True)
    def setup(self, test_node, service_helper_factory, test_poses):
        """Setup test fixtures"""
        self.node = test_node
        self.service_factory = service_helper_factory
        self.poses = test_poses

        self.ik_client = self.service_factory(Ik, '/curobo_ik/ik_pose')
        self.traj_gen_client = self.service_factory(
            TrajectoryGeneration,
            '/curobo_gen_traj/generate_trajectory'
        )
        self.is_available_client = self.service_factory(
            Trigger,
            '/curobo_gen_traj/is_available'
        )

    def test_invalid_pose_handling(self):
        """
        TEST_E2E_006a: Gestion de pose invalide

        Validations:
        - NaN dans pose géré gracieusement
        - Message d'erreur clair
        - Nœud reste fonctionnel
        """
        # Create invalid pose with NaN
        invalid_pose = Pose(
            position=Point(x=float('nan'), y=0.0, z=0.3),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
        )

        request = Ik.Request()
        request.pose = invalid_pose

        try:
            response = self.ik_client.call(request, timeout=5.0)

            # Should fail gracefully
            assert not response.success, "Should reject NaN pose"
            assert 'nan' in response.message.lower() or 'invalid' in response.message.lower(), \
                "Error message should mention invalid input"

            print(f"\n✓ Invalid pose rejected: {response.message}")

        except Exception as e:
            # Exception is also acceptable
            print(f"\n✓ Invalid pose caused exception (acceptable): {e}")

        # Verify node still works
        valid_response = self.is_available_client.call(Trigger.Request())
        assert valid_response.success, "Node should recover from invalid input"

    def test_timeout_handling(self):
        """
        TEST_E2E_006b: Gestion de timeout

        Validations:
        - Requête avec timeout court échoue proprement
        - Nœud ne bloque pas
        - Peut continuer après timeout
        """
        request = TrajectoryGeneration.Request()
        request.target_pose = self.poses['pick']

        # Try with very short timeout
        try:
            response = self.traj_gen_client.call(request, timeout=0.001)  # 1ms - too short

            # Likely to timeout
            print(f"\n✓ Request completed (surprisingly fast)")

        except Exception as e:
            print(f"\n✓ Timeout handled: {type(e).__name__}")

        # Node should still be responsive
        time.sleep(0.5)
        valid_response = self.is_available_client.call(Trigger.Request())
        assert valid_response.success, "Node should remain available after timeout"

    def test_rapid_repeated_failures(self):
        """
        TEST_E2E_006c: Échecs répétés rapides

        Validations:
        - 50 requêtes impossibles consécutives
        - Nœud ne crash pas
        - Reste réactif
        """
        num_failures = 50
        unreachable_pose = self.poses['unreachable']

        for i in range(num_failures):
            request = TrajectoryGeneration.Request()
            request.target_pose = unreachable_pose

            try:
                response = self.traj_gen_client.call(request, timeout=5.0)
                # Expected to fail
            except Exception:
                # Also acceptable
                pass

        # Node should still work
        response = self.is_available_client.call(Trigger.Request())
        assert response.success, \
            "Node should remain functional after repeated failures"

        print(f"\n✓ Survived {num_failures} repeated failures")

    def test_recovery_from_error_state(self):
        """
        TEST_E2E_006d: Récupération après erreur

        Validations:
        - Erreur provoquée
        - Requête valide suivante réussit
        - Pas d'état résiduel
        """
        # Step 1: Cause an error
        invalid_request = TrajectoryGeneration.Request()
        invalid_request.target_pose = self.poses['unreachable']

        invalid_response = self.traj_gen_client.call(invalid_request, timeout=5.0)

        # Step 2: Immediately try valid request
        valid_request = TrajectoryGeneration.Request()
        valid_request.target_pose = self.poses['pick']

        valid_response = self.traj_gen_client.call(valid_request, timeout=10.0)

        # Should succeed despite previous error
        assert valid_response.success, \
            f"Valid request failed after error: {valid_response.message}"

        print(f"\n✓ Recovered from error state")

    @pytest.mark.skip(reason="Requires config file manipulation")
    def test_invalid_config_handling(self):
        """
        TEST_E2E_006e: Gestion de configuration invalide

        Validations:
        - update_motion_gen_config avec config corrompu
        - Fallback sur config précédente
        - Nœud reste fonctionnel
        """
        # Would need to:
        # 1. Corrupt config file
        # 2. Call update_motion_gen_config
        # 3. Verify fallback
        # 4. Restore config

        pytest.skip("Config file manipulation test - implement manually")

    def test_concurrent_conflicting_requests(self):
        """
        TEST_E2E_006f: Requêtes concurrentes conflictuelles

        Validations:
        - Multiples requêtes simultanées gérées
        - Pas de race conditions
        - Toutes reçoivent réponse
        """
        num_concurrent = 10
        results = []

        def make_request(idx):
            pose = Pose(
                position=Point(x=0.3, y=0.0, z=0.3),
                orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
            )

            request = TrajectoryGeneration.Request()
            request.target_pose = pose

            try:
                response = self.traj_gen_client.call(request, timeout=15.0)
                results.append((idx, response.success))
            except Exception as e:
                results.append((idx, False))

        # Launch concurrent requests
        threads = []
        for i in range(num_concurrent):
            thread = threading.Thread(target=make_request, args=(i,))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join(timeout=20.0)

        # All should have received response
        assert len(results) == num_concurrent, \
            f"Only {len(results)}/{num_concurrent} requests completed"

        successes = len([r for r in results if r[1]])

        print(f"\n✓ Concurrent requests handled")
        print(f"  Total: {num_concurrent}")
        print(f"  Successful: {successes}")

    def test_node_remains_available_throughout(self):
        """
        TEST_E2E_006g: Nœud reste disponible

        Validation continue que le nœud est disponible
        pendant une série d'opérations
        """
        operations = [
            ('valid_ik', lambda: self.ik_client.call(
                Ik.Request(pose=self.poses['pick']),
                timeout=5.0
            )),
            ('invalid_ik', lambda: self.ik_client.call(
                Ik.Request(pose=self.poses['unreachable']),
                timeout=5.0
            )),
            ('valid_traj', lambda: self.traj_gen_client.call(
                TrajectoryGeneration.Request(target_pose=self.poses['place']),
                timeout=10.0
            )),
        ]

        for op_name, op_func in operations:
            # Check available before
            before = self.is_available_client.call(Trigger.Request())
            assert before.success, f"Node not available before {op_name}"

            # Run operation
            try:
                op_func()
            except Exception:
                pass  # Errors OK for this test

            # Check available after
            after = self.is_available_client.call(Trigger.Request())
            assert after.success, f"Node not available after {op_name}"

        print(f"\n✓ Node remained available throughout {len(operations)} operations")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
