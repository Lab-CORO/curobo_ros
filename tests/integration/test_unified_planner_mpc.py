#!/usr/bin/env python3
"""
Tests d'intégration pour unified_planner (branche MPC).

Tests du nœud curobo_trajectory_planner qui implémente:
- Classic planner
- MPC planner  
- Changement dynamique de planner

Usage:
    pytest tests/integration/test_unified_planner_mpc.py -v
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner
from curobo_msgs.action import SendTrajectory

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fixtures.test_poses import TestPoses
from fixtures.test_robot_configs import TestRobotConfig


class TestUnifiedPlannerMPC:
    """Tests pour le nœud unified_planner de la branche MPC."""

    @classmethod
    def setup_class(cls):
        """Initialize ROS2."""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Shutdown ROS2."""
        rclpy.shutdown()

    def setup_method(self):
        """Create test node."""
        self.node = Node('test_unified_planner_mpc')
        
        self.set_planner_client = self.node.create_client(
            SetPlanner,
            '/unified_planner/set_planner'
        )
        self.list_planners_client = self.node.create_client(
            Trigger,
            '/unified_planner/list_planners'
        )
        self.gen_traj_client = self.node.create_client(
            TrajectoryGeneration,
            '/unified_planner/generate_trajectory'
        )
        self.execute_action = ActionClient(
            self.node,
            SendTrajectory,
            '/unified_planner/execute_trajectory'
        )

    def teardown_method(self):
        """Destroy node."""
        self.node.destroy_node()

    def test_services_available(self):
        """Test que tous les services sont disponibles."""
        services = [
            (self.set_planner_client, '/unified_planner/set_planner'),
            (self.list_planners_client, '/unified_planner/list_planners'),
            (self.gen_traj_client, '/unified_planner/generate_trajectory'),
        ]
        
        for client, service_name in services:
            assert client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT), \
                f"Service {service_name} non disponible"

    def test_action_server_available(self):
        """Test que l'action server est disponible."""
        assert self.execute_action.wait_for_server(
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT
        ), "Action server /unified_planner/execute_trajectory non disponible"

    def test_list_planners(self):
        """Test liste des planners disponibles."""
        assert self.list_planners_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        
        request = Trigger.Request()
        future = self.list_planners_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        
        response = future.result()
        assert response is not None
        assert response.success
        assert len(response.message) > 0
        
    def test_set_planner_classic(self):
        """Test changement vers Classic planner."""
        assert self.set_planner_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        
        request = SetPlanner.Request()
        request.planner_type = SetPlanner.Request.CLASSIC
        
        future = self.set_planner_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        
        response = future.result()
        assert response is not None
        assert response.success, f"Échec changement vers Classic: {response.message}"
        
    def test_set_planner_mpc(self):
        """Test changement vers MPC planner."""
        assert self.set_planner_client.wait_for_service(timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        
        request = SetPlanner.Request()
        request.planner_type = SetPlanner.Request.MPC
        
        future = self.set_planner_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, 
            future, 
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2  # MPC peut prendre plus de temps
        )
        
        response = future.result()
        assert response is not None
        assert response.success, f"Échec changement vers MPC: {response.message}"
        
    def test_switch_classic_to_mpc(self):
        """Test changement Classic → MPC."""
        # Classic
        req = SetPlanner.Request()
        req.planner_type = SetPlanner.Request.CLASSIC
        future = self.set_planner_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        
        # MPC
        req.planner_type = SetPlanner.Request.MPC
        future = self.set_planner_client.call_async(req)
        rclpy.spin_until_future_complete(
            self.node, 
            future, 
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )
        
        response = future.result()
        assert response is not None
        assert response.success

    def test_generate_trajectory_with_classic(self):
        """Test génération trajectoire avec Classic."""
        # Set Classic
        set_req = SetPlanner.Request()
        set_req.planner_type = SetPlanner.Request.CLASSIC
        future = self.set_planner_client.call_async(set_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TestRobotConfig.SERVICE_TIMEOUT)
        
        # Generate trajectory
        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_1()
        
        future = self.gen_traj_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node, 
            future, 
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )
        
        response = future.result()
        assert response is not None

    def test_generate_trajectory_with_mpc(self):
        """Test génération trajectoire avec MPC."""
        # Set MPC
        set_req = SetPlanner.Request()
        set_req.planner_type = SetPlanner.Request.MPC
        future = self.set_planner_client.call_async(set_req)
        rclpy.spin_until_future_complete(
            self.node, 
            future, 
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )
        
        # Generate trajectory (MPC setup goal buffer)
        traj_req = TrajectoryGeneration.Request()
        traj_req.target_pose = TestPoses.reach_pose_1()
        
        future = self.gen_traj_client.call_async(traj_req)
        rclpy.spin_until_future_complete(
            self.node, 
            future, 
            timeout_sec=TestRobotConfig.SERVICE_TIMEOUT * 2
        )
        
        response = future.result()
        assert response is not None


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
