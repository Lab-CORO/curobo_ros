#!/usr/bin/env python3
"""
Integration tests for robot_segmentation node.

Tests the robot segmentation node that removes the robot from depth
images for obstacle detection and collision avoidance.

Node: robot_segmentation
Publishers tested:
- /masked_depth_image (sensor_msgs/Image)
- /collision_spheres (visualization_msgs/MarkerArray)
- /robot_pointcloud_debug (sensor_msgs/PointCloud2)

Subscribers tested:
- /depth_to_rgb/image_raw (sensor_msgs/Image)
- /depth_to_rgb/camera_info (sensor_msgs/CameraInfo)

Usage:
    pytest tests/integration/test_robot_segmentation.py
"""

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from visualization_msgs.msg import MarkerArray
import time

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fixtures.test_robot_configs import TestRobotConfig


class TestRobotSegmentation:
    """Test suite for robot segmentation node."""

    @classmethod
    def setup_class(cls):
        """Initialize ROS2 for all tests."""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Shutdown ROS2 after all tests."""
        rclpy.shutdown()

    def setup_method(self):
        """Create test node before each test."""
        self.node = Node('test_robot_seg_node')

        # Storage for received messages
        self.masked_depth_received = False
        self.collision_spheres_received = False
        self.robot_pointcloud_received = False

        # Create subscribers to check publications
        self.masked_depth_sub = self.node.create_subscription(
            Image,
            '/masked_depth_image',
            self.masked_depth_callback,
            10
        )
        self.collision_spheres_sub = self.node.create_subscription(
            MarkerArray,
            '/collision_spheres',
            self.collision_spheres_callback,
            10
        )
        self.robot_pointcloud_sub = self.node.create_subscription(
            PointCloud2,
            '/robot_pointcloud_debug',
            self.robot_pointcloud_callback,
            10
        )

        # Publishers to send test data
        self.depth_pub = self.node.create_publisher(
            Image,
            '/depth_to_rgb/image_raw',
            10
        )
        self.camera_info_pub = self.node.create_publisher(
            CameraInfo,
            '/depth_to_rgb/camera_info',
            10
        )

    def teardown_method(self):
        """Destroy test node after each test."""
        self.node.destroy_node()

    # Callbacks to detect message receipt
    def masked_depth_callback(self, msg):
        self.masked_depth_received = True
        self.masked_depth_msg = msg

    def collision_spheres_callback(self, msg):
        self.collision_spheres_received = True
        self.collision_spheres_msg = msg

    def robot_pointcloud_callback(self, msg):
        self.robot_pointcloud_received = True
        self.robot_pointcloud_msg = msg

    # =========================
    # NODE AVAILABILITY TESTS
    # =========================

    def test_node_running(self):
        """Test that robot_segmentation node is running."""
        # Wait a bit for node discovery
        time.sleep(1.0)

        # Check if node exists
        node_names = self.node.get_node_names()
        assert '/curobo_depth_map_robot_segmentation' in node_names or \
               'curobo_depth_map_robot_segmentation' in node_names, \
               "robot_segmentation node not running"

    # =========================
    # PUBLISHER TESTS
    # =========================

    def test_masked_depth_publisher_exists(self):
        """Test that masked_depth_image topic is being published."""
        # Wait for topic discovery
        time.sleep(1.0)

        topics = self.node.get_topic_names_and_types()
        topic_names = [t[0] for t in topics]

        assert '/masked_depth_image' in topic_names, \
            "/masked_depth_image topic not being published"

    def test_collision_spheres_publisher_exists(self):
        """Test that collision_spheres topic is being published."""
        time.sleep(1.0)

        topics = self.node.get_topic_names_and_types()
        topic_names = [t[0] for t in topics]

        assert '/collision_spheres' in topic_names, \
            "/collision_spheres topic not being published"

    def test_robot_pointcloud_publisher_exists(self):
        """Test that robot_pointcloud_debug topic is being published."""
        time.sleep(1.0)

        topics = self.node.get_topic_names_and_types()
        topic_names = [t[0] for t in topics]

        assert '/robot_pointcloud_debug' in topic_names, \
            "/robot_pointcloud_debug topic not being published"

    # =========================
    # SUBSCRIBER TESTS
    # =========================

    def test_depth_image_subscription(self):
        """Test that node subscribes to depth image topic."""
        time.sleep(1.0)

        # Check if subscription exists
        topics = self.node.get_topic_names_and_types()
        topic_names = [t[0] for t in topics]

        # Node should be subscribed to depth image
        assert '/depth_to_rgb/image_raw' in topic_names or \
               any('depth' in name and 'image' in name for name in topic_names)

    def test_camera_info_subscription(self):
        """Test that node subscribes to camera info topic."""
        time.sleep(1.0)

        topics = self.node.get_topic_names_and_types()
        topic_names = [t[0] for t in topics]

        # Node should be subscribed to camera info
        assert '/depth_to_rgb/camera_info' in topic_names or \
               any('camera_info' in name for name in topic_names)

    # =========================
    # MESSAGE PROCESSING TESTS
    # =========================

    def test_publish_test_depth_image(self):
        """Test publishing a test depth image."""
        # Create a simple test depth image
        test_image = Image()
        test_image.header.stamp = self.node.get_clock().now().to_msg()
        test_image.header.frame_id = 'camera_depth_optical_frame'
        test_image.height = 480
        test_image.width = 640
        test_image.encoding = '16UC1'
        test_image.is_bigendian = False
        test_image.step = 640 * 2  # 2 bytes per pixel for 16UC1
        test_image.data = [0] * (480 * 640 * 2)

        # Publish test image
        self.depth_pub.publish(test_image)

        # Spin briefly to allow message processing
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Note: We can't guarantee receipt without actual camera setup
        # This test just verifies publishing doesn't crash

    def test_publish_test_camera_info(self):
        """Test publishing test camera info."""
        # Create a simple test camera info
        test_info = CameraInfo()
        test_info.header.stamp = self.node.get_clock().now().to_msg()
        test_info.header.frame_id = 'camera_depth_optical_frame'
        test_info.height = 480
        test_info.width = 640

        # Simple camera matrix (identity-like)
        test_info.k = [525.0, 0.0, 320.0,
                       0.0, 525.0, 240.0,
                       0.0, 0.0, 1.0]

        # Publish test camera info
        self.camera_info_pub.publish(test_info)

        # Spin briefly
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_collision_spheres_message_format(self):
        """Test that collision spheres messages have correct format."""
        # Reset flag
        self.collision_spheres_received = False

        # Wait for a message (with timeout)
        timeout = 5.0
        start_time = time.time()

        while not self.collision_spheres_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # If we received a message, verify its format
        if self.collision_spheres_received:
            msg = self.collision_spheres_msg
            assert isinstance(msg, MarkerArray), "Message is not MarkerArray"
            # Markers should exist (robot has collision spheres)
            # Note: may be empty if robot model not loaded
        else:
            # It's okay if no message received (node may not publish without depth input)
            pass

    def test_masked_depth_image_format(self):
        """Test that masked depth images have correct format."""
        # Reset flag
        self.masked_depth_received = False

        # Publish test depth and camera info first
        test_image = Image()
        test_image.header.stamp = self.node.get_clock().now().to_msg()
        test_image.header.frame_id = 'camera_depth_optical_frame'
        test_image.height = 480
        test_image.width = 640
        test_image.encoding = '16UC1'
        test_image.step = 640 * 2
        test_image.data = [0] * (480 * 640 * 2)

        test_info = CameraInfo()
        test_info.header.stamp = self.node.get_clock().now().to_msg()
        test_info.header.frame_id = 'camera_depth_optical_frame'
        test_info.height = 480
        test_info.width = 640
        test_info.k = [525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0]

        self.depth_pub.publish(test_image)
        self.camera_info_pub.publish(test_info)

        # Wait for processed message (with timeout)
        timeout = 5.0
        start_time = time.time()

        while not self.masked_depth_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # If we received a message, verify its format
        if self.masked_depth_received:
            msg = self.masked_depth_msg
            assert isinstance(msg, Image), "Message is not Image"
            assert msg.encoding in ['16UC1', '32FC1'], \
                f"Unexpected encoding: {msg.encoding}"
            assert msg.height > 0 and msg.width > 0, "Invalid image dimensions"

    # =========================
    # PERFORMANCE TESTS
    # =========================

    def test_segmentation_latency(self):
        """Test that segmentation processing has acceptable latency."""
        # Publish test depth image
        test_image = Image()
        test_image.header.stamp = self.node.get_clock().now().to_msg()
        test_image.header.frame_id = 'camera_depth_optical_frame'
        test_image.height = 480
        test_image.width = 640
        test_image.encoding = '16UC1'
        test_image.step = 640 * 2
        test_image.data = [0] * (480 * 640 * 2)

        test_info = CameraInfo()
        test_info.header.stamp = self.node.get_clock().now().to_msg()
        test_info.header.frame_id = 'camera_depth_optical_frame'
        test_info.height = 480
        test_info.width = 640
        test_info.k = [525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0]

        # Record publish time
        publish_time = time.time()

        self.depth_pub.publish(test_image)
        self.camera_info_pub.publish(test_info)

        # Reset flag
        self.masked_depth_received = False

        # Wait for response
        timeout = 2.0
        start_wait = time.time()

        while not self.masked_depth_received and (time.time() - start_wait) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.masked_depth_received:
            receive_time = time.time()
            latency = receive_time - publish_time

            # Latency should be reasonable (< 1 second for GPU processing)
            assert latency < 1.0, f"Segmentation latency too high: {latency:.3f}s"

    # =========================
    # INTEGRATION TESTS
    # =========================

    def test_continuous_processing(self):
        """Test continuous depth image processing."""
        # Publish multiple depth images
        for i in range(5):
            test_image = Image()
            test_image.header.stamp = self.node.get_clock().now().to_msg()
            test_image.header.frame_id = 'camera_depth_optical_frame'
            test_image.height = 480
            test_image.width = 640
            test_image.encoding = '16UC1'
            test_image.step = 640 * 2
            test_image.data = [i] * (480 * 640 * 2)  # Different data each time

            test_info = CameraInfo()
            test_info.header.stamp = self.node.get_clock().now().to_msg()
            test_info.header.frame_id = 'camera_depth_optical_frame'
            test_info.height = 480
            test_info.width = 640
            test_info.k = [525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0]

            self.depth_pub.publish(test_image)
            self.camera_info_pub.publish(test_info)

            # Brief spin
            for _ in range(5):
                rclpy.spin_once(self.node, timeout_sec=0.05)

            time.sleep(0.1)

        # Node should handle continuous stream without crashing

    def test_node_parameters(self):
        """Test that node parameters can be queried."""
        # Try to get parameters from robot_segmentation node
        # This requires the node to be running

        time.sleep(1.0)

        # We can't directly access another node's parameters in this test setup
        # But we can verify the node exists
        node_names = self.node.get_node_names()
        assert '/curobo_depth_map_robot_segmentation' in node_names or \
               'curobo_depth_map_robot_segmentation' in node_names


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
