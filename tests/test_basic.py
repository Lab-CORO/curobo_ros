#!/usr/bin/env python3
"""
Basic test to verify curobo_gen_traj node is running and ready.

This test:
1. Checks if the node is running
2. Verifies required services are available
3. Calls the is_available service to check warmup status

Usage:
    python3 tests/test_basic.py
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys
import time


class BasicTest(Node):
    def __init__(self):
        super().__init__('basic_test_node')
        self.get_logger().info("🧪 Starting basic curobo_ros test...")

    def test_node_running(self):
        """Check if curobo_gen_traj node is running"""
        self.get_logger().info("📋 Test 1: Checking if curobo_gen_traj node is running...")

        node_names = self.get_node_names()
        if '/curobo_gen_traj' in node_names:
            self.get_logger().info("✅ curobo_gen_traj node is running")
            return True
        else:
            self.get_logger().error("❌ curobo_gen_traj node is NOT running")
            self.get_logger().error("   Please start it with: ros2 launch curobo_ros gen_traj.launch.py")
            return False

    def test_services_available(self):
        """Check if required services are available"""
        self.get_logger().info("📋 Test 2: Checking if required services are available...")

        required_services = [
            '/curobo_gen_traj/is_available',
            '/curobo_gen_traj/generate_trajectory',
            '/curobo_gen_traj/add_object',
            '/curobo_gen_traj/remove_object',
            '/curobo_gen_traj/remove_all_objects',
        ]

        all_available = True
        for service_name in required_services:
            service_available = service_name in self.get_service_names_and_types_by_node(
                'curobo_gen_traj', '/curobo_gen_traj'
            )

            if service_available:
                self.get_logger().info(f"  ✅ {service_name}")
            else:
                self.get_logger().error(f"  ❌ {service_name} NOT FOUND")
                all_available = False

        if all_available:
            self.get_logger().info("✅ All required services are available")
        else:
            self.get_logger().error("❌ Some services are missing")

        return all_available

    def test_node_ready(self):
        """Check if node is ready (warmup complete)"""
        self.get_logger().info("📋 Test 3: Checking if node completed warmup...")

        # Create service client
        client = self.create_client(Trigger, '/curobo_gen_traj/is_available')

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ /curobo_gen_traj/is_available service not available")
            return False

        # Call the service
        request = Trigger.Request()
        future = client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info("✅ Node is ready! Warmup complete.")
                self.get_logger().info(f"   Message: {response.message}")
                return True
            else:
                self.get_logger().warn("⚠️  Node is still warming up...")
                self.get_logger().warn(f"   Message: {response.message}")
                self.get_logger().warn("   Please wait a few seconds and try again.")
                return False
        else:
            self.get_logger().error("❌ Service call failed or timed out")
            return False


def main():
    """Main test function"""
    print("\n" + "="*60)
    print("  curobo_ros Basic Verification Test")
    print("="*60 + "\n")

    rclpy.init()
    test_node = BasicTest()

    # Run all tests
    results = []

    # Test 1: Node running
    results.append(("Node Running", test_node.test_node_running()))
    time.sleep(0.5)

    # Test 2: Services available (only if node is running)
    if results[0][1]:
        results.append(("Services Available", test_node.test_services_available()))
        time.sleep(0.5)
    else:
        results.append(("Services Available", False))

    # Test 3: Node ready (only if services are available)
    if results[1][1]:
        results.append(("Node Ready", test_node.test_node_ready()))
    else:
        results.append(("Node Ready", False))

    # Print summary
    print("\n" + "="*60)
    print("  Test Summary")
    print("="*60)
    for test_name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"  {test_name:.<40} {status}")
    print("="*60 + "\n")

    # Overall result
    all_passed = all(result[1] for result in results)
    if all_passed:
        print("🎉 All tests passed! Your curobo_ros installation is working correctly.")
        print("\nNext steps:")
        print("  1. Try generating a trajectory:")
        print("     ros2 service call /curobo_gen_traj/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \\")
        print("       \"{target_pose: {position: {x: 0.5, y: 0.2, z: 0.3}, orientation: {w: 1.0, x: 0, y: 0, z: 0}}}\"")
        print("\n  2. Read the tutorials:")
        print("     - doc/tutorials/1_first_trajectory.md")
        print("     - doc/tutorials/02-adding-your-robot.md")
        exit_code = 0
    else:
        print("⚠️  Some tests failed. Please check the output above for details.")
        print("\nCommon issues:")
        print("  - Node not running: Launch it with 'ros2 launch curobo_ros gen_traj.launch.py'")
        print("  - Node still warming up: Wait 30-60 seconds after launch")
        print("  - Services missing: Check if the launch completed successfully")
        print("\nSee doc/troubleshooting.md for more help.")
        exit_code = 1

    test_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
