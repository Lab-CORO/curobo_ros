#!/usr/bin/env python3
"""Benchmark get_voxel_grid at configurable resolution.

Usage (node must already be running):
    ros2 run curobo_ros benchmark_voxel_grid
    ros2 run curobo_ros benchmark_voxel_grid --ros-args -p voxel_size:=0.02
    ros2 run curobo_ros benchmark_voxel_grid --ros-args -p voxel_size:=0.02 -p n_runs:=10
"""

import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from curobo_msgs.srv import GetVoxelGrid


TARGET_NODE = '/unified_planner'


class VoxelBenchmark(Node):

    def __init__(self):
        super().__init__('voxel_benchmark')
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('n_runs', 5)

    def run(self):
        voxel_size = self.get_parameter('voxel_size').value
        n_runs = int(self.get_parameter('n_runs').value)

        # Set voxel_size on the planner node
        param_client = self.create_client(
            SetParameters, f'{TARGET_NODE}/set_parameters')
        if not param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('set_parameters service not available - is the node running?')
            return

        pval = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(voxel_size))
        req = SetParameters.Request(parameters=[Parameter(name='voxel_size', value=pval)])
        fut = param_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or not fut.result().results[0].successful:
            self.get_logger().error(f'Failed to set voxel_size={voxel_size}')
            return
        self.get_logger().info(f'voxel_size set to {voxel_size} m on {TARGET_NODE}')

        # Compute expected grid dimensions
        import math
        extent = 2.0  # default mapper_extent_xyz
        n_side = math.ceil(extent / voxel_size)
        n_total = n_side ** 3
        self.get_logger().info(
            f'Expected grid: {n_side}^3 = {n_total:,} voxels '
            f'({n_total * 4 / 1024**2:.1f} MB as uint32)')

        # Get voxel grid client
        vg_client = self.create_client(GetVoxelGrid, f'{TARGET_NODE}/get_voxel_grid')
        if not vg_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('get_voxel_grid service not available')
            return

        # Warmup call (triggers GPU buffer pre-allocation)
        self.get_logger().info('Warmup call (pre-allocates GPU buffers)...')
        t0 = time.perf_counter()
        fut = vg_client.call_async(GetVoxelGrid.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        t_warmup = time.perf_counter() - t0
        if not fut.done() or fut.result() is None:
            self.get_logger().error('Warmup call timed out')
            return
        resp = fut.result()
        n_occ = sum(resp.voxel_grid.data)
        actual_size = (resp.voxel_grid.size_x, resp.voxel_grid.size_y, resp.voxel_grid.size_z)
        self.get_logger().info(
            f'Warmup: {t_warmup*1000:.1f} ms  |  '
            f'grid={actual_size}  |  occupied={n_occ:,}')

        # Timed runs
        times = []
        for i in range(n_runs):
            fut = vg_client.call_async(GetVoxelGrid.Request())
            t0 = time.perf_counter()
            rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
            dt = time.perf_counter() - t0
            times.append(dt)
            self.get_logger().info(f'  run {i+1}/{n_runs}: {dt*1000:.1f} ms')

        mean_ms = sum(times) / len(times) * 1000
        min_ms  = min(times) * 1000
        max_ms  = max(times) * 1000
        self.get_logger().info(
            f'\n'
            f'=== Benchmark results - voxel_size={voxel_size} m ===\n'
            f'  Grid size : {actual_size[0]}x{actual_size[1]}x{actual_size[2]}'
            f' = {actual_size[0]*actual_size[1]*actual_size[2]:,} voxels\n'
            f'  Occupied  : {n_occ:,}\n'
            f'  Warmup    : {t_warmup*1000:.1f} ms  (GPU buffer allocation)\n'
            f'  Mean      : {mean_ms:.1f} ms\n'
            f'  Min/Max   : {min_ms:.1f} / {max_ms:.1f} ms\n'
            f'  ({n_runs} runs, buffers pre-allocated)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VoxelBenchmark()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
