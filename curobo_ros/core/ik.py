"""
Standalone IK node (v2).

v2 notes:
- IKSolver / IKSolverConfig → InverseKinematics / InverseKinematicsCfg.
  These are created through ConfigWrapperIK, so most of the v2 wiring lives
  in `config_wrapper_motion.py`.
- TensorDeviceType → DeviceCfg. We only read device/dtype from the wrapper.
- `collision_checker_type=BLOX` is gone: collision is a flag on the Cfg.
"""

import rclpy
import std_msgs.msg
import torch
from rclpy.node import Node
from sensor_msgs.msg import JointState

from curobo.types.math import Pose

from .config_wrapper_motion import ConfigWrapperIK
from curobo_ros.robot.robot_context import RobotContext


class IK(Node):
    def __init__(self):
        super().__init__('curobo_ik')

        self.declare_parameter('voxel_size', 0.5)
        self.declare_parameter('init_batch_size', 1000)
        self.size_init = self.get_parameter(
            'init_batch_size'
        ).get_parameter_value().integer_value

        self.robot_context = RobotContext(self, 0.03)
        self.config_wrapper = ConfigWrapperIK(self, self.robot_context)

        self._device = self.config_wrapper._device
        self._dtype = self.config_wrapper._ops_dtype

    # ----- ROS service callbacks -----

    def ik_callback(self, request, response):
        ok, ik_result = self.get_ik([request.pose])
        if not ok:
            response.success = False
            return response

        for index, j in enumerate(ik_result.solution.cpu().numpy()):
            joint = JointState()
            joint.position = j[0].tolist()

            valid = std_msgs.msg.Bool()
            valid.data = bool(ik_result.success.cpu().numpy()[index][0])

            response.joint_states_valid = valid
            response.joint_states = joint
            response.success = True
        return response

    def ik_batch_callback(self, request, response):
        ok, ik_result = self.get_ik(request.poses)
        if not ok:
            response.success = False
            return response

        for index, j in enumerate(ik_result.solution.cpu().numpy()):
            joint = JointState()
            joint.position = j[0].tolist()

            valid = std_msgs.msg.Bool()
            valid.data = bool(ik_result.success.cpu().numpy()[index][0])

            response.joint_states_valid.append(valid)
            response.joint_states.append(joint)
            response.success = True
        return response

    # ----- Helpers -----

    def get_ik(self, poses):
        """Solve IK for a list of geometry_msgs/Pose. Returns (ok, result)."""
        if len(poses) == 0:
            self.get_logger().info("0 pose requested")
            return False, []

        try:
            if len(poses) != self.size_init:
                self.config_wrapper.set_ik_gen_config(self, None, None)
                self.size_init = len(poses)
                self.ik_init()
        except Exception:
            self.size_init = 0
            return False, []

        positions = [[p.position.x, p.position.y, p.position.z] for p in poses]
        orientations = [
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            for p in poses
        ]

        goal = Pose(
            torch.tensor(positions, dtype=self._dtype, device=self._device),
            torch.tensor(orientations, dtype=self._dtype, device=self._device),
        )

        try:
            result = self.ik_solver.solve_batch(goal)
        except Exception:
            try:
                self.ik_init()
                result = self.ik_solver.solve_batch(goal)
            except Exception:
                self.size_init = 0
                return False, []
        torch.cuda.synchronize()
        return True, result

    def ik_init(self):
        """Prime CUDA kernels with a random batch at the current size."""
        q_sample = self.ik_solver.sample_configs(self.size_init)
        kin_state = self.ik_solver.fk(q_sample)
        goal = Pose(kin_state.ee_position, kin_state.ee_quaternion)
        self.ik_solver.solve_batch(goal)
        torch.cuda.synchronize()
        self.get_logger().info("Init done")

    def add_collisions(self):
        return True


def main(args=None):
    rclpy.init(args=args)
    ik = IK()
    rclpy.spin(ik)
    ik.destroy_node()
    rclpy.shutdown()
