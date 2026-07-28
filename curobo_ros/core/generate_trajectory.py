#!/usr/bin/env python3
"""
Standalone trajectory generation node (v2).

v2 notes:
- MotionGen → MotionPlanner (wired via ConfigWrapperMotion).
- `MotionGenPlanConfig` is gone: per-call params are keyword args on `plan_pose`.
- `plan_single` → `plan_pose`. Goal is a `GoalToolPose` wrapping a `ToolPose`.
- `TensorDeviceType` → `DeviceCfg`. Device/dtype is read from the wrapper.
"""

import time

import rclpy
import torch
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from curobo_msgs.srv import TrajectoryGeneration
from curobo_msgs.action import SendTrajectory

from curobo.types import JointState, Pose, GoalToolPose

from .config_wrapper_motion import ConfigWrapperMotion
from curobo_ros.robot.robot_context import RobotContext


class CuRoboTrajectoryMaker(Node):
    """
    Standalone node that plans and executes a single-goal trajectory.

    The wider workflow (service → plan_pose → robot command) is unchanged from v1.
    The internals moved off MotionGen/MotionGenPlanConfig onto the v2 MotionPlanner.
    """

    def __init__(self):
        super().__init__('curobo_gen_traj')

        self.robot_context = RobotContext(self)
        self.config_wrapper = ConfigWrapperMotion(self, self.robot_context)
        # Buffer epoch returned by set_command() in generate_trajectrory_callback,
        # paired with send_trajectrory(expect_epoch=...) in execute_callback —
        # they are separate calls (service, then later an action), so another
        # set_command() could otherwise land in between. See M2 in the
        # pre-publication audit.
        self._command_epoch = None

        self.declare_parameter('max_attempts', 1)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('time_dilation_factor', 0.5)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('collision_activation_distance', 0.025)

        self._device = self.config_wrapper._device
        self._dtype = self.config_wrapper._ops_dtype

        # Builds and warms up self.motion_planner (alias self.motion_gen).
        self.config_wrapper.set_motion_gen_config(self, None, None)

        self._action_server = ActionServer(
            self,
            SendTrajectory,
            self.get_name() + "/send_trajectrory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.send_trajectory_srv = self.create_service(
            TrajectoryGeneration,
            self.get_name() + '/generate_trajectory',
            self.generate_trajectrory_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info("Ready to generate trajectories")

    # ---- Service callbacks ----

    def generate_trajectrory_callback(self, request: TrajectoryGeneration, response):
        """Plan a trajectory from the robot's current joint state to a goal pose."""
        start_state = JointState.from_position(
            torch.tensor([self.robot_context.get_joint_pose()],
                         dtype=self._dtype, device=self._device)
        )

        # v2 Pose.from_list expects [x, y, z, qw, qx, qy, qz] (wxyz).
        pose = Pose.from_list([
            request.target_pose.position.x,
            request.target_pose.position.y,
            request.target_pose.position.z,
            request.target_pose.orientation.w,
            request.target_pose.orientation.x,
            request.target_pose.orientation.y,
            request.target_pose.orientation.z,
        ])
        tool_frame = self.motion_planner.tool_frames[0]
        goal = GoalToolPose.from_poses({tool_frame: pose})

        max_attempts = self.get_parameter('max_attempts').get_parameter_value().integer_value

        try:
            result = self.motion_planner.plan_pose(
                goal,
                start_state,
                max_attempts=max_attempts,
            )
            if result is None or not bool(
                result.success.item() if hasattr(result.success, 'item') else result.success
            ):
                response.success = False
                response.message = "Trajectory planning failed (no solution)"
                return response
            traj = result.get_interpolated_plan()
            pos, vel, acc = traj.position, traj.velocity, traj.acceleration
            if pos.ndim == 3:
                pos, vel, acc = pos[0], vel[0], acc[0]
            self._command_epoch = self.robot_context.set_command(
                traj.joint_names,
                vel.tolist(),
                acc.tolist(),
                pos.tolist(),
            )
            response.success = True
            response.message = "Trajectory generated"
        except Exception as e:
            response.success = False
            response.message = f"Error: trajectory could not be generated ({e})"
            self.get_logger().error(response.message)
        return response

    def execute_callback(self, goal_handle):
        """Send the planned trajectory to the robot and publish progress feedback."""
        # expect_epoch guards against another set_command() landing between
        # generate_trajectrory_callback (which set _command_epoch) and this
        # call — see JointCommandStrategy.buffer_epoch / M2.
        if not self.robot_context.send_trajectrory(expect_epoch=self._command_epoch):
            self.get_logger().error(
                "Refusing to execute - the planned trajectory was superseded "
                "by a newer command before execution started."
            )
            self.is_goal_up = False
            result_msg = SendTrajectory.Result()
            result_msg.success = False
            result_msg.message = "Trajectory superseded before execution"
            goal_handle.abort()
            return result_msg

        start_time = time.time()
        progression = self.robot_context.get_progression()
        time_dilation_factor = self.get_parameter(
            'time_dilation_factor'
        ).get_parameter_value().double_value
        while progression < 1.0 and self.is_goal_up is True:
            if (time.time() - start_time) > time_dilation_factor:
                feedback_msg = SendTrajectory.Feedback()
                feedback_msg.step_progression = self.robot_context.get_progression()
                goal_handle.publish_feedback(feedback_msg)
                progression = self.robot_context.get_progression()
                start_time = time.time()

        self.robot_context.stop_robot()
        self.is_goal_up = False
        result_msg = SendTrajectory.Result()
        result_msg.success = True
        result_msg.message = "Goal reached"
        goal_handle.succeed()
        return result_msg

    def goal_callback(self, goal):
        self.get_logger().info("Received goal request")
        self.is_goal_up = True
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.robot_context.stop_robot()
        self.is_goal_up = False
        self.get_logger().info("Canceling goal")
        return True


def main(args=None):
    rclpy.init(args=args)
    node_curobo = CuRoboTrajectoryMaker()

    executor = MultiThreadedExecutor()
    executor.add_node(node_curobo)

    try:
        node_curobo.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node_curobo.get_logger().info('Keyboard interrupt, shutting down.\n')
    node_curobo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
