#!/usr/bin/env python3

import torch
import numpy as np

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from curobo_msgs.srv import TrajectoryGeneration
from curobo_msgs.action import MpcMove

from .config_wrapper_motion import ConfigWrapperMPC

from curobo_ros.robot.robot_context import RobotContext



from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.rollout.rollout_base import Goal
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
from .marker_publisher import MarkerPublisher


class CuroboMPC(Node):

    def __init__(self):
        node_name = 'curobo_mpc'
        super().__init__(node_name)

        # Create the robot strategy
        self.robot_context = RobotContext(self, 0.03) # TODO issue dt
        self.config_wrapper = ConfigWrapperMPC(self, self.robot_context)

        # Load curobo config
        self.tensor_args = TensorDeviceType()

        # self.config_wrapper.(self, None, None)
        
        # Create Action
        self._action_server = ActionServer(
            self,
            MpcMove,                   # The action type
            self.get_name() + "/mpc_move",     # The action name
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        

    def execute_callback(self, goal_handle):

        # Get Robot pose
        self.start_state = JointState.from_position(torch.Tensor([self.robot_context.get_joint_pose()]).to(device=self.tensor_args.device))

        # Get goal pose
        self.goal_pose = Pose.from_list([
            goal_handle.request.target_pose.position.x, goal_handle.request.target_pose.position.y, goal_handle.request.target_pose.position.z,
            goal_handle.request.target_pose.orientation.x, goal_handle.request.target_pose.orientation.y,
            goal_handle.request.target_pose.orientation.z, goal_handle.request.target_pose.orientation.w
        ])

        goal = Goal(
            current_state=self.start_state,
            goal_pose=self.goal_pose,
        )
        goal_buffer = self.mpc.setup_solve_single(goal, 1)

        converged = False
        tstep = 0
        traj_list = []
        mpc_time = []
        self.mpc.update_goal(goal_buffer)
        current_state = self.start_state  # .clone()
        while not converged and self.is_goal_up:
            st_time = time.time()
            # current_state.position += 0.1
            # print(current_state.position)
            result = self.mpc.step(current_state, 1)

            # print(mpc.get_visual_rollouts().shape)
            # exit()
            torch.cuda.synchronize()
            if tstep > 5:
                mpc_time.append(time.time() - st_time)
            # goal_buffer.current_state.position[:] = result.action.position
            # result.action.position += 0.1
            current_state.copy_(result.action)
            # goal_buffer.current_state.velocity[:] = result.action.vel
            traj_list.append(result.action.get_state_tensor())
            
            tstep += 1
            # if tstep % 10 == 0:
            #    print(result.metrics.pose_error.item(), result.solve_time, mpc_time[-1])
            if result.metrics.pose_error.item() < 0.01:
                converged = True
            if tstep > 1000:
                break
        print(traj_list)
        self.get_logger().info("End request")
        result_msg = MpcMove.Result()
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
    node_curobo = CuroboMPC()

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
