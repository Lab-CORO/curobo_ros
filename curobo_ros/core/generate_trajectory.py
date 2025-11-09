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
from curobo_msgs.action import SendTrajectory

from .config_wrapper_motion import ConfigWrapperMotion

from curobo_ros.robot.robot_context import RobotContext



from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig, PoseCostMetric
from .marker_publisher import MarkerPublisher



class CuRoboTrajectoryMaker(Node):
    '''
    This class is a node that generates a trajectory for the robot to reach a given goal.
    It uses the motion generation module to generate the trajectory and the robot context to send the commands to the robot.
    It also uses the camera context to get the camera observations and the robot context to send the commands to the robot.
    The classic workflow is to set a goal from the ros2 service. The trajectory generated is show to rviz with a ghost robot. 
    To send the commands to an exeterne robot, an action is available. The feedback is percentage of advevncament of the trajectory.
    This class use a config wrapper so it has many other sesrvice available (add object, get voxel map....)
    '''
    def __init__(self):
        node_name = 'curobo_gen_traj'
        super().__init__(node_name)
        
        # Create the robot strategy
        self.robot_context = RobotContext(self, 0.03) # TODO issue dt
        self.config_wrapper = ConfigWrapperMotion(self, self.robot_context)

        # Trajectory generation parameters
        self.declare_parameter('max_attempts', 1)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('time_dilation_factor', 0.5)

        # World configuration parameters
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('collision_activation_distance', 0.025)

        # Publishers and subscribers
        self.marker_publisher = MarkerPublisher()

        self.tensor_args = TensorDeviceType()

        self.config_wrapper.set_motion_gen_config(self, None, None)

        # create an action server to send the trajectory to the robot.
        self._action_server = ActionServer(
            self,
            SendTrajectory,                   # The action type
            self.get_name() + "/send_trajectrory",     # The action name
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        #  create a service server to generate the trajectory
        self.send_trajectory_srv = self.create_service(
            TrajectoryGeneration, self.get_name() + '/generate_trajectory', self.generate_trajectrory_callback, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info("Ready to generate trajectories")

    def generate_trajectrory_callback(self,  request: TrajectoryGeneration, response):
        '''
        This method generates a trajectory to the goal pose(s).
        Supports both single and batch trajectory generation with optional constraints.
        All trajectories are processed as batches internally.
        '''
        try:
            max_attempts = self.get_parameter(
                'max_attempts').get_parameter_value().integer_value
            timeout = self.get_parameter(
                'timeout').get_parameter_value().double_value
            time_dilation_factor = self.get_parameter(
                'time_dilation_factor').get_parameter_value().double_value

            # Create plan config
            plan_config = MotionGenPlanConfig(
                max_attempts=max_attempts,
                timeout=timeout,
                time_dilation_factor=time_dilation_factor,
            )

            # Add constraints if provided
            if request.hold_partial_pose and len(request.hold_vec_weight) == 7:
                # Convert hold_vec_weight to tensor [qw, qx, qy, qz, px, py, pz]
                hold_vec = self.tensor_args.to_device(request.hold_vec_weight)
                pose_cost_metric = PoseCostMetric(
                    hold_partial_pose=True,
                    hold_vec_weight=hold_vec,
                )
                plan_config.pose_cost_metric = pose_cost_metric
                self.get_logger().info(f"Using pose constraints: {request.hold_vec_weight}")

            # Build target list: use batch_targets if provided, otherwise use single target_pose
            targets = request.batch_targets if len(request.batch_targets) > 0 else [request.target_pose]

            num_requested = len(targets)

            self.get_logger().info(f"Planning for {num_requested} target(s)")

            # Get the warmup batch_size
            warmup_batch_size = self.warmup_batch_size

            # Check if requested batch size exceeds warmup batch size
            if num_requested > warmup_batch_size:
                error_msg = (f"Requested {num_requested} targets but motion_gen was warmed up with batch_size={warmup_batch_size}. "
                            f"Please set 'batch_size' parameter to at least {num_requested} and call "
                            f"'/curobo_gen_traj/update_motion_gen_config' service to rewarmup.")
                self.get_logger().error(error_msg)
                response.success = False
                response.message = error_msg
                response.success_indices = []
                return response

            # Build goal_poses_list
            goal_poses_list = []
            for target in targets:
                pose = [
                    target.position.x, target.position.y, target.position.z,
                    target.orientation.w, target.orientation.x,
                    target.orientation.y, target.orientation.z
                ]
                goal_poses_list.append(pose)

            # Pad goal_poses_list to match warmup_batch_size by repeating last pose
            if num_requested < warmup_batch_size:
                self.get_logger().info(f"Padding batch from {num_requested} to {warmup_batch_size} (repeating last pose)")
                last_pose = goal_poses_list[-1]
                for _ in range(warmup_batch_size - num_requested):
                    goal_poses_list.append(last_pose)

            batch_size = len(goal_poses_list)  # Should equal warmup_batch_size

            # Get Robot pose
            current_joint_pose = self.robot_context.get_joint_pose()

            # Always use the planning method that matches warmup
            if warmup_batch_size == 1:
                # Single mode - use plan_single (warmed up for single)
                self.start_state = JointState.from_position(
                    torch.Tensor([current_joint_pose]).to(device=self.tensor_args.device)
                )
                self.goal_pose = Pose.from_list(goal_poses_list[0], tensor_args=self.tensor_args)

                result = self.motion_gen.plan_single(
                    self.start_state,
                    self.goal_pose,
                    plan_config,
                )
            else:
                # Batch mode - use plan_batch with exact warmup_batch_size (with padding if needed)
                start_positions = torch.Tensor([current_joint_pose for _ in range(batch_size)]).to(device=self.tensor_args.device)
                self.start_state = JointState.from_position(start_positions)

                # Stack poses into batch
                self.goal_pose = Pose.from_batch_list(goal_poses_list, tensor_args=self.tensor_args)
                
                # Plan batch trajectories
                result = self.motion_gen.plan_goalset(
                    self.start_state,
                    self.goal_pose,
                    plan_config,
                )
                self.get_logger().info("TEST")
            # Check which trajectories succeeded (only consider requested ones, not padded)
            if warmup_batch_size == 1:
                # Single mode - result.success is a scalar boolean
                success = result.success.item() if hasattr(result.success, 'item') else result.success
                success_indices = [0] if success else []
            else:
                # Batch mode - result.success is a tensor of booleans
                success_mask = result.success
                all_success_indices = torch.nonzero(success_mask).squeeze(-1).cpu().tolist()
                # Handle case where success_indices is a single int
                if isinstance(all_success_indices, int):
                    all_success_indices = [all_success_indices]
                # Only keep indices that correspond to requested targets (not padded ones)
                success_indices = [idx for idx in all_success_indices if idx < num_requested]

            response.success_indices = success_indices

            if len(success_indices) > 0:
                # Get the trajectory
               
                self.get_logger().info("coucou")
                # Extract the successful trajectory
                if warmup_batch_size > 1:
                    traj_single = result.get_successful_paths() #.get_interpolated_plan()
                    self.get_logger().info(traj_single)
                    # Get specific trajectory from batch
                    # first_success_idx = success_indices[0]
                    # traj_single = JointState(
                    #     position=traj.position[first_success_idx],
                    #     velocity=traj.velocity[first_success_idx],
                    #     acceleration=traj.acceleration[first_success_idx],
                    #     joint_names=traj.joint_names
                    # )
                else:
                    traj = result.get_interpolated_plan()
                    # Single trajectory - already in correct format from plan_single
                    traj_single = traj

                # Send command to robot context
                self.robot_context.set_command(
                    traj_single.joint_names,
                    traj_single.velocity.tolist(),
                    traj_single.acceleration.tolist(),
                    traj_single.position.tolist()
                )
                response.success = True
                response.message = f"Trajectory generated. {len(success_indices)}/{num_requested} requested succeeded"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f"No trajectories succeeded (requested: {num_requested}, warmup_batch_size: {warmup_batch_size})"
                self.get_logger().error(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Error during trajectory generation: {str(e)}"
            response.success_indices = []
            self.get_logger().error(f"Exception during trajectory generation: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())

        return response
        
    def execute_callback(self, goal_handle):
        '''
        This method sends the trajectory to the robot (currently with a twist message). Currently there is not verifications (TODO). 
        '''
        self.robot_context.send_trajectrory()

        # Here we wait the message from leeloo to 
        start_time = time.time()
        progression = self.robot_context.get_progression()
        time_dilation_factor = self.get_parameter(
                'time_dilation_factor').get_parameter_value().double_value
        while progression < 1.0 and self.is_goal_up is True:
            if (time.time() - start_time) > time_dilation_factor:
                feedback_msg = SendTrajectory.Feedback()
                feedback_msg.step_progression = self.robot_context.get_progression()
                goal_handle.publish_feedback(feedback_msg)
                progression = self.robot_context.get_progression()
                start_time = time.time()

        # Stop the robot and clean the command list at the end.
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
        # self.robot_context.set_send_to_robot(False)
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
