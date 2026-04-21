#!/usr/bin/env python3
"""
Unified trajectory planner node (v2) using Strategy Pattern.

Supports multiple planning strategies (Classic, MPC, Multi-Point, Joint-Space)
and allows dynamic switching between them.

v2 notes:
- MotionGen → MotionPlanner (wired via ConfigWrapperMotion as `node.motion_planner`).
- MpcSolver → ModelPredictiveControl (wired via ConfigWrapperMPC as `node.mpc`).
- TensorDeviceType → DeviceCfg. We read device/dtype from the wrapper.
- WorldConfig → Scene (obstacle_manager.get_scene()). The `shared_world_cfg`
  is now a Scene reference.
- No more `world_coll_checker` sharing: v2's Scene is the single source of
  truth, propagated via update_world(scene).
- ground plane lives on the Scene via ObstacleManager, not `world_cfg.add_obstacle`.
"""

import time
import traceback

import rclpy
import torch
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState as JointStateMsg
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner, GetPlanners
from curobo_msgs.action import SendTrajectory

from curobo.types import DeviceCfg, JointState, ToolPose, GoalToolPose
from curobo.scene import Cuboid

from curobo_ros.robot.robot_context import RobotContext
from curobo_ros.core.config_wrapper_motion import ConfigWrapperMotion, ConfigWrapperMPC
from curobo_ros.core.ik_services import IKServices
from curobo_ros.core.fk_services import FKServices
from curobo_ros.planners import (
    PlannerFactory,
    PlannerManager,
    ClassicPlanner,
    MPCPlanner,
    SinglePlanner,
)


class UnifiedPlannerNode(Node):
    """Unified trajectory planning node with dynamic strategy switching (v2)."""

    def __init__(self):
        super().__init__('unified_planner')

        # v2 device config — kept as `tensor_args` for backward-compat with
        # code paths that read `.device` / `.dtype` from it.
        self.tensor_args = DeviceCfg(device='cuda', dtype=torch.float32)

        self.robot_context = RobotContext(self, 0.03)

        self.declare_parameter('planner_type', 'classic')
        self.declare_parameter('max_attempts', 1)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('time_dilation_factor', 0.5)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('collision_activation_distance', 0.025)
        self.declare_parameter('convergence_threshold', 0.01)
        self.declare_parameter('max_mpc_iterations', 1000)

        # Lazy init: only motion wrapper up front; MPC wrapper is built on demand.
        self.config_wrapper_motion = ConfigWrapperMotion(self, self.robot_context)
        self.config_wrapper_mpc = None

        # Shared Scene for all planners — references ObstacleManager's Scene.
        # All planners see the same obstacles after update_world(scene).
        self.shared_scene = self.config_wrapper_motion.obstacle_manager.get_scene()
        # Legacy alias for any call sites still using the old name.
        self.shared_world_cfg = self.shared_scene

        # Solvers (created on demand).
        self.motion_planner = None  # v2 alias
        self.motion_gen = None      # legacy alias, kept for older code paths
        self.mpc = None

        # Shared IK — same Scene as MotionPlanner.
        self.ik_services = IKServices(self, self.config_wrapper_motion)

        # FK — depends only on robot YAML (no scene).
        self.fk_services = FKServices(
            self, self.config_wrapper_motion.robot_config_file
        )

        self.planner_manager = PlannerManager(self, self.config_wrapper_motion)

        initial_planner = self.get_parameter('planner_type').get_parameter_value().string_value
        self._warmup_initial_planner(initial_planner)
        self.planner_manager.set_current_planner(initial_planner)

        self.generate_trajectory_srv = self.create_service(
            TrajectoryGeneration,
            f'{self.get_name()}/generate_trajectory',
            self.generate_trajectory_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.set_planner_srv = self.create_service(
            SetPlanner,
            f'{self.get_name()}/set_planner',
            self.set_planner_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.get_planners_srv = self.create_service(
            GetPlanners,
            f'{self.get_name()}/get_planners',
            self.get_planners_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._action_server = ActionServer(
            self,
            SendTrajectory,
            f'{self.get_name()}/execute_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        from geometry_msgs.msg import Pose as PoseMsg
        self.mpc_goal_sub = self.create_subscription(
            PoseMsg,
            f'{self.get_name()}/mpc_goal',
            self.mpc_goal_callback,
            10,
        )

        self.get_logger().info(
            "Unified planner ready with initial planner: "
            f"{self.planner_manager.get_current_planner().get_planner_name()}"
        )

    # ------------------------------------------------------------------
    # Warmup
    # ------------------------------------------------------------------

    def _warmup_initial_planner(self, planner_type: str):
        self.get_logger().info(f"Warming up {planner_type} planner...")

        if planner_type == 'classic':
            self._warmup_classic()
        elif planner_type == 'mpc':
            self._warmup_mpc()
        else:
            self._warmup_classic()
            self.get_logger().warn(
                f"Planner '{planner_type}' not fully wired, falling back to classic warmup"
            )

        self.get_logger().info(f"{planner_type} planner ready")

    def _warmup_classic(self):
        """Warm up MotionPlanner for Classic/MultiPoint/JointSpace planners."""
        if self.motion_planner is None:
            self.get_logger().info("  -> Initializing MotionPlanner...")
            self.config_wrapper_motion.set_motion_gen_config(self, None, None)
            # The wrapper sets both self.motion_planner and self.motion_gen.
            SinglePlanner.set_motion_planner(self.motion_planner)
            self.get_logger().info("  -> MotionPlanner ready and shared with SinglePlanner")
        else:
            self.get_logger().info("  -> MotionPlanner already initialized (cache)")

    def _warmup_mpc(self):
        """Warm up the v2 ModelPredictiveControl solver on demand."""
        if self.mpc is None:
            self.get_logger().info("  -> Initializing MPC solver...")

            # Ensure a ground plane exists on the shared Scene.
            obs_mgr = self.config_wrapper_motion.obstacle_manager
            ground_exists = any(c.name == "ground" for c in self.shared_scene.cuboid)
            if not ground_exists:
                ground_plane = Cuboid(
                    name="ground",
                    pose=[0, 0, -0.1, 1, 0, 0, 0],
                    dims=[3.0, 3.0, 0.01],
                    color=[0.5, 0.5, 0.5, 1.0],
                )
                obs_mgr._append('cuboid', ground_plane)
                self.get_logger().info("  -> Added ground plane to shared Scene")

            # Build (or reuse) the MPC wrapper — this creates `self.mpc`.
            if self.config_wrapper_mpc is None:
                self.config_wrapper_mpc = ConfigWrapperMPC(self, self.robot_context)
            # After ConfigWrapperMPC.__init__, self.mpc is set.
            self.get_logger().info("  -> MPC solver ready")
        else:
            self.get_logger().info("  -> MPC solver already initialized (cache)")

    def update_all_solvers_world(self, scene=None):
        """Propagate scene updates to all initialized solvers."""
        scene = scene if scene is not None else self.shared_scene

        if self.motion_planner is not None:
            self.motion_planner.update_world(scene)

        if self.mpc is not None:
            self.mpc.update_world(scene)

        self.ik_services.update_world()

    # ------------------------------------------------------------------
    # Plan / execute callbacks
    # ------------------------------------------------------------------

    def generate_trajectory_callback(self, request: TrajectoryGeneration, response):
        try:
            planner = self.planner_manager.get_current_planner()
            if planner is None:
                response.success = False
                response.message = "No planner selected"
                return response

            if hasattr(request, 'start_pose') and request.start_pose.position:
                start_joint_pose = list(request.start_pose.position)
                self.get_logger().info(
                    f"Using start position from request: "
                    f"{[f'{x:.3f}' for x in start_joint_pose]}"
                )
            else:
                start_joint_pose = self.robot_context.get_joint_pose()
                self.get_logger().info(
                    f"Using robot current position: "
                    f"{[f'{x:.3f}' for x in start_joint_pose]}"
                )

            start_state = JointState.from_position(
                torch.tensor(
                    [start_joint_pose],
                    dtype=self.tensor_args.dtype,
                    device=self.tensor_args.device,
                )
            )

            config = self._get_planner_config(planner)
            self._setup_planner(planner)

            self.get_logger().info(f"Planning with {planner.get_planner_name()}")
            result = planner.plan(start_state, request, config, self.robot_context)

            response.success = result.success
            response.message = result.message

            if result.success and result.trajectory is not None:
                traj = result.trajectory

                if hasattr(planner, 'motion_planner') and planner.motion_planner is not None:
                    response.dt = float(planner.motion_planner.interpolation_dt)
                else:
                    response.dt = 0.03

                trajectory_msgs = []
                n_waypoints = len(traj.position)
                for i in range(n_waypoints):
                    waypoint = JointStateMsg()
                    if hasattr(traj, 'joint_names') and traj.joint_names is not None:
                        waypoint.name = list(traj.joint_names)
                    waypoint.position = traj.position[i].cpu().tolist()
                    waypoint.velocity = traj.velocity[i].cpu().tolist()
                    trajectory_msgs.append(waypoint)
                response.trajectory = trajectory_msgs

                self.get_logger().info(
                    f"Planning succeeded: {result.message} "
                    f"(trajectory: {n_waypoints} waypoints, dt: {response.dt}s)"
                )
            else:
                response.trajectory = []
                response.dt = 0.0
                if result.success:
                    self.get_logger().info(f"Planning succeeded: {result.message}")
                else:
                    self.get_logger().error(f"Planning failed: {result.message}")

            return response

        except Exception as e:
            self.get_logger().error(f"Trajectory generation error: {e}")
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Error: {e}"
            response.trajectory = []
            response.dt = 0.0
            return response

    def execute_callback(self, goal_handle):
        try:
            planner = self.planner_manager.get_current_planner()
            if planner is None:
                result_msg = SendTrajectory.Result()
                result_msg.success = False
                result_msg.message = "No planner selected"
                goal_handle.abort()
                return result_msg

            self.get_logger().info(f"Executing with {planner.get_planner_name()}")
            success = planner.execute(self.robot_context, goal_handle)

            result_msg = SendTrajectory.Result()
            result_msg.success = success
            result_msg.message = "Execution completed" if success else "Execution failed"
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result_msg

        except Exception as e:
            self.get_logger().error(f"Execution error: {e}")
            self.get_logger().error(traceback.format_exc())
            result_msg = SendTrajectory.Result()
            result_msg.success = False
            result_msg.message = f"Error: {e}"
            goal_handle.abort()
            return result_msg

    def set_planner_callback(self, request: SetPlanner.Request, response: SetPlanner.Response):
        try:
            previous = self.planner_manager.get_current_planner()
            previous_name = previous.get_planner_name() if previous else "None"

            key, error = PlannerFactory.switch_planner(request.planner_type, self.planner_manager)
            if error:
                response.success = False
                response.message = error
                response.previous_planner = previous_name
                response.current_planner = previous_name
                self.get_logger().error(error)
                return response

            planner = self.planner_manager.get_current_planner()
            self._setup_planner(planner)

            response.success = True
            response.message = f"Successfully switched to {planner.get_planner_name()}"
            response.previous_planner = previous_name
            response.current_planner = planner.get_planner_name()
            self.get_logger().info(
                f"Planner switch: {previous_name} -> {planner.get_planner_name()}"
            )

        except Exception as e:
            response.success = False
            response.message = f"Failed to switch planner: {e}"
            response.previous_planner = previous_name if 'previous_name' in locals() else "Unknown"
            response.current_planner = response.previous_planner
            self.get_logger().error(response.message)
            self.get_logger().error(traceback.format_exc())

        return response

    def mpc_goal_callback(self, msg):
        """Receive MPC goal updates from a topic; stored as raw data."""
        planner = self.planner_manager.get_current_planner()
        from curobo_ros.planners.mpc_planner import MPCPlanner
        if isinstance(planner, MPCPlanner):
            planner.latest_goal_from_topic = [
                msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z,
            ]
            self.get_logger().debug(
                f"MPC goal updated from topic: "
                f"[{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]"
            )
        else:
            self.get_logger().warn(
                f"Received MPC goal but current planner is {planner.get_planner_name()}"
            )

    def get_planners_callback(self, request: GetPlanners.Request, response: GetPlanners.Response):
        current_type = self.planner_manager.get_current_planner_type()
        catalog = PlannerFactory.get_catalog()

        response.planner_names = [name for _, _, name in catalog]
        response.planner_ids = [int(eid) for _, eid, _ in catalog]

        response.current_planner_name = 'Unknown'
        response.current_planner_id = 255
        for key, eid, name in catalog:
            if key == current_type:
                response.current_planner_name = name
                response.current_planner_id = int(eid)
                break

        response.success = True
        self.get_logger().info(
            f"GetPlanners: {len(catalog)} planners, current={response.current_planner_name}"
        )
        return response

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _setup_planner(self, planner):
        if isinstance(planner, ClassicPlanner):
            if self.motion_planner is None:
                self.get_logger().info("On-demand warmup: Classic planner")
                self._warmup_classic()
            planner.set_motion_gen(self.motion_planner)

        elif isinstance(planner, MPCPlanner):
            if self.mpc is None:
                self.get_logger().info("On-demand warmup: MPC planner")
                self._warmup_mpc()
            planner.set_mpc_solver(self.mpc)

    def _get_planner_config(self, planner) -> dict:
        if isinstance(planner, ClassicPlanner):
            return {
                'max_attempts': self.get_parameter('max_attempts').value,
                'timeout': self.get_parameter('timeout').value,
                'time_dilation_factor': self.get_parameter('time_dilation_factor').value,
            }
        if isinstance(planner, MPCPlanner):
            return {
                'convergence_threshold': self.get_parameter('convergence_threshold').value,
                'max_iterations': self.get_parameter('max_mpc_iterations').value,
            }
        return {}

    def goal_callback(self, goal):
        self.get_logger().info("Received execution goal")
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.robot_context.stop_robot()
        planner = self.planner_manager.get_current_planner()
        if hasattr(planner, 'cancel'):
            planner.cancel()
        self.get_logger().info("Goal cancelled")
        return rclpy.action.CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedPlannerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info('Unified planner running, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
