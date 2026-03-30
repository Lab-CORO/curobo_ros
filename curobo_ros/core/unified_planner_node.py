#!/usr/bin/env python3
"""
Unified trajectory planner node using Strategy Pattern.

This node supports multiple planning strategies (Classic, MPC, etc.)
and allows dynamic switching between them.
"""

import torch
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import JointState as JointStateMsg
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner, GetPlanners
from curobo_msgs.action import SendTrajectory, MpcMove

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.geom.types import Cuboid

from curobo_ros.robot.robot_context import RobotContext
from curobo_ros.core.config_wrapper_motion import ConfigWrapperMotion, ConfigWrapperMPC
from curobo_ros.core.ik_services import IKServices
from curobo_ros.core.fk_services import FKServices
from curobo_ros.planners import PlannerFactory, PlannerManager, ClassicPlanner, MPCPlanner, SinglePlanner


class UnifiedPlannerNode(Node):
    """
    Unified trajectory planning node with multiple strategies.

    Supports:
    - Classic motion generation (open-loop)
    - Model Predictive Control (closed-loop)
    - Dynamic planner switching via ROS service

    Services:
    - /generate_trajectory: Plan a trajectory
    - /set_planner: Switch active planner

    Actions:
    - /execute_trajectory: Execute planned trajectory
    """

    def __init__(self):
        super().__init__('unified_planner')

        # Initialize tensor arguments
        self.tensor_args = TensorDeviceType()

        # Robot context for command execution
        self.robot_context = RobotContext(self, 0.03)

        # Declare planner selection parameter
        self.declare_parameter('planner_type', 'classic')
        self.declare_parameter('max_attempts', 1)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('time_dilation_factor', 0.5)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('collision_activation_distance', 0.025)
        self.declare_parameter('convergence_threshold', 0.01)
        self.declare_parameter('max_mpc_iterations', 1000)

        # Initialize ONLY the base config wrapper
        # Other wrappers will be created on-demand (lazy loading)
        self.config_wrapper_motion = ConfigWrapperMotion(self, self.robot_context)
        self.config_wrapper_mpc = None  # Created on-demand when MPC is first used

        # Shared world_cfg for all planners - references ObstacleManager's world_cfg
        # This is a reference, not a copy, so all planners see the same obstacles
        self.shared_world_cfg = self.config_wrapper_motion.obstacle_manager.get_world_cfg()

        # IMPORTANT: shared_world_cfg is a reference to obstacle_manager.world_cfg.
        # When obstacles are added/removed via ROS services, all planners automatically
        # see the changes after update_world_config() is called.

        # Initialize solvers (created on-demand)
        self.motion_gen = None
        self.mpc = None

        # IK service — lazy warmup, shares obstacles with MotionGen
        self.ik_services = IKServices(self, self.config_wrapper_motion)

        # FK service — lazy warmup, depends only on robot_cfg (no world needed)
        self.fk_services = FKServices(self, self.config_wrapper_motion.robot_cfg)

        # Planner manager (handles caching)
        self.planner_manager = PlannerManager(self, self.config_wrapper_motion)

        # Get initial planner type
        initial_planner = self.get_parameter('planner_type').get_parameter_value().string_value

        # Warmup only the initial planner
        self._warmup_initial_planner(initial_planner)

        # Set initial planner (will be retrieved from cache)
        self.planner_manager.set_current_planner(initial_planner)

        # Create services
        self.generate_trajectory_srv = self.create_service(
            TrajectoryGeneration,
            f'{self.get_name()}/generate_trajectory',
            self.generate_trajectory_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.set_planner_srv = self.create_service(
            SetPlanner,
            f'{self.get_name()}/set_planner',
            self.set_planner_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Service to get available planners (structured, for RViz plugin)
        self.get_planners_srv = self.create_service(
            GetPlanners,
            f'{self.get_name()}/get_planners',
            self.get_planners_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Create action server (unified for all planner types)
        self._action_server = ActionServer(
            self,
            SendTrajectory,
            f'{self.get_name()}/execute_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Create subscription for MPC goal updates (for real-time tracking)
        from geometry_msgs.msg import Pose as PoseMsg
        self.mpc_goal_sub = self.create_subscription(
            PoseMsg,
            f'{self.get_name()}/mpc_goal',
            self.mpc_goal_callback,
            10
        )

        self.get_logger().info(
            f"Unified planner ready with initial planner: "
            f"{self.planner_manager.get_current_planner().get_planner_name()}"
        )

    def _warmup_initial_planner(self, planner_type: str):
        """
        Warmup only the initial planner (lazy loading).

        Args:
            planner_type: Type of planner to warmup ('classic', 'mpc', etc.)
        """
        self.get_logger().info(f"Warming up {planner_type} planner...")

        if planner_type == 'classic':
            self._warmup_classic()
        elif planner_type == 'mpc':
            self._warmup_mpc()
        else:
            # For future planners (batch, constrained), default to classic config
            self._warmup_classic()
            self.get_logger().warn(
                f"Planner '{planner_type}' not fully implemented yet, "
                "using classic warmup"
            )

        self.get_logger().info(f"✅ {planner_type} planner ready")

    def _warmup_classic(self):
        """Warmup MotionGen for Classic/Batch/Constrained planners."""
        if self.motion_gen is None:
            self.get_logger().info("  → Initializing MotionGen...")
            self.config_wrapper_motion.set_motion_gen_config(self, None, None)
            self.motion_gen = self.motion_gen  # Set by config wrapper

            # Share MotionGen instance with all SinglePlanner children
            # This allows switching between SinglePlanner-based planners without re-warmup
            SinglePlanner.set_motion_gen(self.motion_gen)
            self.get_logger().info("  → MotionGen ready and shared with SinglePlanner hierarchy")
        else:
            self.get_logger().info("  → MotionGen already initialized (using cache)")

    def _warmup_mpc(self):
        """Warmup MPC solver on-demand."""
        if self.mpc is None:
            self.get_logger().info("  → Initializing MPC solver...")

            # Get robot config from existing wrapper
            robot_cfg = self.config_wrapper_motion.robot_cfg

            # Add ground plane to shared world_cfg if not already there
            ground_exists = any(obj.name == "ground" for obj in self.shared_world_cfg.objects)
            if not ground_exists:
                ground_plane = Cuboid(
                    name="ground",
                    pose=[0, 0, -0.1, 1, 0, 0, 0],
                    dims=[3.0, 3.0, 0.01],
                    color=[0.5, 0.5, 0.5, 1.0]
                )
                self.shared_world_cfg.add_obstacle(ground_plane)
                self.get_logger().info("  → Added ground plane to shared world_cfg")

            # Create MPC config, sharing world_coll_checker with MotionGen if available.
            # This avoids duplicating obstacle tensors in VRAM — both solvers use the
            # same CUDA tensors. update_world() only needs to be called once.
            shared_checker = self.motion_gen.world_coll_checker if self.motion_gen is not None else None
            if shared_checker is not None:
                self.get_logger().info("  → Sharing world_coll_checker with MotionGen (no extra VRAM)")

            mpc_config = MpcSolverConfig.load_from_robot_config(
                robot_cfg,
                self.shared_world_cfg,
                store_rollouts=True,
                step_dt=0.03,
                world_coll_checker=shared_checker,
            )

            self.mpc = MpcSolver(mpc_config)
            self.get_logger().info("  → MPC solver ready")
        else:
            self.get_logger().info("  → MPC solver already initialized (using cache)")

    def update_all_solvers_world(self, world_cfg):
        """
        Propagate a world configuration update to all initialized solvers.

        Solvers that share the same world_coll_checker instance (VRAM sharing) only
        need one update — the identity check avoids the redundant call.
        """
        if self.motion_gen is not None:
            self.motion_gen.world_coll_checker.clear_cache()
            self.motion_gen.update_world(world_cfg)

        if (self.mpc is not None and
                (self.motion_gen is None or
                 self.mpc.world_coll_checker is not self.motion_gen.world_coll_checker)):
            self.mpc.world_coll_checker.clear_cache()
            self.mpc.update_world(world_cfg)

        self.ik_services.update_world()

    def generate_trajectory_callback(self, request: TrajectoryGeneration, response):
        """
        Generate a trajectory using the current planner.

        This service plans but doesn't execute. Use the action to execute.
        """
        try:
            # Get current planner
            planner = self.planner_manager.get_current_planner()

            if planner is None:
                response.success = False
                response.message = "No planner selected"
                return response

            # Get robot state - check if start pose is provided in request
            if hasattr(request, 'start_pose') and request.start_pose.position:
                # Use start_pose from request (works for both classic and multipoint)
                start_joint_pose = list(request.start_pose.position)
                self.get_logger().info(
                    f"📍 Using start position from request: {[f'{x:.3f}' for x in start_joint_pose]}"
                )
            else:
                # Fall back to current robot position
                start_joint_pose = self.robot_context.get_joint_pose()
                self.get_logger().info(
                    f"📍 Using robot current position for planning: {[f'{x:.3f}' for x in start_joint_pose]}"
                )

            start_state = JointState.from_position(
                torch.Tensor([start_joint_pose])
                .to(device=self.tensor_args.device)
            )

            # Build config from parameters
            config = self._get_planner_config(planner)

            # Initialize planner if needed
            self._setup_planner(planner)

            # Plan
            # Each planner extracts its goal from the request (target_pose or target_poses)
            self.get_logger().info(
                f"Planning with {planner.get_planner_name()}"
            )

            result = planner.plan(start_state, request, config, self.robot_context)

            # Build response
            response.success = result.success
            response.message = result.message

            # Populate trajectory and dt fields
            if result.success and result.trajectory is not None:
                # Get trajectory from result
                traj = result.trajectory

                # Get interpolation dt
                if hasattr(planner, 'motion_gen') and planner.motion_gen is not None:
                    response.dt = float(planner.motion_gen.interpolation_dt)
                else:
                    response.dt = 0.03  # Default fallback

                # Convert cuRobo JointState to ROS2 sensor_msgs/JointState[]
                trajectory_msgs = []
                n_waypoints = len(traj.position)

                for i in range(n_waypoints):
                    waypoint = JointStateMsg()

                    # Set joint names if available
                    if hasattr(traj, 'joint_names') and traj.joint_names is not None:
                        waypoint.name = list(traj.joint_names)

                    # Convert tensors to lists
                    waypoint.position = traj.position[i].cpu().tolist()
                    waypoint.velocity = traj.velocity[i].cpu().tolist()

                    trajectory_msgs.append(waypoint)

                response.trajectory = trajectory_msgs

                self.get_logger().info(
                    f"Planning succeeded: {result.message} "
                    f"(trajectory: {n_waypoints} waypoints, dt: {response.dt}s)"
                )
            else:
                # Empty trajectory for failed planning
                response.trajectory = []
                response.dt = 0.0

                if result.success:
                    self.get_logger().info(
                        f"Planning succeeded: {result.message}"
                    )
                else:
                    self.get_logger().error(
                        f"Planning failed: {result.message}"
                    )

            return response

        except Exception as e:
            self.get_logger().error(f"Trajectory generation error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

            response.success = False
            response.message = f"Error: {str(e)}"
            response.trajectory = []
            response.dt = 0.0
            return response

    def execute_callback(self, goal_handle):
        """
        Execute the planned trajectory using the current planner.
        """
        try:
            planner = self.planner_manager.get_current_planner()

            if planner is None:
                result_msg = SendTrajectory.Result()
                result_msg.success = False
                result_msg.message = "No planner selected"
                goal_handle.abort()
                return result_msg

            self.get_logger().info(
                f"Executing with {planner.get_planner_name()}"
            )

            # Execute using the planner's strategy
            success = planner.execute(self.robot_context, goal_handle)

            # Build result
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
            import traceback
            self.get_logger().error(traceback.format_exc())

            result_msg = SendTrajectory.Result()
            result_msg.success = False
            result_msg.message = f"Error: {str(e)}"
            goal_handle.abort()
            return result_msg

    def set_planner_callback(self, request: SetPlanner.Request, response: SetPlanner.Response):
        """
        Switch the active planner using enum-based service.

        Usage:
            ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"
        """
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
            self.get_logger().info(f"✅ Planner switch: {previous_name} → {planner.get_planner_name()}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to switch planner: {str(e)}"
            response.previous_planner = previous_name if 'previous_name' in locals() else "Unknown"
            response.current_planner = response.previous_planner
            self.get_logger().error(response.message)
            import traceback
            self.get_logger().error(traceback.format_exc())

        return response


    def mpc_goal_callback(self, msg):
        """
        Callback for MPC goal updates (real-time tracking).

        Receives goal pose from RViz plugin and updates MPC planner during execution.
        NOTE: do NOT create CUDA tensors here — this runs on the ROS2 executor thread
        and may race with CUDA graph capture in the MPC execution thread.
        Store raw Python data; the MPC thread will convert it to a Pose safely.
        """
        # Get current planner
        planner = self.planner_manager.get_current_planner()

        # Only update if MPC planner is active
        from curobo_ros.planners.mpc_planner import MPCPlanner
        if isinstance(planner, MPCPlanner):
            # Store raw list — converted to cuRobo Pose inside the MPC execution thread
            planner.latest_goal_from_topic = [
                msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
            ]
            self.get_logger().debug(
                f"MPC goal updated from topic: [{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]"
            )
        else:
            self.get_logger().warn(
                f"Received MPC goal but current planner is {planner.get_planner_name()}"
            )

    def get_planners_callback(self, request: GetPlanners.Request, response: GetPlanners.Response):
        """
        Return the list of available planners with their enum IDs.

        Usage:
            ros2 service call /unified_planner/get_planners curobo_msgs/srv/GetPlanners
        """
        current_type = self.planner_manager.get_current_planner_type()
        catalog = PlannerFactory.get_catalog()

        response.planner_names = [name for _, _, name in catalog]
        response.planner_ids   = [int(eid) for _, eid, _ in catalog]

        response.current_planner_name = 'Unknown'
        response.current_planner_id   = 255
        for key, eid, name in catalog:
            if key == current_type:
                response.current_planner_name = name
                response.current_planner_id   = int(eid)
                break

        response.success = True
        self.get_logger().info(
            f"GetPlanners: {len(catalog)} planners, current={response.current_planner_name}"
        )
        return response

    def _setup_planner(self, planner):
        """
        Initialize planner-specific components on-demand.

        This performs lazy warmup: if the planner's solver isn't initialized yet,
        it will be warmed up now. Subsequent switches to the same planner will
        be instant (retrieved from cache).
        """
        if isinstance(planner, ClassicPlanner):
            # Warmup MotionGen if not already done
            if self.motion_gen is None:
                self.get_logger().info("On-demand warmup: Classic planner")
                self._warmup_classic()

            planner.set_motion_gen(self.motion_gen)

        elif isinstance(planner, MPCPlanner):
            # Warmup MPC if not already done
            if self.mpc is None:
                self.get_logger().info("On-demand warmup: MPC planner")
                self._warmup_mpc()

            planner.set_mpc_solver(self.mpc)

            # MPC now uses shared world_cfg, no need to update config wrapper

    def _get_planner_config(self, planner) -> dict:
        """Build configuration dictionary for planner."""
        if isinstance(planner, ClassicPlanner):
            return {
                'max_attempts': self.get_parameter('max_attempts').value,
                'timeout': self.get_parameter('timeout').value,
                'time_dilation_factor': self.get_parameter('time_dilation_factor').value,
            }

        elif isinstance(planner, MPCPlanner):
            return {
                'convergence_threshold': self.get_parameter('convergence_threshold').value,
                'max_iterations': self.get_parameter('max_mpc_iterations').value,
            }

        return {}

    def goal_callback(self, goal):
        """Accept all goals."""
        self.get_logger().info("Received execution goal")
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation."""
        self.robot_context.stop_robot()

        # Cancel MPC if active
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
