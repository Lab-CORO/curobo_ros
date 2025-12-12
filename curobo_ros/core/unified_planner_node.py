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

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState as JointStateMsg
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner, SetCollisionChecker
from curobo_msgs.action import SendTrajectory, MpcMove

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig
from curobo.geom.types import Cuboid

from curobo_ros.robot.robot_context import RobotContext
from curobo_ros.core.config_wrapper_motion import ConfigWrapperMotion, ConfigWrapperMPC
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

        # Also add a service to list available planners
        self.list_planners_srv = self.create_service(
            Trigger,
            f'{self.get_name()}/list_planners',
            self.list_planners_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Service to switch collision checker (MESH/BLOX)
        self.set_collision_checker_srv = self.create_service(
            SetCollisionChecker,
            f'{self.get_name()}/set_collision_checker',
            self.set_collision_checker_callback,
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

            # Create MPC config using shared world_cfg
            mpc_config = MpcSolverConfig.load_from_robot_config(
                robot_cfg,
                self.shared_world_cfg,  # Use shared world_cfg!
                store_rollouts=True,
                step_dt=0.03,
            )

            self.mpc = MpcSolver(mpc_config)
            self.get_logger().info("  → MPC solver ready (sharing world_cfg with other planners)")
        else:
            self.get_logger().info("  → MPC solver already initialized (using cache)")

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

        Where:
            0 = CLASSIC
            1 = MPC
            2 = BATCH
            3 = CONSTRAINED
            4 = MULTIPOINT
            5 = JOINT_SPACE
        """
        try:
            # Get current planner before switching
            previous = self.planner_manager.get_current_planner()
            previous_name = previous.get_planner_name() if previous else "None"

            # Map enum to planner type string
            match request.planner_type:
                case SetPlanner.Request.CLASSIC:
                    planner_type = 'classic'
                case SetPlanner.Request.MPC:
                    planner_type = 'mpc'
                case SetPlanner.Request.BATCH:
                    planner_type = 'batch'
                case SetPlanner.Request.CONSTRAINED:
                    planner_type = 'constrained'
                case SetPlanner.Request.MULTIPOINT:
                    planner_type = 'multi_point'
                case SetPlanner.Request.JOINT_SPACE:
                    planner_type = 'joint_space'
                case _:
                    response.success = False
                    response.message = f"Unknown planner type: {request.planner_type}"
                    response.previous_planner = previous_name
                    response.current_planner = previous_name
                    self.get_logger().error(response.message)
                    return response

            # Check if planner is available
            available = PlannerFactory.get_available_planners()
            if planner_type not in available:
                response.success = False
                response.message = (
                    f"Planner '{planner_type}' not yet implemented. "
                    f"Available: {available}"
                )
                response.previous_planner = previous_name
                response.current_planner = previous_name
                self.get_logger().warn(response.message)
                return response

            # Switch planner
            self.planner_manager.set_current_planner(planner_type)

            # Setup the new planner
            planner = self.planner_manager.get_current_planner()
            self._setup_planner(planner)

            # Update response
            response.success = True
            response.message = f"Successfully switched to {planner.get_planner_name()}"
            response.previous_planner = previous_name
            response.current_planner = planner.get_planner_name()

            self.get_logger().info(
                f"✅ Planner switch: {previous_name} → {planner.get_planner_name()}"
            )
            self.get_logger().info(
                f"Updated world: {self.config_wrapper_motion.obstacle_manager.get_world_cfg()} cuboids, meshes"
            )

        except Exception as e:
            response.success = False
            response.message = f"Failed to switch planner: {str(e)}"
            response.previous_planner = previous_name if 'previous_name' in locals() else "Unknown"
            response.current_planner = previous_name if 'previous_name' in locals() else "Unknown"
            self.get_logger().error(response.message)
            import traceback
            self.get_logger().error(traceback.format_exc())

        return response

    def set_collision_checker_callback(
        self,
        request: SetCollisionChecker.Request,
        response: SetCollisionChecker.Response
    ):
        """
        Switch collision checker type (MESH/BLOX/PRIMITIVE).

        This recreates the entire MotionGen instance but preserves obstacles.

        Usage:
            ros2 service call /unified_planner/set_collision_checker curobo_msgs/srv/SetCollisionChecker "{checker_type: 1}"

        Where:
            0 = BLOX (voxel-based, good for point clouds)
            1 = MESH (mesh-based, precise for mesh obstacles)
            2 = PRIMITIVE (cuboids only, fastest)
        """
        try:
            # Map request to CollisionCheckerType
            from curobo.geom.sdf.world import CollisionCheckerType

            match request.checker_type:
                case SetCollisionChecker.Request.BLOX:
                    new_checker = CollisionCheckerType.BLOX
                    checker_name = "BLOX"
                case SetCollisionChecker.Request.MESH:
                    new_checker = CollisionCheckerType.MESH
                    checker_name = "MESH"
                case SetCollisionChecker.Request.PRIMITIVE:
                    new_checker = CollisionCheckerType.PRIMITIVE
                    checker_name = "PRIMITIVE"
                case _:
                    response.success = False
                    response.message = f"Unknown checker type: {request.checker_type}"
                    return response

            # Get current checker
            current_checker = self.config_wrapper_motion.current_collision_checker
            current_name = str(current_checker).split('.')[-1] if current_checker else "None"

            response.previous_checker = current_name

            # Check if already using this checker
            if current_checker == new_checker:
                response.success = True
                response.message = f"Already using {checker_name} checker"
                response.current_checker = checker_name
                response.num_obstacles_restored = (
                    len(self.config_wrapper_motion.cuboid_list) +
                    len(self.config_wrapper_motion.mesh_list)
                )
                return response

            self.get_logger().info(
                f"Switching collision checker: {current_name} → {checker_name}"
            )

            # Save obstacle counts before recreation
            num_cuboids = len(self.config_wrapper_motion.cuboid_list)
            num_meshes = len(self.config_wrapper_motion.mesh_list)

            self.get_logger().info(
                f"Saving {num_cuboids} cuboids and {num_meshes} meshes"
            )

            # Change checker type
            self.config_wrapper_motion.collision_checker_type = new_checker

            # Recreate MotionGen (obstacles are automatically restored inside)
            success = self.config_wrapper_motion.recreate_motion_gen(self)

            if not success:
                response.success = False
                response.message = "Failed to recreate MotionGen"
                response.current_checker = current_name
                response.num_obstacles_restored = 0
                return response

            # Update response
            response.success = True
            response.message = (
                f"Successfully switched to {checker_name} checker. "
                f"Restored {num_cuboids} cuboids and {num_meshes} meshes."
            )
            response.current_checker = checker_name
            response.num_obstacles_restored = num_cuboids + num_meshes

            self.get_logger().info(f"✅ {response.message}")

            return response

        except Exception as e:
            self.get_logger().error(f"Error switching collision checker: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

            response.success = False
            response.message = f"Error: {str(e)}"
            response.current_checker = current_name if 'current_name' in locals() else "Unknown"
            response.num_obstacles_restored = 0
            return response

    def mpc_goal_callback(self, msg):
        """
        Callback for MPC goal updates (real-time tracking).

        Receives goal pose from RViz plugin and updates MPC planner during execution.
        """
        from curobo.types.math import Pose

        # Convert ROS Pose message to cuRobo Pose
        new_goal_pose = Pose.from_list([
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        ])

        # Get current planner
        planner = self.planner_manager.get_current_planner()

        # Only update if MPC planner is active
        from curobo_ros.planners.mpc_planner import MPCPlanner
        if isinstance(planner, MPCPlanner):
            # Store goal for MPC execute() loop to pick up
            planner.latest_goal_from_topic = new_goal_pose
            self.get_logger().debug(
                f"MPC goal updated from topic: [{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]"
            )
        else:
            self.get_logger().warn(
                f"Received MPC goal but current planner is {planner.get_planner_name()}"
            )

    def list_planners_callback(self, request: Trigger, response: Trigger.Response):
        """
        List all available planners with their enum values.

        Usage:
            ros2 service call /unified_planner/list_planners std_srvs/srv/Trigger
        """
        available = PlannerFactory.get_available_planners()
        current_type = self.planner_manager.get_current_planner_type()

        # Build message with enum values
        planner_info = []

        # Helper to get enum value
        enum_map = {
            'classic': (SetPlanner.Request.CLASSIC, 'CLASSIC'),
            'mpc': (SetPlanner.Request.MPC, 'MPC'),
            'batch': (SetPlanner.Request.BATCH, 'BATCH'),
            'constrained': (SetPlanner.Request.CONSTRAINED, 'CONSTRAINED'),
        }

        for planner_type in ['classic', 'mpc', 'batch', 'constrained']:
            enum_val, enum_name = enum_map.get(planner_type, (None, 'UNKNOWN'))
            marker = "→" if planner_type == current_type else " "
            status = "✓" if planner_type in available else "✗"

            planner_info.append(
                f"{marker} {status} {enum_name} ({enum_val}): {planner_type}"
            )

        response.success = True
        response.message = (
            f"Current: {current_type}\n\n"
            "Available planners:\n" + "\n".join(planner_info) + "\n\n"
            "Usage: ros2 service call /unified_planner/set_planner "
            'curobo_msgs/srv/SetPlanner "{planner_type: N}"'
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
