#!/usr/bin/env python3
"""
Unified trajectory planner node (v2) using Strategy Pattern.

Supports multiple planning strategies (Classic, MPC, Multi-Point, Joint-Space)
and allows dynamic switching between them.

v2 notes:
- MotionGen → MotionPlanner (wired via ConfigWrapperMotion as `node.motion_planner`).
- MpcSolver → ModelPredictiveControl (built by MPCController from the shared
  context, published as `node.mpc`).
- TensorDeviceType → DeviceCfg. We read device/dtype from the wrapper.
- WorldConfig → Scene (obstacle_manager.get_scene()), the single source of
  truth, propagated to solvers via update_world(scene).
- Perception: camera data → Mapper (ObstacleManager) → ESDF VoxelGrid in the
  Scene, refreshed on-demand before each plan via refresh_perception_world().
- ground plane lives on the Scene via ObstacleManager, not `world_cfg.add_obstacle`.
"""

import time
import traceback

import rclpy
import torch
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState as JointStateMsg
from std_srvs.srv import Trigger
from curobo_msgs.srv import AttachObject, TrajectoryGeneration, SetPlanner, GetPlanners
from curobo_msgs.action import GraspPlan, SendTrajectory

from curobo.types import DeviceCfg, JointState
from curobo.scene import Cuboid

from curobo_ros.robot.robot_context import RobotContext
from curobo_ros.core.config_wrapper_motion import ConfigWrapperMotion
from curobo_ros.core.ik_services import IKServices
from curobo_ros.core.fk_services import FKServices
from curobo_ros.planners import (
    GraspPlanner,
    PlannerFactory,
    PlannerManager,
    ReactiveController,
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
        # Feedback publish cadence (s) during open-loop execution. NOTE: this is
        # NOT a speed control in v2 — set robot speed natively in the robot YAML
        # cspace (velocity_scale / max_acceleration / max_jerk), then call
        # update_motion_gen_config to rebuild.
        self.declare_parameter('time_dilation_factor', 0.5)
        self.declare_parameter('voxel_size', 0.05)
        # Publish rate (Hz) of the sparse voxel grid topic. <= 0 disables it.
        self.declare_parameter('sparse_voxel_publish_rate', 7.0)
        self.declare_parameter('collision_activation_distance', 0.025)
        # Output sampling step (s) of the interpolated trajectory (trajopt).
        self.declare_parameter('interpolation_dt', 0.025)
        self.declare_parameter('convergence_threshold', 0.01)
        self.declare_parameter('max_mpc_iterations', 1000)
        # Reactive (MPC) solver build params — read by MPCController.build_solver().
        self.declare_parameter('mpc_step_dt', 0.03)
        self.declare_parameter('mpc_horizon_steps', 30)
        # Reactive (Retarget/teleop) build params — read by RetargetController.
        self.declare_parameter('retarget_position_weight', 1.0)
        self.declare_parameter('retarget_orientation_weight', 1.0)
        self.declare_parameter('retarget_use_mpc', False)
        # Lifetime (s) of a pre-planned (preview) trajectory cached by
        # generate_trajectory and reused by the execute action.
        self.declare_parameter('trajectory_cache_ttl', 30.0)

        # Single shared context for every solver (robot + obstacles + scene +
        # collision cache). The MPC solver is built lazily from this same context.
        self.config_wrapper_motion = ConfigWrapperMotion(self, self.robot_context)

        # Shared Scene for all planners — references ObstacleManager's Scene.
        # All planners see the same obstacles after update_world(scene).
        self.shared_scene = self.config_wrapper_motion.obstacle_manager.get_scene()

        # Solvers (created on demand).
        self.motion_planner = None  # v2 alias
        self.motion_gen = None      # legacy alias, kept for older code paths
        self.mpc = None             # reactive: ModelPredictiveControl
        self.retargeter = None      # reactive: MotionRetargeter (teleop)

        # Native grasp pipeline (its own plan_grasp action; reuses motion_planner).
        self.grasp_planner = GraspPlanner(self, self.config_wrapper_motion)

        # Cached open-loop plan from a generate_trajectory (preview) call, reused
        # by the execute action when its target matches. See _pending_plan_*.
        self._pending_plan = None  # {'planner': key, 'signature': sig, 'stamp': monotonic}

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
        self.clear_trajectory_srv = self.create_service(
            Trigger,
            f'{self.get_name()}/clear_trajectory',
            self.clear_trajectory_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Reentrant group so cancel_callback can run while a long-running
        # reactive execute_callback is still servoing (MultiThreadedExecutor).
        self._action_server = ActionServer(
            self,
            SendTrajectory,
            f'{self.get_name()}/execute_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Native grasp pipeline action (plan_grasp): goalset -> approach -> grasp
        # -> [attach] -> lift. Separate from execute_trajectory (grasp-specific
        # inputs). Reentrant so cancel can run during execution.
        self._grasp_action_server = ActionServer(
            self,
            GraspPlan,
            f'{self.get_name()}/plan_grasp',
            execute_callback=self.grasp_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        # Attach a scene obstacle to the arm's attached_object link independently
        # of a grasp action (useful for pre-positioned objects, simulation, tests).
        self.attach_object_srv = self.create_service(
            AttachObject,
            f'{self.get_name()}/attach_object',
            self.attach_object_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        # Release a previously attached (grasped) object.
        self.detach_object_srv = self.create_service(
            Trigger,
            f'{self.get_name()}/detach_object',
            self.detach_object_callback,
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

        if planner_type in ('mpc', 'model_predictive_control'):
            self._warmup_mpc()
        elif planner_type in ('retarget', 'motion_retargeting', 'teleop'):
            self._warmup_reactive('retarget')
        elif planner_type in ('classic', 'multi_point', 'joint_space',
                              'motion_gen', 'multipoint'):
            self._warmup_classic()
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

    def _ensure_ground_plane(self):
        """Ensure a ground-plane cuboid exists on the shared Scene (once)."""
        obs_mgr = self.config_wrapper_motion.obstacle_manager
        if not any(c.name == "ground" for c in obs_mgr.get_scene().cuboid):
            obs_mgr._append('cuboid', Cuboid(
                name="ground",
                pose=[0, 0, -0.1, 1, 0, 0, 0],
                dims=[3.0, 3.0, 0.01],
                color=[0.5, 0.5, 0.5, 1.0],
            ))
            self.get_logger().info("  -> Added ground plane to shared Scene")

    def _warmup_reactive(self, key: str):
        """Build a reactive controller's solver on demand from the shared context.

        Each reactive controller's build_solver() publishes its solver where the
        node expects it (self.mpc / self.retargeter).
        """
        self._ensure_ground_plane()
        self.planner_manager.get_planner(key).ensure_solver()
        self.get_logger().info(f"  -> Reactive '{key}' solver ready")

    def _warmup_mpc(self):
        """Warm up the v2 ModelPredictiveControl solver on demand."""
        if self.mpc is not None:
            self.get_logger().info("  -> MPC solver already initialized (cache)")
            return
        self.get_logger().info("  -> Initializing MPC solver...")
        self._warmup_reactive('mpc')

    def update_all_solvers_world(self, scene=None):
        """Propagate scene updates to all initialized solvers."""
        scene = scene if scene is not None else self.shared_scene

        if self.motion_planner is not None:
            self.motion_planner.update_world(scene)

        # Reactive controllers each own their collision model; delegate to the
        # controller's update_world() override (no node dependency on internals).
        if self.mpc is not None:
            self.planner_manager.get_planner('mpc').update_world(scene)
        if self.retargeter is not None:
            self.planner_manager.get_planner('retarget').update_world(scene)

        self.ik_services.update_world()

    def refresh_perception_world(self):
        """Recompute the perception ESDF and push it to all solvers.

        Called on-demand before each plan (and per step during MPC) so the
        collision world reflects the latest camera data — no background timer,
        so no race with CUDA graph capture.
        """
        obs = self.config_wrapper_motion.obstacle_manager
        if obs.refresh_esdf():
            self.update_all_solvers_world(obs.get_scene())

    def rebuild_solvers_for_cache_change(self):
        """Rebuild all active solvers after a collision-cache change.

        The collision cache is allocated at solver creation, so a change
        requires recreating the solvers (a world update is not sufficient).
        Registered as ObstacleManager's cache-change observer.
        """
        self.get_logger().info("Collision cache changed — rebuilding solvers...")

        # Motion planner (present after the initial warmup).
        if self.motion_planner is not None:
            self.config_wrapper_motion.set_motion_gen_config(self, None, None)
            SinglePlanner.set_motion_planner(self.motion_planner)

        # IK (only if it was initialized). IKServices reads the canonical
        # (motion) cache directly, so no sync is needed.
        self.ik_services.rebuild()

        # Reactive controllers (only if initialized). Built from the SAME shared
        # cache, so just rebuild their solvers — no manual cache copy needed.
        if self.mpc is not None:
            self.planner_manager.get_planner('mpc').rebuild_solver()
        if self.retargeter is not None:
            self.planner_manager.get_planner('retarget').rebuild_solver()

        self.get_logger().info("Solvers rebuilt after cache change")

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

            _, start_state = self._resolve_start_state(request)

            config = self._get_planner_config(planner)
            self._setup_planner(planner)

            # Refresh the perception-based collision world before planning so
            # the plan accounts for the latest camera data.
            self.refresh_perception_world()

            self.get_logger().info(f"Planning with {planner.get_planner_name()}")
            result = planner.plan(start_state, request, config, self.robot_context)

            response.success = result.success
            response.message = result.message

            # Preview workflow: cache a successful open-loop trajectory so the
            # execute action can reuse it (matching target) without recomputing.
            # Reactive controllers have no trajectory to cache.
            if result.success and planner.is_open_loop():
                self._store_pending_plan(start_state, request)
            elif not planner.is_open_loop():
                self._pending_plan = None

            if result.success and result.trajectory is not None:
                traj = result.trajectory

                # v2: MotionPlanner no longer exposes `interpolation_dt` directly —
                # it lives on the trajopt solver config. Fall back to the node's
                # configured default if we can't reach it.
                response.dt = 0.03
                mp = getattr(planner, 'motion_planner', None)
                trajopt = getattr(mp, 'trajopt_solver', None) if mp is not None else None
                trajopt_cfg = getattr(trajopt, 'config', None)
                dt_val = getattr(trajopt_cfg, 'interpolation_dt', None)
                if dt_val is not None:
                    response.dt = float(dt_val)

                # v2: traj.position may be [B, T, D]; the trajectory message
                # wants one JointState per waypoint (T messages, D floats each).
                pos_tensor = traj.position
                vel_tensor = traj.velocity
                while pos_tensor.ndim > 2:
                    pos_tensor = pos_tensor[0]
                    if vel_tensor is not None:
                        vel_tensor = vel_tensor[0]

                trajectory_msgs = []
                n_waypoints = pos_tensor.shape[0]
                pos_list = pos_tensor.detach().cpu().tolist()
                vel_list = (
                    vel_tensor.detach().cpu().tolist()
                    if vel_tensor is not None else None
                )
                for i in range(n_waypoints):
                    waypoint = JointStateMsg()
                    if hasattr(traj, 'joint_names') and traj.joint_names is not None:
                        waypoint.name = list(traj.joint_names)
                    waypoint.position = pos_list[i]
                    if vel_list is not None:
                        waypoint.velocity = vel_list[i]
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
        """Unified 'generate + execute' action for every controller.

        Open-loop: reuse a matching cached (preview) trajectory or plan one, then
        execute it to completion (terminates on its own).
        Reactive: set the goal then servo continuously; terminates only on cancel
        or error, signalling `on_target` through the action feedback.
        """
        result_msg = SendTrajectory.Result()
        try:
            planner = self.planner_manager.get_current_planner()
            if planner is None:
                result_msg.success = False
                result_msg.message = "No planner selected"
                goal_handle.abort()
                return result_msg

            goal = goal_handle.request
            _, start_state = self._resolve_start_state(goal)
            config = self._get_planner_config(planner)
            self._setup_planner(planner)

            if planner.is_open_loop():
                reuse = (bool(getattr(goal, 'allow_cached', True))
                         and self._pending_plan_matches(start_state, goal))
                if reuse:
                    self.get_logger().info("Reusing cached (pre-planned) trajectory")
                else:
                    self.refresh_perception_world()
                    self.get_logger().info(f"Planning with {planner.get_planner_name()}")
                    result = planner.plan(start_state, goal, config, self.robot_context)
                    if not result.success:
                        result_msg.success = False
                        result_msg.message = f"Planning failed: {result.message}"
                        goal_handle.abort()
                        return result_msg
                self._pending_plan = None  # consumed
            else:
                # Reactive: (re)set the goal on the solver before servoing.
                self.refresh_perception_world()
                self.get_logger().info(f"Planning with {planner.get_planner_name()}")
                result = planner.plan(start_state, goal, config, self.robot_context)
                if not result.success:
                    result_msg.success = False
                    result_msg.message = f"Planning failed: {result.message}"
                    goal_handle.abort()
                    return result_msg

            self.get_logger().info(f"Executing with {planner.get_planner_name()}")
            success = planner.execute(self.robot_context, goal_handle)

            # Cancel takes precedence over the planner's return value.
            if goal_handle.is_cancel_requested:
                result_msg.success = False
                result_msg.message = "Execution canceled"
                goal_handle.canceled()
                return result_msg

            result_msg.success = bool(success)
            result_msg.message = "Execution completed" if success else "Execution failed"
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result_msg

        except Exception as e:
            self.get_logger().error(f"Execution error: {e}")
            self.get_logger().error(traceback.format_exc())
            result_msg.success = False
            result_msg.message = f"Error: {e}"
            if goal_handle.is_active:
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
        """Receive live reactive-goal updates from a topic; stored as raw data.

        This is the ROS mapping of cuRobo's continuous `update_goal_tool_poses`:
        the active reactive controller retargets on its next loop iteration.
        """
        planner = self.planner_manager.get_current_planner()
        if isinstance(planner, ReactiveController):
            planner.latest_goal = [
                msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z,
            ]
            self.get_logger().debug(
                f"Reactive goal updated from topic: "
                f"[{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]"
            )
        else:
            self.get_logger().warn(
                f"Received reactive goal but current planner is {planner.get_planner_name()}"
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
        # Open-loop planners share one MotionPlanner instance (class-level).
        if isinstance(planner, SinglePlanner):
            if self.motion_planner is None:
                self.get_logger().info("On-demand warmup: open-loop planner")
                self._warmup_classic()
            planner.set_motion_gen(self.motion_planner)

        # Reactive controllers build their own solver from the shared context.
        # ensure_solver() is idempotent (no-op once built), so just make sure a
        # ground plane exists and let the active controller build/reuse its solver.
        elif isinstance(planner, ReactiveController):
            if planner.solver is None:
                self.get_logger().info("On-demand warmup: reactive controller")
                self._ensure_ground_plane()
            planner.ensure_solver()

    def _get_planner_config(self, planner) -> dict:
        if isinstance(planner, SinglePlanner):
            # plan_pose only honors max_attempts in v2 (timeout / time_dilation
            # are not solver args anymore — speed lives in the robot YAML cspace).
            return {
                'max_attempts': self.get_parameter('max_attempts').value,
            }
        if isinstance(planner, ReactiveController):
            return {
                'convergence_threshold': self.get_parameter('convergence_threshold').value,
                'max_iterations': self.get_parameter('max_mpc_iterations').value,
            }
        return {}

    # ------------------------------------------------------------------
    # Start state + pending-plan (preview cache) helpers
    # ------------------------------------------------------------------

    def _resolve_start_state(self, src):
        """Return (start_joint_list, start_state) from a request/goal start_pose.

        Falls back to the robot's current joint pose when start_pose is empty.
        Shared by the generate_trajectory service and the execute action.
        """
        start_pose = getattr(src, 'start_pose', None)
        if start_pose is not None and len(start_pose.position) > 0:
            start_joint_pose = list(start_pose.position)
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
        return start_joint_pose, start_state

    def _store_pending_plan(self, start_state, req):
        """Cache the just-planned open-loop trajectory's identity for reuse."""
        self._pending_plan = {
            'planner': self.planner_manager.get_current_planner_type(),
            'signature': self._target_signature(start_state, req),
            'stamp': time.monotonic(),
        }

    def _pending_plan_matches(self, start_state, req) -> bool:
        """True if the cached plan is fresh and targets the same goal/start."""
        pp = self._pending_plan
        if pp is None:
            return False
        ttl = self.get_parameter('trajectory_cache_ttl').get_parameter_value().double_value
        if (time.monotonic() - pp['stamp']) > ttl:
            return False
        if pp['planner'] != self.planner_manager.get_current_planner_type():
            return False
        return self._signatures_match(pp['signature'], self._target_signature(start_state, req))

    @staticmethod
    def _pose_tuple(p):
        return (
            p.position.x, p.position.y, p.position.z,
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z,
        )

    def _target_signature(self, start_state, req) -> dict:
        start = [float(x) for x in start_state.position[0].cpu().tolist()]
        tp = getattr(req, 'target_pose', None)
        poses = getattr(req, 'target_poses', None)
        return {
            'start': start,
            'target_pose': self._pose_tuple(tp) if tp is not None else None,
            'target_poses': [self._pose_tuple(p) for p in poses] if poses else None,
            'target_joints': [float(x) for x in (getattr(req, 'target_joint_positions', []) or [])],
        }

    @staticmethod
    def _poses_match(a, b, pos_tol, ori_tol) -> bool:
        if a is None and b is None:
            return True
        if a is None or b is None:
            return False
        if any(abs(a[i] - b[i]) > pos_tol for i in range(3)):
            return False
        dot = sum(a[3 + i] * b[3 + i] for i in range(4))
        return (1.0 - abs(dot)) < ori_tol

    def _signatures_match(self, a, b, pos_tol=1e-3, ori_tol=1e-2, joint_tol=1e-3) -> bool:
        if len(a['start']) != len(b['start']) or any(
                abs(x - y) > joint_tol for x, y in zip(a['start'], b['start'])):
            return False
        aj, bj = a['target_joints'], b['target_joints']
        if len(aj) != len(bj) or any(abs(x - y) > joint_tol for x, y in zip(aj, bj)):
            return False
        if not self._poses_match(a['target_pose'], b['target_pose'], pos_tol, ori_tol):
            return False
        ap, bp = a['target_poses'], b['target_poses']
        if (ap is None) != (bp is None):
            return False
        if ap is not None:
            if len(ap) != len(bp):
                return False
            if any(not self._poses_match(x, y, pos_tol, ori_tol) for x, y in zip(ap, bp)):
                return False
        return True

    def clear_trajectory_callback(self, request, response):
        """Discard any cached (preview) trajectory on user request."""
        had = self._pending_plan is not None
        self._pending_plan = None
        response.success = True
        response.message = "Cached trajectory cleared" if had else "No cached trajectory"
        self.get_logger().info(response.message)
        return response

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

    # ------------------------------------------------------------------
    # Native grasp pipeline (plan_grasp action) + attach/detach
    # ------------------------------------------------------------------

    @staticmethod
    def _as_bool(t) -> bool:
        """Reduce a cuRobo success tensor (or plain bool/None) to a Python bool."""
        if t is None:
            return False
        if hasattr(t, 'any'):
            try:
                return bool(t.any().item())
            except Exception:
                return bool(t)
        return bool(t)

    def grasp_callback(self, goal_handle):
        """Plan (and optionally execute) a native cuRobo grasp pipeline."""
        result_msg = GraspPlan.Result()
        try:
            goal = goal_handle.request

            # The grasp planner reuses the shared MotionPlanner.
            if self.motion_planner is None:
                self.get_logger().info("On-demand warmup for grasp: MotionPlanner")
                self._warmup_classic()
            SinglePlanner.set_motion_planner(self.motion_planner)

            _, start_state = self._resolve_start_state(goal)
            self.refresh_perception_world()

            self.get_logger().info(
                f"Planning grasp ({len(goal.grasp_candidates)} candidate(s))")
            fb = GraspPlan.Feedback()
            fb.phase = "PLANNING"
            fb.progression = 0.0
            goal_handle.publish_feedback(fb)

            result = self.grasp_planner.plan(start_state, goal)

            result_msg.approach_success = self._as_bool(result.approach_success)
            result_msg.grasp_success = self._as_bool(result.grasp_success)
            result_msg.lift_success = self._as_bool(result.lift_success)
            success = self._as_bool(result.success)
            result_msg.success = success
            result_msg.message = result.status or (
                "Grasp planned" if success else "Grasp planning failed")
            result_msg.dt = GraspPlanner.dt_of(result)
            result_msg.approach_trajectory = GraspPlanner.traj_to_msgs(
                result.approach_interpolated_trajectory,
                result.approach_interpolated_last_tstep)
            result_msg.grasp_trajectory = GraspPlanner.traj_to_msgs(
                result.grasp_interpolated_trajectory,
                result.grasp_interpolated_last_tstep)
            result_msg.lift_trajectory = GraspPlanner.traj_to_msgs(
                result.lift_interpolated_trajectory,
                result.lift_interpolated_last_tstep)

            if not success:
                self.get_logger().error(f"Grasp planning failed: {result_msg.message}")
                fb.phase = "FAILED"
                goal_handle.publish_feedback(fb)
                goal_handle.abort()
                return result_msg

            if not goal.execute:
                self.get_logger().info("Grasp planned (execute=false)")
                goal_handle.succeed()
                return result_msg

            self.get_logger().info("Executing grasp pipeline")
            exec_ok = self.grasp_planner.execute(
                self.robot_context, result, goal, goal_handle)

            if goal_handle.is_cancel_requested:
                result_msg.success = False
                result_msg.message = "Grasp execution canceled"
                goal_handle.canceled()
                return result_msg

            result_msg.success = bool(exec_ok)
            result_msg.message = "Grasp completed" if exec_ok else "Grasp execution failed"
            if exec_ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result_msg

        except Exception as e:
            self.get_logger().error(f"Grasp error: {e}")
            self.get_logger().error(traceback.format_exc())
            result_msg.success = False
            result_msg.message = f"Error: {e}"
            if goal_handle.is_active:
                goal_handle.abort()
            return result_msg

    def attach_object_callback(self, request, response):
        """Attach a scene obstacle to the arm at its current joint configuration."""
        try:
            if self.motion_planner is None:
                self._warmup_classic()
            _, current_state = self._resolve_start_state(None)
            self.grasp_planner.attach(request.object_name, current_state)
            response.success = True
            response.message = f"Attached '{request.object_name}'"
        except Exception as e:
            response.success = False
            response.message = f"Attach error: {e}"
            self.get_logger().error(response.message)
        return response

    def detach_object_callback(self, request, response):
        """Release a previously attached (grasped) object from the arm."""
        try:
            released = self.grasp_planner.detach()
            response.success = True
            response.message = (
                f"Detached '{released}'" if released else "Nothing attached")
        except Exception as e:
            response.success = False
            response.message = f"Detach error: {e}"
            self.get_logger().error(response.message)
        return response


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
