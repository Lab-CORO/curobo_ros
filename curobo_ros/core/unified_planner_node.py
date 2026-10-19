#!/usr/bin/env python3
"""
Unified trajectory planner node (v2) using Strategy Pattern.

Supports multiple planning strategies (Classic, MPC, Multi-Point, Joint-Space)
and allows dynamic switching between them.

v2 notes:
- MotionGen → MotionPlanner (built by ConfigWrapperMotion, held by SolverHost
  as 'motion').
- MpcSolver → ModelPredictiveControl (built by MPPIController from the shared
  context, held by SolverHost as 'mpc').
- TensorDeviceType → DeviceCfg. We read device/dtype from the wrapper.
- WorldConfig → Scene (obstacle_manager.get_scene()), the single source of
  truth, propagated to solvers via update_world(scene).
- Perception: camera data → Mapper (ObstacleManager) → ESDF VoxelGrid in the
  Scene, refreshed on-demand before each plan via refresh_perception_world().
- ground plane lives on the Scene via ObstacleManager, not `world_cfg.add_obstacle`.
"""

import os
import threading
import time
import traceback

import rclpy
import torch
import tf2_ros
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState as JointStateMsg
from std_srvs.srv import Trigger
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner, GetPlanners
from curobo_msgs.action import SendTrajectory

from curobo.types import DeviceCfg, JointState

from curobo_ros.robot.robot_context import RobotContext
from curobo_ros.core.config_wrapper_motion import ConfigWrapperMotion
from curobo_ros.core.attachment_services import AttachmentServices
from curobo_ros.core.ik_services import IKServices
from curobo_ros.core.fk_services import FKServices
from curobo_ros.core.config_wrapper import resolve_use_cuda_graph
from curobo_ros.core.solver_host import SolverHost
from curobo_ros.core import parameters
from curobo_ros.planners import (
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

        # Serializes goal admission: only one execute_trajectory goal may be
        # active at a time (open-loop or reactive). Guards against two
        # concurrent servo/execute loops driving the robot at once. Set in
        # goal_callback (accept-time, closing the race with execute_callback)
        # and cleared in execute_callback's finally. Also consulted by
        # set_planner_callback to refuse switching planners mid-goal.
        self._goal_lock = threading.Lock()
        self._goal_active = False

        # Every parameter is declared here, from the one schema
        # (core/parameters.py: type, default, description, when it applies).
        # MUST precede RobotContext, which reads robot and interpolation_dt.
        parameters.declare_all(self)

        self.robot_context = RobotContext(self)

        # Single shared context for every solver (robot + obstacles + scene +
        # collision cache). The MPC solver is built lazily from this same context.
        # Sole owner of the solvers, GPU access and CUDA-graph ownership
        # (ADR-0001). Created before cameras/solvers so it always exists when
        # their callbacks fire.
        self.solver_host = SolverHost(self.get_logger(), resolve_use_cuda_graph(self))
        self.config_wrapper_motion = ConfigWrapperMotion(self, self.robot_context)

        # Adopt the canonical joint order/DOF into the RobotContext descriptor now
        # that the kinematics exist (RobotContext is built before the kin model).
        self.robot_context.bind_kinematics(self.config_wrapper_motion.kin_model)

        # Shared Scene for all planners — references ObstacleManager's Scene.
        # All planners see the same obstacles after update_world(scene).
        self.shared_scene = self.config_wrapper_motion.obstacle_manager.get_scene()

        # TF buffer for AttachmentServices: AttachObject.srv's header.frame_id
        # lets a caller express the payload pose in any TF frame (typically
        # the gripper link) instead of only the planner's own base frame.
        # Node-level rather than inside AttachmentServices: a TransformListener
        # subscribes to /tf and has no business living inside a service.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Attach/detach a payload to the arm's attached_object link
        # (standalone feature — pre-positioned objects, simulation, tests).
        # Registers its own attach_object/detach_object services.
        self.attachment_services = AttachmentServices(self, self.config_wrapper_motion)

        # Cached open-loop plan from a generate_trajectory (preview) call, reused
        # by the execute action when its target matches. See _pending_plan_*.
        self._pending_plan = None  # {'planner': key, 'signature': sig, 'stamp': monotonic}

        # Shared IK — same Scene as MotionPlanner.
        self.ik_services = IKServices(self, self.config_wrapper_motion)

        # FK poses are purely geometric (robot YAML only); its collision
        # checker (poses_valid) shares the same Scene as MotionPlanner/IK.
        self.fk_services = FKServices(self, self.config_wrapper_motion)

        self.planner_manager = PlannerManager(self, self.config_wrapper_motion)

        initial_planner = self.get_parameter('planner_type').get_parameter_value().string_value
        self._warmup_initial_planner(initial_planner)
        self.planner_manager.set_current_planner(initial_planner)
        # Startup warms exactly one solver: the first activation adopts it as
        # graph owner without releasing, so the first plan reuses its warmup graph.
        self.solver_host.activate(self.planner_manager.get_current_planner().SOLVER_KEY)

        # Pre-warm the voxel-grid primitive rasterization path while the executor
        # is not yet spinning (no service can be served until __init__ returns),
        # so the first get_voxel_grid call never pays kernel compilation.
        self.config_wrapper_motion.obstacle_manager.prewarm_voxel_rasterization(self)

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

        from geometry_msgs.msg import Pose as PoseMsg
        self.mpc_goal_sub = self.create_subscription(
            PoseMsg,
            f'{self.get_name()}/mpc_goal',
            self.mpc_goal_callback,
            10,
        )

        # Every parameter is declared by now (descriptor-dependent ones by
        # RobotContext / ConfigManager): report the overrides nothing reads,
        # then log what later runtime changes reach.
        parameters.warn_unused_overrides(self)
        parameters.install_change_hook(self)

        self.get_logger().info(
            "Unified planner ready with initial planner: "
            f"{self.planner_manager.get_current_planner().get_planner_name()}"
        )

    # ------------------------------------------------------------------
    # Warmup
    # ------------------------------------------------------------------

    def _warmup_initial_planner(self, planner_type: str):
        self.get_logger().info(f"Warming up {planner_type} planner...")

        # Warm exactly the solver the initial planner will use: the first
        # SolverHost.activate() adopts it as graph owner without releasing.
        key = PlannerFactory.canonical_key(planner_type)
        if key == 'mpc':
            self._warmup_mpc()
        elif key == 'lbfgs':
            self._warmup_lbfgs()
        elif key == 'retarget':
            self._warmup_reactive('retarget')
        elif key in ('classic', 'multi_point', 'joint_space'):
            self._warmup_classic()
        else:
            self._warmup_classic()
            self.get_logger().warn(
                f"Planner '{planner_type}' not fully wired, falling back to classic warmup"
            )

        self.get_logger().info(f"{planner_type} planner ready")

    def _warmup_classic(self):
        """Warm up MotionPlanner for Classic/MultiPoint/JointSpace planners."""
        if self.solver_host.get('motion') is None:
            self.get_logger().info("  -> Initializing MotionPlanner...")
            self.solver_host.ensure('motion')
            self.get_logger().info("  -> MotionPlanner ready")
        else:
            self.get_logger().info("  -> MotionPlanner already initialized (cache)")

    def _ensure_ground_plane(self):
        """No-op: the ground is the `floor` cuboid of the loaded world file.

        Both shipped worlds put it at z=-0.8 (config/floor_world.yml here,
        leeloo_world.yaml in the leeloo deployment).

        This method used to add a `ground` cuboid at z=-0.1 at runtime. That
        extra cuboid landed AFTER the voxelization SceneCollision had been
        pre-allocated (sized on the world file's three cuboids), so it overflowed
        that cache on every perception refresh -> CPU fallback (~10s) that froze
        the MPC loop. Removed: the robot base sits ~80cm above the floor, which
        is therefore collision ground enough.

        Kept as a no-op rather than deleted because the reactive warmup path
        still calls it (see _warmup_reactive).
        """
        return

    def _warmup_reactive(self, key: str):
        """Build a reactive controller's solver on demand (held by SolverHost)."""
        self._ensure_ground_plane()
        self.planner_manager.get_planner(key).ensure_solver()
        self.get_logger().info(f"  -> Reactive '{key}' solver ready")

    def _warmup_mpc(self):
        """Warm up the v2 ModelPredictiveControl solver on demand."""
        if self.solver_host.get('mpc') is not None:
            self.get_logger().info("  -> MPC solver already initialized (cache)")
            return
        self.get_logger().info("  -> Initializing MPC solver...")
        self._warmup_reactive('mpc')

    def _warmup_lbfgs(self):
        """Warm up the LBFGSController's ModelPredictiveControl solver on demand."""
        if self.solver_host.get('lbfgs') is not None:
            self.get_logger().info("  -> LBFGS solver already initialized (cache)")
            return
        self.get_logger().info("  -> Initializing LBFGS solver...")
        self._warmup_reactive('lbfgs')

    def update_all_solvers_world(self, scene=None, active_only=False):
        """Propagate scene updates to all initialized solvers.

        ``active_only=True`` (used by the reactive servo loop's periodic
        perception refresh, see refresh_perception_world) skips every solver
        that is NOT the currently selected planner. Measured 2026-08-18 on a
        Jetson Orin AGX (lbfgs_debug logs): with only LBFGS actively
        servoing, update_world[motion_planner]=20.1ms fired on EVERY
        perception refresh anyway -- ~49% of the whole refresh_perception_world
        cost -- purely because a MotionPlanner (open-loop SinglePlanner) had
        been warmed up earlier in the session and switching away from it
        (SetPlanner) never tears its solver down (see _warmup_reactive). That
        stale push is pure waste: whichever planner is NOT active right now
        gets a full, unrestricted refresh_perception_world() anyway right
        before its own next plan() call (see the call site above
        set_planner_callback's _setup_planner), so skipping it here on every
        OTHER planner's servo tick loses nothing. Default stays False (push
        to every initialized solver) for every other caller -- e.g. AddObject/
        RemoveObject/SetCollisionCache -- where a manual scene edit should
        reach every solver immediately, not just the active one.
        """
        scene = scene if scene is not None else self.shared_scene
        obstacle_manager = self.config_wrapper_motion.obstacle_manager
        # PlannerFactory (planner_factory.py:33) is the single source of truth
        # for which planners exist. This method no longer needs to know
        # anything about SinglePlanner/ReactiveController itself -- "has a
        # solver", "push scene to it", and "which solver instance is this"
        # are now TrajectoryPlanner interface methods (has_solver/
        # update_world/world_identity, see that class), each overridden
        # where the concept differs (SinglePlanner: one shared MotionPlanner
        # across all its subclasses; ReactiveController: exclusive
        # per-instance solver).
        current_planner = self.planner_manager.get_current_planner() if active_only else None

        # DIAGNOSTIC (default off => normal behaviour). Withholds the perception
        # ESDF voxel layer from the solvers so only analytic primitives remain,
        # to test whether the voxel layer is what pegs con_scene_collision at its
        # sentinel value. SAFETY: with this enabled the solvers do NOT see
        # camera-observed obstacles, so run it only with a clear workspace.
        if not self.get_parameter('push_esdf_to_solvers').get_parameter_value().bool_value:
            scene = obstacle_manager.primitives_only_scene()
            self.get_logger().warn(
                "push_esdf_to_solvers=false: solvers see analytic primitives ONLY "
                "(no camera obstacles) - diagnostic mode",
                throttle_duration_sec=5.0)
        elif obstacle_manager.collision_cache["voxel"] is None:
            # No voxel cache pre-allocated in the solvers (SetCollisionCache
            # blox=0) — a scene carrying an ESDF layer would raise "Voxel cache
            # not initialized" inside update_world. Degrade gracefully: no
            # camera-based collision avoidance instead of a hard planning failure.
            scene = obstacle_manager.primitives_only_scene()
            self.get_logger().warn(
                "Voxel collision cache disabled (SetCollisionCache blox=0): "
                "solvers see analytic primitives ONLY, no camera obstacles",
                throttle_duration_sec=5.0)

        # Debug-only timing breakdown (cf. debug 2026-08-18: isolating whether
        # a slow perception refresh is native cuRobo work, e.g.
        # load_collision_model's voxel upload, or our own overhead -- and
        # whether it's being paid MORE than once per refresh because a
        # planner that was warmed up earlier in the session but isn't the
        # one actively running still gets pushed the same scene every time,
        # see this method's own docstring / _warmup_reactive: switching
        # SetPlanner away from a controller never tears its solver down).
        debug = self._debug_solver_update_timing()
        t0 = None
        current_identity = current_planner.world_identity() if current_planner is not None else None

        # Single uniform loop over every planner in PlannerFactory's catalog
        # -- no isinstance(SinglePlanner)/isinstance(ReactiveController)
        # here, and no separate MotionPlanner special case. A planner
        # only gets the scene if it actually has a live solver, its
        # world_identity() hasn't already been updated THIS call (dedup:
        # ClassicPlanner/MultiPointPlanner/JointSpacePlanner all resolve to
        # the SAME shared MotionPlanner -- without this a full refresh would
        # push the identical scene into it 3 times), and (when active_only)
        # its identity matches the currently active planner's. get_planner()
        # only constructs the lightweight Python wrapper (cheap, cached) --
        # has_solver() is False until SolverHost has built that planner's
        # solver, so a
        # never-used catalog entry costs nothing extra here.
        updated_identities = set()
        for key, _eid, _name in PlannerFactory.get_catalog():
            planner_obj = self.planner_manager.get_planner(key)
            if debug:
                t0 = time.monotonic()
            identity = planner_obj.world_identity()
            if (planner_obj.has_solver()
                    and identity not in updated_identities
                    and (not active_only or identity == current_identity)):
                planner_obj.update_world(scene)
                updated_identities.add(identity)
            if debug:
                self._log_solver_update_ms(key, t0)

        # ik_services is queried independently of whichever planner is
        # "active" (GetIK can be called anytime) -- never skipped by
        # active_only.
        self.ik_services.update_world(scene)
        if debug:
            self._log_solver_update_ms('ik_services', t0)

        # fk_services' collision checker (poses_valid), same reasoning as
        # ik_services above -- queried independently of the active planner.
        self.fk_services.update_world(scene)
        if debug:
            self._log_solver_update_ms('fk_services', t0)

    def _debug_solver_update_timing(self) -> bool:
        return bool(self.get_parameter('mpc_debug').value)

    def _log_solver_update_ms(self, name: str, t0: float):
        ms = (time.monotonic() - t0) * 1000.0
        # Only log solvers that actually did work (non-None) -- a 0.0-ish
        # entry would just be the None-check branch, not a real data point.
        # if ms > 0.05:
            # self.get_logger().info(f"update_world[{name}]={ms:.1f}ms")

    def refresh_perception_world(self, active_only=False):
        """Recompute the perception ESDF and push it to all solvers.

        Called on-demand before each plan (and per step during MPC) so the
        collision world reflects the latest camera data — no background timer,
        so no race with CUDA graph capture.

        ``active_only`` is forwarded to update_all_solvers_world (see its
        docstring) — the reactive servo loop's periodic call passes True,
        every other caller (pre-plan refresh, etc.) keeps the default False.
        """
        obs = self.config_wrapper_motion.obstacle_manager
        debug = self._debug_solver_update_timing()
        # ESDF recompute + world push are GPU ops — hold the lock so they never
        # overlap a concurrent depth integrate / graph capture.
        with self.solver_host.exclusive():
            t0 = time.monotonic() if debug else None
            refreshed = obs.refresh_esdf()
            if debug:
                self._log_solver_update_ms('compute_esdf+inspect', t0)
            if refreshed:
                self.update_all_solvers_world(obs.get_scene(), active_only=active_only)

    def rebuild_solvers_for_cache_change(self):
        """Rebuild all active solvers after a collision-cache change.

        The collision cache is allocated at solver creation, so a change
        requires recreating the solvers (a world update is not sufficient).
        Registered as ObstacleManager's cache-change observer.

        Held under exclusive GPU access: this (re)captures CUDA graphs
        (ADR-0001) — without it, a concurrent camera integrate()/perception
        refresh could invalidate the graph being captured here
        (cudaErrorStreamCaptureInvalidated).
        """
        self.get_logger().info(
            "Collision cache changed - rebuilding solvers (blocking, ~20s)...")

        # Every built solver, in registration order (MotionPlanner, IK, FK,
        # then the reactive controllers); each rebuild replays the attached
        # payload (SolverHost.on_built); only the graph owner is primed again.
        self.solver_host.rebuild_all()

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
            # plan() may (re)capture the MotionGen CUDA graph — hold the lock so a
            # concurrent depth integrate can't invalidate the capture (mirrors the
            # execute-action path). Without this, the camera callback's
            # mapper.integrate() races the capture -> cudaErrorStreamCaptureInvalidated.
            with self.solver_host.exclusive():
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
                # configured interpolation_dt param (curobo_ros's own authority
                # on dt) if we can't reach it, rather than an unrelated literal.
                response.dt = self.get_parameter(
                    'interpolation_dt').get_parameter_value().double_value
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
            return self._execute_goal(goal_handle, goal, planner, result_msg)
        finally:
            with self._goal_lock:
                self._goal_active = False

    def _execute_goal(self, goal_handle, goal, planner, result_msg):
        try:
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
                    # First plan captures the MotionGen CUDA graph — hold the
                    # lock so a concurrent depth integrate can't invalidate it.
                    with self.solver_host.exclusive():
                        result = planner.plan(start_state, goal, config, self.robot_context)
                    if not result.success:
                        self.get_logger().error(
                            f"Planning failed in execute path: {result.message}")
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

            with self._goal_lock:
                if self._goal_active:
                    response.success = False
                    response.message = (
                        "Cannot switch planner while an execution goal is active "
                        "— cancel it first."
                    )
                    response.previous_planner = previous_name
                    response.current_planner = previous_name
                    self.get_logger().error(response.message)
                    return response

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
            planner.set_live_goal([
                msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z,
            ])
            self.get_logger().debug(
                f"Reactive goal updated from topic: "
                f"[{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]"
            )
        else:
            name = planner.get_planner_name() if planner is not None else "none"
            self.get_logger().warn(
                f"Received reactive goal but current planner is {name} - goal ignored",
                throttle_duration_sec=5.0)

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
        # Exclusive GPU access, like every other graph-capturing path here:
        # _warmup_classic() captures the seed-IK graph, and a capture is
        # process-global (ADR-0001, incident 2026-08-08). The depth callback
        # drops its frames during the ~6 s warmup -- by design.
        with self.solver_host.exclusive():
            # Enforce a single live CUDA graph before this planner runs: if it
            # isn't the current graph owner, release every other solver's captured
            # graph so this one re-captures (on its next call) as the sole live
            # graph. Cheap no-op when the owner is unchanged (repeated plans reuse
            # the graph).
            self.solver_host.activate(planner.SOLVER_KEY)

            # Open-loop planners share the one 'motion' solver.
            if isinstance(planner, SinglePlanner):
                if self.solver_host.get('motion') is None:
                    self.get_logger().info("On-demand warmup: open-loop planner")
                    self._warmup_classic()

            # Reactive controllers build their own solver from the shared context.
            # ensure_solver() is idempotent (no-op once built), so just make sure a
            # ground plane exists and let the active controller build/reuse its solver.
            elif isinstance(planner, ReactiveController):
                if planner.solver is None:
                    self.get_logger().info("On-demand warmup: reactive controller")
                    self._ensure_ground_plane()
                planner.ensure_solver()
                # Capture every CUDA graph NOW, under exclusive access, so the
                # servo loop only ever replays. Covers both a fresh solver and
                # one whose graphs activate() just released. Not gated on
                # use_cuda_graph: the retargeter's sub-solvers capture
                # regardless (MotionRetargeterCfg forwards no flag); priming
                # is a no-op when nothing is pending. See
                # ReactiveController.prime_cuda_graphs.
                planner.prime_cuda_graphs()

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
                'convergence_threshold_rad': self.get_parameter('convergence_threshold_rad').value,
                'convergence_hold_steps': self.get_parameter('convergence_hold_steps').value,
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
        """Admit at most one execute_trajectory goal at a time.

        Two concurrent execute() loops (open-loop or reactive) would both
        stream commands to the robot. The flag is set here (accept-time) so
        there is no race with execute_callback starting; it is cleared in
        execute_callback's finally, however the goal ends (succeed / abort /
        cancel / exception).
        """
        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warn(
                    "Rejecting execution goal: another goal is already active "
                    "- cancel it first."
                )
                return rclpy.action.GoalResponse.REJECT
            self._goal_active = True
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
    # Dump a Python traceback on a fatal signal (SIGSEGV/SIGABRT). A GPU illegal
    # access surfaces as a native segfault (process exit -11) with no Python
    # error otherwise — faulthandler shows which CuRobo call was executing.
    # Redundant with PYTHONFAULTHANDLER=1 set in the launch file; harmless if run
    # standalone without that env.
    import faulthandler
    faulthandler.enable()

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
    # rclpy's own SIGINT handler already shuts the context down, so calling
    # shutdown() unconditionally raises "rcl_shutdown already called" and the
    # process exits 1 on every clean Ctrl-C — which made a normal stop
    # indistinguishable from a crash in the test suites' exit-code checks.
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
