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

import threading
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
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner, GetPlanners
from curobo_msgs.action import SendTrajectory

from curobo.types import DeviceCfg, JointState

from curobo_ros.robot.robot_context import RobotContext
from curobo_ros.core.config_wrapper_motion import ConfigWrapperMotion
from curobo_ros.core.attachment_services import AttachmentServices
from curobo_ros.core.ik_services import IKServices
from curobo_ros.core.fk_services import FKServices
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

        # Serializes curobo CUDA-graph *capture* (classic first plan, MPC
        # cold-start) against concurrent GPU work on other executor threads —
        # notably the depth-camera callback's mapper.integrate(). A GPU op
        # launched while a stream is capturing invalidates the capture
        # (cudaErrorStreamCaptureInvalidated). Created before cameras/solvers so
        # it always exists when callbacks fire. RLock: the holder thread may
        # re-enter; a different thread (depth callback) fails the non-blocking
        # acquire and simply skips its frame.
        self.gpu_lock = threading.RLock()

        # Serializes goal admission: only one execute_trajectory goal may be
        # active at a time (open-loop or reactive). Guards against two
        # concurrent servo/execute loops driving the robot at once. Set in
        # goal_callback (accept-time, closing the race with execute_callback)
        # and cleared in execute_callback's finally. Also consulted by
        # set_planner_callback to refuse switching planners mid-goal.
        self._goal_lock = threading.Lock()
        self._goal_active = False

        # Output sampling step (s) of the interpolated trajectory (trajopt) —
        # curobo_ros is the authority on this value (see resolve_interpolation_dt);
        # it is what every JointCommandStrategy stamps into time_from_start.
        # MUST be declared before RobotContext is constructed below: RobotContext
        # reads it (via resolve_interpolation_dt) to build its strategies.
        self.declare_parameter('interpolation_dt', 0.025)

        self.robot_context = RobotContext(self)

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
        # NOTE: the TSDF decay knob is `decay_half_life_s` (seconds), declared by
        # ObstacleManager._load_perception_params. The raw per-integrate
        # `decay_factor` is no longer exposed: it is derived from the half-life
        # and the cameras' combined frame rate.

        self.declare_parameter('collision_activation_distance', 0.025)
        self.declare_parameter('convergence_threshold', 0.01)
        self.declare_parameter('max_mpc_iterations', 1000)
        # Capture/replay CUDA graphs in the solvers (faster, but a captured
        # MotionGen graph can be invalidated by intervening MPC activity — see
        # the MPC->Classic re-warmup in _setup_planner). Override at runtime with
        # the CUROBO_USE_CUDA_GRAPH env var (0 disables) for A/B testing.
        self.declare_parameter('use_cuda_graph', True)
        # Diagnostic toggle (see update_all_solvers_world): set false to withhold
        # the perception ESDF from the solvers. Leave true for normal operation —
        # false disables camera-based collision avoidance.
        self.declare_parameter('push_esdf_to_solvers', True)
        # Reactive (MPC) solver build params — read by MPCController.build_solver().
        self.declare_parameter('mpc_step_dt', 0.03)
        self.declare_parameter('mpc_horizon_steps', 30)
        # LBFGS iterations per optimize call. cuRobo's defaults (200 warm-start,
        # 300 cold-start) are tuned for the offline getting-started demo, not
        # real-time control — they made optimize_action_sequence() take ~1-1.3s
        # per call on this hardware (vs. optimization_dt=0.03s), so the arm went
        # long stretches uncorrected then jumped, causing overshoot/oscillation.
        # NOTE: cuRobo's LBFGS requires num_iters to be a MULTIPLE of its inner
        # loop size (25) — e.g. 25/50/75/100 are valid, 10 raises ValueError.
        # Isolated test (franka.yml, no real hardware): 25/100 -> ~18ms/call
        # (vs ~74ms/call at 200/300) and converges FASTER in wall-clock terms
        # despite fewer iterations per call (more, cheaper corrections beat
        # fewer, expensive ones for a receding-horizon controller).
        # 5/10 are the MPPI values; the 25/100 in the note above are the L-BFGS
        # ones, kept here because they pair with mpc_solver_type below.
        self.declare_parameter('mpc_warm_start_iters', 5)
        self.declare_parameter('mpc_cold_start_iters', 10)
        # MPC solver selection. The default is 'mppi_acceleration' (MPPI in
        # ACCELERATION space), the recipe validated on the real M1013 -- it holds
        # in the postures where L-BFGS + B-spline stalls.
        # The two iteration counts above are tuned FOR THIS DEFAULT. Switching to
        # 'lbfgs_bspline' means also passing mpc_warm_start_iters:=25
        # mpc_cold_start_iters:=100 -- cuRobo's L-BFGS requires multiples of 25
        # (see the note above), and 5/10 would leave it untuned.
        self.declare_parameter('mpc_solver_type', 'mppi_acceleration')
        self.declare_parameter('mpc_mppi_num_particles', 400)
        self.declare_parameter('mpc_vel_feedback_alpha', 1.0)
        # Fixed-interval command pacing (seconds). 0.0 = off (re-solve/re-send as
        # fast as the solve allows, ~70ms — replaces the previous window before the
        # bridge finishes it). >0 = hold each command window for this long before
        # re-solving/re-sending, and read the real robot state only AFTER it has
        # executed (fresh + velocity-consistent). Set to the window duration
        # (interpolation_steps*2 * mpc_step_dt = 8*0.03 = 0.24) to fully execute
        # each window. cf. debug 2026-07-16.
        self.declare_parameter('mpc_command_interval', 0.24)
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

        # Adopt the canonical joint order/DOF into the RobotContext descriptor now
        # that the kinematics exist (RobotContext is built before the kin model).
        self.robot_context.bind_kinematics(self.config_wrapper_motion.kin_model)

        # Shared Scene for all planners — references ObstacleManager's Scene.
        # All planners see the same obstacles after update_world(scene).
        self.shared_scene = self.config_wrapper_motion.obstacle_manager.get_scene()

        # Solvers (created on demand).
        self.motion_planner = None  # v2 alias
        self.motion_gen = None      # legacy alias, kept for older code paths
        self.mpc = None             # reactive: ModelPredictiveControl
        self.retargeter = None      # reactive: MotionRetargeter (teleop)

        # Which solver currently owns the single live CUDA graph. Two captured
        # graphs cannot safely coexist in one CUDA context: the inactive one's
        # baked device addresses get invalidated by the active one's allocator
        # activity, so replaying it segfaults in cuGraphLaunch. We enforce
        # "exactly one live graph" by releasing every other solver's graph
        # whenever the owner changes (see _ensure_exclusive_graph). Keys:
        # 'motion' for all open-loop planners (they share one MotionPlanner),
        # 'reactive:<name>' for each reactive controller. None = no owner yet.
        self._active_graph_owner = None

        # True whenever the next call into a reactive solver may CAPTURE a CUDA
        # graph (fresh solver, or right after a release) rather than just replay
        # one. Capture is process-global — no other thread may issue ANY CUDA op
        # while it's in progress — so callers use take_graph_capture_pending() to
        # decide whether their next solver call needs gpu_lock. Guards only the
        # step(s) that can actually capture; steps that only replay stay
        # unguarded so the perception thread isn't starved. cf. debug 2026-07-28.
        self._graph_capture_pending = True
        self._graph_capture_pending_lock = threading.Lock()

        # Attach/detach a scene obstacle to the arm's attached_object link
        # (standalone feature — pre-positioned objects, simulation, tests).
        # Registers its own attach_object/detach_object services.
        self.attachment_services = AttachmentServices(self, self.config_wrapper_motion)

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
        # Startup warms exactly one solver, so it is already the sole graph
        # owner — record it so the first plan reuses that warmup graph instead
        # of needlessly releasing and re-capturing it.
        self._active_graph_owner = self._graph_owner_key(
            self.planner_manager.get_current_planner())

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
        obstacle_manager = self.config_wrapper_motion.obstacle_manager

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
        # ESDF recompute + world push are GPU ops — hold the lock so they never
        # overlap a concurrent depth integrate / graph capture.
        with self.gpu_lock:
            if obs.refresh_esdf():
                self.update_all_solvers_world(obs.get_scene())

    def rebuild_solvers_for_cache_change(self):
        """Rebuild all active solvers after a collision-cache change.

        The collision cache is allocated at solver creation, so a change
        requires recreating the solvers (a world update is not sufficient).
        Registered as ObstacleManager's cache-change observer.

        Held under gpu_lock: this (re)captures CUDA graphs, same invariant as
        every other graph-capturing path in this node (see gpu_lock's other
        acquisitions) — without it, a concurrent camera integrate()/perception
        refresh could invalidate the graph being captured here
        (cudaErrorStreamCaptureInvalidated).
        """
        self.get_logger().info(
            "Collision cache changed - rebuilding solvers (blocking, ~20s)...")

        with self.gpu_lock:
            # Motion planner (present after the initial warmup).
            if self.motion_planner is not None:
                self.config_wrapper_motion.set_motion_gen_config(self, None, None)
                SinglePlanner.set_motion_planner(self.motion_planner)

            # IK (only if it was initialized). IKServices reads the canonical
            # (motion) cache directly, so no sync is needed.
            self.ik_services.rebuild()

            # Reactive controllers (only if initialized). Built from the SAME
            # shared cache, so just rebuild their solvers — no manual cache copy.
            if self.mpc is not None:
                self.planner_manager.get_planner('mpc').rebuild_solver()
            if self.retargeter is not None:
                self.planner_manager.get_planner('retarget').rebuild_solver()

            # Rebuilt solvers hold no graph yet — their first step() will
            # capture, so mark it pending (see take_graph_capture_pending).
            self._set_graph_capture_pending()

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
            with self.gpu_lock:
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
                    with self.gpu_lock:
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
        # Held under gpu_lock, same invariant as every other graph-capturing path
        # here (refresh_perception_world, rebuild_solvers_for_cache_change, both
        # planner.plan() call sites). This one was MISSING it, and that is not
        # theoretical: switching MPC -> Classic mid-session with the cameras live
        # crashed the node (2026-08-08). _warmup_classic() captures the seed-IK
        # CUDA graph, and torch.cuda.graph() defaults to capture_error_mode
        # "global" -- while a capture is live, ANY CUDA op from ANY thread of the
        # process fails with cudaErrorStreamCaptureUnsupported. The depth callback
        # honours its half of the contract (non-blocking acquire then skip the
        # frame, camera_depth_map_strategy.py:157), but a lock nobody holds is
        # always free, so it ran mapper.integrate() straight into the capture.
        # The capture aborted while leaving the stream in
        # cudaStreamCaptureStatusActive, which poisoned the CUDA context for the
        # whole process: 106 consecutive CUDA failures, no recovery short of a
        # node restart.
        #
        # gpu_lock is an RLock and is the OUTERMOST lock in the documented order
        # (robot_context.py:17-37), so acquiring it here is deadlock-free even
        # when a caller already holds it. Cost: the depth callback drops its
        # frames during the ~6 s warmup -- exactly the designed behaviour.
        with self.gpu_lock:
            # Enforce a single live CUDA graph before this planner runs: if it
            # isn't the current graph owner, release every other solver's captured
            # graph so this one re-captures (on its next call) as the sole live
            # graph. Cheap no-op when the owner is unchanged (repeated plans reuse
            # the graph).
            self._ensure_exclusive_graph(planner)

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

    # ------------------------------------------------------------------
    # CUDA graph exclusivity (only the active solver holds a live graph)
    # ------------------------------------------------------------------

    def _graph_owner_key(self, planner):
        """Stable key identifying which captured graph a planner would use.

        All open-loop planners share one MotionPlanner (one graph) -> 'motion'.
        Each reactive controller owns its own solver/graph -> 'reactive:<name>'.
        """
        if isinstance(planner, SinglePlanner):
            return 'motion'
        if isinstance(planner, ReactiveController):
            return f'reactive:{planner.get_planner_name()}'
        return None

    def _ensure_exclusive_graph(self, planner):
        """Guarantee the given planner is the sole owner of a live CUDA graph."""
        self._ensure_exclusive_graph_key(self._graph_owner_key(planner))

    def _ensure_exclusive_graph_key(self, owner_key):
        """Make owner_key the sole live-graph owner, releasing all others.

        No-op if graphs are disabled or the owner is unchanged. Otherwise release
        every captured graph (the incoming solver re-captures cleanly, alone, on
        its next call) and record the new owner. See _active_graph_owner.
        """
        if not self.config_wrapper_motion.use_cuda_graph:
            return
        if owner_key is None or owner_key == self._active_graph_owner:
            return
        self.get_logger().info(
            f"CUDA graph owner {self._active_graph_owner} -> {owner_key}: releasing "
            f"other captured graphs so only the active solver holds one")
        self._release_all_solver_cuda_graphs()
        self._active_graph_owner = owner_key

    def _release_all_solver_cuda_graphs(self):
        """Free every solver's captured CUDA graph (frees the graph + its pool).

        reset_cuda_graph() -> GraphExecutor.reset() calls CUDAGraph.reset(), which
        releases the graph's private memory pool; the solver itself and its shared
        collision world are untouched (it re-captures on next use). Held under the
        GPU lock so it can't overlap a concurrent capture / depth integrate.
        """
        with self.gpu_lock:
            if self.motion_planner is not None:
                for name in ('trajopt_solver', 'ik_solver'):
                    self._safe_reset_graph(getattr(self.motion_planner, name, None))
            self._safe_reset_graph(self.mpc)
            self._safe_reset_graph(self.retargeter)
            # Every solver now holds no graph — its next call captures.
            self._set_graph_capture_pending()

    def _set_graph_capture_pending(self):
        """Mark that the next reactive solver call may capture a CUDA graph."""
        with self._graph_capture_pending_lock:
            self._graph_capture_pending = True

    def take_graph_capture_pending(self) -> bool:
        """Atomically take-and-clear: True if the next solver call may capture.

        Callers (ReactiveController._step_guard) use this to decide whether
        their next step() needs gpu_lock — capture is process-global (see
        _graph_capture_pending's docstring), replay is not. Take-and-clear
        mirrors _take_live_goal(): exactly one caller sees True per release.
        """
        with self._graph_capture_pending_lock:
            pending = self._graph_capture_pending
            self._graph_capture_pending = False
            return pending

    def _safe_reset_graph(self, solver):
        """Release a solver's captured CUDA graph(s); never raise into callers.

        Uses reset_cuda_graph(), NOT destroy(): SolverCore.destroy() shares its
        body with reset_cuda_graph() (resets optimizer/metrics_rollout/additional
        rollouts) but drops both of reset_cuda_graph()'s guards (use_cuda_graph,
        _task_initialized). Calling it unconditionally on metrics_rollout was
        tried and found to orphan its constraint tensors -- con_scene_collision
        froze at a huge constant instead of tracking live geometry (see debug
        2026-07-25) -- while cost_tool_pose_pos kept updating from the same
        get_current_metrics() call, proving the rollout's buffers were left in a
        stale/half-reset state rather than actually corrupted geometry.

        reset_cuda_graph() alone still has the original gap: IKSolver.reset_cuda_graph()
        never reaches the nested SeedIKSolver's private Levenberg-Marquardt
        GraphExecutor. That gap is what caused the original cuGraphLaunch segfault
        on Classic after MPC. So instead of destroy(), reach seed_ik_solver
        explicitly and narrowly: SeedIKSolver.destroy() only resets its own two
        GraphExecutors and touches no rollout/collision state, so it's safe to
        call unconditionally. Also recurse into a nested ik_solver (MpcSolver owns
        one for goal-state IK; MpcSolver.reset_cuda_graph() only forwards to
        self.core and never releases it, so its seed-IK graphs were never being
        freed at all).
        """
        if solver is None:
            return
        try:
            if hasattr(solver, 'reset_cuda_graph'):
                solver.reset_cuda_graph()
            seed_ik_solver = getattr(solver, 'seed_ik_solver', None)
            if seed_ik_solver is not None:
                seed_ik_solver.destroy()
            nested_ik_solver = getattr(solver, 'ik_solver', None)
            if nested_ik_solver is not None and nested_ik_solver is not solver:
                self._safe_reset_graph(nested_ik_solver)
        except Exception as e:
            self.get_logger().warn(
                f"CUDA graph release failed on {type(solver).__name__}: {e}")

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
