"""
Single schema of the unified planner's ROS parameters.

Every parameter of the node is listed once in ``SPECS``: type and default,
description, and *when* a change takes effect (its ``kind``):

- ``STARTUP``: read once at node start. Declared ``read_only``: change it in
  the launch file and restart.
- ``BUILD``: read when a solver is built. A runtime change is accepted but
  only reaches the solvers listed in ``solvers`` when they are (re)built; the
  on-set hook logs which ones still hold the old value.
- ``LIVE``: re-read on each use (next request, goal or cycle).
- ``STATUS``: written by the node itself (not ``read_only``: rclpy would then
  refuse the node's own ``set_parameters``).

``declare_all`` declares everything at node start, with a
``ParameterDescriptor`` built from the spec, so ``ros2 param describe`` shows
it and ``ros2 param set`` works before the first goal or switch. Shared code
that also runs in other nodes (``RobotContext``, the command strategies,
diagnostics) calls ``declare``, which is idempotent and falls back to the same
spec. A few defaults depend on the robot descriptor (``DESCRIPTOR``): the call
site that knows the descriptor passes the value.

The MPPI and L-BFGS cost/optimizer trees (``mpc_mppi.*``, ``lbfgs.*``) keep
their defaults next to the controllers (``_MPPI_DEFAULTS``,
``_LBFGS_DEFAULTS``); ``declare_all`` declares one parameter per leaf.
"""

from dataclasses import dataclass
from typing import Any, Tuple

from rcl_interfaces.msg import (
    FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult)

STARTUP, BUILD, LIVE, STATUS = 'startup', 'build', 'live', 'status'

# Default supplied by the call site, from the robot descriptor.
DESCRIPTOR = object()

_MOTION = ('motion',)
_MPPI = ('mpc',)
_LBFGS = ('lbfgs',)
_RETARGET = ('retarget',)

# When each solver is (re)built, for the "takes effect" text.
_BUILT_WHEN = {
    'motion': 'the MotionPlanner (startup, update_motion_gen_config, set_collision_cache)',
    'mpc': 'the MPPI solver (first switch to mpc, set_collision_cache)',
    'lbfgs': 'the L-BFGS solver (first switch to lbfgs, set_collision_cache)',
    'retarget': 'the retargeter (first switch to retarget, set_collision_cache)',
}


@dataclass(frozen=True)
class Param:
    """One parameter: type and default, description, when it applies."""

    name: str
    default: Any
    kind: str
    description: str
    solvers: Tuple[str, ...] = ()   # BUILD: solver keys that read it
    bounds: Tuple = None            # (low, high), inclusive; int or float
    note: str = ''                  # extra "takes effect" detail

    def constraints(self) -> str:
        """When a change takes effect (descriptor additional_constraints)."""
        if self.kind == STARTUP:
            text = 'startup: set at launch, read-only at runtime'
        elif self.kind == BUILD:
            text = 'build: read when building ' + '; '.join(
                _BUILT_WHEN[key] for key in self.solvers)
        elif self.kind == LIVE:
            text = 'live: re-read on each use'
        else:
            text = 'status: written by the node'
        return f'{text}. {self.note}' if self.note else text

    def descriptor(self) -> ParameterDescriptor:
        """Build the ParameterDescriptor of this spec."""
        d = ParameterDescriptor(
            description=self.description,
            additional_constraints=self.constraints(),
            read_only=self.kind == STARTUP,
        )
        if self.bounds is not None:
            low, high = self.bounds
            if isinstance(low, int) and isinstance(high, int):
                d.integer_range = [IntegerRange(from_value=low, to_value=high, step=1)]
            else:
                d.floating_point_range = [FloatingPointRange(
                    from_value=float(low), to_value=float(high), step=0.0)]
        return d


_SPECS = [
    # ---- robot and world ----------------------------------------------------
    Param('robot', 'doosan_m1013', STARTUP,
          'Robot descriptor: robots/<name>.yaml or a path (model, driver topics, '
          'default strategy)'),
    Param('robot_config_file', DESCRIPTOR, STARTUP,
          'cuRobo robot YAML. Empty = the one resolved from the robot descriptor'),
    Param('base_link', DESCRIPTOR, STARTUP,
          'Robot base frame (default from the robot descriptor)'),
    Param('world_file', '', STARTUP,
          'Static world YAML loaded at startup. Empty = empty world, no floor'),
    Param('cameras_config_file', '', STARTUP,
          'Camera configuration YAML. Empty = no cameras'),
    Param('control_strategy', DESCRIPTOR, STATUS,
          'How commands reach the robot (emulator, joint_speed, joint_pose). Sets '
          'the startup strategy; change it at runtime with set_robot_strategy'),
    Param('node_is_available', False, STATUS,
          'True once the startup warmup has completed'),

    # ---- planning -----------------------------------------------------------
    Param('planner_type', 'classic', STARTUP,
          'Planner selected at startup (classic, mpc, lbfgs, multi_point, '
          'joint_space, retarget). Switch at runtime with set_planner'),
    Param('use_cuda_graph', True, STARTUP,
          'Capture/replay CUDA graphs in the solvers. CUROBO_USE_CUDA_GRAPH=0 '
          'overrides it'),
    Param('interpolation_dt', 0.025, BUILD,
          'Time step (s) of every output trajectory. Must match the command '
          'period of the robot driver', _MOTION + _MPPI + _LBFGS, (0.001, 1.0),
          'The command strategies re-read it at set_robot_strategy'),
    Param('collision_activation_distance', 0.025, BUILD,
          'Distance (m) at which the collision cost activates', _MOTION, (0.0, 1.0),
          'get_collision_distance reads it on each call'),
    Param('max_attempts', 1, LIVE,
          'Planning retries per request', bounds=(1, 1000)),
    Param('time_dilation_factor', 0.5, LIVE,
          'Feedback publish period (s) during open-loop execution. Not a speed '
          'control: set speed in the robot YAML cspace'),
    Param('trajectory_cache_ttl', 30.0, LIVE,
          'Lifetime (s) of a trajectory cached by generate_trajectory'),
    Param('push_esdf_to_solvers', True, LIVE,
          'Diagnostic: false withholds the camera ESDF from the solvers '
          '(disables camera-based avoidance)'),

    # ---- perception ---------------------------------------------------------
    Param('voxel_size', 0.05, STARTUP,
          'Voxel size (m) of the perception ESDF and the voxel collision cache; '
          'default resolution of get_voxel_grid', bounds=(0.001, 1.0)),
    Param('sparse_voxel_publish_rate', 7.0, STARTUP,
          'Publish rate (Hz) of voxel_grid_sparse. <= 0 disables it'),
    Param('use_mapper', True, STARTUP,
          'Enable the cuRobo Mapper (camera ESDF perception)'),
    Param('mapper_extent_xyz', [2.0, 2.0, 2.0], STARTUP,
          'Perception volume (m) around mapper_grid_center'),
    Param('mapper_grid_center', [0.0, 0.0, 0.0], STARTUP,
          'Centre of the perception volume, in the robot base frame'),
    Param('mapper_voxel_size', 0.02, STARTUP,
          'Internal TSDF voxel size (m)', bounds=(0.001, 1.0)),
    Param('mapper_image_height', 480, STARTUP,
          'Depth-projection height: every depth frame is resized to it'),
    Param('mapper_image_width', 640, STARTUP,
          'Depth-projection width: every depth frame is resized to it'),
    Param('mapper_depth_min', 0.1, STARTUP, 'Near depth clipping (m)'),
    Param('mapper_depth_max', 5.0, STARTUP, 'Far depth clipping (m)'),
    Param('decay_half_life_s', 0.7, STARTUP,
          'Time (s) for an unobserved voxel to lose half its TSDF weight. '
          '<= 0 disables the decay'),

    # ---- reactive control: common ------------------------------------------
    Param('convergence_threshold', 0.01, LIVE,
          'End-effector position error (m) considered on target'),
    Param('convergence_threshold_rad', 0.05, LIVE,
          'End-effector orientation error (rad) considered on target'),
    Param('convergence_hold_steps', 1, LIVE,
          'Consecutive in-tolerance steps before on_target goes true',
          bounds=(1, 100000)),
    Param('max_mpc_iterations', 1000, LIVE,
          'Safety cap on servo steps per goal'),
    Param('mpc_debug', False, LIVE,
          'Per-step diagnostic CSVs and [MPC DIAG] logs'),
    Param('lbfgs_debug', False, LIVE,
          'Perception timing logs (node-wide; unrelated to lbfgs.lbfgs_debug)'),
    Param('diagnostic_csv_dir', '', LIVE,
          'Directory of the diagnostic CSVs. Empty = the ROS log directory'),

    # ---- MPPI (planner mpc) -------------------------------------------------
    Param('mpc_step_dt', 0.03, BUILD, 'MPPI optimization time step (s)', _MPPI,
          (0.001, 1.0)),
    Param('mpc_horizon_steps', 30, BUILD, 'MPPI receding horizon length', _MPPI,
          (1, 10000)),
    Param('mpc_warm_start_iters', 5, BUILD, 'MPPI iterations per solve after the first',
          _MPPI, (1, 100000)),
    Param('mpc_cold_start_iters', 10, BUILD, 'MPPI iterations on the first solve',
          _MPPI, (1, 100000)),
    Param('mpc_mppi_num_particles', 400, BUILD, 'MPPI particle count', _MPPI,
          (1, 1000000)),
    Param('mpc_vel_feedback_alpha', 1.0, BUILD,
          'Blend of the measured velocity into the MPPI state', _MPPI, (0.0, 1.0)),
    Param('mpc_command_interval', 0.12, LIVE,
          'MPPI command pacing (s): each window executes before re-solving. '
          '0 = re-solve as fast as possible', note='Re-read at each goal'),

    # ---- L-BFGS (planner lbfgs) ---------------------------------------------
    Param('lbfgs_apply_custom_config', False, BUILD,
          "Use the lbfgs.* cost/optimizer tree instead of cuRobo's stock config",
          _LBFGS),
    Param('lbfgs_warm_start_iters', 0, BUILD,
          'L-BFGS iterations per solve after the first; multiple of 25. '
          '0 = cuRobo default', _LBFGS, (0, 100000)),
    Param('lbfgs_cold_start_iters', 0, BUILD,
          'L-BFGS iterations on the first solve; multiple of 25. 0 = cuRobo default',
          _LBFGS, (0, 100000)),
    Param('lbfgs_command_points', 6, BUILD,
          'Points per publish; n-1 are executed, the last is a spare (min 2)',
          _LBFGS),
    Param('lbfgs_publish_points', 11, LIVE,
          'Points actually published (>= lbfgs_command_points); the extra tail '
          'only plays when a solve runs late', note='Re-read at each goal'),
    Param('lbfgs_dynamic_m', True, BUILD,
          'Count executed points from the measured publish cycle, not the '
          'nominal one', _LBFGS),
    Param('lbfgs_profile_sync', False, BUILD,
          'Debug: synchronize CUDA at every phase boundary (slows the loop)',
          _LBFGS),

    # ---- retarget -----------------------------------------------------------
    Param('retarget_position_weight', 1.0, BUILD, 'Retarget position weight',
          _RETARGET),
    Param('retarget_orientation_weight', 1.0, BUILD, 'Retarget orientation weight',
          _RETARGET),
    Param('retarget_use_mpc', False, BUILD,
          'Follow frames after the first with the MPC sub-solver instead of '
          'local IK', _RETARGET),

    # ---- command strategies -------------------------------------------------
    Param('max_joint_accel_dps', DESCRIPTOR, LIVE,
          'joint_speed strategy: joint acceleration clamp (deg/s^2) against the '
          'measured velocity'),
    Param('joint_speed_debug_csv', False, LIVE,
          'joint_speed strategy: per-publish diagnostic CSV'),
]

SPECS = {p.name: p for p in _SPECS}

# Nested trees: prefix -> (module, defaults attribute, solver keys).
TREES = {
    'mpc_mppi': ('curobo_ros.planners.mppi_planner', '_MPPI_DEFAULTS', _MPPI),
    'lbfgs': ('curobo_ros.planners.lbfgs_planner', '_LBFGS_DEFAULTS', _LBFGS),
}

# Parameters that no longer exist: an override is ignored, so say why.
REMOVED = {
    'decay_factor': ('the TSDF decay is derived from decay_half_life_s (seconds) '
                     "and the cameras' frame_rate_hz: set decay_half_life_s"),
    'mpc_solver_type': ('the recipe follows the planner: set_planner mpc = MPPI, '
                        'lbfgs = L-BFGS'),
}


def declare(node, name: str, default=DESCRIPTOR):
    """Declare ``name`` from its spec if not yet declared; return its value.

    ``default`` overrides the spec's (required for DESCRIPTOR defaults).
    """
    if not node.has_parameter(name):
        spec = SPECS[name]
        value = spec.default if default is DESCRIPTOR else default
        if value is DESCRIPTOR:
            raise ValueError(f"parameter '{name}' needs a default from the call site")
        node.declare_parameter(name, value, spec.descriptor())
    return node.get_parameter(name).value


def declare_all(node) -> None:
    """Declare every static-default parameter and the nested trees."""
    import importlib

    from curobo_ros.planners.mpc_common import _flatten

    for spec in _SPECS:
        if spec.default is not DESCRIPTOR:
            declare(node, spec.name)
    for prefix, (module, attr, solvers) in TREES.items():
        tree = Param(prefix, None, BUILD, '', solvers)
        descriptor_text = tree.constraints()
        defaults = getattr(importlib.import_module(module), attr)
        for dotted, value in _flatten('', defaults).items():
            name = f'{prefix}.{dotted}'
            if not node.has_parameter(name):
                node.declare_parameter(name, value, ParameterDescriptor(
                    description=f'{prefix} cost/optimizer tree (see config/mpc/)',
                    additional_constraints=descriptor_text))


def spec_of(name: str):
    """Return the Param of ``name`` (nested-tree leaves included), or None."""
    if name in SPECS:
        return SPECS[name]
    prefix = name.split('.', 1)[0]
    if prefix in TREES and '.' in name:
        return Param(name, None, BUILD, '', TREES[prefix][2])
    return None


def install_change_hook(node) -> None:
    """Log what a runtime change reaches. Call after declare_all.

    Declarations also run on-set callbacks; they are told apart because the
    parameter is not declared yet. Never rejects: read_only does that.
    """
    log = node.get_logger()

    def on_set(params):
        for p in params:
            spec = spec_of(p.name)
            if spec is None or spec.kind != BUILD or not node.has_parameter(p.name):
                continue
            host = getattr(node, 'solver_host', None)
            built = [key for key in spec.solvers
                     if host is not None and host.get(key) is not None]
            if built:
                log.warn(f"'{p.name}' changed: {built} already built and keep the "
                         f'old value until rebuilt ({spec.constraints()})')
            else:
                log.info(f"'{p.name}' changed: {spec.constraints()}")
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(on_set)


def warn_unused_overrides(node) -> None:
    """Warn about launch overrides no declared parameter consumes.

    rclpy silently keeps overrides of undeclared names, so a typo or a removed
    parameter would otherwise be dropped while the operator believes it works.
    Call once every parameter is declared.
    """
    # rclpy has no public accessor: Node fills _parameter_overrides from the
    # command line, the params files and NodeOptions.
    for name in sorted(getattr(node, '_parameter_overrides', {})):
        # A schema name not declared yet is declared on use (e.g.
        # max_joint_accel_dps, by the joint_speed strategy): not unused.
        if node.has_parameter(name) or name in SPECS:
            continue
        if name in REMOVED:
            node.get_logger().warn(
                f"Parameter '{name}' no longer exists and is IGNORED: {REMOVED[name]}")
        else:
            node.get_logger().warn(
                f"Parameter '{name}' is set but not used by this node: IGNORED")
