"""Registry of joint-command CONTROL STRATEGIES (control modes).

Strategies describe HOW joints are commanded, not which robot: ``emulator`` (no
hardware), ``joint_speed`` (stream a JointTrajectory with velocities, external
bridge turns it into speed commands), ``joint_pose`` (command target positions).
The robot (model + driver topics) is chosen separately by the RobotDescriptor.

Single source of truth, lazy-imported (a strategy's ROS deps load only if used).
Add a control mode = one line here (or ``register_strategy`` at runtime); adding a
ROBOT never touches this file.
"""

# key -> (module path, class name); imported lazily on first use.
_STRATEGY_REGISTRY = {
    'emulator':    ('curobo_ros.robot.emulator_strategy',    'EmulatorStrategy'),
    'joint_speed': ('curobo_ros.robot.joint_speed_strategy', 'JointSpeedStrategy'),
    'joint_pose':  ('curobo_ros.robot.joint_pose_strategy',  'JointPoseStrategy'),
}


def register_strategy(key: str, module_path: str, class_name: str) -> None:
    """Register (or override) a control strategy at runtime."""
    _STRATEGY_REGISTRY[key] = (module_path, class_name)


def available_strategies():
    """Return the list of registered control-strategy keys."""
    return sorted(_STRATEGY_REGISTRY.keys())


def create_strategy(key, node, dt, description):
    """Instantiate the control strategy ``key`` (lazy import). Returns None if unknown."""
    entry = _STRATEGY_REGISTRY.get(key)
    if entry is None:
        return None
    module_path, class_name = entry
    import importlib
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)
    return cls(node, dt, description)
