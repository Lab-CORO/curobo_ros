import threading
from functools import partial

from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from curobo_msgs.srv import SetRobotStrategy

from curobo_ros.core.config_wrapper import resolve_interpolation_dt
from curobo_ros.robot.ghost_strategy import GhostStrategy
from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from curobo_ros.robot.robot_registry import create_strategy, available_strategies
from curobo_ros.robot.robot_description import load_robot_description

# ---------------------------------------------------------------------------
# Lock order (strict, total), robot command path:
#
#   gpu_lock (node, RLock)  >  strategy_lock (RobotContext)  >  buffer_lock
#   (per-strategy, RLock, see JointCommandStrategy)
#
#   _pending_lock / _live_goal_lock / _goal_lock (reactive_controller.py /
#   unified_planner_node.py) are leaves: never held while acquiring another
#   lock.
#
# gpu_lock -> strategy_lock already exists today (unified_planner_node.py
# `with self.gpu_lock: planner.plan(...)` -> single_planner.py
# `robot_context.set_command(...)`); the reverse never occurs anywhere under
# curobo_ros/robot/. This module only adds a rank BELOW strategy_lock.
#
# No-deadlock argument:
#  1. Leaves are never held while acquiring another lock (verified: the
#     pending-action locks only guard plain assignments).
#  2. buffer_lock is terminal: strategy modules under curobo_ros/robot/ never
#     reference strategy_lock or gpu_lock, and every outgoing call made while
#     holding buffer_lock (e.g. a topic publish) is done via a snapshot taken
#     under the lock, then used after it is released.
#  3. strategy_lock never acquires gpu_lock.
#  4. gpu_lock -> strategy_lock is strictly descending, hence consistent.
#  5. The emulator's playback thread only ever acquires buffer_lock (rank 1),
#     holding nothing else.
#  6. Every lock-then-join sequence (e.g. replacing the playback thread) joins
#     OUTSIDE the lock, so the thread being joined can always still acquire
#     buffer_lock to exit.
# All acquisitions are strictly descending -> the wait-for graph is acyclic.
# ---------------------------------------------------------------------------


class RobotContext:
    '''
    Selects and switches the joint-command CONTROL STRATEGY at runtime.

    Two independent axes:
      - ``robot``            (param): WHICH robot -> descriptor (model + driver topics
                                      + default strategy + joint names).
      - ``control_strategy`` (param): HOW to command joints -> emulator / joint_speed /
                                      joint_pose / ... (robot-agnostic, from the registry).

    Default control strategy comes from the descriptor; it can be overridden at launch
    and switched at runtime via the ``set_robot_strategy`` service (now a string key).
    '''
    robot_strategy: JointCommandStrategy

    def __init__(self, node, dt=0.025):
        self.node = node
        # curobo_ros is the authority on dt (see resolve_interpolation_dt):
        # this reads the interpolation_dt ROS param, falling back to `dt` only
        # if the node hasn't declared it. `dt` itself defaults to the same
        # value as the param's own default so a bare RobotContext(node) is
        # correct even for callers that never touch interpolation_dt.
        self.dt = resolve_interpolation_dt(node, dt)
        self.strategy_lock = threading.Lock()

        # Which robot (model + driver topics + default strategy).
        if not node.has_parameter('robot'):
            node.declare_parameter('robot', 'doosan_m1013')
        robot = node.get_parameter('robot').get_parameter_value().string_value or 'doosan_m1013'
        self.description = load_robot_description(robot)

        # Which control strategy (default = descriptor's, overridable).
        if not node.has_parameter('control_strategy'):
            node.declare_parameter('control_strategy', self.description.strategy_key)
        self.current_strategy_name = node.get_parameter('control_strategy').get_parameter_value().string_value

        self.robot_strategy = self.select_strategy(node, self.dt)
        self.ghost_strategy = GhostStrategy(node, self.dt, self.description)

        self.set_strategy_srv = node.create_service(
            SetRobotStrategy,
            node.get_name() + '/set_robot_strategy',
            partial(self.set_robot_strategy_callback, node)
        )
        self.get_strategy_srv = node.create_service(
            Trigger,
            node.get_name() + '/get_robot_strategy',
            self.get_robot_strategy_callback
        )

        node.get_logger().info(
            f"Control strategy initialized: {self.current_strategy_name} "
            f"(robot: {self.description.name})")

    def bind_kinematics(self, kin):
        '''Adopt canonical joint names/DOF from the built kinematics into the descriptor.'''
        self.description.bind_kinematics(kin)

    def select_strategy(self, node, dt):
        '''Instantiate the control strategy named by the ``control_strategy`` param.

        ``dt`` here is the trajectory sampling step (interpolation_dt) — it has
        nothing to do with the ``time_dilation_factor`` ROS param, which only
        drives open-loop feedback publish cadence. The parameter used to be
        named after that unrelated param, which was misleading at every call
        site.
        '''
        key = node.get_parameter('control_strategy').get_parameter_value().string_value
        strategy = create_strategy(key, node, dt, self.description)
        if strategy is None:
            node.get_logger().warn(
                f"Unknown control strategy: '{key}'. Available: {available_strategies()}")
        return strategy

    def set_robot_strategy_callback(self, node, request, response):
        '''Switch the control strategy at runtime. request.robot_strategy is a string key.'''
        try:
            new_strategy_name = request.robot_strategy
            if new_strategy_name not in available_strategies():
                response.success = False
                response.message = (
                    f"Unknown strategy: '{new_strategy_name}'. "
                    f"Available: {available_strategies()}")
                node.get_logger().error(response.message)
                return response

            response.previous_robot_strategy = self.current_strategy_name

            if new_strategy_name == self.current_strategy_name:
                response.success = True
                response.current_robot_strategy = self.current_strategy_name
                response.message = f"Already using strategy: {new_strategy_name}"
                return response

            node.get_logger().info(
                f"Switching control strategy '{self.current_strategy_name}' -> '{new_strategy_name}'...")

            with self.strategy_lock:
                if self.robot_strategy is not None:
                    try:
                        self.robot_strategy.stop_robot()
                    except Exception as e:
                        node.get_logger().warn(f"Could not stop previous strategy: {e}")

                node.set_parameters([
                    Parameter('control_strategy', Parameter.Type.STRING, new_strategy_name)])
                # Re-read interpolation_dt so a `ros2 param set` done before this
                # switch takes effect on the new strategy (self.dt would
                # otherwise stay pinned to whatever was resolved at construction).
                self.dt = resolve_interpolation_dt(node, self.dt)
                new_strategy = self.select_strategy(node, self.dt)

                if new_strategy is None:
                    response.success = False
                    response.message = f"Strategy '{new_strategy_name}' is not registered"
                    node.get_logger().error(response.message)
                    return response

                self.robot_strategy = new_strategy
                self.current_strategy_name = new_strategy_name
                self.ghost_strategy = GhostStrategy(node, self.dt, self.description)

                response.success = True
                response.current_robot_strategy = new_strategy_name
                response.message = (
                    f"Strategy switched from '{response.previous_robot_strategy}' "
                    f"to '{new_strategy_name}'")
                node.get_logger().info(f"{response.message}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to switch strategy: {str(e)}"
            node.get_logger().error(response.message)
            import traceback
            node.get_logger().error(traceback.format_exc())

        return response

    def get_robot_strategy_callback(self, request, response):
        '''Return the current control strategy name (Trigger).'''
        response.success = True
        response.message = self.current_strategy_name
        return response

    def get_robot_strategies_callback(self, node, request, response):
        '''Return the list of available control strategies (string names).'''
        response.strategy_names = available_strategies()
        response.current_strategy_name = self.current_strategy_name
        response.success = True
        node.get_logger().info(
            f"GetRobotStrategies: {response.strategy_names}, current={self.current_strategy_name}")
        return response

    def set_robot_strategy(self, robot_strategy, node, dt):
        '''DEPRECATED: use the set_robot_strategy service instead.'''
        self.robot_strategy = robot_strategy
        self.ghost_strategy = GhostStrategy(node, dt, self.description)

    def set_command(self, joint_names, vel_command, accel_command, position_command):
        '''Forward a command to the active strategy and the RViz ghost. Thread-safe.

        joint_names=None => resolved INSIDE this critical section instead of
        by the caller — reading robot_context.robot_strategy.get_joint_name()
        before calling this races a concurrent strategy switch (robot_strategy
        can be reassigned, or become a different DOF, between that read and
        this call).

        Returns the new buffer epoch (see JointCommandStrategy.buffer_epoch),
        or None if there is no active strategy. Pair with send_trajectrory(
        expect_epoch=...) when set and send happen in separate calls/threads —
        see set_and_send_command() for the atomic alternative.
        '''
        with self.strategy_lock:
            if joint_names is None:
                joint_names = (self.robot_strategy.get_joint_name()
                                if self.robot_strategy is not None
                                else list(self.description.joint_names))
            epoch = None
            if self.robot_strategy is not None:
                epoch = self.robot_strategy.set_command(
                    joint_names, vel_command, accel_command, position_command)
            self.ghost_strategy.set_command(joint_names, vel_command, accel_command, position_command)
            self.ghost_strategy.send_trajectrory()
            return epoch

    def set_and_send_command(self, joint_names, vel_command, accel_command, position_command) -> bool:
        '''Load the command buffers AND send them, in one transaction.

        RACE FIX: set_command() and send_trajectrory() used to take
        strategy_lock SEPARATELY, so a second producer could overwrite the
        buffers between the two calls and the robot would execute a
        trajectory that was never meant for it.

        joint_names=None => resolved INSIDE this critical section (the caller
        must NOT call get_joint_name() beforehand — that read would itself
        race against a concurrent set_and_send_command/strategy switch).

        Argument order matches set_command() to avoid a call-site trap.
        Lock order: strategy_lock -> buffer_lock (see module docstring).

        Returns True if a strategy was active and the command was sent, False
        if there was no active strategy. NOTE: no JointCommandStrategy's
        send_trajectrory() currently returns a meaningful value on its own
        (fire-and-forget), so this cannot report a downstream failure — only
        "was there a strategy to send to".
        '''
        with self.strategy_lock:
            if joint_names is None:
                joint_names = (self.robot_strategy.get_joint_name()
                                if self.robot_strategy is not None
                                else list(self.description.joint_names))
            self.ghost_strategy.set_command(joint_names, vel_command, accel_command, position_command)
            self.ghost_strategy.send_trajectrory()
            if self.robot_strategy is None:
                return False
            # set_command()/send_trajectrory() each take-and-release buffer_lock
            # internally — without also holding it HERE across both calls, the
            # driver's callback_joint_pose (which only takes buffer_lock) could
            # overwrite joint_names in the gap between them, and
            # send_trajectrory() would then publish those names against THESE
            # positions/velocities. buffer_lock is an RLock precisely so this
            # nesting (outer hold + set_command()'s own internal acquire) works.
            with self.robot_strategy.buffer_lock:
                self.robot_strategy.set_command(
                    joint_names, vel_command, accel_command, position_command)
                self.robot_strategy.send_trajectrory()
            return True

    def get_joint_pose(self):
        with self.strategy_lock:
            if self.robot_strategy is None:
                return [0.0] * self.description.dof
            return self.robot_strategy.get_joint_pose()

    def get_joint_velocity(self):
        """Real, measured joint velocity (driver feedback), if the active
        strategy provides one — else zeros. See JointCommandStrategy.
        """
        with self.strategy_lock:
            if self.robot_strategy is None:
                return [0.0] * self.description.dof
            return self.robot_strategy.get_joint_velocity()

    def get_joint_name(self):
        with self.strategy_lock:
            if self.robot_strategy is None:
                return self.description.joint_names
            return self.robot_strategy.get_joint_name()

    def stop_robot(self):
        with self.strategy_lock:
            if self.robot_strategy is not None:
                self.robot_strategy.stop_robot()

    def get_progression(self):
        with self.strategy_lock:
            if self.robot_strategy is None:
                return 0.0
            return self.robot_strategy.get_progression()

    def send_trajectrory(self, expect_epoch=None) -> bool:
        '''Send the currently loaded command buffers.

        expect_epoch: for producers that call set_command() and
        send_trajectrory() as two SEPARATE calls (e.g. plan() then execute(),
        in different callbacks) — pass the epoch returned by that
        set_command() call. If another set_command() has since superseded the
        buffers (epoch mismatch), this refuses to execute a mismatched
        trajectory and returns False instead of silently sending whatever is
        now in the buffers. Omit (None) for fire-and-forget senders that don't
        need this check (e.g. reactive control's own tight set+send loop).

        Returns True if the command was actually sent, False if refused (no
        active strategy, or an epoch mismatch). NOTE: this is NOT a downstream
        success/failure report — no JointCommandStrategy's send_trajectrory()
        currently returns a meaningful value of its own (fire-and-forget); True
        only means "this call was not refused by the guard above".
        '''
        with self.strategy_lock:
            if self.robot_strategy is None:
                return False
            if expect_epoch is not None and self.robot_strategy.buffer_epoch != expect_epoch:
                self.node.get_logger().error(
                    f"send_trajectrory: buffer epoch mismatch (expected "
                    f"{expect_epoch}, current {self.robot_strategy.buffer_epoch}) "
                    f"- a newer set_command() superseded this one; refusing to "
                    f"execute a mismatched trajectory."
                )
                return False
            self.robot_strategy.send_trajectrory()
            return True

    def get_robot_state(self):
        with self.strategy_lock:
            if self.robot_strategy is None:
                return RobotState.IDLE
            return self.robot_strategy.robot_state
