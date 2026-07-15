import threading
from functools import partial

from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from curobo_msgs.srv import SetRobotStrategy

from curobo_ros.robot.ghost_strategy import GhostStrategy
from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from curobo_ros.robot.robot_registry import create_strategy, available_strategies
from curobo_ros.robot.robot_description import load_robot_description


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

    def __init__(self, node, dt):
        self.node = node
        self.dt = dt
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

        self.robot_strategy = self.select_strategy(node, dt)
        self.ghost_strategy = GhostStrategy(node, dt, self.description)

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

    def select_strategy(self, node, time_dilation_factor):
        '''Instantiate the control strategy named by the ``control_strategy`` param.'''
        key = node.get_parameter('control_strategy').get_parameter_value().string_value
        strategy = create_strategy(key, node, time_dilation_factor, self.description)
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
                node.get_logger().info(f"✅ {response.message}")

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
        '''Forward a command to the active strategy and the RViz ghost. Thread-safe.'''
        with self.strategy_lock:
            if self.robot_strategy is not None:
                self.robot_strategy.set_command(joint_names, vel_command, accel_command, position_command)
            self.ghost_strategy.set_command(joint_names, vel_command, accel_command, position_command)
            self.ghost_strategy.send_trajectrory()

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

    def send_trajectrory(self):
        with self.strategy_lock:
            if self.robot_strategy is None:
                return None
            return self.robot_strategy.send_trajectrory()

    def get_send_to_robot(self):
        with self.strategy_lock:
            if self.robot_strategy is None:
                return False
            return self.robot_strategy.get_send_to_robot()

    def get_robot_state(self):
        with self.strategy_lock:
            if self.robot_strategy is None:
                return RobotState.IDLE
            return self.robot_strategy.robot_state
