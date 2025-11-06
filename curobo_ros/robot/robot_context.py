from curobo_ros.robot.ghost_strategy import GhostStrategy
from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from std_srvs.srv import SetBool, Trigger
import threading
from functools import partial


class RobotContext:
    '''
    The context of the robot if you want to select and change robot strategy dynamically.
    Provides a service to switch between different robot control strategies.
    '''
    robot_strategy: JointCommandStrategy

    def __init__(self, node, dt):
        # Store node and dt for strategy switching
        self.node = node
        self.dt = dt

        # Thread safety for strategy switching
        self.strategy_lock = threading.Lock()

        # Initialize robot type parameter
        node.declare_parameter('robot_type', "doosan_m1013")

        # Track current strategy name
        self.current_strategy_name = node.get_parameter('robot_type').get_parameter_value().string_value

        # Select initial strategy
        self.robot_strategy = self.select_strategy(node, dt)
        self.ghost_strategy = GhostStrategy(node, dt)

        # Create service for dynamic strategy switching
        self.set_strategy_srv = node.create_service(
            Trigger,
            node.get_name() + '/set_robot_strategy',
            partial(self.set_robot_strategy_callback, node)
        )

        # Create service to get current strategy
        self.get_strategy_srv = node.create_service(
            Trigger,
            node.get_name() + '/get_robot_strategy',
            self.get_robot_strategy_callback
        )

        node.get_logger().info(f"Robot strategy initialized: {self.current_strategy_name}")

    def select_strategy(self, node, time_dilation_factor):
        '''
        This method select the robot strategy based on the ros2 param robot_type.
        And instanciate the robot strategy.
        '''
        robot_type = node.get_parameter('robot_type').get_parameter_value().string_value
        match robot_type:
            case "doosan_m1013":
                # The robot lib are loaded here to avoid import error if this robot is not use.
                from curobo_ros.robot.doosan_strategy import DoosanControl
                robot_strategy = DoosanControl(node, time_dilation_factor)
            case "ur5e":
                robot_strategy = None
                node.get_logger().warn("UR5e strategy not yet implemented")
            case "emulator":
                # Robot emulator for testing and visualization
                from curobo_ros.robot.emulator_strategy import EmulatorStrategy
                robot_strategy = EmulatorStrategy(node, time_dilation_factor)
            case _:
                robot_strategy = None
                node.get_logger().warn(f"Unknown robot type: {robot_type}")

        return robot_strategy

    def set_robot_strategy_callback(self, node, request, response):
        '''
        Service callback to dynamically change the robot control strategy.
        The strategy name is read from the ROS parameter 'robot_type'.

        Args:
            node: ROS node
            request: Trigger request (empty)
            response: Trigger response

        Returns:
            Response with success status and message
        '''
        try:
            # Get the requested strategy from parameter
            new_strategy_name = node.get_parameter('robot_type').get_parameter_value().string_value

            # Check if already using this strategy
            if new_strategy_name == self.current_strategy_name:
                response.success = True
                response.message = f"Already using strategy: {new_strategy_name}"
                return response

            node.get_logger().info(f"Switching strategy from '{self.current_strategy_name}' to '{new_strategy_name}'...")

            # Thread-safe strategy switching
            with self.strategy_lock:
                # Store previous strategy
                previous_strategy = self.current_strategy_name

                # Stop current robot if it exists
                if self.robot_strategy is not None:
                    try:
                        self.robot_strategy.stop_robot()
                        node.get_logger().info("Previous robot strategy stopped")
                    except Exception as e:
                        node.get_logger().warn(f"Could not stop previous strategy: {e}")

                # Create new strategy
                new_strategy = self.select_strategy(node, self.dt)

                if new_strategy is None and new_strategy_name not in ["ghost"]:
                    response.success = False
                    response.message = f"Strategy '{new_strategy_name}' is not implemented yet"
                    node.get_logger().error(response.message)
                    return response

                # Switch to new strategy
                self.robot_strategy = new_strategy
                self.current_strategy_name = new_strategy_name

                # Reinitialize ghost strategy (for visualization)
                self.ghost_strategy = GhostStrategy(node, self.dt)

                response.success = True
                response.message = f"Strategy switched from '{previous_strategy}' to '{new_strategy_name}'"
                node.get_logger().info(f"✅ {response.message}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to switch strategy: {str(e)}"
            node.get_logger().error(response.message)
            import traceback
            node.get_logger().error(traceback.format_exc())

        return response

    def get_robot_strategy_callback(self, request, response):
        '''
        Service callback to get the current robot control strategy.

        Args:
            request: Trigger request (empty)
            response: Trigger response

        Returns:
            Response with current strategy name
        '''
        response.success = True
        response.message = self.current_strategy_name
        return response

    def set_robot_strategy(self, robot_strategy, node, dt):
        '''
        DEPRECATED: Use set_robot_strategy_callback via ROS service instead.
        This method is kept for backward compatibility.
        '''
        self.robot_strategy = robot_strategy
        self.ghost_strategy = GhostStrategy(node, dt) 


    def set_command(self, joint_names,  vel_command, accel_command, position_command):
        '''
        Set the velocity, position and acceleration command of the robot.
        The command send to the ghost are directly send to show it on rviz.
        Thread-safe during strategy switching.
        '''
        with self.strategy_lock:
            # If there is a real robot
            if self.robot_strategy is not None:
                self.robot_strategy.set_command(joint_names, vel_command, accel_command, position_command)
            # Active all the time for rviz visualisation
            self.ghost_strategy.set_command(joint_names, vel_command, accel_command, position_command)
            self.ghost_strategy.send_trajectrory()

    def get_joint_pose(self):
        '''
        This method return the pose of the robot.
        Thread-safe during strategy switching.
        '''
        with self.strategy_lock:
            if self.robot_strategy is None:
                return [0.0] * 6  # Default 6-DOF robot
            return self.robot_strategy.get_joint_pose()

    def get_joint_name(self):
        '''
        This method return the joint names of the robot.
        Thread-safe during strategy switching.
        '''
        with self.strategy_lock:
            if self.robot_strategy is None:
                return ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            return self.robot_strategy.get_joint_name()

    def stop_robot(self):
        '''
        The methode send a twits command of 0 to stop the robot.
        The robot will stop at the current position. The list of command (position and velocity are empty)
        and the progression is set to 0.
        Thread-safe during strategy switching.
        '''
        with self.strategy_lock:
            if self.robot_strategy is not None:
                self.robot_strategy.stop_robot()

    def get_progression(self):
        '''
        The methode return the progression of the robot.
        The progression is a float between 0 and 1. 0 mean the robot is at the beginning of the trajectory
        and 1 mean the robot is at the end of the trajectory.
        Thread-safe during strategy switching.
        '''
        with self.strategy_lock:
            if self.robot_strategy is None:
                return 0.0
            return self.robot_strategy.get_progression()

    def send_trajectrory(self):
        '''
        This method set the send_to_robot variable.
        The send_to_robot variable activates the command list send to the robot.
        Thread-safe during strategy switching.
        '''
        with self.strategy_lock:
            if self.robot_strategy is None:
                return None
            return self.robot_strategy.send_trajectrory()

    def get_send_to_robot(self):
        '''
        Get the send_to_robot status.
        Thread-safe during strategy switching.
        '''
        with self.strategy_lock:
            if self.robot_strategy is None:
                return False
            return self.robot_strategy.get_send_to_robot()

