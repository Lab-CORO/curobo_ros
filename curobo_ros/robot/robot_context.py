from curobo_ros.robot.ghost_strategy import GhostStrategy
from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from std_srvs.srv import SetBool


class RobotContext:
    '''
    The context of the robot if you want to change robot
    '''
    # def __init__(self, robot_strategy: JointSpeedCommandStrategy = None)  :
    robot_strategy: JointCommandStrategy

    def __init__(self, node, dt):
        node.declare_parameter('robot_type', "doosan_m1013")
        self.robot_strategy = self.select_strategy(node, dt)
        self.ghost_strategy = GhostStrategy(node, dt)

    def select_strategy(self, node, time_dilation_factor):

        robot_type = node.get_parameter('robot_type').get_parameter_value().string_value
        match robot_type:
            case "doosan_m1013":
                from curobo_ros.robot.doosan_strategy import DoosanControl
                robot_strategy = DoosanControl(node, time_dilation_factor)

            case "ur5e":
                print("ur5e")
            case "emulator":
                robot_strategy = None
            case _:
                robot_strategy = None

        return robot_strategy

    def set_robot_strategy(self, robot_strategy, node, dt):
        self.robot_strategy = robot_strategy
        self.ghost_strategy = GhostStrategy(node, dt) 

    # def send_speed_command(self, vel_command, accel_command, dt):
    #     self.robot_strategy.send_command(node,  vel_command, accel_command, dt)

    def set_command(self, joint_names,  vel_command, accel_command, position_command):
        # If there is a real robot 
        self.robot_strategy.set_command(joint_names, vel_command, accel_command, position_command)
        # Active all the time for rviz visualisation
        self.ghost_strategy.set_command(joint_names, vel_command, accel_command, position_command)
        self.ghost_strategy.send_command()

    def get_joint_pose(self, node):
        return self.robot_strategy.get_joint_pose(node)

    def stop_robot(self):
        self.robot_strategy.stop_robot()

    def get_progression(self):
        return self.robot_strategy.get_progression()

    def set_send_to_robot(self,  data):
        return self.robot_strategy.set_send_to_robot(data)

    def get_send_to_robot(self,):
        return self.robot_strategy.get_send_to_robot()