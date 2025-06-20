from curobo_ros.robot.ghost_strategy import GhostStrategy
from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from std_srvs.srv import SetBool


class RobotContext:
    '''
    The context of the robot if you want to select and change robot
    '''
    robot_strategy: JointCommandStrategy

    def __init__(self, node, dt):
        node.declare_parameter('robot_type', "doosan_m1013")
        self.robot_strategy = self.select_strategy(node, dt)
        self.ghost_strategy = GhostStrategy(node, dt)

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
                print("ur5e")
            case "emulator":
                robot_strategy = None
            case _:
                robot_strategy = None

        return robot_strategy

    def set_robot_strategy(self, robot_strategy, node, dt):
        self.robot_strategy = robot_strategy
        self.ghost_strategy = GhostStrategy(node, dt) 


    def set_command(self, joint_names,  vel_command, accel_command, position_command):
        '''
        Set the velocity, position and acceleration command of the robot.
        The command send to the ghost are directly send to show it on rviz.
        '''
        # If there is a real robot 
        self.robot_strategy.set_command(joint_names, vel_command, accel_command, position_command)
        # Active all the time for rviz visualisation
        self.ghost_strategy.set_command(joint_names, vel_command, accel_command, position_command)
        self.ghost_strategy.send_command()

    def get_joint_pose(self, node):
        '''
        This method return the pose of the robot. 
        '''
        return self.robot_strategy.get_joint_pose(node)

    def stop_robot(self):
        '''
        The methode send a twits command of 0 to stop the robot.
        The robot will stop at the current position. The list of command (position and velocity are empty)
        and the progression is set to 0.
        '''
        self.robot_strategy.stop_robot()

    def get_progression(self):
        '''
        The methode return the progression of the robot.
        The progression is a float between 0 and 1. 0 mean the robot is at the beginning of the trajectory 
        and 1 mean the robot is at the end of the trajectory.
        '''
        return self.robot_strategy.get_progression()

    def send_trajectrory(self,  data):
        '''
        This method set the send_to_robot variable. 
        The send_to_robot variable activates the command list send to the robot.
        '''
        return self.robot_strategy.send_trajectrory(data)

    def get_send_to_robot(self):
        return self.robot_strategy.get_send_to_robot()

