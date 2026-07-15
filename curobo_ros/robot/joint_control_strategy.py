from abc import ABC, abstractmethod
import threading


class RobotState():
    '''
    This class is a enum to represent the robot state
    Currently useless but maybe later...
    '''
    RUNNING = 0
    STOPPED = 1
    READY = 2
    ERROR = 3
    IDLE = 4


class JointCommandStrategy(ABC):
    '''
    Base class (Strategy pattern) for a joint-command CONTROL MODE.

    A strategy describes HOW joints are commanded (emulator / joint speed /
    joint pose / ...), independently of WHICH robot is loaded. The robot-specific
    wiring (command/state/joint_states topics, joint names) comes from the
    RobotDescription passed in, so the same strategy works for any robot.
    '''

    def __init__(self, node, dt, description=None):
        self.node = node
        self.dt = dt
        self.description = description

        # Robot driver topics (empty dict-safe): the descriptor is the source.
        self.params = dict(description.strategy_params) if description is not None else {}

        # Joint names: descriptor fallback (cspace) until real joint_states arrive.
        self.joint_names = list(description.joint_names) if description is not None else []
        self.dof = len(self.joint_names)

        # Command buffers (filled by set_command()).
        self.position_command = []
        self.vel_command = []
        self.accel_command = []
        self.command_index = 0

        self.trajectory_progression = 0.0
        self.robot_state = RobotState.IDLE

        self.send_to_robot = False
        self.buffer_lock = threading.Lock()

    def set_command(self, joint_names, vel_command, accel_command, position_command):
        self.position_command = position_command
        self.vel_command = vel_command
        self.accel_command = accel_command
        self.joint_names = joint_names

    def get_joint_name(self):
        return self.joint_names

    def get_send_to_robot(self):
        with self.buffer_lock:
            return self.send_to_robot

    def get_joint_velocity(self):
        """Real, measured joint velocity, if this strategy's driver provides
        one (e.g. JointSpeedStrategy reads it from /dsr01/joint_states'
        actual_joint_velocity). Default: zeros — not every strategy has real
        velocity feedback (e.g. EmulatorStrategy has no physical driver).
        """
        return [0.0] * self.dof

    @abstractmethod
    def get_joint_pose(self):
        ...

    @abstractmethod
    def stop_robot(self):
        ...

    @abstractmethod
    def get_progression(self):
        ...

    @abstractmethod
    def send_trajectrory(self):
        ...
