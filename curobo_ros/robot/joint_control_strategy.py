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

        # Guards position_command/vel_command/accel_command/command_index/
        # joint_names/trajectory_progression/robot_state, plus any feedback
        # fields a subclass adds (joint_pose/joint_velocity/
        # current_joint_positions) — same callback that rewrites joint_names
        # usually rewrites those together. RLock: the atomic
        # RobotContext.set_and_send_command() holds it across a nested
        # set_command()+send_trajectrory() pair. Rank: below strategy_lock,
        # above nothing (leaf) — see robot_context.py's lock-order docstring.
        self.buffer_lock = threading.RLock()
        # Bumped by every set_command()/stop_robot() call. Lets a producer
        # detect that its buffers were superseded/cleared before it gets to
        # send them (RobotContext.set_and_send_command's expect_epoch), and
        # lets the emulator's playback thread detect preemption mid-playback.
        self._buffer_epoch = 0

    @property
    def buffer_epoch(self) -> int:
        with self.buffer_lock:
            return self._buffer_epoch

    def set_command(self, joint_names, vel_command, accel_command, position_command) -> int:
        """Load new command buffers. Returns the new buffer epoch.

        Stores defensive copies — the caller's lists must not be aliased into
        the strategy (a caller mutating its own list after this call must not
        also mutate what the strategy/robot will execute).
        """
        with self.buffer_lock:
            self.position_command = [list(p) for p in position_command]
            self.vel_command = [list(v) for v in vel_command]
            self.accel_command = [list(a) for a in accel_command]
            self.joint_names = list(joint_names)
            self._buffer_epoch += 1
            return self._buffer_epoch

    def get_joint_name(self):
        with self.buffer_lock:
            return list(self.joint_names)

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
