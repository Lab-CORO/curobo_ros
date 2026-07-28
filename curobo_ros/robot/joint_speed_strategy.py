import math
import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from curobo_ros.core.diagnostics import open_diag_csv


class JointSpeedStrategy(JointCommandStrategy):
    '''Joint-SPEED control: stream a full JointTrajectory (positions + velocities +
    accelerations) to the robot's command topic; an external driver/bridge turns it
    into speed commands. Robot-agnostic — topics come from the RobotDescription.

    Descriptor strategy_params:
      command_topic        (publish JointTrajectory)
      state_topic           (subscribe Float32 progression, optional)
      joint_states_topic    (subscribe JointState feedback, optional)
      max_joint_accel_dps    (deg/s^2, hard velocity-rate limit on outgoing
                              commands — default 45, safety margin under the
                              Doosan's 70 deg/s^2 SpeedJ limit)
      accel_clamp_dt          (s, interval the accel clamp assumes between
                              points — default self.dt/interpolation_dt; set
                              to pin a specific value, e.g. the previously
                              hardcoded 0.02, pending hardware validation)
    '''

    def __init__(self, node, dt, description=None):
        super().__init__(node, dt, description)
        # self.dt (base class) is already the resolved interpolation_dt —
        # curobo_ros is the single authority on trajectory pacing (see
        # resolve_interpolation_dt). It is what gets stamped into
        # time_from_start below.

        command_topic = self.params.get('command_topic', '/execute_trajectory')
        self.pub_trajectory = node.create_publisher(JointTrajectory, command_topic, 10)

        # Hard acceleration clamp on OUTGOING velocity commands, independent of
        # whatever the planner (e.g. MPC re-optimizing every ~90ms) computed.
        # A slow/stale re-optimization can otherwise command a velocity jump far
        # beyond the robot driver's configured joint accel limit (observed on
        # the m1013: commanded deltas implying up to ~500 deg/s^2 against a
        # 70 deg/s^2 SpeedJ limit), tripping controller alarms and producing
        # jerky/oscillating motion.
        max_accel_dps = float(self.params.get('max_joint_accel_dps', 45.0))
        self._max_accel_rad_s2 = math.radians(max_accel_dps)
        # Interval the clamp assumes between consecutive points. Defaults to
        # self.dt (interpolation_dt) — the physically correct choice now that
        # curobo_ros is the single dt authority (see resolve_interpolation_dt):
        # leeloo's real point-to-point interval IS its command period, which is
        # meant to track interpolation_dt (see leeloo's execute_trajectory
        # command_period_s_ param). Override via the descriptor's
        # strategy_params (`accel_clamp_dt`) to pin a specific value instead of
        # following self.dt — e.g. to keep the previously-hardcoded 0.02
        # pending a supervised hardware validation of the new value (this
        # is the one change in the M7 dt cleanup that alters real robot
        # motion: widening this interval makes the clamp more permissive).
        self._accel_clamp_dt = float(self.params.get('accel_clamp_dt', self.dt))

        state_topic = self.params.get('state_topic')
        if state_topic:
            self.sub_trajectory_state = node.create_subscription(
                Float32, state_topic, self.callback_trajectory_state, 10,
                callback_group=MutuallyExclusiveCallbackGroup())

        joint_states_topic = self.params.get('joint_states_topic')
        if joint_states_topic:
            self.sub_joint_state = node.create_subscription(
                JointState, joint_states_topic, self.callback_joint_pose, 10,
                callback_group=MutuallyExclusiveCallbackGroup())

        # Current feedback (sized to the robot's DOF, DOF-agnostic). Both are
        # REAL driver measurements — dsr_hw_interface2.cpp's read() populates
        # joint_position_ and joint_velocities_ from the SAME real-time
        # struct (data->actual_joint_position / actual_joint_velocity), same
        # source, same rate. (Earlier assumption that velocity feedback
        # wasn't reliable was wrong — verified against the driver source.)
        self.joint_pose = [0.0] * self.dof
        self.joint_velocity = [0.0] * self.dof

    def _clamp_velocities(self, vel_list):
        """Hard-limit joint acceleration relative to the REAL, measured
        current velocity (self.joint_velocity), regardless of what the
        planner produced.

        Earlier versions ramped from an assumed "last commanded" value (first
        the theoretical last point of the previous batch, then a flat zero)
        because I believed the Doosan driver had no reliable velocity
        feedback. That was WRONG: dsr_hw_interface2.cpp's read() populates
        joint_velocities_ from data->actual_joint_velocity — the SAME
        real-time struct and update rate as position. The Doosan's own
        SpeedJ safety check almost certainly compares commanded vs its OWN
        measured actual velocity, not vs whatever we last sent (a batch
        rarely finishes playing before the next MPC re-optimization replaces
        the queue — see execute_trajectory.cpp: ``this->trajectory = *msg;``
        overwrites, not appends). Clamping against a "last commanded" guess
        was therefore addressing the wrong quantity, which is why alarms
        persisted even after that fix. Clamping against the REAL measured
        velocity directly targets what the safety check evaluates.
        """
        max_dv = self._max_accel_rad_s2 * self._accel_clamp_dt
        clamped = []
        prev = list(self.joint_velocity)
        for v in vel_list:
            row = [pi + max(-max_dv, min(max_dv, vi - pi)) for vi, pi in zip(v, prev)]
            clamped.append(row)
            prev = row
        return clamped

    def _debug_csv_enabled(self) -> bool:
        # Off by default: per-publish diagnostic CSV, not meant to run in
        # production (every send_trajectrory() call would otherwise write a
        # row and keep a file open for the strategy's whole lifetime).
        if not self.node.has_parameter('joint_speed_debug_csv'):
            self.node.declare_parameter('joint_speed_debug_csv', False)
        return bool(self.node.get_parameter('joint_speed_debug_csv').value)

    def _debug_csv_init(self):
        self._debug_csv = open_diag_csv(self.node, "speedj_publish")
        if self._debug_csv is None:
            return
        self._debug_csv.write_header_once([
            "t_s", "n_points", "real_vel_max_dps", "point0_max_dps",
            "point0_vs_real_accel_dps2", "unclamped_point0_max_dps",
            "clamp_active_j", "intra_batch_accel_max_dps2",
        ])
        self._debug_csv_t0 = time.monotonic()

    def _debug_csv_close(self):
        if getattr(self, "_debug_csv", None) is not None:
            self._debug_csv.close()
        self._debug_csv = None

    def _debug_csv_write(self, vel_clamped):
        if not self._debug_csv_enabled():
            return
        if getattr(self, "_debug_csv", None) is None:
            self._debug_csv_init()
        if self._debug_csv is None:
            return
        deg = math.degrees
        real = self.joint_velocity
        p0 = vel_clamped[0] if vel_clamped else [0.0] * self.dof
        raw0 = self.vel_command[0] if self.vel_command else [0.0] * self.dof
        # delta réellement publié vs vitesse réelle mesurée (ce que le clamp
        # est censé borner à max_dv = max_accel_rad_s2*self._accel_clamp_dt
        # par POINT, donc ici sur le 1er point directement contre le réel).
        accel_p0 = deg(max(abs(a - b) for a, b in zip(p0, real)) / self._accel_clamp_dt)
        clamp_active = [i for i, (c, u) in enumerate(zip(p0, raw0)) if abs(c - u) > 1e-9]
        intra = 0.0
        for i in range(1, len(vel_clamped)):
            d = max(abs(a - b) for a, b in zip(vel_clamped[i], vel_clamped[i - 1]))
            intra = max(intra, deg(d) / self._accel_clamp_dt)
        self._debug_csv.writerow([
            f"{time.monotonic() - self._debug_csv_t0:.3f}", len(vel_clamped),
            f"{deg(max(abs(v) for v in real)):.2f}", f"{deg(max(abs(v) for v in p0)):.2f}",
            f"{accel_p0:.1f}", f"{deg(max(abs(v) for v in raw0)):.2f}",
            str(clamp_active), f"{intra:.1f}",
        ])

    def send_trajectrory(self):
        with self.buffer_lock:
            self.robot_state = RobotState.RUNNING
            msg = JointTrajectory()
            msg.joint_names = self.joint_names

            if len(self.position_command) == 0:
                self.trajectory_progression = 1.0

            vel_clamped = self._clamp_velocities(self.vel_command)
            if self.vel_command:
                self._debug_csv_write(vel_clamped)

            for i in range(len(self.position_command)):
                point = JointTrajectoryPoint()
                point.positions = self.position_command[i]
                point.velocities = vel_clamped[i]
                point.accelerations = self.accel_command[i]
                point.effort = []
                point.time_from_start = Duration(
                    sec=int(self.dt * i), nanosec=int((self.dt * i % 1) * 1e9))
                msg.points.append(point)

            self.position_command = []
            self.vel_command = []
            self.accel_command = []
            self.trajectory_progression = 0.0

        self.pub_trajectory.publish(msg)

    def get_joint_pose(self):
        with self.buffer_lock:
            return list(self.joint_pose)

    def get_joint_velocity(self):
        with self.buffer_lock:
            return list(self.joint_velocity)

    def stop_robot(self):
        with self.buffer_lock:
            self.vel_command = []
            self.position_command = []
            self.accel_command = []
            self.command_index = 0
            self.trajectory_progression = 0.0
            self.robot_state = RobotState.STOPPED
        self.pub_trajectory.publish(JointTrajectory())
        self._debug_csv_close()

    def callback_trajectory_state(self, msg):
        with self.buffer_lock:
            self.trajectory_progression = msg.data

    def callback_joint_pose(self, msg):
        # Positional read in the driver's publication order (DOF-agnostic). The
        # driver's joint names (e.g. on /dsr01/joint_states) need NOT match the
        # curobo cspace names, so a by-name remap would wrongly drop every joint
        # and yield an empty q. curobo only needs the right count/order.
        n = self.dof or len(msg.position)
        with self.buffer_lock:
            self.joint_pose = list(msg.position[:n])
            if msg.velocity:
                self.joint_velocity = list(msg.velocity[:n])
            if msg.name:
                self.joint_names = list(msg.name[:n])

    def get_progression(self):
        with self.buffer_lock:
            return self.trajectory_progression
