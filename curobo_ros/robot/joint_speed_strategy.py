import math
import time
from collections import deque

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from curobo_ros.core.diagnostics import open_diag_csv

# Bounds on the interval the feedback plausibility gate is allowed to assume
# between two joint_states messages (see callback_joint_pose).
#   FLOOR: two callbacks landing in the same instant must not collapse the gate
#   to zero travel and reject a legitimate sample.
#   CEILING: after a real gap (startup, a stalled driver, a paused executor) we
#   genuinely do not know where the arm went, so the gate must OPEN, not close.
#   At 0.5 s the window is ~360 deg per joint, i.e. no gate at all — the safe
#   direction, since refusing feedback forever is worse than accepting it.
_FEEDBACK_DT_FLOOR_S = 0.005
_FEEDBACK_DT_CEIL_S = 0.5


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
                              Doosan's 70 deg/s^2 SpeedJ limit. Also exposed as
                              a live ROS param of the same name — `ros2 param
                              set <node> max_joint_accel_dps <value>` takes
                              effect on the next send_trajectrory() call, no
                              restart needed. Set it very high (e.g. 1e6) to
                              effectively disable the clamp for a test.)
      accel_clamp_dt          (s, interval the accel clamp assumes between
                              points — default self.dt/interpolation_dt; set
                              to pin a specific value, e.g. the previously
                              hardcoded 0.02, pending hardware validation)
      feedback_max_speed_dps  (deg/s, plausibility gate on INCOMING joint
                              feedback — see callback_joint_pose. Default 720,
                              ~4x the fastest M1013 axis)
      feedback_reject_limit   (consecutive rejected samples before the gate
                              gives up and accepts anyway — default 3)
      velocity_filter_window  (int, number of joint_states samples averaged
                              into get_joint_velocity_filtered() AND
                              get_joint_acceleration_filtered() — a plain
                              moving average over the raw ~100Hz feedback,
                              not the per-resolve EMA that used to live in
                              ReactiveController. Default 10.)
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
        # Live ROS param, not just a descriptor default: lets a test disable
        # the clamp (e.g. `ros2 param set <node> max_joint_accel_dps 1e6`)
        # without restarting the node or touching the robot descriptor. Value
        # is re-read every _clamp_velocities() call, so changes apply
        # immediately.
        default_max_accel_dps = float(self.params.get('max_joint_accel_dps', 45.0))
        if not node.has_parameter('max_joint_accel_dps'):
            node.declare_parameter('max_joint_accel_dps', default_max_accel_dps)
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

        # Plain moving average over the raw ~100Hz joint_states feedback,
        # computed here (at the source's real rate) rather than in
        # ReactiveController._close_state_loop, which only samples whatever
        # self.joint_velocity holds at its own ~4-8Hz resolve rate — most of
        # the 100Hz stream was never seen by that per-resolve EMA.
        velocity_filter_window = int(self.params.get('velocity_filter_window', 10))
        self._velocity_window = deque(maxlen=max(1, velocity_filter_window))
        self.joint_velocity_filtered = [0.0] * self.dof

        # Same idea, one derivative up: acceleration from a finite difference
        # of the FILTERED velocity above, taken over a ~velocity_filter_window
        # long baseline (oldest vs. newest sample currently in the history),
        # not a raw sample-to-sample diff at the ~10ms message rate. This is
        # what grounds ReactiveController's extrapolated acceleration against
        # reality instead of it being a pure mirror of the solver's own
        # pred_acc.
        #
        # Differentiating at dt~10ms amplifies velocity measurement noise by
        # ~1/dt (a modest +-0.01 rad/s of encoder noise becomes +-1 rad/s^2 =
        # +-57 dps^2 PER SAMPLE) -- a 10-sample moving average on that only
        # cuts the noise by sqrt(10)~3.16x, nowhere near enough. Stretching
        # the baseline to ~velocity_filter_window samples (~100ms at 100Hz,
        # instead of ~10ms) divides that amplification by the same factor
        # the window would have averaged over anyway, without adding much
        # extra lag since it's already differencing the smoothed signal.
        self._filt_vel_history = deque(maxlen=max(1, velocity_filter_window) + 1)
        self.joint_acceleration_filtered = [0.0] * self.dof

        # Plausibility gate on the feedback above (callback_joint_pose).
        self._feedback_max_speed_rad_s = math.radians(
            float(self.params.get('feedback_max_speed_dps', 720.0)))
        self._feedback_reject_limit = int(self.params.get('feedback_reject_limit', 3))
        self._last_good_pose = None
        self._last_good_time = 0.0
        self._reject_streak = 0
        self._reject_total = 0

        # Declared eagerly (was lazy, in _debug_csv_enabled()): send_trajectrory()
        # is only reached once a command is actually being sent, so `ros2 param
        # set .../joint_speed_debug_csv true` failed with "undeclared parameter"
        # before the first one — the same trap mpc_debug had. Declaring here, at
        # strategy construction (node startup), makes it settable up front.
        if not node.has_parameter('joint_speed_debug_csv'):
            node.declare_parameter('joint_speed_debug_csv', False)

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
        max_accel_dps = float(self.node.get_parameter('max_joint_accel_dps').value)
        max_dv = math.radians(max_accel_dps) * self._accel_clamp_dt
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
        # Delta actually published vs. measured real velocity -- what the clamp
        # is meant to bound to max_dv = max_accel_rad_s2 * self._accel_clamp_dt
        # PER POINT, so here the first point straight against the real value.
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

    def get_joint_velocity_filtered(self):
        with self.buffer_lock:
            return list(self.joint_velocity_filtered)

    def get_joint_acceleration_filtered(self):
        with self.buffer_lock:
            return list(self.joint_acceleration_filtered)

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
        pos = list(msg.position[:n])
        now = time.monotonic()
        with self.buffer_lock:
            if len(pos) != n or not self._feedback_is_plausible(pos, now):
                return
            self.joint_pose = pos
            self._last_good_pose = pos
            self._last_good_time = now
            if msg.velocity:
                self.joint_velocity = list(msg.velocity[:n])
                self._velocity_window.append(self.joint_velocity)
                self.joint_velocity_filtered = [
                    sum(samples) / len(self._velocity_window)
                    for samples in zip(*self._velocity_window)
                ]
                self._filt_vel_history.append((self.joint_velocity_filtered, now))
                if len(self._filt_vel_history) == self._filt_vel_history.maxlen:
                    baseline_vel, baseline_t = self._filt_vel_history[0]
                    dt = now - baseline_t
                    if dt > 1e-6:
                        self.joint_acceleration_filtered = [
                            (v - bv) / dt
                            for v, bv in zip(self.joint_velocity_filtered, baseline_vel)
                        ]
            if msg.name:
                self.joint_names = list(msg.name[:n])

    def _feedback_is_plausible(self, pos, now) -> bool:
        '''Reject a joint_states sample the arm cannot physically have reached.

        This is not defensive paranoia: measured 2026-08-07 on three
        consecutive hardware runs (mpc_diag_20260807_140507 / _140539 /
        _141605), /dsr01/joint_states carried 5 samples that stepped 125-200 deg
        in ~120 ms and were followed by a sample resuming the true trajectory
        exactly where the previous one left it. The arm never went there. All
        five share a signature: every component inside +-0.5 rad with at least
        one pinned at exactly 0.50003 rad. There is a single publisher on that
        topic (joint_state_broadcaster, /dsr01) with the six expected names, so
        it is the driver's own read path, not a competing publisher.

        Without this gate those samples reach the MPC as ground truth through
        ReactiveController._close_state_loop: the run above fed the optimizer a
        phantom 1.88 m / 114 deg pose error on one step.

        Rejecting means running one cycle on the previous position, which is
        strictly better than running on a wrong one. But NEVER freeze: after
        _feedback_reject_limit consecutive rejections the gate gives up and
        accepts, because a persistent mismatch means our own reference is the
        stale one (arm moved while we were not listening), and stale feedback
        held forever is the more dangerous failure.

        Caller must hold buffer_lock.
        '''
        prev = self._last_good_pose
        if prev is None or len(prev) != len(pos):
            self._reject_streak = 0
            return True

        dt = min(max(now - self._last_good_time, _FEEDBACK_DT_FLOOR_S), _FEEDBACK_DT_CEIL_S)
        bound = self._feedback_max_speed_rad_s * dt
        worst = max(abs(a - b) for a, b in zip(pos, prev))
        if worst <= bound:
            self._reject_streak = 0
            return True

        self._reject_streak += 1
        self._reject_total += 1
        detail = (f"jump {math.degrees(worst):.1f} deg in {dt * 1000:.0f} ms "
                  f"(gate {math.degrees(bound):.1f} deg), "
                  f"q={[round(math.degrees(v), 2) for v in pos]}, "
                  f"{self._reject_total} rejected so far")
        if self._reject_streak >= self._feedback_reject_limit:
            # Distinct call site from the WARN below on purpose: rclpy indexes
            # throttling on file+line, not on the message text.
            self.node.get_logger().error(
                f"joint feedback: {self._reject_streak} implausible samples in a "
                f"row - accepting anyway rather than freezing on stale state. {detail}")
            self._reject_streak = 0
            return True
        self.node.get_logger().warn(
            f"joint feedback: rejected implausible sample. {detail}",
            throttle_duration_sec=1.0)
        return False

    def get_progression(self):
        with self.buffer_lock:
            return self.trajectory_progression
