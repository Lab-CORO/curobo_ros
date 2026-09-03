#!/usr/bin/env python3
"""Forensics for the Doosan `[SpeedJ] Acceleration is over max value` alarms.

WHY THIS EXISTS
---------------
The M1013 raises level-1 / index-1216 alarms during LBFGS MPC execution even
though the planner is well inside its own limits. Two facts frame the problem:

  * The 70 deg/s^2 ceiling is NOT a physical limit of the arm. It is a
    hardcoded driver default -- dsr_hw_interface2.cpp:321 pushes
    `float limit[6] = {70.0f,...}` into both set_velj_rt() and set_accj_rt().
  * cuRobo already plans under `max_acceleration: 0.3` rad/s^2 = 17 deg/s^2
    (leeloo_curobo.yaml), i.e. 4x below that ceiling.

So the *plan* is conforming and the *command stream* is not. What the DRCF
evaluates is a realised delta-v between consecutive speedj commands, which
brings in three things nothing in the pipeline bounds:

  H1  QUEUE UNDERRUN. lbfgs_planner republishes every (command_points-1)*dt
      = 400 ms while the queue only holds command_points*dt = 480 ms, i.e. a
      single point of margin. The MPC is logged publishing up to 178 ms late;
      past 80 ms late execute_trajectory.cpp falls into its idle branch and
      commands vel={0,...} with time=0.08 -- a full stop in 80 ms mid-motion,
      followed by a restart at full speed.
  H2  SEAM / TRACKING ERROR. Every solve restarts from the *measured* velocity
      (mpc_vel_feedback_alpha=1.0). The arm lags because it is itself limited
      to 70 deg/s^2, so each new batch starts below the last command -- a step
      at point 0 of every batch, with no acceleration continuity constraint
      across batches. If the DRCF derives a = (vel_cmd - vel_actual)/time,
      that persistent tracking gap alone sustains the alarm.
  H3  INTRA-PLAN noise inside a single batch (least likely).

With time=0.08 s and a 70 deg/s^2 ceiling the real budget is
**delta-v < 5.6 deg/s per command** -- a very tight target.

WHAT IT DOES
------------
Strictly passive: it subscribes, it NEVER publishes on speedj_rt_stream. It
keeps a ring buffer of the command stream and, on every alarm seen on /rosout,
freezes the preceding window and attributes the alarm to H1/H2/H3.

ALARM STRING DECODING -- AN ASSUMPTION, NOT A FACT
--------------------------------------------------
The string comes from the closed DRCF firmware (`grep "over max value"` finds
nothing in doosan-robot2/), so it cannot be confirmed from source. Observed:

    a = (13.588, -8.577, -4.988, -8.413, 5.243) > (-70.305, 70.0, 70.0, 70.0, 70.0)

Reading SHIFTED (assumed): 6 accelerations then 5 limits -- the 6th
acceleration occupies the first slot of the second group and the 6th limit is
truncated. Under it every logged line has exactly one joint over 70, which is
coherent. Reading NAIVE (5 vs 5) would make the first "limit" -70.305, then
-47.822, then 9.972, which is meaningless.

This tool does NOT hardcode either reading: it keeps the raw line, parses every
numeric group, and reports the offending joints under BOTH readings. A
disagreement between them is information, not noise.

USAGE
-----
    source /home/ros2_ws/install/setup.bash
    python3 src/curobo_ros/scripts/speedj_alarm_forensics.py

    --window S     look-back analysed per alarm (default 1.5 s)
    --limit D      acceleration ceiling in deg/s^2 (default 70)
    --outdir DIR   CSV destination (default /home/ros2_ws/diag)
    --quiet        alarm verdicts only, no periodic live line

Two CSVs land in --outdir, in the conventions of core/diagnostics.py so they
sit next to mpc_diag_* / speedj_publish_*:
    speedj_alarm_*.csv   one row per alarm, with its verdict
    speedj_trace_*.csv   one row per command, continuous (feeds a baseline)
"""

import argparse
import glob
import math
import os
import re
import signal
import sys
from collections import deque, Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from dsr_msgs2.msg import SpeedjRtStream, RobotError

from curobo_ros.core.diagnostics import open_diag_csv

NDOF = 6

# The alarm text we trigger on. Matched on the DRCF's `param :` line, which is
# the only one of the multi-line OnLogAlarm dump carrying the numbers.
ALARM_TEXT = "Acceleration is over max value"
# Same family, worth catching too: it points at set_velj_rt's 70 deg/s ceiling
# rather than set_accj_rt's, and would otherwise look like a silent limit.
VELOCITY_ALARM_TEXT = "Velocity is over max value"

# The alarm never reaches /rosout. dsr_controller2.cpp:3152 logs it through
# RCLCPP_INFO(rclcpp::get_logger("dsr_controller2"), ...) -- a FREE logger, not
# a node logger, and in ROS 2 only node-attached loggers publish to /rosout.
# Confirmed empirically: three alarms fired during a traced run and /rosout
# carried none of them. They do land in the node's own log file, which is what
# we tail. /rosout stays subscribed anyway: it costs nothing and would catch a
# node-logger alarm if the driver ever grows one.
_LOG_LINE_RE = re.compile(r"^\[\w+\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$")

_NUM_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")
_GROUP_RE = re.compile(r"\(([^)]*)\)")

# A command whose payload is all-zero while the arm is still moving faster than
# this is the H1 signature: a full stop injected mid-motion. Well above encoder
# noise, well below any velocity the MPC actually commands.
_MOVING_THRESHOLD_DPS = 2.0


def parse_alarm_numbers(text):
    """Return (groups, flat) for an alarm line.

    `groups` is the list of numeric groups as they appear between parentheses;
    `flat` is every number in order. Both readings are derived from these, so
    neither is baked into the parser.
    """
    groups = []
    for g in _GROUP_RE.findall(text):
        nums = [float(n) for n in _NUM_RE.findall(g)]
        if nums:
            groups.append(nums)
    flat = [float(n) for n in _NUM_RE.findall(text)]
    return groups, flat


def decode_readings(groups, flat, limit_dps2):
    """Attribute the alarm to joints under both candidate readings.

    Returns {reading_name: (accels, offenders)} where `offenders` lists
    (joint_index, value) for |value| > limit. Either entry may be None when the
    message does not have the shape that reading expects.
    """
    out = {}

    # SHIFTED: 6 accelerations first, whatever follows is limits (possibly
    # truncated). This is the reading the log is consistent with.
    if len(flat) >= NDOF:
        acc = flat[:NDOF]
        out["shifted"] = (acc, [(i, v) for i, v in enumerate(acc)
                                if abs(v) > limit_dps2])
    else:
        out["shifted"] = None

    # NAIVE: first parenthesised group is the acceleration vector as printed.
    if groups:
        acc = groups[0]
        out["naive"] = (acc, [(i, v) for i, v in enumerate(acc)
                              if abs(v) > limit_dps2])
    else:
        out["naive"] = None
    return out


def fmt_vec(v, width=8, prec=2):
    return "[" + " ".join(f"{x:{width}.{prec}f}" for x in v) + "]"


class SpeedjAlarmForensics(Node):

    def __init__(self, args):
        super().__init__("speedj_alarm_forensics")
        self.args = args
        self.limit = args.limit
        self.window_s = args.window

        # Ring buffers. Sized generously past --window so a late alarm still
        # finds its cause: the DRCF -> driver -> /rosout path adds latency, and
        # the offending command may be several hundred ms behind the report.
        self.cmds = deque(maxlen=2000)      # (t, vel_dps[6], acc[6], time)
        self.states = deque(maxlen=4000)    # (t, vel_dps[6], pos_deg[6])
        self.batches = deque(maxlen=200)    # (t, n_points)
        self.errors = deque(maxlen=200)     # (t, RobotError)

        self.n_alarms = 0
        self.verdicts = Counter()
        self.reading_disagreements = 0
        self.worst_cmd_accel = 0.0
        self.worst_track_accel = 0.0
        self.n_zero_payload = 0
        self.n_cmds = 0

        # VOLATILE on purpose for /rosout: its publishers are TRANSIENT_LOCAL
        # with depth 1000, and a durable subscription would replay a backlog of
        # stale alarms from before this tool started as if they were live.
        rosout_qos = QoSProfile(
            depth=200,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST)

        self.create_subscription(
            SpeedjRtStream, "/dsr01/dsr_controller2/speedj_rt_stream",
            self.cb_cmd, 50)
        self.create_subscription(
            JointState, "/dsr01/joint_states", self.cb_state, 50)
        self.create_subscription(
            JointTrajectory, "/leeloo/execute_trajectory", self.cb_batch, 10)
        self.create_subscription(
            RobotError, "/dsr01/error", self.cb_error, 20)
        self.create_subscription(Log, "/rosout", self.cb_rosout, rosout_qos)

        if not self.has_parameter("diagnostic_csv_dir"):
            self.declare_parameter("diagnostic_csv_dir", args.outdir)

        self.alarm_csv = open_diag_csv(self, "speedj_alarm")
        if self.alarm_csv:
            self.alarm_csv.write_header_once([
                "t_s", "verdict", "worst_joint", "worst_cmd_accel_dps2",
                "worst_track_accel_dps2", "offender_age_ms", "at_seam",
                "zero_payload_in_window", "batch_age_ms", "n_cmds_in_window",
                "offenders_shifted", "offenders_naive", "readings_agree",
                "raw",
            ])
        self.trace_csv = open_diag_csv(self, "speedj_trace")
        if self.trace_csv:
            self.trace_csv.write_header_once([
                "t_s", "dt_ms", "speedj_time_s", "at_seam", "zero_payload",
                "cmd_vel_max_dps", "meas_vel_max_dps",
                "cmd_accel_max_dps2", "cmd_accel_argmax_joint",
                "track_accel_max_dps2", "track_accel_argmax_joint",
            ])

        # Alarm source: tail the driver's log file (see _LOG_LINE_RE above).
        self._log_path = None
        self._log_fh = None
        self._log_glob = args.alarm_log or os.path.join(
            os.environ.get("ROS_LOG_DIR") or os.path.expanduser("~/.ros/log"),
            "ros2_control_node_*.log")
        self._open_alarm_log(seek_end=True)
        self.create_timer(0.1, self.poll_alarm_log)

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        if not args.quiet:
            self.create_timer(5.0, self.live_line)

        self.get_logger().info(
            f"passive forensics up: limit={self.limit} deg/s^2, "
            f"window={self.window_s}s, budget at time="
            f"{args.command_period}s is "
            f"dv < {self.limit * args.command_period:.2f} deg/s per command")

    # ---------------- clock ----------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------- subscriptions ----------------

    def cb_cmd(self, msg):
        t = self.now()
        vel = list(msg.vel[:NDOF])          # already deg/s (execute_trajectory converts)
        acc = list(msg.acc[:NDOF])
        prev = self.cmds[-1] if self.cmds else None
        self.cmds.append((t, vel, acc, float(msg.time)))
        self.n_cmds += 1

        meas = self.state_at(t)
        zero_payload = self.is_zero_payload(vel, meas)
        if zero_payload:
            self.n_zero_payload += 1

        ca, cj = self.cmd_accel(prev, (t, vel, acc, float(msg.time)))
        ta, tj = self.track_accel((t, vel, acc, float(msg.time)), meas)
        self.worst_cmd_accel = max(self.worst_cmd_accel, ca)
        self.worst_track_accel = max(self.worst_track_accel, ta)

        if self.trace_csv:
            dt_ms = (t - prev[0]) * 1e3 if prev else float("nan")
            self.trace_csv.writerow([
                f"{t - self.t0:.3f}", f"{dt_ms:.1f}", f"{msg.time:.4f}",
                int(self.at_seam(t)), int(zero_payload),
                f"{max(abs(v) for v in vel):.2f}",
                f"{max(abs(v) for v in meas):.2f}" if meas else "",
                f"{ca:.1f}", cj if cj is not None else "",
                f"{ta:.1f}" if meas else "", tj if tj is not None else "",
            ])

    def cb_state(self, msg):
        n = min(NDOF, len(msg.position))
        pos = [math.degrees(p) for p in msg.position[:n]]
        vel = [math.degrees(v) for v in msg.velocity[:n]] if len(msg.velocity) >= n \
            else [0.0] * n
        self.states.append((self.now(), vel, pos))

    def cb_batch(self, msg):
        self.batches.append((self.now(), len(msg.points)))

    def cb_error(self, msg):
        self.errors.append((self.now(), msg))

    def cb_rosout(self, msg):
        if ALARM_TEXT in msg.msg:
            self.on_alarm(msg.msg, "accel")
        elif VELOCITY_ALARM_TEXT in msg.msg:
            self.on_alarm(msg.msg, "vel")

    # ---------------- derived quantities ----------------

    def state_at(self, t):
        """Measured velocity (deg/s) from the newest joint_states at or before t."""
        best = None
        for ts, vel, _pos in reversed(self.states):
            if ts <= t:
                best = vel
                break
        if best is None and self.states:
            best = self.states[0][1]
        return best

    def is_zero_payload(self, vel, meas):
        """H1 signature: an all-zero command while the arm is still moving.

        Detected on the PAYLOAD, not on a timing gap: execute_trajectory's timer
        keeps ticking at command_period during an underrun, so the commands
        arrive perfectly on time -- they just carry a stop.
        """
        if any(abs(v) > 1e-9 for v in vel):
            return False
        return bool(meas) and max(abs(v) for v in meas) > _MOVING_THRESHOLD_DPS

    def cmd_accel(self, prev, cur):
        """(max |dv/time|, argmax joint) between two consecutive commands."""
        if prev is None:
            return 0.0, None
        dt = cur[3] if cur[3] > 1e-6 else (cur[0] - prev[0])
        if dt <= 1e-6:
            return 0.0, None
        vals = [abs(a - b) / dt for a, b in zip(cur[1], prev[1])]
        m = max(vals)
        return m, vals.index(m)

    def track_accel(self, cur, meas):
        """(max |(vel_cmd - vel_meas)/time|, argmax joint).

        This is the quantity the DRCF most plausibly checks, and the one that
        stays high on a perfectly smooth commanded profile when the arm simply
        cannot keep up (H2).
        """
        if not meas:
            return 0.0, None
        dt = cur[3] if cur[3] > 1e-6 else 0.08
        n = min(len(meas), NDOF)
        vals = [abs(cur[1][i] - meas[i]) / dt for i in range(n)]
        if not vals:
            return 0.0, None
        m = max(vals)
        return m, vals.index(m)

    def at_seam(self, t, span=None):
        """True if a fresh trajectory batch landed just before this command.

        A batch arrival overwrites the queue (execute_trajectory.cpp does
        `this->trajectory = *msg;`), so the next command is point 0 of a new
        plan -- the boundary H2 is about.
        """
        # 1.5x the command period: wide enough to survive timer jitter between
        # the batch landing and the next tick consuming it, narrow enough that
        # it still only ever covers the first command or two of a batch (which
        # arrive every ~400 ms).
        span = span if span is not None else 1.5 * self.args.command_period
        return any(t - span < tb <= t for tb, _n in self.batches)

    # ---------------- alarm log tail ----------------

    def _newest_log(self):
        matches = glob.glob(self._log_glob)
        return max(matches, key=os.path.getmtime) if matches else None

    def _open_alarm_log(self, seek_end):
        """(Re)open the newest matching log file.

        `seek_end` only on the very first open: we must not replay alarms that
        fired before this tool started, but a file that APPEARS later (the
        driver was restarted mid-session) begins after us and is read whole.
        """
        path = self._newest_log()
        if path is None or path == self._log_path:
            if path is None:
                self.get_logger().warn(
                    f"no alarm log matching {self._log_glob} -- alarms will NOT "
                    f"be captured; pass --alarm-log")
            return
        try:
            fh = open(path, "r", errors="replace")
        except OSError as e:
            self.get_logger().warn(f"cannot open alarm log {path}: {e}")
            return
        if seek_end:
            fh.seek(0, os.SEEK_END)
        if self._log_fh:
            self._log_fh.close()
        self._log_fh, self._log_path = fh, path
        self.get_logger().info(f"alarm source: {path}")

    def poll_alarm_log(self):
        self._open_alarm_log(seek_end=False)   # picks up a driver restart
        if self._log_fh is None:
            return
        for line in self._log_fh.readlines():
            if ALARM_TEXT not in line and VELOCITY_ALARM_TEXT not in line:
                continue
            m = _LOG_LINE_RE.match(line.strip())
            # Use the line's OWN ROS timestamp, not now(): the 10 Hz poll adds
            # up to 100 ms of skew, which at 80 ms per command would shift the
            # window by more than a full command.
            t = float(m.group(1)) if m else self.now()
            text = m.group(3) if m else line.strip()
            self.on_alarm(text, "accel" if ALARM_TEXT in line else "vel", t)

    # ---------------- alarm handling ----------------

    def on_alarm(self, raw, kind, t_alarm=None):
        t_alarm = self.now() if t_alarm is None else t_alarm
        self.n_alarms += 1
        groups, flat = parse_alarm_numbers(raw)
        readings = decode_readings(groups, flat, self.limit)

        off_shift = readings["shifted"][1] if readings["shifted"] else []
        off_naive = readings["naive"][1] if readings["naive"] else []
        agree = sorted(i for i, _ in off_shift) == sorted(i for i, _ in off_naive)
        if not agree:
            self.reading_disagreements += 1

        win = [c for c in self.cmds if t_alarm - self.window_s <= c[0] <= t_alarm]

        worst_ca, worst_cj, worst_ct, worst_seam = 0.0, None, None, False
        worst_ta, worst_tj = 0.0, None
        zero_payload = False
        prev = None
        for c in win:
            ca, cj = self.cmd_accel(prev, c)
            if ca > worst_ca:
                worst_ca, worst_cj, worst_ct = ca, cj, c[0]
                worst_seam = self.at_seam(c[0])
            meas = self.state_at(c[0])
            ta, tj = self.track_accel(c, meas)
            if ta > worst_ta:
                worst_ta, worst_tj = ta, tj
            if self.is_zero_payload(c[1], meas):
                zero_payload = True
            prev = c

        batch_age = (t_alarm - self.batches[-1][0]) * 1e3 if self.batches else float("nan")
        batch_age_s = f"{batch_age:.0f}ms" if self.batches else "n/a (aucun batch recu)"

        # Ordered so the cheapest, most specific signature wins. A stop injected
        # mid-motion explains an over-limit acceleration on its own, whichever
        # else is also true in the window.
        if zero_payload:
            verdict = "H1_queue_underrun"
        elif worst_ca > self.limit and worst_seam:
            verdict = "H2_batch_seam"
        elif worst_ca > self.limit:
            verdict = "H3_intra_plan"
        elif worst_ta > self.limit:
            verdict = "H2_tracking_error"
        else:
            verdict = "UNEXPLAINED"
        self.verdicts[verdict] += 1

        if self.alarm_csv:
            self.alarm_csv.writerow([
                f"{t_alarm - self.t0:.3f}", verdict,
                worst_cj if worst_cj is not None else "",
                f"{worst_ca:.1f}", f"{worst_ta:.1f}",
                f"{(t_alarm - worst_ct) * 1e3:.0f}" if worst_ct else "",
                int(worst_seam), int(zero_payload), f"{batch_age:.0f}",
                len(win),
                ";".join(f"j{i}={v:.2f}" for i, v in off_shift),
                ";".join(f"j{i}={v:.2f}" for i, v in off_naive),
                int(agree), raw.replace("\n", " "),
            ])

        self.print_alarm(t_alarm, kind, raw, readings, verdict, win,
                         worst_ca, worst_cj, worst_ta, worst_tj,
                         worst_seam, zero_payload, batch_age_s, agree)

    def print_alarm(self, t_alarm, kind, raw, readings, verdict, win,
                    worst_ca, worst_cj, worst_ta, worst_tj,
                    worst_seam, zero_payload, batch_age_s, agree):
        w = sys.stdout.write
        w("\n" + "=" * 78 + "\n")
        w(f"ALARM #{self.n_alarms} ({kind}) t={t_alarm - self.t0:8.3f}s  "
          f"VERDICT: {verdict}\n")
        w(f"  raw: {raw.strip()}\n")
        for name in ("shifted", "naive"):
            r = readings[name]
            if r is None:
                w(f"  reading {name:8s}: not applicable to this message shape\n")
                continue
            acc, off = r
            tag = ", ".join(f"j{i}={v:.2f}" for i, v in off) or "none over limit"
            w(f"  reading {name:8s}: a={fmt_vec(acc)} -> {tag}\n")
        if not agree:
            w("  !! the two readings disagree on the offending joint -- "
              "worth reconciling against the computed accelerations below\n")

        w(f"  window: {len(win)} commands over {self.window_s:.2f}s, "
          f"last batch {batch_age_s} ago\n")
        w(f"  worst commanded  dv/time : {worst_ca:7.1f} deg/s^2 "
          f"(joint {worst_cj}, {'AT BATCH SEAM' if worst_seam else 'intra-batch'})\n")
        w(f"  worst tracking gap/time  : {worst_ta:7.1f} deg/s^2 (joint {worst_tj})\n")
        w(f"  zero-payload stop in window: {'YES -> H1' if zero_payload else 'no'}\n")

        w("  last commands before the alarm (deg/s):\n")
        prev = None
        for c in win[-6:]:
            ca, cj = self.cmd_accel(prev, c)
            meas = self.state_at(c[0])
            flags = []
            if self.at_seam(c[0]):
                flags.append("SEAM")
            if self.is_zero_payload(c[1], meas):
                flags.append("STOP")
            if ca > self.limit:
                flags.append("OVER")
            w(f"    t-{(t_alarm - c[0]) * 1e3:6.0f}ms cmd={fmt_vec(c[1])} "
              f"meas={fmt_vec(meas) if meas else 'n/a':>10s} "
              f"a={ca:7.1f}(j{cj}) time={c[3]:.3f} {' '.join(flags)}\n")
            prev = c
        w("=" * 78 + "\n")
        sys.stdout.flush()

    # ---------------- live / summary ----------------

    def live_line(self):
        self.get_logger().info(
            f"cmds={self.n_cmds} alarms={self.n_alarms} "
            f"zero_payload={self.n_zero_payload} "
            f"peak_cmd_accel={self.worst_cmd_accel:.1f} "
            f"peak_track_accel={self.worst_track_accel:.1f} deg/s^2 "
            f"(limit {self.limit})"
            + ("" if self.states else "  [no joint_states yet]"))

    def summary(self):
        w = sys.stdout.write
        w("\n" + "=" * 78 + "\nSESSION SUMMARY\n")
        w(f"  commands seen        : {self.n_cmds}\n")
        w(f"  alarms captured      : {self.n_alarms}\n")
        w(f"  zero-payload stops   : {self.n_zero_payload}"
          f"   (H1 events, alarm or not)\n")
        w(f"  peak commanded accel : {self.worst_cmd_accel:.1f} deg/s^2\n")
        w(f"  peak tracking accel  : {self.worst_track_accel:.1f} deg/s^2\n")
        w(f"  limit                : {self.limit:.1f} deg/s^2\n")
        if self.verdicts:
            w("  verdicts:\n")
            for k, v in self.verdicts.most_common():
                w(f"    {k:22s} {v:4d}  ({100.0 * v / self.n_alarms:.0f}%)\n")
        if self.reading_disagreements:
            w(f"  alarm-string readings disagreed on {self.reading_disagreements} "
              f"of {self.n_alarms} alarms\n")
        w("=" * 78 + "\n")
        sys.stdout.flush()
        for c in (self.alarm_csv, self.trace_csv):
            if c:
                c.close()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--window", type=float, default=1.5,
                   help="look-back analysed per alarm, seconds (default 1.5)")
    p.add_argument("--limit", type=float, default=70.0,
                   help="acceleration ceiling, deg/s^2 (default 70, "
                        "= set_accj_rt in dsr_hw_interface2.cpp:321)")
    p.add_argument("--outdir", default="/home/ros2_ws/diag",
                   help="CSV destination (default /home/ros2_ws/diag)")
    p.add_argument("--command-period", type=float, default=0.08,
                   dest="command_period",
                   help="ExecuteTrajectory's command_period, seconds (default "
                        "0.08) -- sets the batch-seam detection window")
    p.add_argument("--alarm-log", default=None, dest="alarm_log",
                   help="path or glob of the driver log carrying the alarms "
                        "(default: newest $ROS_LOG_DIR/ros2_control_node_*.log). "
                        "The alarm does NOT reach /rosout -- see _LOG_LINE_RE.")
    p.add_argument("--quiet", action="store_true",
                   help="suppress the periodic live line")
    args = p.parse_args()

    rclpy.init()
    node = SpeedjAlarmForensics(args)
    # Also honour SIGTERM: this tool is typically stopped with a `timeout` or a
    # kill from a test script, and the session summary is the whole point --
    # losing it because the signal was not Ctrl-C would defeat the run.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
