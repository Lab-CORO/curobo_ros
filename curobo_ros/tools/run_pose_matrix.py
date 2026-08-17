#!/usr/bin/env python3
"""Replay a fixed matrix of Cartesian goals against the reactive controller.

Why this exists
---------------
On 2026-08-07 three consecutive runs of the same MPPI configuration produced
41%, 92% and 92% null-space cancellation, and a fix was declared effective on
the strength of a single run that had simply targeted a different goal. Any
conclusion drawn from one trajectory is unfalsifiable: behaviour varies
qualitatively with how close the goal sits to the wrist singularity. This
replays the same labelled set every time so a change can be attributed.

Two modes
---------
Build the matrix once, from joint poses that are reachable by construction::

    ros2 run curobo_ros run_pose_matrix --from-joint-poses \\
        $(ros2 pkg prefix leeloo_calibration)/share/leeloo_calibration/config/calibration_poses.yaml \\
        --out $(ros2 pkg prefix curobo_ros)/share/curobo_ros/config/pose_matrix.yaml --count 10

Then replay it, as often as you like::

    ros2 run curobo_ros run_pose_matrix --matrix <pose_matrix.yaml> \\
        --out-dir /home/ros2_ws/diag --label before-clamp-removal

Each pose gets a row in ``pose_matrix_summary_*.csv``. Point analyze_mpc_seam
at that file (``--matrix``) to compare two campaigns.

The planner's own per-step CSVs (mpc_diag_*.csv) are written independently and
only when ``mpc_debug`` is true — launch with ``mpc_debug:=true`` to get both.

Lives in ``curobo_ros/tools/`` rather than ``scripts/`` because ``ros2 run``
only looks in ``lib/<pkg>/``, where console_scripts are the only thing that
lands — a file installed through ``data_files`` goes to ``share/`` and stays
invisible. See the note next to the entry_points block in setup.py.
"""

import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from curobo_msgs.action import SendTrajectory
from curobo_msgs.srv import Fk, SetPlanner, WarmupFK
from curobo_ros.core.diagnostics import open_diag_csv

DEFAULT_PLANNER_NODE = "/curobo_trajectory_planner"
# Wrist singularity: on the M1013 axes 4 and 6 become collinear as axis 5 nears
# zero, so |j5| is the natural distance-to-singularity label. Below this the
# pose is "near"; the matrix deliberately spans both sides.
SINGULAR_J5_RAD = 0.35  # ~20 deg


class PoseMatrixRunner(Node):
    def __init__(self, planner_node: str, out_dir: str):
        super().__init__("pose_matrix_runner")
        if out_dir:
            self.declare_parameter("diagnostic_csv_dir", out_dir)
        self._planner_node = planner_node.rstrip("/")
        self._fk_cli = self.create_client(Fk, f"{self._planner_node}/fk")
        self._warmup_cli = self.create_client(WarmupFK, f"{self._planner_node}/warmup_fk")
        self._set_planner_cli = self.create_client(
            SetPlanner, f"{self._planner_node}/set_planner")
        self._exec_cli = ActionClient(
            self, SendTrajectory, f"{self._planner_node}/execute_trajectory")
        # Latest feedback from the in-flight goal, refreshed by _feedback_cb.
        self._fb = None
        # Handle of the goal currently being chased, so Ctrl-C can cancel it.
        self._handle = None

    # -- helpers ---------------------------------------------------------

    def _call(self, client, request, what: str, timeout: float = 30.0):
        """Blocking service call on a spinning executor, or None on failure."""
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"{what}: service unavailable at {client.srv_name}")
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error(f"{what}: timed out after {timeout}s")
            return None
        return future.result()

    def select_planner_mpc(self) -> bool:
        req = SetPlanner.Request()
        req.planner_type = SetPlanner.Request.MPC
        res = self._call(self._set_planner_cli, req, "set_planner")
        if res is None:
            return False
        # Already-MPC is reported as success by the node; a failure here means
        # the switch itself broke, which invalidates the whole campaign.
        if not res.success:
            self.get_logger().error(f"set_planner failed: {res.message}")
            return False
        self.get_logger().info(f"Planner: {res.current_planner}")
        return True

    # -- matrix construction --------------------------------------------

    def joint_poses_to_cartesian(self, joint_poses):
        """FK a list of joint-position lists through the planner's own model.

        Uses the node's /fk service rather than loading cuRobo kinematics here,
        so the poses are produced by exactly the model that will chase them.
        """
        warm = self._call(self._warmup_cli, WarmupFK.Request(batch_size=len(joint_poses)),
                          "warmup_fk", timeout=120.0)
        if warm is None or not warm.success:
            self.get_logger().error("warmup_fk failed - cannot build the matrix")
            return None
        req = Fk.Request()
        for positions in joint_poses:
            js = JointState()
            js.position = [float(v) for v in positions]
            req.joint_states.append(js)
        res = self._call(self._fk_cli, req, "fk", timeout=120.0)
        if res is None or not res.poses:
            self.get_logger().error("fk returned no poses")
            return None
        return res.poses

    # -- replay ----------------------------------------------------------

    def _feedback_cb(self, msg):
        self._fb = msg.feedback

    def cancel_inflight(self):
        """Cancel the goal being chased, if any. Safe to call twice.

        This MUST run on Ctrl-C too. Reactive control never terminates on its
        own, so killing the client does not stop the arm: on 2026-08-07 a Ctrl-C
        27 s into a 30 s pose left the planner servoing an uncancelled goal for
        another ~70 s unattended, its FK error diverging from 0.33 m back out to
        0.73 m (mpc_diag_20260807_143422.csv, 943 steps over 129 s of a run the
        operator had already interrupted).
        """
        handle, self._handle = self._handle, None
        if handle is None:
            return
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=10.0)

    def run_pose(self, entry, timeout_s: float, settle_s: float):
        """Send one goal, wait for convergence or timeout, cancel, report."""
        p = entry["pose"]
        goal = SendTrajectory.Goal()
        goal.target_pose = Pose()
        goal.target_pose.position.x = float(p["x"])
        goal.target_pose.position.y = float(p["y"])
        goal.target_pose.position.z = float(p["z"])
        goal.target_pose.orientation.w = float(p["qw"])
        goal.target_pose.orientation.x = float(p["qx"])
        goal.target_pose.orientation.y = float(p["qy"])
        goal.target_pose.orientation.z = float(p["qz"])

        self._fb = None
        if not self._exec_cli.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("execute_trajectory action server unavailable")
            return None

        send_future = self._exec_cli.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=30.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"pose {entry['index']}: goal rejected")
            return None

        self._handle = handle
        t0 = time.monotonic()
        converged_at = None
        while time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._fb is not None and self._fb.on_target:
                converged_at = time.monotonic() - t0
                break

        # Reactive control never terminates on its own (the loop exits on cancel
        # or error), so the cancel is the normal end of a pose, not a failure.
        self.cancel_inflight()

        fb = self._fb
        row = {
            "index": entry["index"],
            "j5_rad": entry.get("j5_rad", float("nan")),
            "near_singularity": entry.get("near_singularity", False),
            "converged": converged_at is not None,
            "time_to_converge_s": converged_at if converged_at is not None else float("nan"),
            "final_pos_err_m": fb.position_error if fb else float("nan"),
            "final_rot_err_rad": fb.orientation_error if fb else float("nan"),
            "hold_count": fb.hold_count if fb else 0,
        }
        verdict = "CONVERGED" if row["converged"] else "TIMEOUT"
        self.get_logger().info(
            f"pose {row['index']:>3}  {verdict:<9} "
            f"pos={row['final_pos_err_m']:.4f}m "
            f"rot={math.degrees(row['final_rot_err_rad']):.2f}deg "
            f"hold={row['hold_count']}  |j5|={row['j5_rad']:.2f}rad")

        # Let the arm come to rest before the next goal, so one pose's residual
        # motion is not measured as the next pose's starting transient.
        settle_end = time.monotonic() + settle_s
        while time.monotonic() < settle_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return row


def load_joint_poses(path):
    with open(path, "r") as f:
        doc = yaml.safe_load(f)
    return [p["positions"] for p in doc.get("poses", []) if p.get("positions")]


def transit_min_j5(a, b, samples=33):
    """Smallest |j5| along a straight joint-space line from pose `a` to `b`.

    The goal's own |j5| is a poor singularity label: measured on the 50 taught
    calibration poses, every one sits above 0.73 rad (42 deg), yet runs through
    them still showed corr(j4,j6) = -0.99. The wrist singularity is crossed
    EN ROUTE, not arrived at. This approximates that crossing.

    It is a proxy, not the truth: MPPI plans against a Cartesian cost and does
    not follow a joint-space line. Treat it as "this transition plausibly passes
    near the singularity", and confirm against the null-space check on the
    resulting mpc_diag CSV.
    """
    return min(abs(a[4] + (b[4] - a[4]) * k / (samples - 1)) for k in range(samples))


def read_joint_limits(urdf_path, dof=6):
    """(lower, upper) per arm joint from the URDF, or None if unreadable.

    Parsed from the URDF rather than hardcoded because the limits differ per
    robot, and a wrong hardcoded value here would silently pass poses that the
    controller then refuses.
    """
    try:
        import xml.etree.ElementTree as ET
        root = ET.parse(urdf_path).getroot()
        limits = []
        for j in root.iter("joint"):
            name = j.get("name", "")
            lim = j.find("limit")
            # The arm's own joints only: the URDF also carries the Ridgeback
            # base and the gripper knuckles, whose limits are irrelevant here.
            if lim is None or not name.lower().startswith("joint"):
                continue
            lo, up = lim.get("lower"), lim.get("upper")
            if lo is None or up is None:
                continue
            limits.append((float(lo), float(up)))
        return limits[:dof] if len(limits) >= dof else None
    except Exception:
        return None


def within_limits(jp, limits, margin):
    """True when every joint sits at least `margin` inside its bounds."""
    return all(lo + margin <= q <= up - margin
               for q, (lo, up) in zip(jp, limits))


def select_spanning_singularity(joint_poses, count):
    """Pick `count` poses spread across the |j5| range.

    The singularity gradient is the variable that made three runs disagree, so
    the matrix must sample it deliberately rather than take the first N poses,
    which would over-represent whatever posture the operator happened to teach.
    """
    scored = sorted(((abs(p[4]), i, p) for i, p in enumerate(joint_poses)),
                    key=lambda t: t[0])
    if count >= len(scored):
        return scored
    # Even strides through the sorted list: guarantees both the most singular
    # and the most well-conditioned pose are present, plus a spread between.
    step = (len(scored) - 1) / (count - 1) if count > 1 else 1
    return [scored[int(round(k * step))] for k in range(count)]


def build_matrix(runner, joint_poses_path, out_path, count, urdf_path, limit_margin):
    joint_poses = load_joint_poses(joint_poses_path)
    if not joint_poses:
        runner.get_logger().error(f"no poses in {joint_poses_path}")
        return 1
    runner.get_logger().info(f"{len(joint_poses)} joint poses read; selecting {count}")

    # Joint-limit filter, BEFORE the singularity spread. select_spanning_
    # singularity deliberately takes the extremes of the |j5| range, which on
    # the taught calibration library means the pose with the LARGEST |j5| --
    # i.e. the one closest to joint 5's bound. Run without this filter on
    # 2026-08-07, the matrix contained a goal at |j5| = 2.286 rad against a
    # +-2.356 rad limit: 4 degrees of headroom, and the arm hit the stop.
    # Maximising singularity coverage and avoiding the bounds pull in opposite
    # directions; the bound wins, because a pose the controller cannot legally
    # hold measures nothing.
    limits = read_joint_limits(urdf_path)
    if limits is None:
        runner.get_logger().warn(
            f"could not read joint limits from {urdf_path} - NOT filtering "
            "near-limit poses. Check the matrix by hand before running it.")
    else:
        n_before = len(joint_poses)
        joint_poses = [p for p in joint_poses if within_limits(p, limits, limit_margin)]
        dropped = n_before - len(joint_poses)
        lim_txt = ", ".join(f"j{i+1} +-{up:.2f}" for i, (_, up) in enumerate(limits))
        runner.get_logger().info(
            f"joint limits ({lim_txt}) rad; margin {limit_margin} rad "
            f"({math.degrees(limit_margin):.0f} deg) -> dropped {dropped}, "
            f"{len(joint_poses)} candidates remain")
        if not joint_poses:
            runner.get_logger().error(
                "every pose is within the limit margin - lower --limit-margin "
                "or teach poses further from the bounds")
            return 1

    chosen = select_spanning_singularity(joint_poses, count)
    poses = runner.joint_poses_to_cartesian([p for _, _, p in chosen])
    if poses is None:
        return 1

    entries = []
    for n, ((j5, src_idx, jp), pose) in enumerate(zip(chosen, poses)):
        # The transition INTO this pose, i.e. from its predecessor in replay
        # order. The first entry has no predecessor, so it carries its own |j5|.
        prev_jp = chosen[n - 1][2] if n > 0 else jp
        t_min = transit_min_j5(prev_jp, jp)
        entries.append({
            "index": src_idx + 1,
            "j5_rad": round(float(j5), 4),
            "transit_min_j5_rad": round(float(t_min), 4),
            "near_singularity": bool(min(j5, t_min) < SINGULAR_J5_RAD),
            "joint_positions": [round(float(v), 6) for v in jp],
            "pose": {
                "x": round(float(pose.position.x), 6),
                "y": round(float(pose.position.y), 6),
                "z": round(float(pose.position.z), 6),
                "qw": round(float(pose.orientation.w), 6),
                "qx": round(float(pose.orientation.x), 6),
                "qy": round(float(pose.orientation.y), 6),
                "qz": round(float(pose.orientation.z), 6),
            },
        })

    doc = {
        "metadata": {
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "source": joint_poses_path,
            "singular_j5_rad": SINGULAR_J5_RAD,
            "note": ("Cartesian goals obtained by FK through the planner's own "
                     "kinematic model, so every pose is reachable by "
                     "construction. j5_rad is |joint 5| at the goal, the "
                     "distance to the wrist singularity where axes 4 and 6 go "
                     "collinear. transit_min_j5_rad is the smallest |j5| along "
                     "a joint-space line from the PREVIOUS pose in this file, "
                     "a proxy for crossing the singularity en route -- which is "
                     "where it actually happens. near_singularity is true when "
                     "either drops below singular_j5_rad."),
        },
        "poses": entries,
    }
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False)
    near = sum(1 for e in entries if e["near_singularity"])
    runner.get_logger().info(
        f"wrote {out_path}: {len(entries)} poses, {near} near the singularity "
        f"(|j5| < {SINGULAR_J5_RAD} rad), goal |j5| range "
        f"{entries[0]['j5_rad']:.2f}..{entries[-1]['j5_rad']:.2f} rad")
    if near == 0:
        runner.get_logger().warn(
            "NO pose in this matrix approaches the wrist singularity, at the "
            "goal or in transit. It cannot discriminate the behaviour that "
            "motivated the matrix (41% vs 92% null-space cancellation between "
            "otherwise identical runs). Measured on the 50 taught calibration "
            "poses: min |j5| = 0.74 rad (42 deg) — that library was recorded "
            "for calibration, where the wrist stays well conditioned. Teach a "
            "few deliberately near-singular poses with pose_saver_node "
            "(|j5| under 0.35 rad) and rebuild, or the campaign will only ever "
            "sample the easy half of the workspace.")
    return 0


def replay_matrix(runner, matrix_path, label, timeout_s, settle_s):
    with open(matrix_path, "r") as f:
        doc = yaml.safe_load(f)
    entries = doc.get("poses", [])
    if not entries:
        runner.get_logger().error(f"no poses in {matrix_path}")
        return 1
    if not runner.select_planner_mpc():
        return 1

    csv_out = open_diag_csv(runner, f"pose_matrix_summary_{label}" if label
                            else "pose_matrix_summary")
    if csv_out is not None:
        csv_out.write_header_once([
            "index", "j5_rad", "near_singularity", "converged",
            "time_to_converge_s", "final_pos_err_m", "final_rot_err_deg",
            "hold_count"])

    rows = []
    for entry in entries:
        row = runner.run_pose(entry, timeout_s, settle_s)
        if row is None:
            continue
        rows.append(row)
        if csv_out is not None:
            csv_out.writerow([
                row["index"], f"{row['j5_rad']:.4f}", int(row["near_singularity"]),
                int(row["converged"]), f"{row['time_to_converge_s']:.2f}",
                f"{row['final_pos_err_m']:.5f}",
                f"{math.degrees(row['final_rot_err_rad']):.3f}",
                row["hold_count"]])
    if csv_out is not None:
        csv_out.close()

    ok = sum(1 for r in rows if r["converged"])
    runner.get_logger().info(f"=== {ok}/{len(rows)} poses converged ===")
    near = [r for r in rows if r["near_singularity"]]
    far = [r for r in rows if not r["near_singularity"]]
    for name, group in (("near singularity", near), ("well-conditioned", far)):
        if group:
            conv = sum(1 for r in group if r["converged"])
            runner.get_logger().info(f"  {name:<18}: {conv}/{len(group)} converged")
    # Non-zero exit when any pose failed, so this is usable as a gate in a
    # scripted campaign rather than something a human has to eyeball.
    return 0 if ok == len(rows) and rows else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from-joint-poses", metavar="YAML",
                    help="build the matrix by FK-ing a joint-pose YAML")
    ap.add_argument("--out", help="output path for --from-joint-poses")
    ap.add_argument("--count", type=int, default=10,
                    help="poses to select when building (default 10)")
    ap.add_argument("--urdf",
                    default=os.path.join(get_package_share_directory('leeloo'), 'urdf', 'leeloo.urdf'),
                    help="URDF used to read joint limits when building the matrix")
    ap.add_argument("--limit-margin", type=float, default=0.20,
                    help="reject candidate poses closer than this (rad) to any joint "
                         "limit (default 0.20 rad = 11 deg)")
    ap.add_argument("--matrix", help="pose_matrix.yaml to replay")
    ap.add_argument("--out-dir", default="",
                    help="directory for the summary CSV (default: ROS log dir)")
    ap.add_argument("--label", default="",
                    help="tag folded into the summary filename, e.g. 'before-clamp'")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="seconds to wait for convergence per pose (default 30)")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="seconds of rest between poses (default 2)")
    ap.add_argument("--planner-node", default=DEFAULT_PLANNER_NODE)
    args, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])

    if not args.from_joint_poses and not args.matrix:
        ap.error("one of --from-joint-poses or --matrix is required")
    if args.from_joint_poses and not args.out:
        ap.error("--from-joint-poses requires --out")

    rclpy.init(args=ros_args)
    runner = PoseMatrixRunner(args.planner_node, args.out_dir)
    try:
        if args.from_joint_poses:
            return build_matrix(runner, args.from_joint_poses, args.out, args.count,
                                args.urdf, args.limit_margin)
        return replay_matrix(runner, args.matrix, args.label, args.timeout, args.settle)
    except KeyboardInterrupt:
        runner.get_logger().info("interrupted - cancelling the in-flight goal")
        try:
            runner.cancel_inflight()
        except Exception as e:
            runner.get_logger().error(
                f"could not cancel on interrupt ({e}) - the arm may still be "
                "servoing; cancel by hand or stop the planner")
        return 130
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
