#!/usr/bin/env python3
"""Correlate the MPC/joint-speed diagnostic CSVs with the Doosan's SpeedJ
acceleration alarms, to settle where the over-limit acceleration comes from.

Two competing explanations for the `[SpeedJ] Acceleration is over max value`
alarms (index 1216, limit 70 deg/s^2):

  A. BATCH SEAM  - each MPC batch is truncated mid-ramp: the bridge plays
     mpc_command_interval / interpolation_dt points, then the next batch
     overwrites the queue at index 0 starting from the (lagging) measured
     velocity. Predicts: accel_boundary_dps2 spikes, accel_win_max_dps2 stays
     at/below the clamp setting.

  B. INTRA-PLAN  - the MPPI plan itself is noisy within a single window.
     Predicts the opposite: accel_win_max_dps2 spikes, accel_boundary_dps2
     stays low.

Usage
-----
    python3 analyze_mpc_seam.py mpc_diag_*.csv [--speedj speedj_publish_*.csv]
                                               [--log log.txt]

`--log` is optional: alarm timestamps are absolute (ROS wall clock) while the
CSV `t_s` column is relative to the goal, so they are reported side by side
rather than joined on a shared clock.
"""

import argparse
import csv
import glob
import math
import os
import re
import statistics
import sys

ALARM_RE = re.compile(r"a = \(([^)]*)\)")
LIMIT_DPS2 = 70.0


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def fnum(row, key):
    """Float value of a column, or None when absent/unparseable."""
    v = row.get(key)
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def column(rows, key):
    return [v for v in (fnum(r, key) for r in rows) if v is not None]


def describe(name, vals, limit=None):
    if not vals:
        print(f"  {name:<26} (absent)")
        return
    over = f"  over {limit:.0f}: {sum(1 for v in vals if abs(v) > limit):>4}/{len(vals)}" if limit else ""
    print(
        f"  {name:<26} max {max(vals):8.1f}  p95 {percentile(vals, 95):8.1f}  "
        f"median {statistics.median(vals):8.1f}{over}"
    )


def percentile(vals, p):
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def parse_alarms(path):
    """Peak |acceleration| per SpeedJ alarm in the launch log.

    The controller prints 6 accelerations but its format string only has 5
    slots per tuple, so the 6th value spills into the first slot of the
    'limit' tuple. Both tuples are parsed and every value that is not exactly
    the limit is treated as an acceleration.
    """
    peaks = []
    with open(path, errors="replace") as f:
        for line in f:
            if "Acceleration is over max value" not in line:
                continue
            groups = re.findall(r"\(([^)]*)\)", line)
            vals = []
            for g in groups:
                for tok in g.split(","):
                    tok = tok.strip()
                    if not tok or "deg" in tok:
                        continue
                    try:
                        v = float(tok)
                    except ValueError:
                        continue
                    if abs(abs(v) - LIMIT_DPS2) > 1e-6:  # drop the real limits
                        vals.append(v)
            if vals:
                peaks.append(max(abs(v) for v in vals))
    return peaks


def correlation(a, b):
    """Pearson correlation, or nan if either series is constant."""
    if len(a) < 3:
        return float("nan")
    ma, mb = statistics.mean(a), statistics.mean(b)
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    da = sum((x - ma) ** 2 for x in a) ** 0.5
    db = sum((y - mb) ** 2 for y in b) ** 0.5
    return num / (da * db) if da * db else float("nan")


def report_convergence(rows, pos_tol_m=0.005, rot_tol_deg=2.0):
    """Position AND orientation convergence, side by side.

    Orientation is reported separately because is_on_target() in
    reactive_controller.py gates on POSITION ONLY — a run can be declared
    on-target, and an action ended, with the tool pointing anywhere. Reading
    only fk_err_m therefore hides a whole failure mode.

    Uses fk_rot_err_deg (real FK geodesic angle). Older CSVs carry
    pose_rot_err_rad instead, which is mis-scaled by ~1e-3 and quantised to
    ~16 distinct values per run — refuse to report it rather than print a
    number that means nothing.
    """
    pos = column(rows, "fk_err_m")
    rot = column(rows, "fk_rot_err_deg")
    print("\n=== Convergence (position vs orientation) ===")
    if pos:
        print(f"  position     start {pos[0]:7.4f} m    min {min(pos):7.4f} m    final {pos[-1]:7.4f} m")
    if not rot:
        print("  orientation  NOT RECORDED in this CSV (no fk_rot_err_deg column).")
        if column(rows, "pose_rot_err_rad"):
            print("               pose_rot_err_rad is present but unusable (mis-scaled"
                  " ~1e-3, ~16 quantisation steps). Rebuild curobo_ros and re-run.")
        return
    print(f"  orientation  start {rot[0]:7.2f} deg  min {min(rot):7.2f} deg  final {rot[-1]:7.2f} deg")

    pos_ok = bool(pos) and pos[-1] < pos_tol_m
    rot_ok = rot[-1] < rot_tol_deg
    if pos_ok and not rot_ok:
        print(f"\n  !! POSITION CONVERGED, ORIENTATION DID NOT ({rot[-1]:.2f} deg > {rot_tol_deg} deg).")
        print("     is_on_target() only checks position, so the action can end here")
        print("     with the tool mis-aimed and nothing in the feedback saying so.")
        if rot[-1] > min(rot) * 1.5 and min(rot) < rot_tol_deg:
            print(f"     Orientation reached {min(rot):.2f} deg then DRIFTED BACK — the arm")
            print("     traded rotation for position late in the approach.")
        else:
            print("     Orientation never converged at all. Suspect, in order:")
            print("       - cspace_cfg squared_l2_regularization_weight[0] (velocity):")
            print("         damped-least-squares sacrifices the ill-conditioned")
            print("         direction first, which near a wrist singularity IS the")
            print("         orientation. Set it back to 0.0 to test.")
            print("       - the IK anchor: _ensure_ik_solver uses")
            print("         orientation_tolerance=0.05 rad (2.9 deg), so the anchor")
            print("         it hands MPPI is itself only accurate to ~3 deg.")
    elif pos_ok and rot_ok:
        print("\n  Both converged.")


# Arm joint limits (rad), M1013 as declared in leeloo.urdf. Only the bounded
# pair matters in practice: j1/j2/j4/j6 span +-360 deg and are never the
# binding constraint.
JOINT_LIMITS_RAD = [6.2832, 6.2832, 2.35619, 6.2832, 2.35619, 6.2832]
# Same value as the planner's constraint_cfg activation_distance and
# run_pose_matrix's --limit-margin: inside this band the constraint is pushing.
LIMIT_BAND_RAD = 0.20


def report_joint_limits(rows):
    """Closest approach of each joint to its bound over the run.

    Needed because con_cspace_bound cannot answer this: it is the summed cspace
    constraint over all five terms (position, velocity, acceleration, jerk,
    torque) and is dominated by the jerk violation on essentially every step,
    so it is non-zero whether or not a position bound was ever approached.
    """
    q = {i: column(rows, f"q_j{i}_deg") for i in range(1, 7)}
    if not all(q.values()):
        print("\n=== Joint limits ===")
        print("  NOT RECORDED in this CSV (no q_j*_deg columns). Rebuild"
              " curobo_ros and re-run.")
        return
    print("\n=== Joint limits (closest approach) ===")
    worst = []
    for i in range(1, 7):
        lim_deg = math.degrees(JOINT_LIMITS_RAD[i - 1])
        head = min(lim_deg - abs(v) for v in q[i])
        band = math.degrees(LIMIT_BAND_RAD)
        mark = ""
        if head <= 0:
            mark = "  <-- AT/BEYOND THE LIMIT"
        elif head < band:
            mark = "  <-- inside the constraint band"
        print(f"  j{i}: limit +-{lim_deg:6.1f} deg, closest approach "
              f"{head:7.2f} deg of headroom{mark}")
        if head < band:
            worst.append((i, head))
    if worst:
        i, head = min(worst, key=lambda t: t[1])
        print(f"\n  !! Joint {i} came within {head:.2f} deg of its bound.")
        if head <= 0:
            print("     It REACHED the stop. The hard constraint in constraint_cfg")
            print("     did not hold it; nothing downstream can fix that, the joint")
            print("     was already there when the next window was committed.")
        print("     Two distinct causes, and they need different fixes:")
        print("       - the GOAL sits near the bound: filter the pose set")
        print("         (run_pose_matrix --limit-margin);")
        print("       - the goal is legal but the arm drifts there in transit:")
        print("         that is unconstrained null-space motion, and the lever is")
        print("         cspace_cfg's squared_l2_regularization_weight, not the")
        print("         constraint. Compare the goal's own joint value to decide.")


def report_nullspace(rows, active_dps=10.0):
    """Flag joint motion that cancels itself out — i.e. lives in the Jacobian's
    null space and moves the tool not at all.

    The M1013 has a spherical wrist (axes 4-5-6), so axes 4 and 6 become
    collinear as axis 5 nears zero: any (j4, -j4) pair is then exactly a
    null-space motion. It is invisible to tool_pose_cfg (it does not move the
    tool), so with no velocity regularization nothing penalizes it and MPPI
    samples it freely. Measured 2026-08-07 before the fix: corr(j4, j6) =
    -0.999 with 92% of j4 cancelled, peaking at +46.7/-46.5 deg/s of pure
    thrash. See squared_l2_regularization_weight in mpc_planner.py.

    Only steps above active_dps are considered: near standstill the velocities
    are noise and their correlation is meaningless.
    """
    try:
        j = {i: column(rows, f"vfirst_j{i}_dps") for i in range(1, 7)}
    except Exception:
        return
    if not all(j.values()):
        return
    idx = [k for k in range(len(rows))
           if max(abs(j[i][k]) for i in range(1, 7)) > active_dps]
    print(f"\n=== Null-space check ({len(idx)} steps above {active_dps:.0f} deg/s) ===")
    if len(idx) < 3:
        print("  too few moving steps to judge - run a longer or faster motion.")
        return
    worst = None
    for a, b in ((4, 6), (1, 5), (3, 6), (1, 3), (2, 5)):
        va = [j[a][k] for k in idx]
        vb = [j[b][k] for k in idx]
        c = correlation(va, vb)
        # Cancellation only means anything for anti-correlated pairs.
        mean_abs = statistics.mean(abs(x) for x in va)
        mean_sum = statistics.mean(abs(x + y) for x, y in zip(va, vb))
        cancel = (1.0 - mean_sum / mean_abs) * 100.0 if mean_abs > 1e-6 else 0.0
        print(f"  corr(j{a},j{b}) = {c:6.3f}   |j{a}| mean {mean_abs:5.1f} deg/s, "
              f"cancelled by j{b}: {cancel:5.1f}%")
        if c < -0.9 and cancel > 50.0 and (worst is None or cancel > worst[2]):
            worst = (a, b, cancel, mean_abs)
    if worst:
        a, b, cancel, mean_abs = worst
        # Anti-correlation alone does NOT mean the optimizer is misbehaving.
        # Counting sign changes separates the two cases that look identical in
        # the correlation number but call for opposite responses:
        #   - a smooth ramp with no sign flips is the wrist reconfiguring
        #     through the null space, plausibly a manoeuvre the arm NEEDS;
        #   - sign flips are real chatter, i.e. sampling noise surviving into
        #     the command.
        # Getting this wrong cost a wasted tuning round on 2026-08-07.
        va = [j[a][k] for k in idx]
        flips = sum(1 for p, q in zip(va, va[1:]) if (p >= 0) != (q >= 0))
        print(f"\n  j{a} and j{b} are anti-correlated and {cancel:.0f}% cancelling: "
              f"{mean_abs:.0f} deg/s of\n     joint motion producing no tool motion. "
              "Near-singular configuration\n     (wrist singularity if this is j4/j6, "
              "i.e. axis 5 near zero).")
        print(f"     j{a} sign changes over the moving phase: {flips}/{len(idx)} steps.")
        if flips <= max(1, len(idx) // 10):
            print("\n     -> SMOOTH RECONFIGURATION, not thrash. A sustained one-way")
            print("        counter-rotation is the wrist walking through the null space")
            print("        to change posture. Do NOT damp it with")
            print("        squared_l2_regularization_weight - that suppresses a")
            print("        manoeuvre the arm may need to reach the goal ORIENTATION.")
            print("        If it is genuinely unnecessary, the fix is upstream: the IK")
            print("        seed / arm posture, not the cost weights.")
        else:
            print("\n     -> CHATTER: the sign flips make this sampling noise reaching")
            print("        the command. Damp it with squared_l2_regularization_weight,")
            print("        index 1 (ACCELERATION) rather than index 0 (velocity):")
            print("        acceleration penalizes the oscillation, velocity penalizes")
            print("        the useful approach speed along with it.")
        print("\n     Before changing any weight, check its authority: compare")
        print("     cost_cspace against cost_tool_pose_pos over the MOVING steps.")
        print("     A term two decades below the pose cost there cannot damp anything,")
        print("     however sensible its value looks.")
    else:
        print("  no cancelling pair - joint motion is producing tool motion.")


def report_matrix(paths):
    """Compare one or more pose_matrix_summary CSVs from run_pose_matrix.

    A single campaign is a report card; two are a regression test. Poses are
    split by `near_singularity`, because that split is the whole reason the
    matrix exists — an aggregate pass rate hides the case that actually varies.
    """
    campaigns = []
    for path in paths:
        rows = read_csv(path)
        if not rows:
            print(f"  {path}: empty")
            continue
        campaigns.append((os.path.basename(path), rows))
    if not campaigns:
        sys.exit("no usable pose_matrix_summary CSV")

    for name, rows in campaigns:
        print(f"\n=== Pose matrix: {name}  ({len(rows)} poses) ===")
        print("  idx  |j5|goal transit  conv  t_conv   pos_err   rot_err  hold")
        for r in rows:
            near = "*" if str(r.get("near_singularity", "0")).strip() in ("1", "True") else " "
            conv = "yes" if str(r.get("converged", "0")).strip() in ("1", "True") else "NO "
            print(f"  {r.get('index',''):>3}{near} "
                  f"{fnum(r,'j5_rad') or float('nan'):7.3f} "
                  f"{fnum(r,'transit_min_j5_rad') or float('nan'):7.3f} "
                  f"  {conv} "
                  f"{fnum(r,'time_to_converge_s') or float('nan'):7.2f}s "
                  f"{fnum(r,'final_pos_err_m') or float('nan'):8.4f}m "
                  f"{fnum(r,'final_rot_err_deg') or float('nan'):7.2f}d "
                  f"{r.get('hold_count',''):>4}")
        print("  (* = near the wrist singularity, at the goal or in transit)")

        groups = {"near singularity": [], "well-conditioned": []}
        for r in rows:
            key = ("near singularity"
                   if str(r.get("near_singularity", "0")).strip() in ("1", "True")
                   else "well-conditioned")
            groups[key].append(r)
        print()
        for key, grp in groups.items():
            if not grp:
                continue
            conv = sum(1 for r in grp
                       if str(r.get("converged", "0")).strip() in ("1", "True"))
            rot = [fnum(r, "final_rot_err_deg") for r in grp]
            rot = [v for v in rot if v is not None]
            pos = [fnum(r, "final_pos_err_m") for r in grp]
            pos = [v for v in pos if v is not None]
            print(f"  {key:<18}: {conv}/{len(grp)} converged"
                  + (f", median pos {statistics.median(pos):.4f}m" if pos else "")
                  + (f", median rot {statistics.median(rot):.2f}deg" if rot else ""))

    if len(campaigns) < 2:
        return
    print("\n=== Regression: first vs last campaign ===")
    base, head = campaigns[0], campaigns[-1]
    by_idx = {r.get("index"): r for r in base[1]}
    for r in head[1]:
        b = by_idx.get(r.get("index"))
        if b is None:
            continue
        for field, unit, fmt in (("final_pos_err_m", "m", "{:+.4f}"),
                                 ("final_rot_err_deg", "deg", "{:+.2f}")):
            new, old = fnum(r, field), fnum(b, field)
            if new is None or old is None:
                continue
            delta = new - old
            # Only call out changes big enough to be a real move rather than
            # run-to-run scatter: 1 mm and 0.5 deg.
            thresh = 0.001 if unit == "m" else 0.5
            if abs(delta) > thresh:
                arrow = "WORSE" if delta > 0 else "better"
                print(f"  pose {r.get('index'):>3} {field:<18} "
                      f"{fmt.format(delta)}{unit}  {arrow}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mpc_csv", nargs="?", help="mpc_diag_*.csv (newest if omitted)")
    ap.add_argument("--speedj", help="speedj_publish_*.csv")
    ap.add_argument("--log", help="launch log containing the SpeedJ alarms")
    ap.add_argument("--matrix", nargs="+", metavar="CSV",
                    help="pose_matrix_summary_*.csv from run_pose_matrix; pass two "
                         "(oldest first) to diff two campaigns")
    ap.add_argument("--interval", type=float, default=0.24,
                    help="mpc_command_interval in seconds, used as the solve-time budget "
                         "for CSVs recorded before cmd_interval_ms existed (default 0.24)")
    ap.add_argument("--csv-prefix", default="mpc_diag",
                    help="auto-discovery glob prefix when mpc_csv is omitted, e.g. "
                         "'lbfgs_diag' for LBFGSController's CSVs (default 'mpc_diag'). "
                         "Note: this script's column set (accel_win/accel_boundary/vbc/"
                         "vexec/vfirst/vlast) is MPCController's per-horizon schema -- "
                         "record_tick()'s one-point-per-call CSV (lbfgs_planner.py) uses "
                         "different column names, so most fields will report '(absent)'; "
                         "this flag only helps you find the file, not a schema translation.")
    args = ap.parse_args()

    if args.matrix:
        report_matrix(args.matrix)
        if not args.mpc_csv:
            return

    mpc_path = args.mpc_csv or (
        sorted(glob.glob(f"**/{args.csv_prefix}_*.csv", recursive=True)) or [None])[-1]
    if not mpc_path:
        sys.exit(f"no {args.csv_prefix}_*.csv found - run with the matching debug param true first")

    rows = read_csv(mpc_path)
    print(f"=== MPC diagnostics: {mpc_path}  ({len(rows)} steps) ===")
    if not rows:
        sys.exit("CSV is empty - was a goal actually executed?")

    win = column(rows, "accel_win_max_dps2")
    bound = column(rows, "accel_boundary_dps2")
    describe("accel_win_max_dps2", win, LIMIT_DPS2)
    describe("accel_boundary_dps2", bound, LIMIT_DPS2)
    describe("vbc_max_dps", column(rows, "vbc_max_dps"))
    describe("vfirst_max_dps", column(rows, "vfirst_max_dps"))
    describe("vexec_max_dps", column(rows, "vexec_max_dps"))
    describe("vlast_max_dps", column(rows, "vlast_max_dps"))
    describe("solve_ms", column(rows, "solve_ms"))
    describe("dt_step_ms", column(rows, "dt_step_ms"))
    report_convergence(rows)
    report_joint_limits(rows)

    # Real-time budget: the paced servo loop can only send a fresh plan if the
    # solve finishes inside mpc_command_interval. When solve_ms creeps up to it
    # the producer starves, ticks are missed and convergence degrades — that is
    # what mpc_warm_start_iters=10 did on 2026-08-07 (105ms -> 199ms median).
    # The budget is the SEND interval (mpc_command_interval), not dt_step_ms:
    # in paced mode the producer loop period is structurally ~solve_ms, so
    # comparing those two flags every healthy run.
    solve = column(rows, "solve_ms")
    interval = column(rows, "cmd_interval_ms")
    budget = statistics.median(interval) if interval else args.interval * 1000.0
    if solve and budget > 0:
        msolve = statistics.median(solve)
        if msolve > 0.6 * budget:
            print(f"\n  !! solve_ms median {msolve:.0f}ms vs send interval {budget:.0f}ms: "
                  "under 40% margin.\n"
                  "     Reduce mpc_warm_start_iters / mpc_mppi_num_particles, or raise\n"
                  "     mpc_command_interval, before reading anything else here.")
        if not interval:
            print(f"  (send interval assumed {budget:.0f}ms; pass --interval, or re-record "
                  "to get cmd_interval_ms)")

    report_nullspace(rows)

    srows = []
    if args.speedj:
        srows = read_csv(args.speedj)
        print(f"\n=== JointSpeedStrategy publishes: {args.speedj}  ({len(srows)} sends) ===")
        describe("point0_vs_real_accel_dps2", column(srows, "point0_vs_real_accel_dps2"), LIMIT_DPS2)
        describe("intra_batch_accel_dps2", column(srows, "intra_batch_accel_max_dps2"), LIMIT_DPS2)
        describe("real_vel_max_dps", column(srows, "real_vel_max_dps"))
        describe("point0_max_dps", column(srows, "point0_max_dps"))
        describe("unclamped_point0_max_dps", column(srows, "unclamped_point0_max_dps"))
        active = sum(1 for r in srows if (r.get("clamp_active_j") or "[]") != "[]")
        print(f"  {'clamp engaged on':<26} {active}/{len(srows)} sends")

    if args.log:
        # Absolute path, always: `--log log.txt` run from the workspace root once
        # silently picked up a stale /home/ros2_ws/log.txt instead of the live
        # src/log.txt and reported "none" while 16 alarms were being logged.
        log_path = os.path.abspath(args.log)
        peaks = parse_alarms(log_path)
        print(f"\n=== SpeedJ alarms in {log_path} ===")
        if peaks:
            print(f"  {len(peaks)} alarm(s), peak |a| max {max(peaks):.1f} "
                  f"median {statistics.median(peaks):.1f} deg/s^2 (limit {LIMIT_DPS2:.0f})")
        else:
            print("  none")
            with open(log_path, errors="replace") as f:
                text = f.read()
            if "curobo_trajectory_planner" not in text:
                print("  !! This log has no curobo_trajectory_planner output at all -- it is")
                print("     almost certainly the wrong or a stale file. 'none' proves nothing.")
            for other in ("log.txt", "src/log.txt", "../log.txt"):
                cand = os.path.abspath(other)
                if cand != log_path and os.path.isfile(cand) and parse_alarms(cand):
                    print(f"  !! {cand} DOES contain alarms -- did you mean that one?")

    print("\n=== Verdict ===")
    if not win or not bound:
        print("  Inconclusive: the CSV lacks the accel columns.")
        return
    mw, mb = max(win), max(bound)

    # Checked FIRST: a saturated velocity boundary condition produces a large
    # accel_boundary_dps2 that looks exactly like a batch seam but has nothing
    # to do with the pacing, so testing "boundary > window" first sends you off
    # tuning mpc_command_interval for a discontinuity it cannot fix (it did, on
    # 2026-08-07). The tell is vbc_max_dps pinned at one constant while the
    # plan's last point sits far above it: the MPC is being told it is slower
    # than the arm really is, and replans an acceleration every cycle.
    vbc = column(rows, "vbc_max_dps")
    vlast_col = column(rows, "vlast_max_dps")
    vfirst_col = column(rows, "vfirst_max_dps")
    real = column(srows, "real_vel_max_dps") if srows else []
    if vbc and vlast_col:
        cap = max(vbc)
        pinned = sum(1 for v in vbc if abs(v - cap) < 1e-6)
        # Pinning at the cap is only pathological when the ARM is meanwhile
        # slower than the cap: that is the cap feeding the MPC a velocity the
        # arm has already exceeded. If the arm really does reach the cap, the
        # feedback is simply tracking fast motion (observed 2026-08-07: pinned
        # at 30.0 on 7/128 steps while the arm reached 31.6 deg/s -- benign).
        arm_below_cap = bool(real) and max(real) < cap * 0.9
        if pinned >= 3 and (arm_below_cap or not real):
            detail = (f"the arm only reached {max(real):.1f} deg/s"
                      if real else "no speedj CSV to compare the real velocity against")
            print(f"  VELOCITY BOUNDARY CAP: vbc_max_dps pinned at {cap:.1f} deg/s on "
                  f"{pinned}/{len(vbc)} steps\n  while {detail}. The MPC is replanning from a "
                  "velocity the arm has already\n  exceeded, so the plan-to-plan discontinuity is "
                  "the CAP, not the pacing --\n  shortening mpc_command_interval will not fix it.\n"
                  "  -> raise _VBC_CAP_DPS above the working range, and sample the feedback at\n"
                  "     the executed plan point rather than the last (mpc_planner.step()).")
            return

    # Magnitudes steady but the boundary still jumps => the velocity VECTOR is
    # being redistributed across joints between solves, not ramped. That is MPPI
    # re-solve noise; pacing and the cap are both irrelevant to it.
    if vfirst_col and vlast_col and mb > LIMIT_DPS2:
        mf, ml = statistics.median(vfirst_col), statistics.median(vlast_col)
        if max(mf, ml) > 0 and abs(mf - ml) / max(mf, ml) < 0.15:
            print(f"  PLAN-TO-PLAN REDISTRIBUTION: median vfirst {mf:.1f} ~= vlast {ml:.1f} deg/s,\n"
                  f"  so the plan's speed is steady, yet the per-joint boundary jump reaches "
                  f"{mb:.1f}\n  deg/s^2. Successive MPPI solves are splitting the same motion "
                  "differently\n  across joints.\n"
                  "  -> this is solver noise, not pacing: try more particles\n"
                  "     (mpc_mppi_num_particles) or more warm-start iterations.")
            return

    if mb > LIMIT_DPS2 and mb > mw:
        print(f"  BATCH SEAM: boundary {mb:.1f} > window {mw:.1f} deg/s^2 and over the "
              f"{LIMIT_DPS2:.0f} limit,\n  with no cap saturation to explain it.\n"
              "  -> shorten mpc_command_interval so fewer points drift from the measured\n"
              "     velocity before the next batch resets the ramp, and re-measure.")
    elif mw > LIMIT_DPS2:
        print(f"  INTRA-PLAN: window {mw:.1f} > boundary {mb:.1f} deg/s^2.\n"
              "  -> the seam is not the driver; look at MPPI plan noise\n"
              "     (mpc_mppi_num_particles, mpc_warm_start_iters) instead.")
    else:
        print(f"  Neither exceeds {LIMIT_DPS2:.0f} deg/s^2 (window {mw:.1f}, boundary {mb:.1f}).\n"
              "  The over-limit acceleration is then created downstream of curobo_ros —\n"
              "  suspect transport latency between the command and the measured velocity.")


if __name__ == "__main__":
    main()
