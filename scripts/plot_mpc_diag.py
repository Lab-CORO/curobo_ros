#!/usr/bin/env python3
"""Plot + auto-analyze an MPC diagnostic CSV (mpc_diag_*.csv).

Usage:
    python3 plot_mpc_diag.py <csv> [--compare <csv2>] [--out-dir DIR]
        [--convergence-threshold 0.01] [--accel-limit 70]

Outputs (next to the input CSV, or --out-dir):
    <name>_raw.png       -- all raw columns vs time, grouped by panel
    <name>_analysis.png  -- rendered text report
    <name>_report.txt    -- same report, plain text
    <name>_compare.png   -- only if --compare is given

No display required (Agg backend) -- safe to run on a headless Jetson.
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Sourced from curobo_ros/planners/mpc_planner.py: the MPPI boundary-velocity
# feedback cap, and the Doosan SpeedJ acceleration limit seen tripping alarms
# on real hardware (dsr_controller2 "[SpeedJ] Acceleration is over max value").
VBC_CAP_DPS_DEFAULT = 5.0
ACCEL_LIMIT_DPS2_DEFAULT = 70.0
CONVERGENCE_THRESHOLD_DEFAULT = 0.01  # meters, matches mpc_planner.py default


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    for c in df.columns:
        if c != "t_s":
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def _joint_cols(df, prefix):
    return [c for c in df.columns if c.startswith(prefix) and c.endswith("_dps")]


def plot_raw(df: pd.DataFrame, out_path: str, accel_limit: float, vbc_cap: float,
             convergence_threshold: float):
    t = df["t_s"]
    fig, axes = plt.subplots(8, 1, figsize=(13, 26), sharex=True)

    ax = axes[0]
    ax.set_yscale("log")
    ax.plot(t, df["fk_err_m"], label="fk_err_m (real FK)", color="tab:red")
    ax.plot(t, df["pose_pos_err_m"], label="pose_pos_err_m (solver)", color="tab:blue")
    ax.axhline(convergence_threshold, color="gray", linestyle="--", linewidth=1,
               label=f"convergence threshold ({convergence_threshold}m)")
    ax.set_ylabel("position error (m, log)")
    ax2 = ax.twinx()
    ax2.plot(t, df["pose_rot_err_rad"], label="pose_rot_err_rad", color="tab:green", alpha=0.6)
    ax2.set_ylabel("orientation error (rad)")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)
    ax.set_title("Convergence: real FK error vs solver's own pose error")

    ax = axes[1]
    ax.plot(t, df["vfirst_max_dps"], label="vfirst_max_dps")
    ax.plot(t, df["vlast_max_dps"], label="vlast_max_dps")
    ax.plot(t, df["vbc_max_dps"], label="vbc_max_dps")
    ax.axhline(vbc_cap, color="gray", linestyle="--", linewidth=1, label=f"vbc cap ({vbc_cap} dps)")
    ax.set_ylabel("deg/s")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Global commanded velocities")

    ax = axes[2]
    for c in _joint_cols(df, "vfirst_j"):
        ax.plot(t, df[c], label=c, linewidth=1)
    ax.set_ylabel("deg/s")
    ax.legend(loc="upper right", fontsize=7, ncol=3)
    ax.set_title("Per-joint velocity, first horizon point (vfirst)")

    ax = axes[3]
    for c in _joint_cols(df, "vlast_j"):
        ax.plot(t, df[c], label=c, linewidth=1)
    ax.set_ylabel("deg/s")
    ax.legend(loc="upper right", fontsize=7, ncol=3)
    ax.set_title("Per-joint velocity, last horizon point (vlast)")

    ax = axes[4]
    ax.plot(t, df["accel_win_max_dps2"], label="accel_win_max_dps2")
    ax.plot(t, df["accel_boundary_dps2"], label="accel_boundary_dps2")
    ax.axhline(accel_limit, color="red", linestyle="--", linewidth=1,
               label=f"SpeedJ limit ({accel_limit} deg/s²)")
    ax.set_ylabel("deg/s²")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Accelerations (intra-window and boundary discontinuity)")

    ax = axes[5]
    ax.set_yscale("log")
    ax.plot(t, df["dt_step_ms"], label="dt_step_ms (loop period)")
    ax.plot(t, df["solve_ms"], label="solve_ms")
    ax.set_ylabel("ms (log)")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Timing")

    ax = axes[6]
    ax.plot(t, df["con_self_collision"], label="con_self_collision")
    ax.plot(t, df["con_scene_collision"], label="con_scene_collision")
    ax.plot(t, df["con_cspace_bound"], label="con_cspace_bound")
    ax.set_ylabel("constraint violation")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Constraints (0 = no violation)")

    ax = axes[7]
    cost_cols = ["cost_tool_pose_pos", "cost_tool_pose_orient", "cost_cspace"]
    if all(df[c].isna().all() for c in cost_cols):
        ax.text(0.5, 0.5, "cost_* columns unavailable\n"
                           "(weighted cost magnitudes removed from the safe path "
                           "after a CUDA crash, cf. debug 2026-07-20 -- "
                           "use the dedicated graph-free rollout tool to recover them)",
                ha="center", va="center", transform=ax.transAxes, fontsize=10, color="gray")
        ax.set_yticks([])
    else:
        for c in cost_cols:
            ax.plot(t, df[c], label=c)
        ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Cost terms (weighted, horizon-summed)")
    ax.set_xlabel("t_s")

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def analyze(df: pd.DataFrame, convergence_threshold: float, accel_limit: float,
            vbc_cap: float) -> str:
    lines = []

    def section(title, verdict, body_lines):
        lines.append(f"\n{verdict} {title}")
        for b in body_lines:
            lines.append(f"    {b}")

    n = len(df)
    fk_start, fk_end, fk_min = df.fk_err_m.iloc[0], df.fk_err_m.iloc[-1], df.fk_err_m.min()
    converged = fk_end < convergence_threshold
    half = df.iloc[n // 2:]
    plateau = None
    if len(half) >= 3:
        slope = np.polyfit(half.t_s, half.fk_err_m, 1)[0]  # m/s
        if not converged and abs(slope) < 0.002:
            plateau = half.fk_err_m.mean()
    body = [f"fk_err_m: start={fk_start:.4f}m -> end={fk_end:.4f}m (min={fk_min:.4f}m)",
            f"threshold: {convergence_threshold}m"]
    if converged:
        verdict = "[OK]"
        body.append("Converged.")
    elif plateau is not None:
        verdict = "[WARN]"
        body.append(f"Plateau detected (~{plateau:.4f}m, not decreasing) -- did not converge.")
    else:
        verdict = "[WARN]"
        body.append("Not converged, still decreasing at end of log (run ended early / cancelled?).")
    section("Convergence", verdict, body)

    if df.pose_pos_err_m.mean() > 1e-9:
        ratio = df.fk_err_m.mean() / df.pose_pos_err_m.mean()
        body = [f"mean(fk_err_m)={df.fk_err_m.mean():.5f}m  "
                f"mean(pose_pos_err_m)={df.pose_pos_err_m.mean():.6f}m  ratio={ratio:.0f}x"]
        if ratio > 10:
            verdict = "[WARN]"
            body.append("Solver believes it is near-converged (its own pose error is tiny) while "
                        "the REAL end-effector (FK) error is far larger -> the state fed back to "
                        "the solver (or the executed command) disagrees with reality. Check "
                        "_close_state_loop / _state_from_action and execution pacing.")
        else:
            verdict = "[OK]"
            body.append("Solver's own pose error tracks the real FK error reasonably.")
        section("Solver <-> real-world agreement", verdict, body)

    pct_self = 100.0 * (df.con_self_collision > 0).mean()
    pct_scene = 100.0 * (df.con_scene_collision > 0).mean()
    body = [f"self_collision > 0 in {pct_self:.0f}% of steps (max={df.con_self_collision.max():.2f})",
            f"scene_collision > 0 in {pct_scene:.0f}% of steps (max={df.con_scene_collision.max():.2f})"]
    if pct_self > 50 or pct_scene > 5:
        verdict = "[WARN]"
        body.append("The arm operates persistently close to (or inside) a collision constraint "
                    "boundary -- this can suppress motion or cause erratic corrections.")
    elif pct_self > 0 or pct_scene > 0:
        verdict = "[INFO]"
    else:
        verdict = "[OK]"
    section("Collision constraints", verdict, body)

    pct_bd = 100.0 * (df.accel_boundary_dps2 > accel_limit).mean()
    pct_win = 100.0 * (df.accel_win_max_dps2 > accel_limit).mean()
    body = [f"accel_boundary_dps2 > {accel_limit}: {pct_bd:.0f}% of steps "
            f"(max={df.accel_boundary_dps2.max():.1f})",
            f"accel_win_max_dps2  > {accel_limit}: {pct_win:.0f}% of steps "
            f"(max={df.accel_win_max_dps2.max():.1f})"]
    if pct_bd > 0:
        verdict = "[WARN]"
        body.append("Expect real-hardware 'SpeedJ Acceleration is over max value' alarms at these "
                    "boundary discontinuities (batch-to-batch re-solve jumps).")
    else:
        verdict = "[OK]"
    section("Accelerations vs SpeedJ limit", verdict, body)

    med = df.dt_step_ms.median()
    pauses = df[df.dt_step_ms > 3 * med]
    body = [f"solve_ms: mean={df.solve_ms.mean():.1f}ms max={df.solve_ms.max():.1f}ms",
            f"dt_step_ms median={med:.1f}ms"]
    if len(pauses) > 0:
        verdict = "[INFO]"
        body.append(f"{len(pauses)} pause(s) > 3x median at t_s={list(np.round(pauses.t_s.values, 2))}"
                     " (cold start / perception refresh / re-goal are common causes).")
    else:
        verdict = "[OK]"
    section("Timing", verdict, body)

    pct_cap = 100.0 * np.isclose(df.vbc_max_dps, vbc_cap, atol=0.05).mean()
    body = [f"vbc_max_dps at cap ({vbc_cap} dps) in {pct_cap:.0f}% of steps"]
    verdict = "[INFO]" if pct_cap > 0 else "[OK]"
    section("Velocity-continuity cap saturation", verdict, body)

    nan_cols = [c for c in df.columns if df[c].isna().all()]
    if nan_cols:
        section("Data availability", "[INFO]",
                 [f"All-NaN columns: {', '.join(nan_cols)}",
                  "cost_tool_pose_*/cost_cspace are expected NaN: removed from the safe "
                  "extraction path after a CUDA device-side-assert crash (cf. debug 2026-07-20)."])

    return "\n".join(lines).strip() + "\n"


def render_report_png(report: str, out_path: str):
    fig = plt.figure(figsize=(11, max(6, 0.28 * report.count("\n") + 1)))
    fig.text(0.02, 0.98, report, family="monospace", fontsize=9, va="top", ha="left")
    plt.axis("off")
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def plot_compare(df1, df2, label1, label2, out_path):
    cols = ["fk_err_m", "accel_boundary_dps2", "con_scene_collision"]
    fig, axes = plt.subplots(len(cols), 1, figsize=(12, 3 * len(cols)), sharex=False)
    for ax, c in zip(axes, cols):
        ax.plot(df1.t_s, df1[c], label=label1)
        ax.plot(df2.t_s, df2[c], label=label2)
        ax.set_title(c)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", help="Path to mpc_diag_*.csv")
    p.add_argument("--compare", metavar="CSV2", help="Second CSV to overlay for comparison")
    p.add_argument("--out-dir", default=None, help="Output directory (default: alongside the CSV)")
    p.add_argument("--convergence-threshold", type=float, default=CONVERGENCE_THRESHOLD_DEFAULT)
    p.add_argument("--accel-limit", type=float, default=ACCEL_LIMIT_DPS2_DEFAULT)
    p.add_argument("--vbc-cap", type=float, default=VBC_CAP_DPS_DEFAULT)
    args = p.parse_args()

    if not os.path.isfile(args.csv):
        print(f"CSV not found: {args.csv}", file=sys.stderr)
        sys.exit(1)

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.csv))
    base = os.path.splitext(os.path.basename(args.csv))[0]
    os.makedirs(out_dir, exist_ok=True)

    df = load_csv(args.csv)

    raw_png = os.path.join(out_dir, f"{base}_raw.png")
    plot_raw(df, raw_png, args.accel_limit, args.vbc_cap, args.convergence_threshold)
    print(f"Raw plots -> {raw_png}")

    report = analyze(df, args.convergence_threshold, args.accel_limit, args.vbc_cap)
    print(report)
    report_txt = os.path.join(out_dir, f"{base}_report.txt")
    with open(report_txt, "w") as f:
        f.write(report)
    print(f"Report -> {report_txt}")

    analysis_png = os.path.join(out_dir, f"{base}_analysis.png")
    render_report_png(report, analysis_png)
    print(f"Analysis figure -> {analysis_png}")

    if args.compare:
        df2 = load_csv(args.compare)
        base2 = os.path.splitext(os.path.basename(args.compare))[0]
        compare_png = os.path.join(out_dir, f"{base}_vs_{base2}_compare.png")
        plot_compare(df, df2, base, base2, compare_png)
        print(f"Comparison -> {compare_png}")


if __name__ == "__main__":
    main()
