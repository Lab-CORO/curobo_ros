#!/usr/bin/env python3
"""Optuna auto-tuner for the LBFGS MPC cost weights (config/mpc/lbfgs_mpc.yaml),
driving the FULL bringup_leeloo.launch.py sim:=true (Gazebo + execute_trajectory
+ ros2_control) chain per trial -- not a standalone cuRobo solver.

Why the full chain and not a lighter standalone/emulator harness: a
pure-Python standalone MPC loop (see the earlier repro_lbfgs_ab.py) skips the
producer/consumer pacing race in reactive_controller.py::_execute_paced, the
real execution lag/state feedback of the actual robot bridge, and the
live-goal republish churn -- all real dynamics of the system actually being
tuned. It also can't be trusted to reproduce results measured on the real
node, since there is no live way to reload a cost YAML on an already-running
node (PlannerManager caches planner instances; see planner_factory.py). See
the plan this script was built from for the full reasoning.

Each Optuna trial writes one candidate lbfgs_mpc.yaml, then runs each
reference goal in its OWN fresh launch (see run_goal_in_fresh_launch's
docstring for why: goals sharing one launch back-to-back confound the
second goal's score with wherever the first goal left the robot). Per goal:
  1. Launches `ros2 launch leeloo bringup_leeloo.launch.py sim:=true
     enable_calibration:=false mpc_debug:=true mpc_lbfgs_config_file:=<cand>
     diag_dir:=<trial_dir>` as a subprocess, in its own ROS_DOMAIN_ID (fully
     isolated from any other ROS graph on this machine, including any real
     robot-network CycloneDDS peers) and its own process group.
  2. Waits for /curobo_trajectory_planner/set_planner to become available as
     the readiness signal -- NOT /unified_planner/set_planner, which is only
     the name under the integration test's own launch setup. Real measured
     startup here is ~15-20s.
  3. Selects the MPC planner, sends a SendTrajectory goal, lets it run for
     --sim-duration-s, cancels it -- the exact sequence curobo_ros's own
     generated integration tests use (test/test_test_planners.py:
     test_09_set_planner_mpc / test_10_execute_mpc_reactive).
  4. Parses the mpc_diag_*.csv the run itself wrote (mpc_debug:=true), using
     the exec_fk_err_m / exec_fk_rot_err_deg / horizon_churn_max_deg columns
     added to mpc_planner.py this session -- no extra ROS subscriber needed.
  5. Tears the node down (SIGINT the whole process group, SIGKILL fallback).

Cost: ~65-70s/goal-launch at the default --sim-duration-s (measured, not
estimated -- see the plan this script was built from). Two goals per trial,
so budget roughly 2x that per Optuna trial. GPU is shared with any other
CUDA process on this machine -- everything else (topics, services, DDS
discovery) is isolated via ROS_DOMAIN_ID and a dedicated loopback-only
CycloneDDS profile (tune_mpc_cyclonedds.xml) that keeps this harness off the
workstation's shared, Wi-Fi-bound, real-robot-network profile.

Usage:
    # Workspace must already be sourced (same convention as the other
    # scripts here -- see analyze_mpc_seam.py / plot_mpc_diag.py).
    python3 tune_mpc.py --n-trials 15
    python3 tune_mpc.py --n-trials 2 --sim-duration-s 6   # smoke test first
"""
import argparse
import copy
import csv
import glob
import math
import os
import signal
import statistics
import subprocess
import sys
import time

import yaml

DIVERGENCE_PENALTY = 1e6
CONVERGENCE_THRESHOLD_M = 0.01           # matches unified_planner_node.py's declared default
CONVERGENCE_THRESHOLD_RAD = 0.05         # matches unified_planner_node.py's declared default
CONVERGENCE_THRESHOLD_DEG = math.degrees(CONVERGENCE_THRESHOLD_RAD)
CHURN_TIEBREAK_SCALE = 0.01

# The two goals characterized this debugging session: one that converges
# cleanly (q_j5 stays 65-85 deg, well off the spherical-wrist singularity)
# and one that plateaus (q_j5 wanders -8..33 deg, near it). Grounding the
# search in real, previously-observed behavior rather than arbitrary poses.
DEFAULT_GOALS = [
    {
        "name": "converging_194406",
        "position": (0.3853, 0.3876, 0.3175),
        "orientation_wxyz": (0.6914, 0.0, 0.7225, -0.0),
    },
    {
        "name": "near_singular_173037",
        "position": (0.6006, 0.2910, 0.2935),
        "orientation_wxyz": (0.6820, 0.0, 0.7313, -0.0),
    },
]

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
BASE_LBFGS_YAML = os.path.join(
    os.path.dirname(__file__), "..", "config", "mpc", "lbfgs_mpc.yaml")
# The workstation's default CYCLONEDDS_URI (~/.cyclone/profile.xml) is tuned
# for the real multi-machine Leeloo network (multicast off, bound to the
# Wi-Fi interface, static robot-network peers) and does not reliably
# discover independent same-host processes. Every harness process (this one
# and each trial subprocess) uses this loopback-only profile instead -- see
# tune_mpc_cyclonedds.xml's own header comment.
CYCLONEDDS_PROFILE = os.path.join(os.path.dirname(__file__), "tune_mpc_cyclonedds.xml")


def load_base_yaml(path: str) -> dict:
    """Same loader mpc_planner.py::_load_mpc_config uses -- needed so
    "1.0e-3"-style floats already in lbfgs_mpc.yaml parse correctly instead
    of silently becoming strings (see that function's docstring)."""
    from curobo._src.util.config_io import Loader as _CUROBO_YAML_LOADER
    with open(path, "r") as f:
        return yaml.load(f, Loader=_CUROBO_YAML_LOADER)


def build_candidate_yaml(base: dict, params: dict, out_path: str) -> str:
    cfg = copy.deepcopy(base)
    cost_cfg = cfg["rollout"]["cost_cfg"]
    cost_cfg["tool_pose_cfg"]["weight"] = [params["pos_weight"], params["orient_weight"]]
    cost_cfg["tool_pose_cfg"]["_non_terminal_pose_axes_weight_factor"] = \
        [params["non_terminal_factor"]] * 6
    cost_cfg["cspace_cfg"]["squared_l2_regularization_weight"][0] = params["vel_damping"]
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        yaml.safe_dump(cfg, f, default_flow_style=None, sort_keys=False)
    return out_path


def launch_trial_node(candidate_yaml: str, trial_dir: str, ros_domain_id: int,
                       log_path: str) -> subprocess.Popen:
    cmd = [
        "ros2", "launch", "leeloo", "bringup_leeloo.launch.py",
        "sim:=true", "enable_calibration:=false", "gui:=false", "use_rviz:=False",
        "mpc_debug:=true",
        f"mpc_lbfgs_config_file:={candidate_yaml}",
        f"diag_dir:={trial_dir}",
    ]
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(ros_domain_id)
    env["CYCLONEDDS_URI"] = CYCLONEDDS_PROFILE
    logf = open(log_path, "w")
    return subprocess.Popen(
        cmd, cwd=REPO_ROOT, env=env, stdout=logf, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def terminate_process_group(proc: subprocess.Popen, timeout: float = 20.0):
    """SIGINT the whole process group first (the graceful path -- observed
    this session to cleanly cascade through Gazebo/ros2_control/curobo in
    ~3-4s when the user Ctrl-C's a real bringup_leeloo session), escalating
    to SIGKILL only if it doesn't exit in time. Must kill the GROUP, not
    just the `ros2 launch` PID -- launch spawns child processes that would
    otherwise leak as orphaned GPU-resident nodes across trials."""
    if proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    try:
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
    except ProcessLookupError:
        pass


def wait_for_planner_service(node, timeout_s: float):
    from curobo_msgs.srv import SetPlanner
    client = node.create_client(SetPlanner, "/curobo_trajectory_planner/set_planner")
    return client if client.wait_for_service(timeout_sec=timeout_s) else None


def set_planner_mpc(node, client) -> bool:
    import rclpy
    from curobo_msgs.srv import SetPlanner
    req = SetPlanner.Request()
    req.planner_type = 1  # MPC, per planner_factory.py's catalog
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    if not future.done() or future.result() is None:
        return False
    return bool(future.result().success)


def run_goal(node, goal_spec: dict, sim_duration_s: float, action_timeout_s: float = 60.0) -> bool:
    import rclpy
    from rclpy.action import ActionClient
    from rosidl_runtime_py import set_message_fields
    from curobo_msgs.action import SendTrajectory

    client = ActionClient(node, SendTrajectory, "/curobo_trajectory_planner/execute_trajectory")
    if not client.wait_for_server(timeout_sec=action_timeout_s):
        return False

    goal = SendTrajectory.Goal()
    x, y, z = goal_spec["position"]
    w, qx, qy, qz = goal_spec["orientation_wxyz"]
    set_message_fields(goal, {
        "target_pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"w": w, "x": qx, "y": qy, "z": qz},
        }
    })

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=action_timeout_s)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        return False

    deadline = time.time() + sim_duration_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    cancel_future = goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=action_timeout_s)
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=action_timeout_s)
    return True


def score_csv(csv_path: str, tail_window_s: float):
    """Returns (score, detail_dict) or (None, reason) if the CSV can't be
    trusted (missing/empty/NaN-poisoned) -- caller applies DIVERGENCE_PENALTY."""
    if not csv_path or not os.path.isfile(csv_path):
        return None, "missing CSV"
    rows = []
    with open(csv_path, "r", newline="") as f:
        for row in csv.DictReader(f):
            try:
                rows.append({
                    "t_s": float(row["t_s"]),
                    "exec_fk_err_m": float(row["exec_fk_err_m"]),
                    "exec_fk_rot_err_deg": float(row["exec_fk_rot_err_deg"]),
                    "horizon_churn_max_deg": float(row["horizon_churn_max_deg"]),
                })
            except (KeyError, ValueError):
                continue
    if not rows:
        return None, "empty/unparseable CSV"
    t_max = rows[-1]["t_s"]
    tail = [r for r in rows if r["t_s"] >= t_max - tail_window_s]
    if not tail:
        tail = rows[-1:]

    def finite(vals):
        return [v for v in vals if math.isfinite(v)]

    fk_err = finite(r["exec_fk_err_m"] for r in tail)
    fk_rot = finite(r["exec_fk_rot_err_deg"] for r in tail)
    churn = finite(r["horizon_churn_max_deg"] for r in tail)
    if not fk_err or not fk_rot:
        return None, "NaN-poisoned tail window"

    pos_term = statistics.mean(fk_err) / CONVERGENCE_THRESHOLD_M
    rot_term = statistics.mean(fk_rot) / CONVERGENCE_THRESHOLD_DEG
    churn_term = CHURN_TIEBREAK_SCALE * (max(churn) if churn else 0.0)
    score = pos_term + rot_term + churn_term
    return score, {
        "tail_fk_err_m": statistics.mean(fk_err),
        "tail_fk_rot_err_deg": statistics.mean(fk_rot),
        "tail_churn_max_deg": max(churn) if churn else float("nan"),
    }


def find_new_csv(trial_dir: str, seen_before: set) -> str:
    now = set(glob.glob(os.path.join(trial_dir, "mpc_diag_*.csv")))
    new = sorted(now - seen_before)
    return new[-1] if new else None


def run_goal_in_fresh_launch(node, goal: dict, candidate_path: str, trial_dir: str, args, tag: str):
    """One goal, one dedicated launch -- each goal gets a fresh Gazebo spawn
    at the robot's home posture, rather than inheriting whatever state the
    PREVIOUS goal happened to leave the robot in. A first full sweep (this
    session, 2026-08-14) ran goals back-to-back in a single launch and it
    confounded the results: the second goal's tail error reflected recovery
    from an arbitrary mid-convergence pose, not a clean repro of the
    historical CSV it was named after. Costs an extra launch/teardown cycle
    per goal but reuses already-validated launch_trial_node/
    terminate_process_group rather than inventing a joint-space "reset to
    home" service call."""
    log_path = os.path.join(trial_dir, f"launch.{tag}.stdout.log")
    proc = launch_trial_node(candidate_path, trial_dir, args.ros_domain_id, log_path)
    try:
        client = wait_for_planner_service(node, args.startup_timeout_s)
        if client is None:
            return DIVERGENCE_PENALTY, {"goal": goal["name"], "reason": "node never became ready"}
        if not set_planner_mpc(node, client):
            return DIVERGENCE_PENALTY, {"goal": goal["name"], "reason": "set_planner(MPC) failed"}

        seen = set(glob.glob(os.path.join(trial_dir, "mpc_diag_*.csv")))
        if not run_goal(node, goal, args.sim_duration_s):
            return DIVERGENCE_PENALTY, {"goal": goal["name"], "reason": "action rejected/timed out"}
        time.sleep(0.5)  # let the CSV writer flush its last rows
        csv_path = find_new_csv(trial_dir, seen)
        score, detail = score_csv(csv_path, args.tail_window_s)
        if score is None:
            return DIVERGENCE_PENALTY, {"goal": goal["name"], "reason": detail}
        return score, {"goal": goal["name"], **detail}
    finally:
        terminate_process_group(proc)


def run_trial(node, params: dict, base_yaml: dict, args, trial_index: int):
    trial_dir = os.path.join(args.storage_dir, f"trial_{trial_index:04d}")
    os.makedirs(trial_dir, exist_ok=True)
    candidate_path = build_candidate_yaml(base_yaml, params, os.path.join(trial_dir, "lbfgs_mpc.yaml"))

    scores, details = [], []
    for i, goal in enumerate(args.goals):
        score, detail = run_goal_in_fresh_launch(node, goal, candidate_path, trial_dir, args, tag=f"goal{i}")
        scores.append(score)
        details.append(detail)
    return statistics.mean(scores), details


def make_objective(node, base_yaml: dict, args):
    def objective(trial):
        params = {
            "pos_weight": trial.suggest_float("pos_weight", 1000.0, 10000.0, log=True),
            "orient_weight": trial.suggest_float("orient_weight", 200.0, 5000.0, log=True),
            "vel_damping": trial.suggest_float("vel_damping", 0.01, 10.0, log=True),
            "non_terminal_factor": trial.suggest_float("non_terminal_factor", 0.01, 0.3),
        }
        score, details = run_trial(node, params, base_yaml, args, trial.number)
        trial.set_user_attr("details", details)
        return score
    return objective


def report_best(study, base_yaml: dict, args):
    best = study.best_trial
    print("\n=== Best trial ===")
    print(f"score = {best.value:.4f}  (trial #{best.number})")
    print("params:")
    current = {
        "pos_weight": base_yaml["rollout"]["cost_cfg"]["tool_pose_cfg"]["weight"][0],
        "orient_weight": base_yaml["rollout"]["cost_cfg"]["tool_pose_cfg"]["weight"][1],
        "vel_damping": base_yaml["rollout"]["cost_cfg"]["cspace_cfg"]["squared_l2_regularization_weight"][0],
        "non_terminal_factor": base_yaml["rollout"]["cost_cfg"]["tool_pose_cfg"]
            ["_non_terminal_pose_axes_weight_factor"][0],
    }
    for k, v in best.params.items():
        print(f"  {k}: {current[k]:.4g} (current)  ->  {v:.4g} (proposed)")

    out_path = build_candidate_yaml(base_yaml, best.params, args.out_yaml)
    print(f"\nFull candidate config written to: {out_path}")
    print("Review it, then sanity-check by hand (see the plan's Verification "
          "section) before adopting it into config/mpc/lbfgs_mpc.yaml.")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--n-trials", type=int, default=15)
    p.add_argument("--sim-duration-s", type=float, default=50.0,
                    help="The historical converging-case CSV this harness's "
                         "goals are drawn from took ~46s to reach steady "
                         "state; a shorter window scores mid-transient "
                         "error for every candidate alike and produces a "
                         "meaningless ranking (confirmed 2026-08-14 at 20s).")
    p.add_argument("--tail-window-s", type=float, default=2.0)
    p.add_argument("--startup-timeout-s", type=float, default=240.0,
                    help="Readiness timeout for /curobo_trajectory_planner/set_planner. "
                         "Gazebo + camera drivers + controller spawners measured "
                         "~130s in a real run this session -- default is generous.")
    p.add_argument("--study-name", type=str, default="lbfgs_mpc_tune")
    p.add_argument("--storage-dir", type=str,
                    default=os.path.join(os.path.dirname(__file__), "tune_runs"))
    p.add_argument("--ros-domain-id", type=int, default=77,
                    help="Fully isolates every trial's ROS graph from any other "
                         "session on this machine (topics/services/discovery). "
                         "Confirm no other session already uses this domain.")
    p.add_argument("--out-yaml", type=str,
                    default=os.path.join(os.path.dirname(__file__), "tune_runs", "best_lbfgs_mpc.yaml"))
    return p.parse_args()


def main():
    args = parse_args()
    args.goals = DEFAULT_GOALS
    os.makedirs(args.storage_dir, exist_ok=True)

    # Must happen before rclpy/DDS ever initializes a participant in this
    # process -- otherwise the harness's own client node joins whatever
    # domain the shell inherited (not the isolated one each trial's
    # subprocess is launched into) and every wait_for_service() call times
    # out silently, since the two graphs simply can't see each other.
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["CYCLONEDDS_URI"] = CYCLONEDDS_PROFILE

    import rclpy
    import optuna

    base_yaml = load_base_yaml(BASE_LBFGS_YAML)

    rclpy.init()
    node = rclpy.create_node("mpc_tuner_client")
    try:
        storage = f"sqlite:///{os.path.join(args.storage_dir, args.study_name)}.db"
        study = optuna.create_study(
            study_name=args.study_name, storage=storage, direction="minimize",
            load_if_exists=True, sampler=optuna.samplers.TPESampler(),
        )
        study.optimize(make_objective(node, base_yaml, args), n_trials=args.n_trials)
        report_best(study, base_yaml, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
