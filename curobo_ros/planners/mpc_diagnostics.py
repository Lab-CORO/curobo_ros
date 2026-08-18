#!/usr/bin/env python3
"""
Per-step diagnostic CSV, cost/horizon introspection, and RViz publishing for
MPCController — split out of mpc_planner.py so step() stays about control,
not I/O. Every method here is read-only with respect to control: it observes
a solve after the fact and never feeds anything back into the solver or
_v_bc. Built once in MPCController.build_solver(), alongside the solver it
observes (same lifetime — a solver rebuild gets a fresh MPCDiagnostics too).
"""

import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from curobo_msgs.msg import MpcCosts
from curobo.types import JointState

from curobo_ros.core.diagnostics import open_diag_csv


class MPCDiagnostics:
    """Diagnostics + RViz publishing for one MPCController solver instance.

    ``fk_position_error`` / ``fk_orientation_error`` are the owning
    controller's bound methods (``ReactiveController._fk_position_error`` /
    ``_fk_orientation_error``): they need the controller's target state
    (``_target_position`` / ``_target_quaternion``), which is
    convergence-tracking state, not diagnostic state, so it stays on the
    controller and is injected here rather than duplicated.

    ``executed_idx`` (the index into the plan the arm will actually have
    reached before the next window replaces it) is a per-step value computed
    once in ``MPCController.step()`` and passed explicitly into every method
    that needs it, rather than read from shared mutable state — the same
    value is needed by both ``horizon_diag`` and ``csv_write`` and must stay
    consistent between them for one step.
    """

    def __init__(self, node, solver, path_frame, step_dt,
                 fk_position_error, fk_orientation_error, csv_prefix: str = "mpc_diag"):
        self.node = node
        self.solver = solver
        self._path_frame = path_frame
        self._step_dt = step_dt
        self._fk_position_error = fk_position_error
        self._fk_orientation_error = fk_orientation_error
        self._csv_prefix = csv_prefix

        # Predicted end-effector path (current MPC horizon), for RViz
        # (nav_msgs/Path renders natively, no custom plugin needed).
        self._path_pub = node.create_publisher(Path, 'mpc_predicted_path', 10)
        # Same data, guaranteed un-trimmed: publish_predicted_path falls back
        # to the 8-point action_sequence whenever robot_state_sequence.tool_poses
        # is unavailable (cf. publish_full_predicted_path). This topic always
        # FKs the full mpc_horizon_steps-length robot_state_sequence itself.
        self._full_path_pub = node.create_publisher(Path, 'mpc_predicted_path_full', 10)
        self._goal_marker_pub = node.create_publisher(Marker, 'mpc_goal_marker', 10)
        # Cost/constraint breakdown, for live inspection via rqt_plot (each
        # named field is individually plottable). See cost_breakdown().
        self._cost_pub = node.create_publisher(MpcCosts, 'mpc_costs', 10)

        self._csv = None
        self._csv_t0 = None
        self._csv_t_prev = None
        self._csv_last_vexec = None
        # Horizon-churn state (see horizon_diag): reset per goal so a new
        # goal's first solve never diffs against the previous goal's horizon.
        self._prev_horizon_q = None
        self._prev_horizon_t = None
        # record_tick's own previous-velocity baseline (one-point-per-call
        # model, LBFGSController) -- deliberately separate from
        # _csv_last_vexec (csv_write's own baseline, MPCController's
        # per-horizon model) so the two continuity computations can never
        # perturb each other even if both ran against the same solver.
        self._tick_last_v = None

    # ---- Debug CSV ----

    def csv_init(self, debug_enabled: bool):
        """Opens a diagnostic CSV (one line per step). Gated by mpc_debug.

        Closes any CSV left open by a previous goal first (setup() is called
        once per goal — without this, each new goal leaked the previous
        file's descriptor).
        """
        self.csv_close()
        if not debug_enabled:
            return
        self._csv = open_diag_csv(self.node, self._csv_prefix)
        if self._csv is not None:
            self._csv_t0 = time.monotonic()
            self._csv_t_prev = self._csv_t0
            self._csv_last_vexec = None
        self._prev_horizon_q = None
        self._prev_horizon_t = None
        self._tick_last_v = None

    def csv_close(self):
        if self._csv is not None:
            self._csv.close()
        self._csv = None

    def horizon_diag(self, result, seq, executed_idx: int) -> dict:
        """Three FK/consistency diagnostics beyond the single-point fk_err_m:

        - exec_fk_*: FK error of the point actually EXECUTED before the next
          resolve (same index csv_write's vexec_max_dps samples -- the last
          point of the trimmed command window, seq[:, -1, :] for both
          backends since command_end_idx is always 8). If this tracks
          fk_err_m, the command window isn't the problem.
        - term_fk_*: FK error of the TERMINAL point of the full
          mpc_horizon_steps-length horizon (result.robot_state_sequence, the
          same source publish_full_predicted_path FKs) -- i.e. what the
          optimizer itself believes it will reach if left alone. If this
          stays large while fk_err_m plateaus, the optimizer is stuck (wrong
          local minimum / singularity), not merely under-executed; pacing
          fixes cannot help that case.
        - horizon_churn_max_deg: max per-joint deviation (degrees) between
          this solve's full horizon and the PREVIOUS solve's full horizon,
          compared over their time-aligned overlap (shifted by however many
          step_dt ticks elapsed between the two solves). A consistent
          receding-horizon controller has low churn solve-to-solve; a
          controller re-landing in a different local minimum each solve
          spikes here -- this is the direct, numeric version of "converges
          maybe 1 time in 10."
        """
        out = {
            'exec_fk_err_m': float('nan'), 'exec_fk_rot_err_deg': float('nan'),
            'term_fk_err_m': float('nan'), 'term_fk_rot_err_deg': float('nan'),
            'horizon_churn_max_deg': float('nan'),
        }
        try:
            npts = seq.position.shape[1]
            kexec = max(0, min(executed_idx, npts - 1))
            exec_state = JointState.from_position(
                seq.position[:, kexec, :], joint_names=self.solver.joint_names)
            out['exec_fk_err_m'] = self._fk_position_error(exec_state)
            out['exec_fk_rot_err_deg'] = math.degrees(self._fk_orientation_error(exec_state))
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] exec_fk failed: {e}", throttle_duration_sec=5.0)

        full_q = None
        try:
            state_seq = getattr(result, 'robot_state_sequence', None)
            if state_seq is not None:
                full_q = state_seq.joint_state.position  # [1, horizon, dof]
                term_state = JointState.from_position(
                    full_q[:, -1, :], joint_names=self.solver.joint_names)
                out['term_fk_err_m'] = self._fk_position_error(term_state)
                out['term_fk_rot_err_deg'] = math.degrees(self._fk_orientation_error(term_state))
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] term_fk failed: {e}", throttle_duration_sec=5.0)

        try:
            now = time.monotonic()
            if full_q is not None:
                full_q_cpu = full_q[0].detach().cpu()  # [horizon, dof]
                if (self._prev_horizon_q is not None and self._prev_horizon_t is not None
                        and self._step_dt > 0.0):
                    shift = int(round((now - self._prev_horizon_t) / self._step_dt))
                    prev = self._prev_horizon_q
                    overlap = min(prev.shape[0] - shift, full_q_cpu.shape[0])
                    if 0 <= shift < prev.shape[0] and overlap > 0:
                        dev = (prev[shift:shift + overlap] - full_q_cpu[:overlap]).abs()
                        out['horizon_churn_max_deg'] = math.degrees(float(dev.max().item()))
                self._prev_horizon_q = full_q_cpu
                self._prev_horizon_t = now
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] horizon_churn failed: {e}", throttle_duration_sec=5.0)

        return out

    def csv_write(
        self, result, solve_ms, breakdown: dict, horizon_diag: dict, *,
        command_interval: float, last_position_error: float,
        last_orientation_error: float, v_bc, executed_idx: int, last_q,
    ):
        if self._csv is None:
            return
        seq = result.action_sequence
        now = time.monotonic()
        dt_step_ms = (now - self._csv_t_prev) * 1000.0  # Loop period: captures 248ms AND pauses ~10s
        self._csv_t_prev = now
        dt = self._step_dt
        deg = math.degrees
        vel = seq.velocity[0]  # [npts, dof]
        npts, dof = vel.shape
        vfirst = vel[0].cpu().tolist()
        vlast = vel[-1].cpu().tolist()
        # Intra-window accel: max over (i,j) of |v[i+1]-v[i]|/dt (MPPI plan noise)
        accel_win = deg((vel[1:] - vel[:-1]).abs().max().item() / dt) if npts > 1 else 0.0
        # Boundary accel, measured against the point the previous window actually
        # EXECUTED (executed_idx, the same one _v_bc is sampled from), not its
        # last point. Comparing against vel[-1] measured a transition the robot
        # never makes — the unexecuted tail of the previous plan — and so just
        # reported the plan's own ramp across that tail: on 2026-08-07 the ratio
        # accel_boundary/accel_win pinned at exactly 8.0, i.e. the 8 unexecuted
        # steps, which is an artifact and not a discontinuity.
        kexec = max(0, min(executed_idx, npts - 1))
        vexec = vel[kexec].cpu().tolist()
        if self._csv_last_vexec is not None:
            accel_bd = deg(max(abs(a - b) for a, b in zip(vfirst, self._csv_last_vexec)) / dt)
        else:
            accel_bd = 0.0
        self._csv_last_vexec = vexec
        vbc = v_bc[0].cpu().tolist() if v_bc is not None else [0.0] * dof
        pos_err = getattr(result, 'position_error', None)
        rot_err = getattr(result, 'rotation_error', None)
        pose_pos_err = float(pos_err.reshape(-1)[0].item()) if pos_err is not None else float('nan')
        pose_rot_err = float(rot_err.reshape(-1)[0].item()) if rot_err is not None else float('nan')
        hd = horizon_diag or {}
        self._csv.write_header_once(
            # cmd_interval_ms is constant per run but recorded per row so the CSV
            # is self-describing: the real-time budget for solve_ms is the SEND
            # interval, not dt_step_ms (in paced mode the producer loop period is
            # structurally ~solve_ms, so comparing the two always looks tight).
            ["t_s", "dt_step_ms", "solve_ms", "cmd_interval_ms",
             "fk_err_m", "fk_rot_err_deg", "vfirst_max_dps", "vlast_max_dps",
             "vexec_max_dps",
             "accel_win_max_dps2", "accel_boundary_dps2", "vbc_max_dps",
             "pose_pos_err_m", "pose_rot_err_rad", "cost_tool_pose_pos", "cost_tool_pose_orient",
             "cost_cspace", "con_self_collision", "con_scene_collision", "con_cspace_bound",
             # exec_fk_*: FK error of the point actually executed before the next
             # resolve (should track fk_err_m -- if it doesn't, the command
             # window is desynced from the state estimate). term_fk_*: FK error
             # of the horizon's TERMINAL point -- large+flat here means the
             # optimizer itself is stuck (wrong local minimum), not merely
             # under-executed. horizon_churn_max_deg: max per-joint deviation
             # between this solve's full horizon and the previous one's, over
             # their time-aligned overlap -- the numeric version of "converges
             # ~1 time in 10". See horizon_diag().
             "exec_fk_err_m", "exec_fk_rot_err_deg",
             "term_fk_err_m", "term_fk_rot_err_deg",
             "horizon_churn_max_deg"]
            + [f"vfirst_j{i+1}_dps" for i in range(dof)]
            + [f"vlast_j{i+1}_dps" for i in range(dof)]
            + [f"q_j{i+1}_deg" for i in range(dof)])
        self._csv.writerow(
            [f"{now - self._csv_t0:.3f}", f"{dt_step_ms:.1f}", f"{solve_ms:.1f}",
             f"{command_interval * 1000.0:.1f}",
             f"{last_position_error:.5f}",
             # Degrees, 3 decimals: the old pose_rot_err_rad column plateaued at
             # 1.6e-4 with 5-decimal formatting, i.e. ~16 distinct values over a
             # whole run — it could neither confirm nor refute an orientation
             # complaint. This one resolves 0.001 deg.
             f"{deg(last_orientation_error):.3f}",
             f"{deg(max(abs(v) for v in vfirst)):.2f}",
             f"{deg(max(abs(v) for v in vlast)):.2f}",
             f"{deg(max(abs(v) for v in vexec)):.2f}",
             f"{accel_win:.1f}", f"{accel_bd:.1f}",
             f"{deg(max(abs(v) for v in vbc)):.2f}",
             f"{pose_pos_err:.5f}", f"{pose_rot_err:.5f}",
             f"{breakdown.get('cost_tool_pose_pos', float('nan')):.4f}",
             f"{breakdown.get('cost_tool_pose_orient', float('nan')):.4f}",
             f"{breakdown.get('cost_cspace', float('nan')):.4f}",
             f"{breakdown.get('con_self_collision', float('nan')):.4f}",
             f"{breakdown.get('con_scene_collision', float('nan')):.4f}",
             f"{breakdown.get('con_cspace_bound', float('nan')):.4f}",
             f"{hd.get('exec_fk_err_m', float('nan')):.5f}",
             f"{hd.get('exec_fk_rot_err_deg', float('nan')):.3f}",
             f"{hd.get('term_fk_err_m', float('nan')):.5f}",
             f"{hd.get('term_fk_rot_err_deg', float('nan')):.3f}",
             f"{hd.get('horizon_churn_max_deg', float('nan')):.3f}"]
            + [f"{deg(v):.2f}" for v in vfirst] + [f"{deg(v):.2f}" for v in vlast]
            + [f"{deg(v):.2f}" for v in (last_q or [float('nan')] * dof)])

    def record_tick(
        self, result, *, solve_ms: float,
        last_position_error: float, last_orientation_error: float,
        queue_depth: int = -1, min_queue_depth_seen: int = -1,
        starvation_ticks: int = -1,
        backpressure_wait_ms: float = -1.0, perception_ms: float = -1.0,
        live_goal_ms: float = -1.0, cheap_ms_before_resolve: float = -1.0,
        batch_wall_ms: float = -1.0, loop_iter: int = -1,
    ):
        """One CSV row per ``optimize_next_action()`` call -- the record_tick
        counterpart to ``csv_write``'s one-row-per-horizon-solve for
        LBFGSController's one-point-per-call model (see lbfgs_planner.py). A
        resolve only happens on 1 of every ``interpolation_steps`` (4) calls
        (confirmed empirically 2026-08-17, see the plan's Vérification étape
        0), so ``solve_ms`` here is ~0 on 3 calls out of 4 and a real solve
        time on the 4th -- expected, not a bug, when reading this CSV.

        ``queue_depth``/``min_queue_depth_seen``/``starvation_ticks`` mirror
        ReactiveController._execute_paced's own periodic (1Hz) log line, but
        recorded here per-row instead so a stall can be correlated against
        the exact dt_step_ms/solve_ms spike that caused it, not just a 1Hz
        aggregate. -1 (not 0) is the "caller didn't pass it" sentinel --
        0 is a real, meaningful queue depth (empty queue). cf. debug
        2026-08-18: the periodic log alone couldn't show WHICH batch drained
        the queue to starvation.

        ``backpressure_wait_ms``/``perception_ms``/``live_goal_ms``/
        ``cheap_ms_before_resolve``/``batch_wall_ms``/``loop_iter`` break
        dt_step_ms down into where the time actually went, since solve_ms
        alone only ever explained ~46% of dt_step_ms's variance (measured
        2026-08-18, lbfgs_diag_20260818_070201.csv): dt_step_ms ≈
        backpressure_wait_ms + perception_ms + live_goal_ms +
        cheap_ms_before_resolve + solve_ms + (trailing cheap calls of THIS
        batch, attributed to the NEXT row, same as dt_step_ms itself) +
        scheduling slop. All -1 sentinel when unset (same convention as the
        queue fields above).

        Deliberately does NOT touch ``csv_write``/``horizon_diag`` or their
        state (``_csv_last_vexec``, ``_prev_horizon_q/_t``) -- those stay
        strictly MPCController's per-horizon path. Uses its own
        ``_tick_last_v`` baseline instead (see __init__).
        """
        if self._csv is None:
            return
        action = result.next_action
        now = time.monotonic()
        dt_step_ms = (now - self._csv_t_prev) * 1000.0
        self._csv_t_prev = now
        deg = math.degrees
        q = action.position[0].detach().cpu().tolist()
        dof = len(q)
        v = (action.velocity[0].detach().cpu().tolist()
             if getattr(action, 'velocity', None) is not None else [0.0] * dof)
        # action_dt is the RESOLVE window duration (== optimization_dt), not
        # the per-command dt (== optimization_dt/interpolation_steps) -- see
        # the plan's Contexte point 2. It is still the right denominator here
        # because _tick_last_v is itself one command apart (one call), so the
        # per-call dt to use is optimization_dt/interpolation_steps, i.e.
        # command_dt -- the caller's own paced tick interval
        # (LBFGSController._command_interval == interpolation_dt) is the
        # correct, already-available value; action_dt is logged as raw
        # metadata alongside it for cross-checking, not used in the accel
        # computation.
        command_dt = self._step_dt
        action_dt = float(getattr(result, 'action_dt', 0.0) or 0.0)
        accel = 0.0
        if self._tick_last_v is not None and command_dt > 0.0:
            accel = deg(max(abs(a - b) for a, b in zip(v, self._tick_last_v)) / command_dt)
        self._tick_last_v = v
        breakdown = self.cost_breakdown(result)
        self._csv.write_header_once(
            ["t_s", "dt_step_ms", "solve_ms", "command_dt_ms", "action_dt_ms",
             "fk_err_m", "fk_rot_err_deg", "v_max_dps", "accel_step_dps2",
             "cost_tool_pose_pos", "cost_tool_pose_orient", "cost_cspace",
             "con_self_collision", "con_scene_collision", "con_cspace_bound",
             "queue_depth", "min_queue_depth_seen", "starvation_ticks",
             "backpressure_wait_ms", "perception_ms", "live_goal_ms",
             "cheap_ms_before_resolve", "batch_wall_ms", "loop_iter"]
            + [f"q_j{i+1}_deg" for i in range(dof)]
            + [f"v_j{i+1}_dps" for i in range(dof)])
        self._csv.writerow(
            [f"{now - self._csv_t0:.3f}", f"{dt_step_ms:.1f}", f"{solve_ms:.1f}",
             f"{command_dt * 1000.0:.1f}", f"{action_dt * 1000.0:.1f}",
             f"{last_position_error:.5f}", f"{deg(last_orientation_error):.3f}",
             f"{deg(max(abs(x) for x in v)):.2f}", f"{accel:.1f}",
             f"{breakdown.get('cost_tool_pose_pos', float('nan')):.4f}",
             f"{breakdown.get('cost_tool_pose_orient', float('nan')):.4f}",
             f"{breakdown.get('cost_cspace', float('nan')):.4f}",
             f"{breakdown.get('con_self_collision', float('nan')):.4f}",
             f"{breakdown.get('con_scene_collision', float('nan')):.4f}",
             f"{breakdown.get('con_cspace_bound', float('nan')):.4f}",
             str(queue_depth), str(min_queue_depth_seen), str(starvation_ticks),
             f"{backpressure_wait_ms:.1f}", f"{perception_ms:.1f}",
             f"{live_goal_ms:.1f}", f"{cheap_ms_before_resolve:.1f}",
             f"{batch_wall_ms:.1f}", str(loop_iter)]
            + [f"{deg(x):.2f}" for x in q] + [f"{deg(x):.2f}" for x in v])

    # ---- RViz publishing ----

    def publish_predicted_path(self, result):
        """Publish the MPC's full predicted end-effector path for RViz.

        ``result.action_sequence`` (used elsewhere to command the robot) is
        NOT the full optimized horizon: cuRobo's TrajectoryExecutionManager
        trims it to only ``interpolation_steps * 2`` points — the near-term
        slice meant to be executed before the next re-plan (see
        ``get_command_sequence()``). The full horizon the optimizer actually
        reasoned over (collision costs, goal convergence, etc.) is
        ``result.robot_state_sequence`` instead, already FK'd (untrimmed).
        """
        try:
            state_seq = getattr(result, 'robot_state_sequence', None)
            if state_seq is not None and state_seq.tool_poses is not None:
                ee_pos = state_seq.tool_poses.position.reshape(-1, 3).cpu().tolist()
            else:
                # Fallback: FK on the trimmed action_sequence (partial horizon).
                seq = result.action_sequence
                js = JointState.from_position(seq.position[0], joint_names=self.solver.joint_names)
                fk = getattr(self.solver, 'compute_kinematics', None) or self.solver.kinematics.compute_kinematics
                ee_pos = fk(js).tool_poses.position.reshape(-1, 3).cpu().tolist()
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] predicted path FK failed: {e}", throttle_duration_sec=5.0)
            return
        self._path_pub.publish(self._ee_positions_to_path(ee_pos))

    def _ee_positions_to_path(self, ee_pos) -> Path:
        path = Path()
        path.header.frame_id = self._path_frame
        path.header.stamp = self.node.get_clock().now().to_msg()
        for x, y, z in ee_pos:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def publish_full_predicted_path(self, result):
        """Publish the MPC's full ``mpc_horizon_steps``-length predicted path.

        ``publish_predicted_path`` above silently degrades to an 8-point path
        (``interpolation_steps * 2``) whenever ``robot_state_sequence.tool_poses``
        isn't populated by the metrics rollout. ``robot_state_sequence.joint_state
        .position`` itself has no such gap — the transition model always fills it
        for the whole horizon — so FK it directly here instead of trusting
        ``tool_poses`` to have been computed.
        """
        try:
            state_seq = getattr(result, 'robot_state_sequence', None)
            if state_seq is None:
                return
            if state_seq.tool_poses is not None:
                ee_pos = state_seq.tool_poses.position.reshape(-1, 3).cpu().tolist()
            else:
                js = JointState.from_position(
                    state_seq.joint_state.position[0], joint_names=self.solver.joint_names
                )
                fk = getattr(self.solver, 'compute_kinematics', None) or self.solver.kinematics.compute_kinematics
                ee_pos = fk(js).tool_poses.position.reshape(-1, 3).cpu().tolist()
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] full predicted path FK failed: {e}", throttle_duration_sec=5.0)
            return
        self._full_path_pub.publish(self._ee_positions_to_path(ee_pos))

    def publish_goal_marker(self, raw, applied: bool = True):
        """Publish a sphere Marker at the current Cartesian goal, for RViz.

        Color reflects whether the solver actually accepted the goal: red when
        applied (tracking normally), orange when IK failed and the arm is only
        pose-tracking (may not move) — see MPCController._apply_goal's warn
        path. Without this, the marker showed a goal as "set" even when the
        controller wasn't really tracking it, which is a bad debugging trap.
        cf. debug 2026-07-28.
        """
        marker = Marker()
        marker.header.frame_id = self._path_frame
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "mpc_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(raw[0])
        marker.pose.position.y = float(raw[1])
        marker.pose.position.z = float(raw[2])
        marker.pose.orientation.w = float(raw[3])
        marker.pose.orientation.x = float(raw[4])
        marker.pose.orientation.y = float(raw[5])
        marker.pose.orientation.z = float(raw[6])
        marker.scale.x = marker.scale.y = marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0 if applied else 0.5  # red = tracking, orange = IK failed
        marker.color.b = 0.0
        self._goal_marker_pub.publish(marker)

    # ---- Costs ----

    def cost_breakdown(self, result) -> dict:
        """Per-term COST and CONSTRAINT values (horizon-summed) for this solve.

        CRASH-SAFETY NOTE (cf. debug 2026-07-20): an earlier version read the
        weighted COST magnitudes (tool_pose, cspace/anchor) via
        `compute_metrics_from_action(result.action_buffer)` on the
        OPTIMIZATION rollout (the only one with cost_cfg set by default). That
        rollout runs with use_cuda_graph=True and is shared with the
        optimizer; the extra call rebatches it (num_particles -> 1) while a
        CUDA graph is captured for a different batch size, which triggered a
        device-side assert (`index out of bounds`) on real hardware. A
        device-side assert corrupts the WHOLE CUDA context for the process —
        unrecoverable via try/except, and it took down the next solve,
        _send_command, and perception/voxelization with it. That call has
        been REMOVED. Do not reintroduce compute_metrics_from_action on a
        use_cuda_graph=True rollout shared with the optimizer.

        Costs are recovered safely instead: MPCController.build_solver()
        injects a cost_cfg (mirroring the active branch's
        tool_pose_cfg/cspace_cfg) into the METRICS rollout's config (see
        _build_metrics_rollout_cfg) — a fixed-batch-size rollout, never
        rebatched, so it computes these costs as a side effect of the normal
        solve. get_current_metrics() just returns an attribute
        (`_current_metrics`) already populated during that solve — no
        rebatch, no graph, no extra GPU call. Validated in sandbox: identical
        cost values to the removed dangerous path, zero CUDA errors across
        use_cuda_graph=True runs."""
        try:
            m = self.solver.trajectory_execution_manager.get_current_metrics()
            if m is None:
                return {}
            cc = m.costs_and_constraints
            out = {}
            for name, val in zip(cc.costs.names, cc.costs.values):
                if name == "tool_pose":
                    out["cost_tool_pose_pos"] = float(val[0, :, 0].sum().item())
                    out["cost_tool_pose_orient"] = float(val[0, :, 1].sum().item())
                else:
                    out[f"cost_{name}"] = float(val.sum().item())
            for name, val in zip(cc.constraints.names, cc.constraints.values):
                if name == "cspace":
                    out["con_cspace_bound"] = float(val.sum().item())
                else:
                    out[f"con_{name}"] = float(val.sum().item())
            return out
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] cost breakdown failed: {e}", throttle_duration_sec=5.0)
            return {}

    def publish_costs(
        self, result, breakdown: dict, last_position_error: float, last_orientation_error: float,
    ):
        msg = MpcCosts()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self._path_frame
        msg.fk_err_m = float(last_position_error)
        msg.fk_rot_err_deg = math.degrees(float(last_orientation_error))
        # Deprecated pair, still published so old bags stay parseable -- see the
        # note in MpcCosts.msg. Read fk_err_m / fk_rot_err_deg instead.
        pos_err = getattr(result, 'position_error', None)
        rot_err = getattr(result, 'rotation_error', None)
        msg.pose_pos_err_m = float(pos_err.reshape(-1)[0].item()) if pos_err is not None else 0.0
        msg.pose_rot_err_rad = float(rot_err.reshape(-1)[0].item()) if rot_err is not None else 0.0
        msg.cost_tool_pose_pos = breakdown.get("cost_tool_pose_pos", 0.0)
        msg.cost_tool_pose_orient = breakdown.get("cost_tool_pose_orient", 0.0)
        msg.cost_cspace = breakdown.get("cost_cspace", 0.0)
        msg.con_self_collision = breakdown.get("con_self_collision", 0.0)
        msg.con_scene_collision = breakdown.get("con_scene_collision", 0.0)
        msg.con_cspace_bound = breakdown.get("con_cspace_bound", 0.0)
        self._cost_pub.publish(msg)

    # ---- Step-level debug log ----

    def log_setup_summary(self, start_state, raw, applied: bool):
        ee_pos, ee_quat = None, None
        try:
            kin = self.solver.compute_kinematics(start_state)
            ee_pos = [round(v, 4) for v in kin.tool_poses.position.reshape(-1, 3)[0].cpu().tolist()]
            ee_quat = [round(v, 4) for v in kin.tool_poses.quaternion.reshape(-1, 4)[0].cpu().tolist()]
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] EE pose log failed: {e}")
        self.node.get_logger().debug(
            f"[MPC DIAG] setup: goal_xyz={[round(v, 4) for v in raw[0:3]]} "
            f"goal_quat(wxyz)={[round(v, 4) for v in raw[3:7]]} "
            f"tool_frame={self.solver.tool_frames[0]} "
            f"start_EE_pos={ee_pos} start_EE_quat(wxyz)={ee_quat} "
            f"fk_err={self._fk_position_error(start_state):.4f}m goal_applied={applied}"
        )

    def log_step_summary(self, action, last_position_error: float):
        try:
            vel = action.velocity
            is_horizon = vel.dim() == 3
            v_first = float(vel[:, 0, :].abs().max()) if is_horizon else float(vel.abs().max())
            v_last = float(vel[:, -1, :].abs().max()) if is_horizon else v_first
            n_pts = action.position.shape[1] if action.position.dim() == 3 else 1
            self.node.get_logger().debug(
                f"[MPC DIAG] step: fk_err={last_position_error:.4f}m "
                f"horizon_points={n_pts} |v|first={v_first:.3e} |v|last={v_last:.3e} rad/s",
                throttle_duration_sec=1.0,
            )
        except Exception as e:
            self.node.get_logger().warn(f"[MPC DIAG] step log failed: {e}", throttle_duration_sec=5.0)
