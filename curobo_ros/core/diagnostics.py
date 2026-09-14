#!/usr/bin/env python3
"""
Diagnostic CSV writer shared by mpc_planner and joint_speed_strategy.

Both write one row per control-loop step for offline analysis (see
scripts/plot_mpc_diag.py). Neither is meant to run by default in production:
each is gated by its own ROS parameter and defaults to off. When enabled, the
file goes to the node's ROS logging directory (``ROS_LOG_DIR`` / the ROS home
log dir, or a directory overridden by the ``diagnostic_csv_dir`` parameter) —
never into the package's source tree, which `setup.py` installs from.
"""

import csv
import os
import time

import rclpy.logging


def resolve_diag_dir(node) -> str:
    """Directory for diagnostic CSVs: `diagnostic_csv_dir` param, or the ROS
    logging directory (``ROS_LOG_DIR`` / ``~/.ros/log/<session>``) by default.
    """
    if not node.has_parameter('diagnostic_csv_dir'):
        node.declare_parameter('diagnostic_csv_dir', '')
    override = node.get_parameter('diagnostic_csv_dir').get_parameter_value().string_value
    if override:
        return override
    return rclpy.logging.get_logging_directory()


class DiagCsv:
    """A single diagnostic CSV file: header written once, rows buffered in RAM.

    Never raises into the caller — a diagnostic sink must not break the
    control loop it is observing. Construct via ``open_diag_csv``.

    **Why rows are buffered.** This used to write AND ``flush()`` on every row.
    On the Jetson's eMMC that made the LBFGS servo loop's ``csv_write`` phase
    cost 23.9ms/cycle — 5.4% of a 443ms cycle spent in fsync-adjacent I/O, on a
    loop whose whole budget is 400ms. Buffering moves that cost off the control
    path without changing what ends up in the file.

    **What is traded.** A hard crash (SIGSEGV from the collision kernel, SIGKILL)
    loses whatever has not drained yet. That is bounded on BOTH axes so the loss
    can never be the interesting part of a run: at most ``flush_every`` rows, and
    at most ``flush_interval_s`` of wall time. The time bound matters because
    callers close the file per goal, not per cycle (lbfgs_planner's ``_csv_init``
    closes the PREVIOUS goal's file) — a goal that completes and then sits idle
    would otherwise strand its tail in RAM indefinitely.
    """

    def __init__(self, path: str, flush_every: int = 25, flush_interval_s: float = 2.0):
        self._path = path
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._header_written = False
        self._rows = []
        self._flush_every = max(1, int(flush_every))
        self._flush_interval_s = float(flush_interval_s)
        self._last_drain = time.monotonic()

    def writerow(self, row) -> None:
        # Append only — the actual write happens in _drain(). list.append on a
        # pre-built row is the cheapest thing that can stand where a flush() was.
        self._rows.append(row)
        if (len(self._rows) >= self._flush_every
                or (time.monotonic() - self._last_drain) >= self._flush_interval_s):
            self._drain()

    def _drain(self) -> None:
        """Write every buffered row to disk. Swallows failures like writerow did.

        The buffer is cleared BEFORE the write attempt: if the disk is full or
        the file went away, retrying the same rows on every subsequent call
        would turn a transient I/O failure into an unbounded memory leak in the
        control loop's own process.
        """
        if not self._rows:
            self._last_drain = time.monotonic()
            return
        rows, self._rows = self._rows, []
        try:
            self._writer.writerows(rows)
            self._file.flush()
        except Exception:
            pass
        self._last_drain = time.monotonic()

    def write_header_once(self, header) -> bool:
        """Write `header` only on the first call. Returns True if written now."""
        if self._header_written:
            return False
        self.writerow(header)
        self._header_written = True
        return True

    def close(self) -> None:
        # Drain first, or every row buffered since the last drain is lost —
        # including the end of the run, which is usually the part being looked
        # for (a cancel, a watchdog trip, the final convergence).
        self._drain()
        try:
            self._file.close()
        except Exception:
            pass


def open_diag_csv(node, prefix: str):
    """Open a new timestamped diagnostic CSV, or None if it can't be created.

    Logs (and swallows) any failure — a missing/unwritable log directory must
    never prevent the node itself from starting or running.
    """
    try:
        d = resolve_diag_dir(node)
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        diag = DiagCsv(path)
        node.get_logger().info(f"Diagnostic CSV -> {path}")
        return diag
    except Exception as e:
        node.get_logger().warn(f"Diagnostic CSV ({prefix}) init failed: {e}")
        return None
