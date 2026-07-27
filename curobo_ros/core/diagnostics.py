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
    """A single diagnostic CSV file: header written once, flushed every row.

    Never raises into the caller — a diagnostic sink must not break the
    control loop it is observing. Construct via ``open_diag_csv``.
    """

    def __init__(self, path: str):
        self._path = path
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._header_written = False

    def writerow(self, row) -> None:
        try:
            self._writer.writerow(row)
            self._file.flush()
        except Exception:
            pass

    def write_header_once(self, header) -> bool:
        """Write `header` only on the first call. Returns True if written now."""
        if self._header_written:
            return False
        self.writerow(header)
        self._header_written = True
        return True

    def close(self) -> None:
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
