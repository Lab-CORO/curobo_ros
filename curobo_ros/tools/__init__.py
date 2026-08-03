"""Command-line tools shipped with the package.

Unlike the nodes in ``curobo_ros/core``, these are one-shot utilities run by hand
against an already-running planner (``ros2 run curobo_ros <tool>``). They depend
only on rclpy and the curobo_msgs interfaces.

Offline analysis tools (matplotlib/pandas, no ROS) stay in ``scripts/`` and are
not installed -- see ``scripts/plot_mpc_diag.py``.
"""
