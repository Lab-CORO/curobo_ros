#!/usr/bin/env python3
"""Standalone test: producer/consumer pacing for an action server.

Validates the pattern intended for the real MPC servo loop:
  - PRODUCER: execute_callback's own thread runs continuously (no sleep for
    pacing, just the simulated "solve" time), generating a new value as fast
    as it can and publishing it (under a lock) as the latest candidate. It
    checks goal_handle.is_cancel_requested itself each iteration — no
    threading.Event needed, the loop just ends.
  - CONSUMER: a fixed-interval (240ms) timer takes whatever is latest each
    tick. If it is NEW since the last send, it "sends" it (here: log +
    publish feedback) and marks it consumed. If NOT new (the producer didn't
    finish a fresh one in time), it only warns ("out of time") and sends
    NOTHING that tick — no resend of stale data.

This decouples "how long one solve takes" from "how precisely a command
window boundary is hit": the timer's send is a cheap read+log, never blocked
by production time.

Run:
    python3 test_timer_action_server.py

Test from another terminal (needs the curobo_msgs package sourced):
    ros2 action send_goal /timer_action_test_node/execute_trajectory \\
        curobo_msgs/action/SendTrajectory "{}" --feedback
    # Ctrl-C mid-run to test cancel responsiveness.
"""
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState as JointStateMsg
from curobo_msgs.action import SendTrajectory

TICK_PERIOD_S = 0.24
PRODUCE_TIME_S = 0.250  # simulated solve time per produced value
TARGET_COUNT = 300


class TimerActionTestNode(Node):

    def __init__(self):
        super().__init__('timer_action_test_node')
        self._cb_group = ReentrantCallbackGroup()
        self._active = False

        # Shared between the producer (execute_callback's thread) and the
        # consumer (timer tick, a different executor thread) — protected by
        # _lock since they're read/written cross-thread.
        self._lock = threading.Lock()
        self._latest_value = 0
        self._latest_is_new = False

        self._timer = None

        self._action_server = ActionServer(
            self,
            SendTrajectory,
            f'{self.get_name()}/execute_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info('timer_action_test_node ready')

    def goal_callback(self, goal_request):
        if self._active:
            self.get_logger().warn('Goal rejected: another goal is already active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    # ---- CONSUMER: fixed-interval send, never waits on production --------

    def _on_timer_tick(self, goal_handle):
        with self._lock:
            if self._latest_is_new:
                value = self._latest_value
                self._latest_is_new = False
            else:
                value = None

        if value is None:
            self.get_logger().warn(
                'Timer tick: no new trajectory ready (producer out of time) — nothing sent')
            return

        if value % 25 == 0 or value <= 3:
            self.get_logger().info(f'SEND value={value}/{TARGET_COUNT}')

        fb = SendTrajectory.Feedback()
        fb.state = 'EXECUTING'
        fb.step_progression = float(value) / TARGET_COUNT
        fb.position_error = 0.0
        fb.on_target = False
        fb.joint_command = JointStateMsg()
        fb.joint_command.position = [float(value)]
        goal_handle.publish_feedback(fb)

    def _stop_timer(self):
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

    # ---- PRODUCER: execute_callback IS the producer loop -----------------

    def execute_callback(self, goal_handle):
        self.get_logger().info(
            f'Goal accepted, producing continuously (~{PRODUCE_TIME_S * 1000:.0f}ms/value), '
            f'sending every {TICK_PERIOD_S * 1000:.0f}ms')
        self._active = True
        with self._lock:
            self._latest_value = 0
            self._latest_is_new = False

        self._timer = self.create_timer(
            TICK_PERIOD_S, lambda: self._on_timer_tick(goal_handle),
            callback_group=self._cb_group,
        )

        counter = 0
        cancelled = False
        while counter < TARGET_COUNT:
            if goal_handle.is_cancel_requested:
                cancelled = True
                break

            time.sleep(PRODUCE_TIME_S)  # simulated solve
            counter += 1
            with self._lock:
                self._latest_value = counter
                self._latest_is_new = True

        self._stop_timer()

        result = SendTrajectory.Result()
        if cancelled:
            result.success = False
            result.message = f'cancelled at counter={counter}'
            goal_handle.canceled()
        else:
            result.success = True
            result.message = f'completed, counter={counter}'
            goal_handle.succeed()
        self.get_logger().info(result.message)

        self._active = False
        return result


def main():
    rclpy.init()
    node = TimerActionTestNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
