from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from sensor_msgs.msg import JointState
import threading
import time


class EmulatorStrategy(JointCommandStrategy):
    '''
    Robot emulator strategy for visualization in RViz.
    Publishes JointState messages to simulate robot movement without a real robot.
    This allows testing and visualization of trajectories in RViz.
    '''

    def __init__(self, node, dt, description=None):
        super().__init__(node, dt, description)

        # Publisher for joint states (topic from the descriptor).
        joint_states_topic = self.params.get('joint_states_topic', '/emulator/joint_states')
        self.pub_joint_states = node.create_publisher(JointState, joint_states_topic, 10)

        # Current simulated pose, sized to the robot's DOF (DOF-agnostic).
        self.current_joint_positions = [0.0] * self.dof

        # Thread for trajectory execution simulation
        self.execution_thread = None
        self.stop_execution = threading.Event()

        node.get_logger().info(f"Emulator strategy initialized - Publishing to {joint_states_topic}")

    def _publish_state(self, names, positions, velocities):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
        joint_state_msg.name = names
        joint_state_msg.position = positions
        joint_state_msg.velocity = velocities
        joint_state_msg.effort = []
        self.pub_joint_states.publish(joint_state_msg)

    def _apply_immediate(self, index):
        '''Publish a single point of the current command buffer right away
        (no playback thread) and mark the buffer as fully consumed.'''
        self.stop_execution.set()  # stop any lingering playback thread

        with self.buffer_lock:
            positions = list(self.position_command[index])
            velocities = (
                list(self.vel_command[index]) if self.vel_command
                else [0.0] * len(positions)
            )
            names = list(self.joint_names)
            self.current_joint_positions = positions
            self.robot_state = RobotState.RUNNING
            self.trajectory_progression = 1.0

        self._publish_state(names, positions, velocities)

    def send_trajectrory(self):
        '''
        Start simulating trajectory execution by publishing joint states progressively.
        '''
        with self.buffer_lock:
            n_points = len(self.position_command)

        if n_points == 0:
            self.node.get_logger().warn("No trajectory to execute")
            with self.buffer_lock:
                self.trajectory_progression = 1.0
            return

        # Fast servo path: a single setpoint (reactive / MPC streaming) is applied
        # IMMEDIATELY — no playback thread, no per-point sleep(dt). Without this,
        # each streamed command costs ~dt + thread spawn/join, capping reactive
        # control at a few Hz. With it, the control loop runs at its native rate.
        if n_points == 1:
            self._apply_immediate(0)
            return

        # Reactive controllers (MPC) also stream the FULL predicted horizon on
        # every step (~every 90ms), not a single point — see reactive_controller
        # ._send_command. If the previous multi-point playback thread is still
        # running, we're being re-invoked far faster than that thread can play
        # out (dt=0.02s/point * horizon), a signature open-loop callers never
        # produce (they call send_trajectrory() once and wait for progression
        # to reach 1.0). In that case, threading through the new buffer would
        # just get killed again next step, leaving current_joint_positions
        # (read back into the MPC's state loop) stuck near the start of each
        # horizon. Apply the horizon's last (current-target) point immediately
        # instead — same fast path as the single-point case.
        if self.execution_thread is not None and self.execution_thread.is_alive():
            self._apply_immediate(-1)
            return

        # Stop any previous playback thread and take a consistent snapshot of
        # the buffers this thread will play, under ONE lock hold. The snapshot
        # (not self.position_command etc.) is what the thread operates on — by
        # the time it runs, those attributes may already belong to a NEWER
        # command. epoch is whatever set_command() last bumped it to (the
        # caller always calls set_command() before send_trajectrory()), and
        # tags this specific playback generation.
        self.stop_execution.set()
        prev_thread = self.execution_thread

        with self.buffer_lock:
            positions = [list(p) for p in self.position_command]
            velocities = (
                [list(v) for v in self.vel_command] if self.vel_command
                else [[0.0] * len(p) for p in positions]
            )
            names = list(self.joint_names)
            self.robot_state = RobotState.RUNNING
            self.trajectory_progression = 0.0
            self.command_index = 0
            epoch = self._buffer_epoch

        # Join OUTSIDE the lock: the dying thread only ever needs buffer_lock
        # briefly, per iteration, to notice it's been superseded and exit —
        # joining while holding the lock would deadlock against it.
        if prev_thread is not None and prev_thread.is_alive():
            prev_thread.join(timeout=1.0)

        self.stop_execution.clear()
        self.execution_thread = threading.Thread(
            target=self._execute_trajectory,
            args=(positions, velocities, names, epoch),
            daemon=True,
            name="emulator_trajectory_executor"
        )
        self.execution_thread.start()

    def _execute_trajectory(self, positions_snapshot, velocities_snapshot, names_snapshot, epoch):
        '''
        Thread function that simulates trajectory execution by publishing joint states.

        Pure function of its snapshot — never reads self.position_command /
        self.vel_command / self.joint_names, which may already belong to a
        newer command by the time an iteration runs. Every write to shared
        state happens under a short buffer_lock hold that also re-checks
        self._buffer_epoch == epoch, so a newer set_command()/send_trajectrory()
        /stop_robot() preempts this thread instead of racing it (relying on
        stop_execution alone is not enough: it is a single reused Event that
        gets cleared again for the NEXT generation, so a thread that wakes
        from sleep() after that clear() would otherwise see "not stopped" and
        keep publishing a stale trajectory).
        '''
        try:
            total_points = len(positions_snapshot)
            index = 0

            while index < total_points and not self.stop_execution.is_set():
                positions = positions_snapshot[index]
                velocities = velocities_snapshot[index]

                with self.buffer_lock:
                    if self._buffer_epoch != epoch:
                        return  # preempted by a newer command
                    self.current_joint_positions = positions
                    index += 1
                    self.command_index = index
                    self.trajectory_progression = index / total_points

                self._publish_state(names_snapshot, positions, velocities)

                time.sleep(self.dt)

            with self.buffer_lock:
                if self._buffer_epoch == epoch:
                    self.robot_state = RobotState.IDLE
                    if not self.stop_execution.is_set():
                        self.trajectory_progression = 1.0

        except Exception as e:
            self.node.get_logger().error(f"Emulator execution error: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())
            with self.buffer_lock:
                if self._buffer_epoch == epoch:
                    self.robot_state = RobotState.ERROR

    def get_joint_pose(self):
        '''
        Return the current joint positions of the emulated robot.
        '''
        with self.buffer_lock:
            return list(self.current_joint_positions)

    def get_joint_name(self):
        '''
        Return the joint names of the emulated robot.
        '''
        with self.buffer_lock:
            return list(self.joint_names)

    def wait_for_execution_complete(self, timeout=5.0):
        '''
        Wait for the trajectory execution thread to complete.

        This ensures that the robot's current_joint_positions is fully updated
        before the next planning operation reads it.

        Args:
            timeout: Maximum time to wait in seconds (default: 5.0)

        Returns:
            True if thread completed, False if timeout occurred
        '''
        if self.execution_thread is not None and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=timeout)
            return not self.execution_thread.is_alive()
        return True

    def stop_robot(self):
        '''
        Stop the emulated robot trajectory execution.
        '''
        self.node.get_logger().info("Emulator: Stopping trajectory execution")

        # Signal any playback thread to stop.
        self.stop_execution.set()

        with self.buffer_lock:
            self.vel_command = []
            self.position_command = []
            self.accel_command = []
            self.command_index = 0
            self.trajectory_progression = 0.0
            self.robot_state = RobotState.STOPPED
            # Bump the epoch so a still-running playback thread's per-iteration
            # check sees it has been superseded, even if stop_execution alone
            # would be ambiguous with a subsequent send_trajectrory().
            self._buffer_epoch += 1
            names = list(self.joint_names)
            positions = list(self.current_joint_positions)

        self._publish_state(names, positions, [0.0] * len(names))

    def get_progression(self):
        '''
        Return the trajectory execution progression (0.0 to 1.0).
        '''
        with self.buffer_lock:
            return self.trajectory_progression

    def __del__(self):
        '''
        Cleanup when strategy is destroyed.
        '''
        self.stop_execution.set()
        if self.execution_thread is not None and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=1.0)
