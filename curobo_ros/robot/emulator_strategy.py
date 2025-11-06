from curobo_ros.robot.joint_control_strategy import JointCommandStrategy, RobotState
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import threading
import time


class EmulatorStrategy(JointCommandStrategy):
    '''
    Robot emulator strategy for visualization in RViz.
    Publishes JointState messages to simulate robot movement without a real robot.
    This allows testing and visualization of trajectories in RViz.
    '''

    def __init__(self, node, dt):
        super().__init__(node, dt)

        # Publisher for joint states (standard ROS topic for robot visualization)
        self.pub_joint_states = node.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Emulator state
        self.position_command = []
        self.vel_command = []
        self.accel_command = []
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.command_index = 0
        self.dt = dt
        self.robot_state = RobotState.IDLE
        self.trajectory_progression = 0.0

        self.node = node

        # Thread for trajectory execution simulation
        self.execution_thread = None
        self.stop_execution = threading.Event()

        node.get_logger().info("✅ Emulator strategy initialized - Publishing to /joint_states")

    def send_trajectrory(self):
        '''
        Start simulating trajectory execution by publishing joint states progressively.
        '''
        if len(self.position_command) == 0:
            self.node.get_logger().warn("No trajectory to execute")
            self.trajectory_progression = 1.0
            return

        # Stop any previous execution
        self.stop_execution.set()
        if self.execution_thread is not None and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=1.0)

        # Start new execution thread
        self.stop_execution.clear()
        self.robot_state = RobotState.RUNNING
        self.trajectory_progression = 0.0
        self.command_index = 0

        self.execution_thread = threading.Thread(
            target=self._execute_trajectory,
            daemon=True,
            name="emulator_trajectory_executor"
        )
        self.execution_thread.start()

        self.node.get_logger().info(f"🚀 Emulator: Starting trajectory execution ({len(self.position_command)} points)")

    def _execute_trajectory(self):
        '''
        Thread function that simulates trajectory execution by publishing joint states.
        '''
        try:
            start_time = time.time()
            total_points = len(self.position_command)

            while self.command_index < total_points and not self.stop_execution.is_set():
                # Get current command
                positions = self.position_command[self.command_index]
                velocities = self.vel_command[self.command_index] if self.command_index < len(self.vel_command) else [0.0] * len(positions)

                # Update current position
                self.current_joint_positions = positions

                # Create and publish JointState message
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = positions
                joint_state_msg.velocity = velocities
                joint_state_msg.effort = []

                self.pub_joint_states.publish(joint_state_msg)

                # Update progression
                self.command_index += 1
                self.trajectory_progression = self.command_index / total_points

                # Log progress periodically
                if self.command_index % 50 == 0 or self.command_index == total_points:
                    elapsed = time.time() - start_time
                    self.node.get_logger().info(
                        f"Emulator: {self.trajectory_progression*100:.1f}% complete "
                        f"({self.command_index}/{total_points}) - {elapsed:.2f}s"
                    )

                # Wait for next timestep
                time.sleep(self.dt)

            # Trajectory complete
            if not self.stop_execution.is_set():
                self.trajectory_progression = 1.0
                self.robot_state = RobotState.IDLE
                elapsed = time.time() - start_time
                self.node.get_logger().info(f"✅ Emulator: Trajectory completed in {elapsed:.2f}s")
            else:
                self.node.get_logger().info("⚠️ Emulator: Trajectory execution stopped")

        except Exception as e:
            self.node.get_logger().error(f"❌ Emulator execution error: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())
            self.robot_state = RobotState.ERROR

    def get_joint_pose(self):
        '''
        Return the current joint positions of the emulated robot.
        '''
        return self.current_joint_positions

    def get_joint_name(self):
        '''
        Return the joint names of the emulated robot.
        '''
        return self.joint_names

    def stop_robot(self):
        '''
        Stop the emulated robot trajectory execution.
        '''
        self.node.get_logger().info("🛑 Emulator: Stopping trajectory execution")

        # Signal thread to stop
        self.stop_execution.set()

        # Clear command buffers
        self.vel_command = []
        self.position_command = []
        self.accel_command = []
        self.command_index = 0
        self.trajectory_progression = 0.0
        self.robot_state = RobotState.STOPPED

        # Publish stopped state (zero velocities)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.current_joint_positions
        joint_state_msg.velocity = [0.0] * len(self.joint_names)
        joint_state_msg.effort = []
        self.pub_joint_states.publish(joint_state_msg)

    def get_progression(self):
        '''
        Return the trajectory execution progression (0.0 to 1.0).
        '''
        return self.trajectory_progression

    def __del__(self):
        '''
        Cleanup when strategy is destroyed.
        '''
        self.stop_execution.set()
        if self.execution_thread is not None and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=1.0)
