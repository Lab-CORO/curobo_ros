# MPC Planner Implementation Guide

> **For Developers**: This document provides technical guidance for implementing the MPC planner in curobo_ros. It complements the [Unified Planner Architecture](unified_planner.md) specification and [MPC Planner Tutorial](../tutorials/5_mpc_planner.md) user documentation.

---

## Overview

The MPC (Model Predictive Control) planner implementation will add closed-loop, real-time trajectory planning capabilities to curobo_ros. This guide covers:

1. Architecture integration points
2. cuRobo MPC API usage
3. Real-time control loop design
4. ROS 2 interface implementation
5. Testing and validation strategy

---

## Architecture Integration

### File Structure

```
curobo_ros/
├── planners/
│   ├── __init__.py
│   ├── base_planner.py          # Abstract base (Phase 2)
│   ├── planner_factory.py       # Factory pattern (Phase 2)
│   ├── planner_manager.py       # Runtime switching (Phase 2)
│   ├── classic_planner.py       # Existing MotionGen wrapper (Phase 2)
│   └── mpc_planner.py           # ⭐ MPC implementation (Phase 3)
├── core/
│   ├── config_wrapper.py        # Extend with MPC params
│   └── config_wrapper_motion.py # MPC config integration
└── interfaces/
    └── unified_planner_node.py  # ROS node exposing all planners
```

### Integration with Existing Code

The MPC planner integrates with:

1. **ConfigWrapper** (`curobo_ros/core/config_wrapper.py`)
   - Add MPC-specific parameters
   - Manage solver configuration
   - Handle GPU resource allocation

2. **RobotContext** (`curobo_ros/robot/robot_context.py`)
   - Real-time state feedback loop
   - Command publishing at control frequency
   - Joint state monitoring

3. **CollisionContext** (voxel grid updates)
   - Dynamic obstacle updates during execution
   - Camera integration for real-time perception

---

## cuRobo MPC API Integration

### Understanding cuRobo's MPC

cuRobo provides MPC capabilities through its optimization framework. Key components:

```python
from curobo.wrap.reacher.mpc import MpcSolver
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.types.state import JointState

# MPC solver initialization
mpc_solver = MpcSolver(
    robot_config=robot_config,
    world_model=world_model,
    mpc_config=mpc_config
)
```

### Key cuRobo APIs to Use

1. **MpcSolver.solve_step()** - Single MPC iteration
2. **MpcSolver.update_goal()** - Update target during execution
3. **MpcSolver.update_world()** - Update obstacles in real-time
4. **MpcSolver.get_trajectory()** - Get predicted trajectory
5. **MpcSolver.reset()** - Reset for new planning task

### Configuration Parameters

Map ROS parameters to cuRobo MPC config:

```python
# ROS parameters → cuRobo MpcConfig
mpc_config = MpcConfig(
    horizon=self.get_parameter('mpc_horizon').value,  # 5-15 steps
    dt=1.0 / self.get_parameter('mpc_control_frequency').value,  # 0.01s @ 100Hz
    max_iterations=self.get_parameter('mpc_max_iterations').value,  # 1000
    convergence_threshold=self.get_parameter('mpc_convergence_threshold').value,  # 0.01m
    cost_weights={
        'position': 1.0,
        'rotation': 1.0,
        'smoothness': 0.1,
        'obstacle': 10.0
    }
)
```

---

## MPC Planner Class Design

### Class Structure

```python
# curobo_ros/planners/mpc_planner.py

from .base_planner import TrajectoryPlanner, PlannerResult, ExecutionMode
from curobo.wrap.reacher.mpc import MpcSolver
import rclpy
from rclpy.node import Node
import threading

class MPCPlanner(TrajectoryPlanner):
    """
    Model Predictive Control planner for real-time reactive trajectory planning.

    Implements closed-loop control with continuous replanning at each control step.
    """

    def __init__(self, node: Node, config_wrapper):
        """
        Initialize MPC planner.

        Args:
            node: ROS 2 node for parameter access and communication
            config_wrapper: Configuration wrapper with MPC parameters
        """
        self.node = node
        self.config_wrapper = config_wrapper

        # Load MPC-specific parameters
        self.horizon = node.get_parameter('mpc_horizon').value
        self.control_freq = node.get_parameter('mpc_control_frequency').value
        self.max_iterations = node.get_parameter('mpc_max_iterations').value
        self.convergence_threshold = node.get_parameter('mpc_convergence_threshold').value

        # Initialize cuRobo MPC solver
        self.mpc_solver = self._create_mpc_solver()

        # Control loop state
        self.is_executing = False
        self.execution_thread = None
        self.goal_pose = None

        # Performance monitoring
        self.stats_publisher = node.create_publisher(
            MPCStats,
            'mpc_stats',
            10
        )

        self.node.get_logger().info('MPC Planner initialized')

    def _create_mpc_solver(self) -> MpcSolver:
        """Create and configure cuRobo MPC solver."""
        # Get robot and world config from ConfigWrapper
        robot_config = self.config_wrapper.get_robot_config()
        world_model = self.config_wrapper.get_world_model()

        # Build MPC configuration
        mpc_config = MpcConfig(
            horizon=self.horizon,
            dt=1.0 / self.control_freq,
            max_iterations=self.max_iterations,
            convergence_threshold=self.convergence_threshold,
            # Cost weights for optimization
            cost_weights={
                'position_error': 1.0,
                'rotation_error': 1.0,
                'smoothness': 0.1,
                'obstacle_avoidance': 10.0,
                'joint_limits': 5.0,
                'velocity_limits': 2.0
            }
        )

        return MpcSolver(
            robot_config=robot_config,
            world_model=world_model,
            mpc_config=mpc_config
        )

    def get_execution_mode(self) -> ExecutionMode:
        """MPC uses closed-loop execution."""
        return ExecutionMode.CLOSED_LOOP

    def plan(self, start_state, goal_pose, config) -> PlannerResult:
        """
        Plan initial trajectory to goal using MPC.

        For MPC, this performs initial feasibility check and warm-start.
        The actual trajectory is generated continuously during execute().

        Args:
            start_state: Current robot joint state
            goal_pose: Target end-effector pose
            config: Planning configuration (optional overrides)

        Returns:
            PlannerResult with initial trajectory and feasibility
        """
        self.node.get_logger().info('MPC planning started')

        # Store goal for execution phase
        self.goal_pose = goal_pose

        # Reset MPC solver
        self.mpc_solver.reset()
        self.mpc_solver.update_goal(goal_pose)

        # Perform initial solve for feasibility check
        initial_result = self.mpc_solver.solve_step(
            current_state=start_state,
            goal_pose=goal_pose
        )

        if not initial_result.success:
            self.node.get_logger().error(
                f'MPC initial planning failed: {initial_result.error_msg}'
            )
            return PlannerResult(
                success=False,
                message='MPC initial solve failed - goal may be unreachable',
                trajectory=None
            )

        # Return initial trajectory (will be refined during execution)
        return PlannerResult(
            success=True,
            message='MPC planning succeeded - ready for closed-loop execution',
            trajectory=initial_result.trajectory,
            computation_time=initial_result.solve_time
        )

    def execute(self, robot_context, goal_handle) -> bool:
        """
        Execute trajectory using closed-loop MPC control.

        Continuously replans at control frequency until goal is reached.

        Args:
            robot_context: Interface to robot for state/command
            goal_handle: ROS action goal handle for feedback/cancellation

        Returns:
            True if goal reached successfully, False if failed/cancelled
        """
        self.node.get_logger().info('MPC execution started')

        self.is_executing = True
        control_rate = self.node.create_rate(self.control_freq)

        iteration = 0
        start_time = self.node.get_clock().now()

        try:
            while self.is_executing and rclpy.ok():
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self.node.get_logger().warn('MPC execution cancelled')
                    goal_handle.canceled()
                    return False

                # Get current robot state
                current_state = robot_context.get_current_joint_state()

                # Update world model (dynamic obstacles from cameras)
                self._update_world_model()

                # Solve MPC step
                step_start = self.node.get_clock().now()
                result = self.mpc_solver.solve_step(
                    current_state=current_state,
                    goal_pose=self.goal_pose
                )
                step_time = (self.node.get_clock().now() - step_start).nanoseconds / 1e6

                if not result.success:
                    self.node.get_logger().error(f'MPC step failed: {result.error_msg}')
                    return False

                # Execute first control action
                command = result.trajectory.position[0]  # First waypoint
                robot_context.send_joint_command(command)

                # Compute distance to goal
                current_pose = robot_context.compute_fk(current_state)
                goal_error = self._compute_pose_error(current_pose, self.goal_pose)

                # Publish performance stats
                self._publish_stats(iteration, goal_error, step_time, result)

                # Publish feedback
                feedback = SendTrajectoryFeedback()
                feedback.progress = max(0.0, 1.0 - goal_error / 1.0)  # Normalize
                feedback.current_error = goal_error
                goal_handle.publish_feedback(feedback)

                # Check convergence
                if goal_error < self.convergence_threshold:
                    self.node.get_logger().info(
                        f'MPC converged after {iteration} iterations '
                        f'({(self.node.get_clock().now() - start_time).nanoseconds / 1e9:.2f}s)'
                    )
                    goal_handle.succeed()
                    return True

                # Check max iterations
                if iteration >= self.max_iterations:
                    self.node.get_logger().warn(
                        f'MPC reached max iterations ({self.max_iterations}) '
                        f'without converging (error: {goal_error:.4f}m)'
                    )
                    return False

                iteration += 1
                control_rate.sleep()

            return False

        except Exception as e:
            self.node.get_logger().error(f'MPC execution error: {str(e)}')
            return False

        finally:
            self.is_executing = False

    def _update_world_model(self):
        """Update collision world with latest camera data."""
        # Get latest voxel grid from cameras
        voxel_grid = self.config_wrapper.get_voxel_grid()

        # Update MPC solver's world model
        self.mpc_solver.update_world(voxel_grid)

    def _compute_pose_error(self, current_pose, goal_pose) -> float:
        """Compute position + orientation error to goal."""
        pos_error = np.linalg.norm(
            current_pose.position - goal_pose.position
        )

        # Quaternion distance (simplified)
        rot_error = 1.0 - abs(np.dot(
            current_pose.quaternion,
            goal_pose.quaternion
        ))

        # Weighted combination
        return pos_error + 0.1 * rot_error

    def _publish_stats(self, iteration, error, step_time, result):
        """Publish MPC performance statistics."""
        stats = MPCStats()
        stats.stamp = self.node.get_clock().now().to_msg()
        stats.iteration = iteration
        stats.convergence_error = error
        stats.computation_time = step_time
        stats.horizon_length = self.horizon
        stats.success = result.success

        self.stats_publisher.publish(stats)

    def stop(self):
        """Stop MPC execution immediately."""
        self.is_executing = False
        self.node.get_logger().info('MPC execution stopped')

    def update_parameters(self, params: dict):
        """Dynamically update MPC parameters."""
        if 'mpc_horizon' in params:
            self.horizon = params['mpc_horizon']
            self.mpc_solver.update_config(horizon=self.horizon)

        if 'mpc_convergence_threshold' in params:
            self.convergence_threshold = params['mpc_convergence_threshold']

        # Recreate solver if major config changed
        if 'mpc_control_frequency' in params:
            self.control_freq = params['mpc_control_frequency']
            self.mpc_solver = self._create_mpc_solver()
```

---

## ROS 2 Parameters

Add to `curobo_ros/core/config_wrapper.py`:

```python
# MPC Planner Parameters
self.declare_parameter('mpc_convergence_threshold', 0.01)
self.declare_parameter('mpc_max_iterations', 1000)
self.declare_parameter('mpc_horizon', 10)
self.declare_parameter('mpc_control_frequency', 100.0)
self.declare_parameter('mpc_position_weight', 1.0)
self.declare_parameter('mpc_rotation_weight', 1.0)
self.declare_parameter('mpc_smoothness_weight', 0.1)
self.declare_parameter('mpc_obstacle_weight', 10.0)
```

---

## ROS 2 Messages

Add to `curobo_msgs/msg/MPCStats.msg`:

```
# MPC performance statistics
std_msgs/Header header

int32 iteration              # Current iteration number
float64 convergence_error    # Distance to goal (meters)
float64 computation_time     # Step solve time (ms)
int32 horizon_length         # Planning horizon
bool success                 # Step succeeded
```

---

## Testing Strategy

### Unit Tests

```python
# tests/test_mpc_planner.py

def test_mpc_planner_initialization():
    """Test MPC planner creates successfully."""
    planner = MPCPlanner(node, config_wrapper)
    assert planner.get_execution_mode() == ExecutionMode.CLOSED_LOOP

def test_mpc_planning_feasibility():
    """Test MPC can find initial solution."""
    start = get_test_joint_state()
    goal = get_reachable_pose()

    result = planner.plan(start, goal, {})
    assert result.success
    assert result.trajectory is not None

def test_mpc_execution_convergence():
    """Test MPC execution converges to goal."""
    # Mock robot context
    robot_context = MockRobotContext()
    goal_handle = MockGoalHandle()

    success = planner.execute(robot_context, goal_handle)
    assert success
    assert robot_context.final_error < 0.01
```

### Integration Tests

```python
def test_mpc_with_dynamic_obstacles():
    """Test MPC avoids obstacles added during execution."""
    # Start MPC execution
    threading.Thread(target=planner.execute, args=(robot_context, goal_handle)).start()

    # Wait then add obstacle in path
    time.sleep(1.0)
    add_collision_sphere([0.5, 0.3, 0.4], radius=0.1)

    # MPC should avoid and still reach goal
    wait_for_completion(timeout=10.0)
    assert goal_reached()
    assert no_collisions()
```

---

## Performance Optimization

### GPU Memory Management

- Pre-allocate tensors for MPC solver
- Reuse memory across iterations
- Monitor GPU utilization

### Real-Time Considerations

- Use dedicated high-priority thread for control loop
- Ensure control frequency is maintained
- Add watchdog for missed deadlines

### Benchmarking

Target performance:
- Control frequency: 50-100 Hz
- Step solve time: < 10 ms (at 100 Hz)
- Convergence: < 5 seconds for typical tasks
- GPU usage: 50-80% sustained

---

## Future Enhancements

1. **Adaptive Horizon**: Adjust horizon based on distance to goal
2. **Warm-Start**: Use previous solution for faster convergence
3. **Multi-Goal MPC**: Switch goals dynamically
4. **Learned Cost Functions**: Integrate learned models
5. **Distributed MPC**: Multi-robot coordination

---

## References

- [cuRobo MPC Documentation](https://curobo.org/source/getting_started/2b_mpc_example.html)
- [ROS 2 Real-Time Best Practices](https://design.ros2.org/articles/realtime_background.html)
- [MPC Theory (Wikipedia)](https://en.wikipedia.org/wiki/Model_predictive_control)
- [Unified Planner Architecture](unified_planner.md)
- [MPC Planner Tutorial](../tutorials/5_mpc_planner.md)

---

## Questions / Support

For implementation questions:
1. Check existing robot strategy pattern in `curobo_ros/robot/`
2. Review `config_wrapper.py` for parameter management patterns
3. Refer to cuRobo examples in `curobo/examples/`
4. Open GitHub issue with `[MPC Implementation]` tag
