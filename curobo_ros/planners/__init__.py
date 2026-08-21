"""
Trajectory planning strategies for cuRobo ROS2 integration.

Two families share the same Strategy/Factory switch (SetPlanner / GetPlanners):

Open-loop planners (plan a full trajectory, then execute) — base ``SinglePlanner``:
- ClassicPlanner: single-shot pose planning
- MultiPointPlanner: goalset / multi-waypoint planning
- JointSpacePlanner: joint-space planning

Reactive controllers (closed-loop control loop) — base ``ReactiveController``:
- MPPIController: MPPI built on cuRobo ModelPredictiveControl, via
  optimize_action_sequence() + hand-rolled pacing (registered as 'mpc' in
  PlannerFactory — see planner_factory.py for why)
- LBFGSController: thin wrapper around cuRobo's stock ModelPredictiveControl
  (optimize_action_sequence(), plan/execute-with-overlap scheme -- see
  lbfgs_planner.py's module docstring)
- RetargetController: teleop pose-stream following (cuRobo MotionRetargeter, IK-based)

Both planner families consume the node's single shared context (robot, obstacles,
scene, collision cache), so switching between them keeps the same world.
"""

from .trajectory_planner import TrajectoryPlanner, PlannerResult, ExecutionMode
from .single_planner import SinglePlanner
from .reactive_controller import ReactiveController
from .classic_planner import ClassicPlanner
from .mppi_planner import MPPIController
from .lbfgs_planner import LBFGSController
from .retarget_controller import RetargetController
from .multi_point_planner import MultiPointPlanner
from .joint_space_planner import JointSpacePlanner
from .planner_factory import PlannerFactory, PlannerManager

__all__ = [
    'TrajectoryPlanner',
    'PlannerResult',
    'ExecutionMode',
    'SinglePlanner',
    'ReactiveController',
    'ClassicPlanner',
    'MPPIController',
    'LBFGSController',
    'RetargetController',
    'MultiPointPlanner',
    'JointSpacePlanner',
    'PlannerFactory',
    'PlannerManager',
]
