"""
Trajectory planning strategies for cuRobo ROS2 integration.

Two families share the same Strategy/Factory switch (SetPlanner / GetPlanners):

Open-loop planners (plan a full trajectory, then execute) — base ``SinglePlanner``:
- ClassicPlanner: single-shot pose planning
- MultiPointPlanner: goalset / multi-waypoint planning
- JointSpacePlanner: joint-space planning

Reactive controllers (closed-loop control loop) — base ``ReactiveController``:
- MPCController: Model Predictive Control (real-time, cuRobo ModelPredictiveControl, MPPI or LBFGS
  via optimize_action_sequence + hand-rolled pacing)
- LBFGSController: LBFGS+B-spline MPC built directly on cuRobo's optimize_next_action() API
- RetargetController: teleop pose-stream following (cuRobo MotionRetargeter, IK-based)

Both planner families consume the node's single shared context (robot, obstacles,
scene, collision cache), so switching between them keeps the same world.
"""

from .trajectory_planner import TrajectoryPlanner, PlannerResult, ExecutionMode
from .single_planner import SinglePlanner
from .reactive_controller import ReactiveController
from .classic_planner import ClassicPlanner
from .mpc_planner import MPCController, MPCPlanner
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
    'MPCController',
    'MPCPlanner',
    'LBFGSController',
    'RetargetController',
    'MultiPointPlanner',
    'JointSpacePlanner',
    'PlannerFactory',
    'PlannerManager',
]
