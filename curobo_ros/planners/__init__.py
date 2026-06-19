"""
Trajectory planning strategies for cuRobo ROS2 integration.

Two families share the same Strategy/Factory switch (SetPlanner / GetPlanners):

Open-loop planners (plan a full trajectory, then execute) — base ``SinglePlanner``:
- ClassicPlanner: single-shot pose planning
- MultiPointPlanner: goalset / multi-waypoint planning
- JointSpacePlanner: joint-space planning

Reactive controllers (closed-loop control loop) — base ``ReactiveController``:
- MPCController: Model Predictive Control (real-time, cuRobo ModelPredictiveControl)
- RetargetController: teleop pose-stream following (cuRobo MotionRetargeter, IK-based)

GraspPlanner is a standalone helper (its own ``plan_grasp`` action, not a
SetPlanner strategy) wrapping cuRobo's native grasp pipeline.

Both planner families consume the node's single shared context (robot, obstacles,
scene, collision cache), so switching between them keeps the same world.
"""

from .trajectory_planner import TrajectoryPlanner, PlannerResult, ExecutionMode
from .single_planner import SinglePlanner
from .reactive_controller import ReactiveController
from .classic_planner import ClassicPlanner
from .mpc_planner import MPCController, MPCPlanner
from .retarget_controller import RetargetController
from .grasp_planner import GraspPlanner
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
    'RetargetController',
    'GraspPlanner',
    'MultiPointPlanner',
    'JointSpacePlanner',
    'PlannerFactory',
    'PlannerManager',
]
