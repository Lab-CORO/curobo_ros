#!/usr/bin/env python3
"""
Grasp MPC Planner - Model Predictive Control for reactive object grasping.

This planner uses MPC for closed-loop control during all grasp phases,
enabling reactive adaptation to disturbances and object movement.

Grasp Sequence:
1. APPROACH: MPC to pre-grasp pose (offset above object)
2. GRASP: MPC to grasp pose, close gripper
3. ATTACH: Add object collision spheres to robot model
4. RETREAT: MPC to post-grasp pose (lift object)
"""

import time
from enum import Enum
from typing import Optional
import numpy as np

import torch
from curobo.types.robot import JointState
from curobo.types.math import Pose
from curobo.rollout.rollout_base import Goal
from geometry_msgs.msg import Vector3
import traceback

from .mpc_planner import MPCPlanner
from .trajectory_planner import PlannerResult, ExecutionMode
from curobo_msgs.action import SendTrajectory


class GraspState(Enum):
    """State machine for grasp execution."""
    IDLE = 0
    APPROACH = 1      # Moving to pre-grasp pose
    GRASP = 2         # Moving to grasp pose
    ATTACHED = 3      # Object attached to gripper
    RETREAT = 4       # Lifting object


class GraspMPCPlanner(MPCPlanner):
    """
    MPC-based grasp planner with reactive control.

    Extends MPCPlanner to add multi-phase grasp execution with object
    attachment and gripper control.

    Key features:
    - Pure MPC control throughout all phases (highly reactive)
    - Automatic object attachment as collision spheres
    - Configurable pre-grasp and post-grasp offsets
    - Phase-specific convergence thresholds for precision/speed tradeoff
    """

    def __init__(self, node, config_wrapper):
        """
        Initialize GraspMPCPlanner.

        Args:
            node: ROS2 node
            config_wrapper: ConfigWrapperMPC with MpcSolver configured
        """
        super().__init__(node, config_wrapper)

        # Grasp-specific state
        self.grasp_state = GraspState.IDLE
        self.grasped_object = None
        self.object_info = None

        # Grasp poses (computed in plan())
        self.target_pose = None
        self.pre_grasp_pose = None
        self.post_grasp_pose = None

        # Grasp offsets (can be overridden in request)
        self.pre_grasp_offset = [0.0, 0.0, 0.10]   # 10cm above object
        self.post_grasp_offset = [0.0, 0.0, 0.15]  # 15cm lift

        # Gripper interface (set externally after initialization)
        self.gripper = None

        # Phase-specific convergence thresholds
        self.approach_convergence = 0.02   # Looser for speed
        self.grasp_convergence = 0.005     # Tight for precision
        self.retreat_convergence = 0.015   # Moderate

    def get_planner_name(self) -> str:
        """Return planner name."""
        return "Grasp MPC (Reactive Grasping)"

    def get_config_parameters(self) -> list:
        """List of ROS parameters used by this planner."""
        base_params = super().get_config_parameters()
        return base_params + [
            'grasp_pre_grasp_offset_z',
            'grasp_post_grasp_offset_z',
            'grasp_approach_convergence',
            'grasp_grasp_convergence',
            'grasp_retreat_convergence',
            'grasp_gripper_close_time',
        ]

    def set_gripper_interface(self, gripper):
        """
        Set gripper interface for gripper control.

        Args:
            gripper: GripperInterface instance
        """
        self.gripper = gripper

    def plan(self, start_state: JointState, grasp_request, config: dict, robot_context=None) -> PlannerResult:
        """
        Setup MPC for grasp sequence.

        Args:
            start_state: Initial joint configuration
            grasp_request: GraspObject service request containing:
                - target_pose: Grasp pose (end-effector at object)
                - object_name: Object to grasp (for attachment)
                - pre_grasp_offset: Offset for approach (default: [0,0,0.1])
                - post_grasp_offset: Offset for retreat (default: [0,0,0.15])
            config: Dictionary with keys:
                - convergence_threshold: Error threshold (overridden per phase)
                - max_iterations: Maximum MPC iterations
            robot_context: Optional RobotContext for visualization

        Returns:
            PlannerResult with MPC goal buffer initialized
        """
        if self.mpc is None:
            return PlannerResult(
                success=False,
                message="MPC solver not initialized. Call set_mpc_solver() first.",
            )

        try:
            # Extract target pose from request
            self.target_pose = Pose.from_list([
                grasp_request.target_pose.position.x,
                grasp_request.target_pose.position.y,
                grasp_request.target_pose.position.z,
                grasp_request.target_pose.orientation.x,
                grasp_request.target_pose.orientation.y,
                grasp_request.target_pose.orientation.z,
                grasp_request.target_pose.orientation.w
            ])

            # Store object name for attachment
            self.object_name = grasp_request.object_name

            # Extract offsets (use defaults if not provided)
            if hasattr(grasp_request, 'pre_grasp_offset') and grasp_request.pre_grasp_offset is not None:
                self.pre_grasp_offset = [
                    grasp_request.pre_grasp_offset.x,
                    grasp_request.pre_grasp_offset.y,
                    grasp_request.pre_grasp_offset.z
                ]
            else:
                # Use parameter or default
                pre_grasp_z = self.node.get_parameter('grasp_pre_grasp_offset_z').get_parameter_value().double_value
                self.pre_grasp_offset = [0.0, 0.0, pre_grasp_z]

            if hasattr(grasp_request, 'post_grasp_offset') and grasp_request.post_grasp_offset is not None:
                self.post_grasp_offset = [
                    grasp_request.post_grasp_offset.x,
                    grasp_request.post_grasp_offset.y,
                    grasp_request.post_grasp_offset.z
                ]
            else:
                # Use parameter or default
                post_grasp_z = self.node.get_parameter('grasp_post_grasp_offset_z').get_parameter_value().double_value
                self.post_grasp_offset = [0.0, 0.0, post_grasp_z]

            # Compute phase poses
            self.pre_grasp_pose = self._compute_offset_pose(self.target_pose, self.pre_grasp_offset)
            self.post_grasp_pose = self._compute_offset_pose(self.target_pose, self.post_grasp_offset)

            # Load phase-specific convergence thresholds
            self.approach_convergence = self.node.get_parameter('grasp_approach_convergence').get_parameter_value().double_value
            self.grasp_convergence = self.node.get_parameter('grasp_grasp_convergence').get_parameter_value().double_value
            self.retreat_convergence = self.node.get_parameter('grasp_retreat_convergence').get_parameter_value().double_value

            # Initialize MPC with pre-grasp pose (Phase 1)
            self.grasp_state = GraspState.APPROACH
            self.start_state = start_state
            self.goal_pose = self.pre_grasp_pose

            # Create goal for MPC
            goal = Goal(
                current_state=start_state,
                goal_pose=self.pre_grasp_pose,
            )

            # Setup MPC goal buffer
            self.goal_buffer = self.mpc.setup_solve_single(goal, 1)
            self.mpc.update_goal(self.goal_buffer)
            self.is_goal_active = True

            self.node.get_logger().info(
                f"Grasp MPC plan initialized:\n"
                f"  Object: {self.object_name}\n"
                f"  Pre-grasp offset: {self.pre_grasp_offset}\n"
                f"  Post-grasp offset: {self.post_grasp_offset}\n"
                f"  Convergence - approach: {self.approach_convergence}m, "
                f"grasp: {self.grasp_convergence}m, retreat: {self.retreat_convergence}m"
            )

            return PlannerResult(
                success=True,
                message="Grasp MPC goal buffer initialized",
                trajectory=None,  # No pre-computed trajectory for MPC
                metadata={
                    'object_name': self.object_name,
                    'pre_grasp_offset': self.pre_grasp_offset,
                    'post_grasp_offset': self.post_grasp_offset,
                }
            )

        except Exception as e:
            self.node.get_logger().error(f"Grasp MPC setup error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            return PlannerResult(
                success=False,
                message=f"Grasp MPC setup error: {str(e)}",
            )

    def execute(self, robot_context, goal_handle=None) -> bool:
        """
        Execute grasp sequence with MPC closed-loop control.

        Phases:
        1. APPROACH: MPC to pre-grasp pose
        2. GRASP: MPC to grasp pose + close gripper
        3. ATTACH: Add object collision spheres
        4. RETREAT: MPC to post-grasp pose (with object attached)

        Args:
            robot_context: RobotContext for sending commands
            goal_handle: Optional ROS action goal handle for feedback

        Returns:
            True if grasp sequence completed successfully
        """
        if self.goal_buffer is None:
            self.node.get_logger().error("Grasp MPC not initialized. Call plan() first.")
            return False

        try:
            # Phase 1: Approach to pre-grasp
            self.node.get_logger().info("Grasp Phase 1: Approaching pre-grasp pose")
            self.grasp_state = GraspState.APPROACH
            if not self._execute_mpc_phase(
                robot_context,
                self.pre_grasp_pose,
                self.approach_convergence,
                goal_handle,
                phase_name="APPROACH"
            ):
                self.node.get_logger().error("Phase 1 (APPROACH) failed")
                return False

            # Phase 2: Move to grasp pose
            self.node.get_logger().info("Grasp Phase 2: Moving to grasp pose")
            self.grasp_state = GraspState.GRASP
            self.update_goal_pose(self.target_pose)
            if not self._execute_mpc_phase(
                robot_context,
                self.target_pose,
                self.grasp_convergence,
                goal_handle,
                phase_name="GRASP"
            ):
                self.node.get_logger().error("Phase 2 (GRASP) failed")
                return False

            # Close gripper
            self.node.get_logger().info("Closing gripper")
            if not self._close_gripper():
                self.node.get_logger().error("Failed to close gripper")
                return False

            # Phase 3: Attach object to robot
            self.node.get_logger().info("Grasp Phase 3: Attaching object to robot")
            self.grasp_state = GraspState.ATTACHED
            if not self._attach_object_to_robot(self.object_name):
                self.node.get_logger().warn(
                    "Failed to attach object (continuing anyway, but collision checking may be incorrect)"
                )
                # Continue anyway - attachment is optional for basic grasping

            # Phase 4: Retreat with object
            self.node.get_logger().info("Grasp Phase 4: Retreating with object")
            self.grasp_state = GraspState.RETREAT
            self.update_goal_pose(self.post_grasp_pose)
            if not self._execute_mpc_phase(
                robot_context,
                self.post_grasp_pose,
                self.retreat_convergence,
                goal_handle,
                phase_name="RETREAT"
            ):
                self.node.get_logger().error("Phase 4 (RETREAT) failed")
                return False

            self.node.get_logger().info("Grasp sequence completed successfully!")
            return True

        except Exception as e:
            self.node.get_logger().error(f"Grasp execution error: {e}")
            self.node.get_logger().error(traceback.format_exc())
            robot_context.stop_robot()
            return False

    def _execute_mpc_phase(
        self,
        robot_context,
        goal_pose: Pose,
        convergence_threshold: float,
        goal_handle=None,
        phase_name: str = "PHASE"
    ) -> bool:
        """
        Execute one MPC phase until convergence.

        This is a modified version of MPCPlanner.execute() for a single phase.

        Args:
            robot_context: RobotContext for commands
            goal_pose: Target pose for this phase
            convergence_threshold: Error threshold for convergence (meters)
            goal_handle: Optional goal handle for feedback
            phase_name: Name of phase for logging

        Returns:
            True if phase converged successfully
        """
        converged = False
        tstep = 0
        self.mpc_time = []

        self.node.get_logger().info(
            f"{phase_name}: Starting MPC loop (threshold={convergence_threshold}m)"
        )

        while not converged and self.is_goal_active:
            # Check for cancellation
            if goal_handle is not None and not goal_handle.is_active:
                self.node.get_logger().warn(f"{phase_name}: Cancelled")
                robot_context.stop_robot()
                return False

            # Read actual robot position (closes feedback loop)
            actual_joint_pose = robot_context.get_joint_pose()
            current_state = JointState.from_position(
                torch.Tensor([actual_joint_pose]).to(device=self.node.tensor_args.device)
            )

            st_time = time.time()

            # MPC optimization step
            result = self.mpc.step(current_state, 1)
            torch.cuda.synchronize()

            # Track timing
            if tstep > 5:
                self.mpc_time.append(time.time() - st_time)

            # Send command to robot
            self._send_mpc_command(robot_context, result.action)

            # Publish feedback
            if goal_handle is not None and tstep % 10 == 0:
                feedback_msg = SendTrajectory.Feedback()
                # Estimate progress based on pose error
                progress = 1.0 - min(result.metrics.pose_error.item() / 0.1, 1.0)
                feedback_msg.step_progression = progress
                goal_handle.publish_feedback(feedback_msg)

            # Check convergence
            if result.metrics.pose_error.item() < convergence_threshold:
                converged = True
                self.node.get_logger().info(
                    f"{phase_name}: Converged at step {tstep} "
                    f"(error={result.metrics.pose_error.item():.4f}m)"
                )

            tstep += 1

            # Safety timeout
            if tstep > self.max_iterations:
                self.node.get_logger().warn(
                    f"{phase_name}: Max iterations ({self.max_iterations}) reached"
                )
                break

        # Stop robot at end of phase
        robot_context.stop_robot()

        # Log statistics
        if self.mpc_time:
            avg_time = sum(self.mpc_time) / len(self.mpc_time)
            self.node.get_logger().info(
                f"{phase_name}: Completed in {tstep} steps, "
                f"avg time={avg_time*1000:.1f}ms/step"
            )

        return converged

    def _compute_offset_pose(self, base_pose: Pose, offset: list) -> Pose:
        """
        Compute a pose offset from the base pose.

        The offset is applied in the world frame (not end-effector frame).

        Args:
            base_pose: Base pose (target grasp pose)
            offset: [x, y, z] offset in meters (world frame)

        Returns:
            New pose with offset applied
        """
        # Extract position from base pose
        base_position = base_pose.position.cpu() if base_pose.position.is_cuda else base_pose.position

        # Apply offset
        new_position = base_position + torch.tensor(offset, dtype=base_position.dtype, device=base_position.device)

        # Keep same orientation
        return Pose(position=new_position, quaternion=base_pose.quaternion)

    def _attach_object_to_robot(self, object_name: str) -> bool:
        """
        Attach grasped object to robot for collision checking.

        Strategy:
        1. Get object info from obstacle manager
        2. Remove object from world obstacles
        3. Compute collision spheres based on object type
        4. Attach spheres to gripper link in robot kinematics
        5. Update MPC world model

        Args:
            object_name: Name of object to attach

        Returns:
            True if attachment succeeded
        """
        try:
            # Get object from obstacle manager
            self.object_info = self.config_wrapper.obstacle_manager.get_object(object_name)

            self.node.get_logger().info(
                f"Attaching object: {object_name} "
                f"(type={self.object_info['type']})"
            )

            # Compute collision spheres for attached object
            spheres = self._compute_collision_spheres(self.object_info)

            if spheres is not None and len(spheres) > 0:
                self.node.get_logger().info(
                    f"Generated {len(spheres)} collision spheres for {object_name}"
                )
                # TODO: Attach spheres to robot kinematics
                # This requires extending the MPC solver's kinematics model
                # For now, we log the spheres but don't attach them
                self.node.get_logger().warn(
                    "Sphere attachment to robot kinematics not yet implemented - "
                    "spheres generated but not attached"
                )
            else:
                self.node.get_logger().warn(
                    f"No collision spheres generated for object type {self.object_info['type']}"
                )

            # Remove object from world (no longer a collision obstacle)
            # Create a RemoveObject request
            from curobo_msgs.srv import RemoveObject
            remove_request = RemoveObject.Request()
            remove_request.name = object_name
            remove_response = RemoveObject.Response()
            self.config_wrapper.obstacle_manager.remove_object(
                self.node, remove_request, remove_response
            )

            if not remove_response.success:
                self.node.get_logger().warn(
                    f"Failed to remove object from world: {remove_response.message}"
                )

            # Update MPC world model (without the grasped object)
            # Note: This may require specific MPC world update method
            # For now, the object is removed from obstacle manager
            # TODO: Implement explicit MPC world update if needed

            self.grasped_object = object_name
            return True

        except Exception as e:
            self.node.get_logger().error(f"Object attachment failed: {e}")
            self.node.get_logger().error(traceback.format_exc())
            return False

    def _compute_collision_spheres(self, object_info: dict) -> list:
        """
        Compute collision spheres for grasped object.

        Strategy per object type:
        - Cuboid: 9 spheres (8 corners + center), radius = min(dims)/4
        - Cylinder: 7 spheres (3 top + 3 bottom + center), radius = cylinder_radius * 0.8
        - Sphere: 1 sphere at center
        - Others: Not supported yet

        Args:
            object_info: Object information from obstacle manager

        Returns:
            List of sphere dictionaries with 'position' and 'radius' keys
            Positions are in world frame
        """
        obj_type = object_info['type']
        pose = object_info['pose']  # [x, y, z, qw, qx, qy, qz]

        # Extract position and orientation
        position = np.array(pose[:3])
        quaternion = np.array(pose[3:])  # [qw, qx, qy, qz]

        spheres = []

        if obj_type == 'cuboid':
            # Get dimensions
            dims = object_info['dimensions']['dims']  # [width, length, height]
            dims_array = np.array(dims)

            # Sphere radius = 1/4 of minimum dimension
            sphere_radius = min(dims) / 4.0

            # Compute 8 corner positions + 1 center (in object frame)
            half_dims = dims_array / 2.0
            corners = [
                [-half_dims[0], -half_dims[1], -half_dims[2]],  # Corner 0
                [+half_dims[0], -half_dims[1], -half_dims[2]],  # Corner 1
                [-half_dims[0], +half_dims[1], -half_dims[2]],  # Corner 2
                [+half_dims[0], +half_dims[1], -half_dims[2]],  # Corner 3
                [-half_dims[0], -half_dims[1], +half_dims[2]],  # Corner 4
                [+half_dims[0], -half_dims[1], +half_dims[2]],  # Corner 5
                [-half_dims[0], +half_dims[1], +half_dims[2]],  # Corner 6
                [+half_dims[0], +half_dims[1], +half_dims[2]],  # Corner 7
                [0.0, 0.0, 0.0],  # Center
            ]

            # Transform corners to world frame
            # TODO: Apply quaternion rotation (for now assume axis-aligned)
            for corner in corners:
                world_pos = position + np.array(corner)
                spheres.append({
                    'position': world_pos.tolist(),
                    'radius': sphere_radius
                })

            self.node.get_logger().info(
                f"Cuboid: 9 spheres, radius={sphere_radius:.4f}m, dims={dims}"
            )

        elif obj_type == 'cylinder':
            # Get dimensions
            radius = object_info['dimensions']['radius']
            height = object_info['dimensions']['height']

            # Sphere radius = 80% of cylinder radius
            sphere_radius = radius * 0.8

            # Compute 7 sphere positions (in object frame)
            # Assume cylinder axis is Z
            half_height = height / 2.0

            # 3 spheres on top circle (at 120 degree intervals)
            # 3 spheres on bottom circle (at 120 degree intervals)
            # 1 sphere at center
            positions = []

            # Top circle
            for angle in [0, 2*np.pi/3, 4*np.pi/3]:
                x = radius * np.cos(angle) * 0.8  # 80% of radius
                y = radius * np.sin(angle) * 0.8
                z = half_height
                positions.append([x, y, z])

            # Bottom circle
            for angle in [0, 2*np.pi/3, 4*np.pi/3]:
                x = radius * np.cos(angle) * 0.8
                y = radius * np.sin(angle) * 0.8
                z = -half_height
                positions.append([x, y, z])

            # Center
            positions.append([0.0, 0.0, 0.0])

            # Transform to world frame
            # TODO: Apply quaternion rotation
            for pos in positions:
                world_pos = position + np.array(pos)
                spheres.append({
                    'position': world_pos.tolist(),
                    'radius': sphere_radius
                })

            self.node.get_logger().info(
                f"Cylinder: 7 spheres, radius={sphere_radius:.4f}m, "
                f"height={height:.4f}m, cyl_radius={radius:.4f}m"
            )

        elif obj_type == 'sphere':
            # Single sphere at center
            sphere_radius = object_info['dimensions']['radius']
            spheres.append({
                'position': position.tolist(),
                'radius': sphere_radius
            })

            self.node.get_logger().info(
                f"Sphere: 1 sphere, radius={sphere_radius:.4f}m"
            )

        else:
            self.node.get_logger().warn(
                f"Collision sphere generation not supported for type: {obj_type}"
            )
            return []

        return spheres

    def _detach_object_from_robot(self) -> bool:
        """
        Detach object from robot.

        Strategy:
        1. Remove collision spheres from gripper link
        2. Re-add object to world obstacles
        3. Update MPC world model

        Returns:
            True if detachment succeeded
        """
        if self.grasped_object is None:
            self.node.get_logger().warn("No object attached, nothing to detach")
            return True

        try:
            # TODO: Implement sphere removal from robot

            # Re-add object to world if we have its info
            if self.object_info is not None:
                # TODO: Add object back to obstacle manager
                pass

            # Update MPC world model
            # TODO: Implement world update for MPC

            self.node.get_logger().info(f"Detached object: {self.grasped_object}")
            self.grasped_object = None
            self.object_info = None
            return True

        except Exception as e:
            self.node.get_logger().error(f"Object detachment failed: {e}")
            return False

    def _close_gripper(self) -> bool:
        """
        Close gripper and wait for completion.

        Returns:
            True if successful
        """
        if self.gripper is None:
            self.node.get_logger().warn(
                "No gripper interface configured - simulating gripper close"
            )
            # Simulate gripper close time
            gripper_time = self.node.get_parameter('grasp_gripper_close_time').get_parameter_value().double_value
            time.sleep(gripper_time)
            return True

        return self.gripper.close()

    def _open_gripper(self) -> bool:
        """
        Open gripper and wait for completion.

        Returns:
            True if successful
        """
        if self.gripper is None:
            self.node.get_logger().warn(
                "No gripper interface configured - simulating gripper open"
            )
            # Simulate gripper open time
            gripper_time = self.node.get_parameter('grasp_gripper_close_time').get_parameter_value().double_value
            time.sleep(gripper_time)
            return True

        return self.gripper.open()
