#!/usr/bin/env python3
"""
Multi-point trajectory planner using cuRobo MotionGen.

This planner generates a trajectory that passes through multiple waypoints,
useful for tasks requiring intermediate poses (e.g., pick-and-place with obstacles).
"""

from typing import List, Optional

import torch
from curobo.types.robot import JointState
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig, MotionGenResult
from curobo.util.trajectory import InterpolateType
from curobo.rollout.cost.pose_cost import PoseCostMetric

from .single_planner import SinglePlanner


class MultiPointPlanner(SinglePlanner):
    """
    Multi-point motion generation planner.

    Extends ClassicPlanner to support trajectories through multiple waypoints.
    Uses MotionGen's batch planning to generate a trajectory that visits
    each waypoint in sequence.

    This planner is useful for:
    - Pick-and-place operations with specific approach/retreat poses
    - Navigation through tight spaces requiring specific intermediate poses
    - Tasks requiring precise path constraints

    Execution flow:
    1. plan() - Generate trajectory through all waypoints using MotionGen
    2. execute() - Send combined trajectory to robot

    Example usage:
        >>> # After warmup
        >>> SinglePlanner.set_motion_gen(node.motion_gen)
        >>>
        >>> # Create planner
        >>> planner = MultiPointPlanner(node, config_wrapper)
        >>>
        >>> # Plan through multiple waypoints
        >>> waypoints = [approach_pose, grasp_pose, retreat_pose]
        >>> result = planner.plan(
        >>>     start_state=current_joint_state,
        >>>     goal_pose=waypoints,  # List of Pose objects
        >>>     config={
        >>>         'max_attempts': 2,
        >>>         'timeout': 10.0,
        >>>         'time_dilation_factor': 0.5,
        >>>     }
        >>> )
        >>>
        >>> # Execute
        >>> if result.success:
        >>>     success = planner.execute(robot_context)
    """

    def get_planner_name(self) -> str:
        """Return planner name."""
        return "Multi-Point Motion Generation"

    def _plan_trajectory(
        self,
        start_state: JointState,
        goal_request,
        config: dict
    ) -> MotionGenResult:
        """
        Generate trajectory through multiple waypoints.

        Extracts multiple poses from goal_request.target_poses and plans
        a trajectory that visits each waypoint in sequence.

        Args:
            start_state: Initial joint configuration
            goal_request: TrajectoryGeneration request (uses target_poses field)
            config: Dictionary with keys:
                - max_attempts: Number of planning attempts (default: 1)
                - timeout: Planning timeout in seconds (default: 10.0)
                - time_dilation_factor: Trajectory time scaling (default: 0.5)
                - connect_waypoints: If True, connect waypoints with smooth motion
                                    If False, plan to each waypoint independently
                                    (default: True)

        Returns:
            MotionGenResult with trajectory through all waypoints
        """
        # Extract waypoints from request (MultiPointPlanner uses target_poses)
        waypoints = []
        for pose_msg in goal_request.target_poses:
            waypoint = Pose.from_list([
                pose_msg.position.x,
                pose_msg.position.y,
                pose_msg.position.z,
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w
            ])
            waypoints.append(waypoint)

        # Extract config parameters
        max_attempts = config.get('max_attempts', 1)
        timeout = config.get('timeout', 10.0)
        time_dilation_factor = config.get('time_dilation_factor', 0.5)
        connect_waypoints = config.get('connect_waypoints', False)

        # Prepare per-waypoint constraints
        waypoint_constraints = []
        if (hasattr(goal_request, 'trajectories_contraints') and
            goal_request.trajectories_contraints):

            # Validation
            if len(goal_request.trajectories_contraints) % 6 != 0:
                self.node.get_logger().error(
                    f"trajectories_contraints length ({len(goal_request.trajectories_contraints)}) "
                    f"must be multiple of 6"
                )
                # Continue without constraints
            elif len(goal_request.trajectories_contraints) // 6 != len(waypoints):
                self.node.get_logger().error(
                    f"trajectories_contraints has {len(goal_request.trajectories_contraints)//6} "
                    f"constraint vectors but {len(waypoints)} waypoints"
                )
                # Continue without constraints
            else:
                # Extract constraint vectors (one per waypoint)
                for i in range(len(waypoints)):
                    start_idx = i * 6
                    end_idx = start_idx + 6
                    constraint_vec = list(goal_request.trajectories_contraints[start_idx:end_idx])
                    waypoint_constraints.append(constraint_vec)

                self.node.get_logger().info(
                    f"MultiPointPlanner: {len(waypoint_constraints)} constraint vectors loaded"
                )

        # Plan through waypoints sequentially, stacking trajectories
        # Based on curobo pose_sequence_example.py

        # Start with initial state
        current_state = start_state.clone()
        combined_trajectory = None
        total_motion_time = 0.0

        # Store combined trajectory in instance variable for _process_trajectory
        self._combined_trajectory = None

        for i, waypoint in enumerate(waypoints):
            # Reset velocity and acceleration to zero between waypoints (only if not connecting)
            if not connect_waypoints:
                current_state.velocity[:] = 0.0
                current_state.acceleration[:] = 0.0
            # Otherwise keep velocity/acceleration from previous segment for smooth motion

            waypoint_pos = waypoint.position.cpu().tolist() if hasattr(waypoint.position, 'cpu') else list(waypoint.position)
            current_pos = current_state.position[0].cpu().tolist()

            # Check for constraint for this waypoint
            pose_cost_metric = None
            if waypoint_constraints and i < len(waypoint_constraints):
                constraint_vec = waypoint_constraints[i]

                # Check if at least one constraint is active
                if any(c == 1 for c in constraint_vec):

                    # Convert to PyTorch tensor (CuRobo requires torch.Tensor for .clone())
                    hold_vec_tensor = torch.tensor(
                        constraint_vec,
                        dtype=torch.float32
                    )

                    pose_cost_metric = PoseCostMetric(
                        hold_vec_weight=hold_vec_tensor,
                        hold_partial_pose=True  # Just constrain specified axes
                    )

                    self.node.get_logger().info(
                        f"MultiPointPlanner: Constraint {constraint_vec} applied to waypoint {i}"
                    )

            # Plan to this waypoint
            result = self.motion_gen.plan_single(
                current_state.clone(),
                waypoint,
                MotionGenPlanConfig(
                    max_attempts=max_attempts,
                    timeout=timeout / len(waypoints),  # Divide timeout across segments
                    time_dilation_factor=time_dilation_factor,
                    pose_cost_metric=pose_cost_metric,  # Pass constraint if defined
                ),
            )

            if not result.success.item():
                self.node.get_logger().error(
                    f"Failed to plan to waypoint {i}: {result.status}"
                )
                # Return the failed result
                return result

            # Get trajectory segment (optimized or interpolated based on mode)
            if connect_waypoints:
                # Use optimized (non-interpolated) waypoints for smooth continuous motion
                # We'll interpolate once at the end over all waypoints
                segment = result.optimized_plan
            else:
                # Use interpolated segments with stops at each waypoint
                segment = result.get_interpolated_plan()

            # Stack trajectories
            if combined_trajectory is None:
                combined_trajectory = segment
            else:
                combined_trajectory = combined_trajectory.stack(segment.clone())

            # Track total motion time
            total_motion_time += result.motion_time

            # Update current state to end of this segment for next waypoint
            current_state = segment[-1].unsqueeze(0).clone()


        # If using continuous motion mode: Apply Kunz-Stilman interpolation for truly continuous trajectory
        if connect_waypoints:

            # 1. Rendre la trajectoire contiguë (requis après .stack())
            combined_trajectory = combined_trajectory.clone()

            # 2. Calculer le dt de la trajectoire combinée
            # Utiliser le nombre de waypoints pour estimer le temps total
            num_waypoints = len(combined_trajectory.position)
            # Estimation: ~0.5s entre chaque waypoint optimisé (ajuster selon vos données)
            estimated_total_time = (num_waypoints - 1) * 0.5
            combined_dt = torch.tensor(
                [estimated_total_time / (num_waypoints - 1)],
                device=combined_trajectory.position.device,
                dtype=torch.float32
            )

            # 3. Créer un MotionGenResult avec la trajectoire combinée
            combined_result = MotionGenResult(
                success=torch.tensor([True]),
                valid_query=True,
                optimized_plan=combined_trajectory,
                optimized_dt=combined_dt,
                interpolation_dt=self.motion_gen.interpolation_dt,
                path_buffer_last_tstep=None,  # Will be set by retime_trajectory
            )

            # Sauvegarder la trajectoire originale au cas où Kunz-Stilman échoue
            original_combined_trajectory = combined_trajectory

            try:
                combined_result.retime_trajectory(
                    time_dilation_factor=1.0,  # Pas de scaling (ajuster si besoin de ralentir/accélérer)
                    interpolate_trajectory=True,
                    interpolation_dt=self.motion_gen.interpolation_dt,
                    interpolation_kind=InterpolateType.KUNZ_STILMAN_OPTIMAL,
                    create_interpolation_buffer=True
                )

                # 5. Récupérer la trajectoire continue
                interpolated_traj = combined_result.get_interpolated_plan()

                if interpolated_traj is not None:
                    combined_trajectory = interpolated_traj

                else:
                    self.node.get_logger().warn(
                        "⚠️  Kunz-Stilman returned None - using optimized waypoints"
                    )

            except Exception as e:
                # Si Kunz-Stilman échoue (package manquant ou autre), fallback
                self.node.get_logger().warn(
                    f"⚠️  Kunz-Stilman interpolation failed: {e}. "
                    f"The 'trajectory_smoothing' package is not available in this cuRobo version. "
                    f"Using optimized waypoints without additional interpolation."
                )
                # Restaurer la trajectoire originale
                combined_trajectory = original_combined_trajectory

        # Store combined trajectory for _process_trajectory to use
        self._combined_trajectory = combined_trajectory


        # Return the last result (motion_time and other metadata)
        # The actual trajectory will be retrieved from self._combined_trajectory in _process_trajectory
        return result

    def _process_trajectory(self, trajectory: JointState, config: dict) -> JointState:
        """
        Post-process the multi-point trajectory.

        For multi-point planning, we return the combined trajectory stored in _plan_trajectory
        instead of the single-segment trajectory passed as argument.

        Args:
            trajectory: Raw trajectory from MotionGen (ignored for multi-point)
            config: Configuration dictionary

        Returns:
            Combined multi-waypoint trajectory
        """
        # Return the combined trajectory we built in _plan_trajectory
        if self._combined_trajectory is not None:
            self.node.get_logger().info(
                f"🔍 DEBUG: _process_trajectory returning combined trajectory with {len(self._combined_trajectory.position)} waypoints"
            )
            return self._combined_trajectory
        else:
            # Fallback to single-segment trajectory if not set
            self.node.get_logger().warn("_combined_trajectory not set, using single segment")
            return trajectory

    def get_config_parameters(self) -> list:
        """
        Get configuration parameters for multi-point planner.

        Returns:
            List of parameter names
        """
        # Start with base parameters from SinglePlanner
        params = super().get_config_parameters()

        # Add multi-point specific parameters
        params.extend([
            'connect_waypoints',
        ])

        return params
