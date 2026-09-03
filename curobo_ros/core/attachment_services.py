#!/usr/bin/env python3
"""
Attach/detach services for the unified planner node.

Attaches a payload to the arm's ``attached_object`` link as collision
spheres (fitted by MORPHIT via cuRobo's AttachmentManager) so it moves with
the arm for subsequent planning/control, until ``detach`` releases it. The
robot YAML must declare the ``attached_object`` link (see
docs/tutorials/03-collision-objects.md).

The payload's geometry travels IN the request (AttachObject.srv) -- attach()
never looks anything up in the Scene, and the payload is never added to it.
cuRobo's own contract for an ``Obstacle`` (curobo._src.geom.types) has no
parent-link field: a thing is in the world OR on the robot, never both. By
never inserting the payload into the Scene in the first place, that contract
holds by construction -- there is nothing to remove, nothing that can be
reintroduced by the next world push, and no stale CPU-side pose to freeze
(the bug this design replaces).

Applies to every planner that currently has a live solver (classic/
multi_point/joint_space via the shared MotionPlanner, plus MPC and LBFGS),
and is replayed automatically onto any solver built or rebuilt afterwards
(TrajectoryPlanner.attachment_managers(), ReactiveController.ensure_solver(),
UnifiedPlannerNode.replay_attachment()) — so switching planners mid-transport
never silently drops the payload's collision model. RetargetController is out
of scope (see its attachment_managers() override).

Registers its own services (`<node>/attach_object`, `<node>/detach_object`),
following the same self-registering pattern as ``IKServices`` / ``FKServices``.
"""

import torch

from rclpy.time import Time
from std_srvs.srv import Trigger
from curobo.types import JointState, Pose, DeviceCfg
from curobo.sphere_fit import estimate_sphere_count

from curobo_msgs.srv import AttachObject
from curobo_ros.core.obstacle_manager import build_obstacle

ATTACH_LINK = "attached_object"


class AttachmentServices:
    """Attach/detach a payload to the robot's ``attached_object`` link,
    across every planner that currently has a live solver."""

    def __init__(self, node, config_wrapper):
        self.node = node
        self.config_wrapper = config_wrapper
        self._attached_state = None     # JointState at attach time (cloned)
        self._attached_spheres = None   # fitted sphere_tensor [n,4] (cloned)

        name = node.get_name()
        self.attach_object_srv = node.create_service(
            AttachObject, f'{name}/attach_object', self._attach_object_callback)
        self.detach_object_srv = node.create_service(
            Trigger, f'{name}/detach_object', self._detach_object_callback)

    def active_manager(self):
        """The AttachmentManager backing whichever planner is currently active.

        Prefers the currently active planner's AttachmentManager (so a caller
        reading it sees whichever solver is actually driving the robot);
        falls back to the first live manager found (_targets() walks the full
        catalog, so this is non-None as soon as ANY planner has a solver, not
        just the active one). None if nothing is ready yet.

        Factored out of kinematics() so a caller wanting more than
        ._kinematics (get_collision_distance also needs ._scene_collision)
        can reach the manager itself instead of duplicating this walk.
        """
        pm = getattr(self.node, 'planner_manager', None)
        current = pm.get_current_planner() if pm is not None else None
        if current is not None:
            managers = current.attachment_managers()
            if managers:
                return managers[0]
        managers = self._targets()
        return managers[0] if managers else None

    def kinematics(self):
        """Kinematics carrying the attached-object spheres, for viz.

        Used by ros_service_manager to render the fitted attached-object
        spheres in the collision-sphere viz. See active_manager() for the
        resolution order.
        """
        am = self.active_manager()
        return am._kinematics if am is not None else None

    # ------------------------------------------------------------------
    # Solver discovery
    # ------------------------------------------------------------------

    def _targets(self) -> list:
        """Every distinct cuRobo AttachmentManager currently backing a live solver.

        Mirrors update_all_solvers_world's catalog walk (unified_planner_node.py)
        so this stays in sync with whatever planners exist, without knowing
        about SinglePlanner/ReactiveController itself: get_planner() only
        constructs the lightweight Python wrapper (cheap, cached) -- has_solver()
        gates out any catalog entry that was never actually built.

        Deduped on id(am.kinematics_params) -- the tensor actually written --
        NOT on world_identity(): two managers can share one KinematicsParams
        (MPCSolver.core and MPCSolver.ik_solver both resolve to the same
        already-resolved RobotCfg in this build) while remaining formally
        distinct planner "identities"; writing the shared tensor twice would
        be wasted work, not wrong, but the dedup keeps this cheap regardless
        of what a future cuRobo version does with that sharing.
        """
        pm = getattr(self.node, 'planner_manager', None)
        if pm is None:
            return []
        # Local import: planner_factory -> planner classes (lbfgs_planner,
        # mppi_planner) -> curobo_ros.core.config_wrapper, which would cycle
        # back into this module if imported at module load time.
        from curobo_ros.planners import PlannerFactory

        seen_ids = set()
        managers = []
        for key, _eid, _name in PlannerFactory.get_catalog():
            planner_obj = pm.get_planner(key)
            if not planner_obj.has_solver():
                continue
            for am in planner_obj.attachment_managers():
                kp_id = id(am.kinematics_params)
                if kp_id in seen_ids:
                    continue
                seen_ids.add(kp_id)
                managers.append(am)
        return managers

    def _active_kinematics_param_ids(self) -> set:
        """id(kinematics_params) for the currently active planner's manager(s).

        Used only to classify a write failure during attach() as blocking
        (the solver that will actually move the arm didn't get the payload)
        vs degraded-but-recoverable (some other, currently inactive solver
        didn't).
        """
        pm = getattr(self.node, 'planner_manager', None)
        current = pm.get_current_planner() if pm is not None else None
        if current is None:
            return set()
        return {id(am.kinematics_params) for am in current.attachment_managers()}

    @staticmethod
    def _manager_has_attach_link(am) -> bool:
        return ATTACH_LINK in am.kinematics_params.link_name_to_idx_map

    @staticmethod
    def _identity_offset(state: JointState) -> Pose:
        """Identity pose for AttachmentManager.update()'s world_objects_pose_offset.

        fit_spheres() bakes each obstacle's own world pose into the centers it
        returns (Obstacle.get_trimesh_mesh(transform_with_pose=True), see
        AttachmentManager._obstacles_to_trimesh) -- so the sphere_tensor passed
        to update() is already WORLD-frame, not obstacle-local as update()'s
        own docstring describes. Passing the obstacle's world pose again as
        the offset would double-apply it (what cuRobo's own attach-with-offset
        test does, and it is wrong for this contract); passing None writes
        world coordinates as-is into what is actually a link-local slot (the
        pre-existing bug this rewrite fixes -- see the plan's Étape 0).
        Identity makes update()'s internal obj_to_link_poses collapse to
        ee_poses.inverse(), exactly the transform world-frame centers need.

        Verified numerically before this was written: built ee_poses/centers
        by hand, ran update()'s exact composition with each of None/identity/
        obstacle-world-pose, and round-tripped the result back through FK.
        Only identity reproduced the original obstacle position (see the plan
        file, Étape 0, for the worked numbers) -- not derived from reading
        Pose.multiply()'s convention alone.
        """
        device = state.position.device
        dtype = state.position.dtype
        return Pose(
            position=torch.zeros(1, 3, device=device, dtype=dtype),
            quaternion=torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=device, dtype=dtype),
        )

    def _resolve_world_pose(self, ros_pose, frame_id: str, grasp_end_state: JointState) -> list:
        """World-frame pose list [x, y, z, qw, qx, qy, qz] for a payload pose
        expressed in ``frame_id`` (AttachObject.srv's ``header.frame_id``).

        No TF lookup at all when ``frame_id`` is empty or already the
        planner's base frame -- that keeps a base-frame attach usable even
        without a robot_state_publisher in the graph. Otherwise resolves
        base <- frame_id via TF (rclpy.time.Time() = latest available,
        matching robot_segmentation._tf_matrix's convention) and composes it
        with the given pose: base <- frame_id <- payload.

        The common case is frame_id = the gripper link (tool_frames[0]):
        AttachmentManager.update() later computes
        ee_pose(grasp_end_state)^-1 . world_pose, and because world_pose here
        already equals ee_pose . T_tool_payload, that composition cancels
        back to exactly T_tool_payload -- the payload lands fixed in the
        gripper regardless of which joint configuration grasp_end_state is.
        """
        q = ros_pose.orientation
        local = [ros_pose.position.x, ros_pose.position.y, ros_pose.position.z,
                 q.w, q.x, q.y, q.z]

        base = self.config_wrapper.config_manager.base_link
        if not frame_id or frame_id == base:
            return local

        tf_buffer = getattr(self.node, 'tf_buffer', None)
        if tf_buffer is None:
            raise ValueError(
                f"Cannot resolve frame_id '{frame_id}': no TF buffer on the "
                "planner node."
            )
        try:
            tf = tf_buffer.lookup_transform(base, frame_id, Time())
        except Exception as e:
            raise ValueError(
                f"TF frame '{frame_id}' -> base frame '{base}' unavailable "
                f"({e}). Use header.frame_id: \"\" for the base frame, or a "
                f"real TF frame name -- '{ATTACH_LINK}' itself is a virtual "
                f"extra_link, not a TF frame; use its parent link instead."
            )

        device_cfg = DeviceCfg(
            device=grasp_end_state.position.device,
            dtype=grasp_end_state.position.dtype,
        )
        tr, rot = tf.transform.translation, tf.transform.rotation
        T_base_frame = Pose.from_list(
            [tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w],
            device_cfg=device_cfg, q_xyzw=True,
        )
        local_pose = Pose.from_list(local, device_cfg=device_cfg)
        return T_base_frame.multiply(local_pose).tolist()

    # ------------------------------------------------------------------
    # Attach / detach
    # ------------------------------------------------------------------

    def attach(self, geometry: tuple, grasp_end_state: JointState) -> None:
        """Fit once, write everywhere: fit spheres on ONE manager, then write
        the identical sphere_tensor into every live solver's AttachmentManager.

        ``geometry`` is ``(obj_type, dims, ros_pose, frame_id, mesh_file_path)``
        -- the ROS types stay at the module's edge (extracted by the callback)
        so this method is testable without a message.

        Only one payload at a time: refuses if something is already attached.
        This is not just policy -- ``_attached_spheres``/``_attached_state``
        are single fields, they were never able to represent two payloads,
        and a re-attach that failed on the active solver would otherwise
        reset_link_spheres the PREVIOUS payload's model instead of restoring
        it (see the rollback below).

        Sphere-fitting and every AttachmentManager.update() call are GPU
        operations — held under node.gpu_lock so they can't overlap a
        concurrent CUDA-graph capture or depth-camera integrate.
        """
        if self._attached_spheres is not None:
            raise ValueError(
                "A payload is already attached -- detach it first "
                "(detach_object) before attaching another one."
            )

        managers = self._targets()
        if not managers:
            # Last resort, not a hard requirement: nothing has a live solver
            # yet at all. Warm up Classic so there is at least one
            # AttachmentManager to write to. Only reached when _targets() is
            # truly empty — an LBFGS/MPC-only session that already has A
            # solver never pays this ~25s cost just to attach.
            self.node._warmup_classic()
            managers = self._targets()
        if not managers:
            raise ValueError("No planner has a live solver to attach to.")
        if not any(self._manager_has_attach_link(am) for am in managers):
            raise ValueError(
                f"Robot YAML does not declare the '{ATTACH_LINK}' link — "
                "attach is unavailable for this robot (see "
                "docs/tutorials/03-collision-objects.md)."
            )

        obj_type, dims, ros_pose, frame_id, mesh_file_path = geometry
        if dims[0] <= 0 or dims[1] <= 0 or dims[2] <= 0:
            raise ValueError("Payload dimensions must be positive")
        q = ros_pose.orientation
        if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) < 1e-9:
            raise ValueError(
                "Payload orientation quaternion is ~zero -- send a "
                "normalized quaternion (e.g. w=1 for identity), not "
                "(0, 0, 0, 0)."
            )

        world_pose = self._resolve_world_pose(ros_pose, frame_id, grasp_end_state)
        _, obstacle = build_obstacle(
            ATTACH_LINK, obj_type, world_pose, dims, color=None,
            mesh_file_path=mesh_file_path,
        )

        active_ids = self._active_kinematics_param_ids()
        offset = self._identity_offset(grasp_end_state)

        with self.node.gpu_lock:
            primary = managers[0]
            # cuRobo's automatic fit (num_spheres=None) sizes itself to the
            # obstacle's geometry, not to what the robot YAML allocated for
            # ATTACH_LINK, and aborts the attach when it overruns. Capping the
            # fit at the slot count makes attach always succeed -- but a cap
            # that truncates silently hands back a collision model far coarser
            # than the payload's real shape, and the caller is about to plan
            # motions against exactly that model. So cap, and say so. Slot
            # count is read off the FIRST manager only: every manager is built
            # from the same robot YAML, so they all allocate the same budget.
            n_slots = primary.kinematics_params.get_sphere_index_from_link_name(
                ATTACH_LINK).shape[0]
            n_needed = self._estimate_sphere_need(obstacle)
            if n_needed > n_slots:
                self.node.get_logger().warn(
                    f"Attached payload needs about {n_needed} collision "
                    f"spheres but link '{ATTACH_LINK}' allocates only "
                    f"{n_slots}: its collision model is a coarse approximation "
                    f"that may not cover corners or protrusions. Raise "
                    f"'extra_collision_spheres.{ATTACH_LINK}' in the robot YAML "
                    f"before planning with this payload on real hardware."
                )
            # Only cap when the estimate exceeds the budget; below it, let the
            # estimate stand so a small object isn't padded to the full slot
            # count. n_needed == 0 means the estimate failed -> fall back.
            n_fit = min(n_needed, n_slots) if n_needed > 0 else n_slots

            sphere_tensor = primary.fit_spheres([obstacle], num_spheres=n_fit)

            wrote_any = False
            active_failed = False
            failures = []
            written = []
            for am in managers:
                try:
                    am.update(sphere_tensor, grasp_end_state, ATTACH_LINK,
                              world_objects_pose_offset=offset)
                    wrote_any = True
                    written.append(am)
                except Exception as e:
                    if id(am.kinematics_params) in active_ids:
                        active_failed = True
                    failures.append(str(e))
                    self.node.get_logger().warn(
                        f"attach: write failed on one solver's AttachmentManager: {e}")

            if not wrote_any:
                raise ValueError(
                    f"Attach failed on every solver: {'; '.join(failures)}")

            if active_failed:
                # The solver that will actually move the arm didn't get the
                # payload -- undo the managers that DID succeed rather than
                # leave a half-attached state, and don't memorize anything.
                for am in written:
                    try:
                        am.detach(ATTACH_LINK)
                    except Exception as e:
                        self.node.get_logger().warn(
                            f"attach: rollback failed on one solver's "
                            f"AttachmentManager: {e}")
                raise ValueError(
                    f"Attach failed on the currently active planner's solver "
                    f"({'; '.join(failures)}); rolled back, nothing attached."
                )

            # Memorize state only now, on success (possibly partial: some
            # non-active solver may still be in `failures`, degraded until
            # the next attach/detach -- reapply()/replay can recover it on
            # its next rebuild).
            self._attached_state = grasp_end_state.clone()
            self._attached_spheres = sphere_tensor.clone()

        if failures:
            self.node.get_logger().warn(
                f"Attached payload but {len(failures)} other solver(s) "
                f"did not receive it and are degraded until the next attach/detach."
            )
        self.node.get_logger().info(
            f"Attached payload to link '{ATTACH_LINK}' "
            f"on {len(managers)} solver(s)")

    @staticmethod
    def _estimate_sphere_need(obstacle) -> int:
        """cuRobo's own estimate of how many spheres this geometry needs.

        Same heuristic the AttachmentManager applies when ``num_spheres=None``
        (bounding-box volume, 1 sphere per 15 cm3, capped at 100). Returns 0 if
        it can't be computed, so a failure here never blocks an attach.
        """
        try:
            return estimate_sphere_count(
                obstacle.get_trimesh_mesh(transform_with_pose=True))
        except Exception:
            return 0

    def detach(self) -> bool:
        """Release the attached payload (reset link spheres) on every live
        solver. Returns False if nothing was attached.

        Nothing is injected into the world: the caller is responsible for an
        add_object at the drop pose if it wants one there (deliberate --
        see the plan's Context on why an automatic reinjection is wrong).

        Calls cuRobo's own AttachmentManager.detach() per manager
        (reset_link_spheres -> back to the -100 sentinel radii). Its internal
        re-enable loop is a harmless no-op here: it only re-enables world
        obstacles it tracked disabling via its own attach(), and this design
        never calls that -- spheres are written through update() instead.
        """
        if self._attached_spheres is None:
            return False
        with self.node.gpu_lock:
            for am in self._targets():
                try:
                    am.detach(ATTACH_LINK)
                except Exception as e:
                    self.node.get_logger().warn(
                        f"detach: failed on one solver's AttachmentManager: {e}")
        self._attached_state = None
        self._attached_spheres = None
        self.node.get_logger().info(f"Detached payload from '{ATTACH_LINK}'")
        return True

    # ------------------------------------------------------------------
    # Replay (solvers built/rebuilt after attach)
    # ------------------------------------------------------------------

    def reapply(self, planner=None) -> None:
        """Re-write the already-fitted spheres into every (or one) live
        AttachmentManager. No refit: same sphere_tensor as the original
        attach, so a solver built/rebuilt after attach() carries an IDENTICAL
        collision model, not a second independent MORPHIT fit.

        Call site contract: ``planner`` is an already-built solver's wrapper
        (e.g. from ReactiveController.ensure_solver() right after
        build_solver()) -- pass it explicitly so this reaches a solver that
        may not be reachable via PlannerManager yet. Omit it to replay onto
        every currently live solver (e.g. after a Classic rebuild).

        Idempotent and safe to call whether or not anything is attached, so
        any call site can invoke it unconditionally. Writes unconditionally
        rather than checking am._attached_link_name first: that flag is only
        set by update()/cleared by detach(), so trusting it would silently
        skip a manager whose tensor was reset some other way; update() is an
        in-place tensor write, so re-writing identical values costs nothing.
        """
        if self._attached_spheres is None:
            return
        managers = planner.attachment_managers() if planner is not None else self._targets()
        if not managers:
            return
        offset = self._identity_offset(self._attached_state)
        with self.node.gpu_lock:
            for am in managers:
                try:
                    am.update(self._attached_spheres, self._attached_state,
                              ATTACH_LINK, world_objects_pose_offset=offset)
                except Exception as e:
                    self.node.get_logger().warn(
                        f"reapply: failed to re-attach payload "
                        f"on one solver's AttachmentManager: {e}")

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _attach_object_callback(self, request, response):
        """Attach a payload to the arm at its current joint configuration.

        Refused while an execution goal is active: the CUDA graph a live goal
        may be replaying reads link_spheres by address (see the plan's
        Risques résiduels), and a mid-execution attach would mutate a tensor
        that graph is actively replaying. Same guard/lock pattern as
        set_planner_callback / SetCollisionCache.
        """
        with self.node._goal_lock:
            if self.node._goal_active:
                response.success = False
                response.message = (
                    "Cannot attach an object while an execution goal is "
                    "active — cancel it first."
                )
                self.node.get_logger().error(response.message)
                return response
        try:
            _, current_state = self.node._resolve_start_state(None)
            geometry = (
                request.type,
                [request.dimensions.x, request.dimensions.y, request.dimensions.z],
                request.pose,
                request.header.frame_id,
                request.mesh_file_path,
            )
            self.attach(geometry, current_state)
            response.success = True
            response.message = f"Attached payload to link '{ATTACH_LINK}'"
        except Exception as e:
            response.success = False
            response.message = f"Attach error: {e}"
            self.node.get_logger().error(response.message)
        return response

    def _detach_object_callback(self, request, response):
        """Release a previously attached payload from the arm. Same
        goal-active guard as attach (see its docstring)."""
        with self.node._goal_lock:
            if self.node._goal_active:
                response.success = False
                response.message = (
                    "Cannot detach an object while an execution goal is "
                    "active — cancel it first."
                )
                self.node.get_logger().error(response.message)
                return response
        try:
            released = self.detach()
            response.success = True
            response.message = "Detached payload" if released else "Nothing attached"
        except Exception as e:
            response.success = False
            response.message = f"Detach error: {e}"
            self.node.get_logger().error(response.message)
        return response
