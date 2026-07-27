#!/usr/bin/env python3
"""
Attach/detach services for the unified planner node.

Attaches a scene obstacle to the arm's ``attached_object`` link as collision
spheres (fitted by ``sphere_fit`` via cuRobo's attachment_manager) so it moves
with the arm for subsequent open-loop transport planning, until ``detach``
releases it. The robot YAML must declare the ``attached_object`` link (see
docs/tutorials/03-collision-objects.md).

Standalone feature: independent of any particular planning pipeline. Useful
for pre-positioned objects, simulation, and tests — the caller supplies the
scene obstacle name and the joint state to fit spheres at (usually the
robot's current pose).

Registers its own services (`<node>/attach_object`, `<node>/detach_object`),
following the same self-registering pattern as ``IKServices`` / ``FKServices``.
"""

from std_srvs.srv import Trigger
from curobo.types import JointState

from curobo_msgs.srv import AttachObject

ATTACH_LINK = "attached_object"


class AttachmentServices:
    """Attach/detach a scene obstacle to the robot's ``attached_object`` link."""

    def __init__(self, node, config_wrapper):
        self.node = node
        self.config_wrapper = config_wrapper
        self._attached_name = None  # name of the currently attached obstacle

        name = node.get_name()
        self.attach_object_srv = node.create_service(
            AttachObject, f'{name}/attach_object', self._attach_object_callback)
        self.detach_object_srv = node.create_service(
            Trigger, f'{name}/detach_object', self._detach_object_callback)

    @property
    def attached_name(self):
        """Name of the currently attached obstacle, or None."""
        return self._attached_name

    @property
    def motion_planner(self):
        """The node's shared MotionPlanner (may be None before warmup)."""
        return self.node.motion_planner

    def kinematics(self):
        """The MotionPlanner's kinematics (carries attached-object spheres), or
        None if the planner isn't ready. Used by ros_service_manager to render
        the fitted attached-object spheres in the collision-sphere viz.
        """
        try:
            return self._attachment_manager()._kinematics
        except Exception:
            return None

    def _attachment_manager(self):
        """cuRobo AttachmentManager for the shared MotionPlanner.

        In this build ``MotionPlanner.attachment_manager`` forwards to
        ``trajopt_solver.attachment_manager``, but TrajOptSolver composes
        SolverCore and doesn't re-expose it — reach into ``.core`` instead.
        """
        mp = self.motion_planner
        am = getattr(mp, 'attachment_manager', None)  # property may raise -> None
        if am is None:
            am = mp.trajopt_solver.core.attachment_manager
        return am

    def _has_attach_link(self) -> bool:
        """Whether the loaded robot YAML declares the attach link."""
        mp = self.motion_planner
        if mp is None:
            return False
        kc = mp.kinematics.kinematics_config
        return ATTACH_LINK in kc.link_name_to_idx_map

    # ------------------------------------------------------------------
    # Attach / detach
    # ------------------------------------------------------------------

    def attach(self, object_name: str, grasp_end_state: JointState) -> None:
        """Fit + attach the named scene obstacle to the arm at the given config.

        The obstacle is fetched from the ObstacleManager's authoritative Scene
        (so it works even if the solver's loaded scene_model is out of sync) and
        passed directly to ``attach`` (not ``attach_from_scene``, which looks up
        by name in the solver's scene_model). ``disable_obstacle_names`` also
        disables the world obstacle so it isn't double-counted with the attached
        spheres. Affects the shared MotionPlanner only (open-loop transport).

        Sphere-fitting is a GPU operation — held under node.gpu_lock so it can't
        overlap a concurrent CUDA-graph capture or depth-camera integrate.
        """
        if not self._has_attach_link():
            raise ValueError(
                f"Robot YAML does not declare the '{ATTACH_LINK}' link — "
                "attach is unavailable for this robot (see "
                "docs/tutorials/03-collision-objects.md)."
            )
        scene = self.config_wrapper.obstacle_manager.get_scene()
        obstacle = self._find_obstacle(scene, object_name)
        if obstacle is None:
            raise ValueError(f"Obstacle '{object_name}' not found in the scene")
        with self.node.gpu_lock:
            self._attachment_manager().attach(
                joint_states=grasp_end_state,
                obstacles=[obstacle],
                link_name=ATTACH_LINK,
                disable_obstacle_names=[object_name],
            )
        self._attached_name = object_name
        self.node.get_logger().info(
            f"Attached '{object_name}' to link '{ATTACH_LINK}'")

    @staticmethod
    def _find_obstacle(scene, name: str):
        """Find an obstacle by name across the Scene's typed buckets.

        The ObstacleManager stores obstacles in typed lists (cuboid/sphere/…),
        not in Scene.objects, so Scene.get_obstacle (which reads .objects) misses
        runtime-added ones — search the buckets directly.
        """
        for bucket in ('cuboid', 'sphere', 'capsule', 'cylinder', 'mesh'):
            for obj in (getattr(scene, bucket, None) or []):
                if obj.name == name:
                    return obj
        if getattr(scene, 'objects', None):
            return scene.get_obstacle(name)
        return None

    def detach(self) -> str:
        """Release the attached object (reset link spheres, re-enable the obstacle)."""
        with self.node.gpu_lock:
            self._attachment_manager().detach(ATTACH_LINK)
        released = self._attached_name
        self._attached_name = None
        self.node.get_logger().info(f"Detached '{released}' from '{ATTACH_LINK}'")
        return released or ""

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _attach_object_callback(self, request, response):
        """Attach a scene obstacle to the arm at its current joint configuration."""
        try:
            if self.motion_planner is None:
                self.node._warmup_classic()
            _, current_state = self.node._resolve_start_state(None)
            self.attach(request.object_name, current_state)
            response.success = True
            response.message = f"Attached '{request.object_name}'"
        except Exception as e:
            response.success = False
            response.message = f"Attach error: {e}"
            self.node.get_logger().error(response.message)
        return response

    def _detach_object_callback(self, request, response):
        """Release a previously attached object from the arm."""
        try:
            released = self.detach()
            response.success = True
            response.message = (
                f"Detached '{released}'" if released else "Nothing attached")
        except Exception as e:
            response.success = False
            response.message = f"Detach error: {e}"
            self.node.get_logger().error(response.message)
        return response
