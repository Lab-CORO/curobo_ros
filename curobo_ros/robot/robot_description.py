"""Robot descriptor: single source of truth for a robot's ROS-side wiring.

A descriptor is a small YAML file in ``robots/`` (installed to the package share)
that references a cuRobo robot config and declares only what cuRobo does NOT know:
the robot DRIVER interface (command/state/joint-feedback topics), the joint-command
strategy key, and a display name.

Deliberately NOT here: camera/depth topics. Those belong to the perception setup
(``config/cameras.yaml`` / ``robot_segmentation`` params), not to the robot — the
same cell's camera is shared across robots, and the same robot moves between cells.

Canonical vs descriptor data:
- ``base_link`` / ``joint_names`` / ``dof`` are canonical in the cuRobo config
  (and ``kin_model.joint_names`` once kinematics is built). The descriptor only
  carries an OPTIONAL ``base_link`` override; it never duplicates joint names.
- Paths inside the cuRobo config (``urdf_path`` / ``asset_root_path``) are made
  portable here: ``package://pkg/rest``, absolute, or relative-to-the-yml are all
  resolved to absolute. If any needed resolving, a resolved copy of the cuRobo yml
  is written to a runtime location so every downstream consumer keeps receiving a
  plain file path (the whole codebase passes a path to the v2 factories).

Adding a robot = drop a ``robots/<name>.yaml`` + its cuRobo config; no core edit.
"""

import hashlib
import os
from dataclasses import dataclass, field
from typing import List, Optional

import yaml as _yaml
from ament_index_python.packages import get_package_share_directory
from curobo.config_io import load_yaml

_RESOLVED_DIR = '/tmp/curobo_ros_resolved'


def _resolve_uri(path: Optional[str], base_dir: str) -> Optional[str]:
    """Resolve a ``package://``, absolute, or relative-to-``base_dir`` path."""
    if not path:
        return path
    if path.startswith('package://'):
        rest = path[len('package://'):]
        pkg, _, tail = rest.partition('/')
        return os.path.join(get_package_share_directory(pkg), tail)
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(base_dir, path))


def resolve_curobo_config(curobo_path: str) -> str:
    """Resolve a cuRobo robot-config YAML's ``urdf_path``/``asset_root_path``
    to absolute paths, writing a resolved copy if anything changed.

    Any consumer that loads a cuRobo robot-config YAML DIRECTLY — bypassing
    ``load_robot_description``'s descriptor wrapper, e.g. an explicit
    ``robot_config_file`` override — must call this first. Without it, a
    ``package://``-style or custom-relative ``urdf_path``/``asset_root_path``
    silently fails to resolve, and cuRobo's OWN relative-path resolution
    (relative to its bundled assets dir) kicks in instead — wrong for a path
    that was meant to be relative to the yml file itself.

    Returns the resolved path, or ``curobo_path`` unchanged if nothing needed
    resolving.
    """
    cfg = load_yaml(curobo_path)
    robot_cfg = cfg.get('robot_cfg', cfg)
    kin = robot_cfg.get('kinematics', {})
    yml_dir = os.path.dirname(curobo_path)
    # Keyed on the full source path (not just the yml basename) so that two
    # descriptors referencing identically-named yml files in different
    # directories never resolve to the same /tmp path and overwrite each other.
    digest = hashlib.sha1(os.path.abspath(curobo_path).encode()).hexdigest()[:10]

    # - package://    -> always resolve (explicit, portable).
    # - absolute      -> leave as-is.
    # - bare relative -> only rewrite if it exists relative to the yml dir (a
    #   custom robot shipping its own assets). Otherwise leave it for cuRobo's
    #   native resolution (its bundled configs are relative to get_assets_path).
    changed = False
    for key in ('urdf_path', 'asset_root_path'):
        original = kin.get(key)
        if not original:
            continue
        if original.startswith('package://'):
            kin[key] = _resolve_uri(original, yml_dir)
            changed = True
        elif not os.path.isabs(original):
            candidate = os.path.normpath(os.path.join(yml_dir, original))
            if os.path.exists(candidate):
                kin[key] = candidate
                changed = True

    if not changed:
        return curobo_path

    name = os.path.splitext(os.path.basename(curobo_path))[0]
    os.makedirs(_RESOLVED_DIR, exist_ok=True)
    resolved_path = os.path.join(_RESOLVED_DIR, f'{name}-{digest}.yml')
    with open(resolved_path, 'w') as f:
        _yaml.safe_dump(cfg, f, default_flow_style=None, sort_keys=False)
    return resolved_path


@dataclass
class RobotDescription:
    """Resolved, ready-to-use description of one robot."""

    name: str
    display_name: str
    curobo_config_path: str          # resolved yml path (inner paths absolute)
    robot_cfg_dict: dict             # full parsed robot_cfg (cspace kept here)
    base_link: str
    strategy_key: str
    strategy_params: dict = field(default_factory=dict)
    _joint_names: Optional[List[str]] = None
    _dof: Optional[int] = None

    def bind_kinematics(self, kin) -> None:
        """Adopt the canonical joint order/DOF from a built cuRobo ``Kinematics``."""
        self._joint_names = list(kin.joint_names)
        self._dof = len(self._joint_names)

    @property
    def joint_names(self) -> List[str]:
        """Canonical joint names once bound; cspace fallback before binding ([] if unknown)."""
        return list(self._joint_names) if self._joint_names else []

    @property
    def dof(self) -> int:
        return self._dof or 0


def _default_descriptor_path(name: str) -> str:
    return os.path.join(
        get_package_share_directory('curobo_ros'), 'robots', f'{name}.yaml')


def load_robot_description(name_or_path: str) -> RobotDescription:
    """Load a robot descriptor by name (``robots/<name>.yaml`` in the share) or path.

    Resolves the referenced cuRobo config's ``urdf_path``/``asset_root_path`` and,
    if any were non-absolute, writes a resolved copy so consumers get a plain path.
    """
    desc_path = name_or_path if os.path.isfile(name_or_path) else _default_descriptor_path(name_or_path)
    desc = load_yaml(desc_path)

    name = desc.get('name', os.path.splitext(os.path.basename(desc_path))[0])
    display_name = desc.get('display_name', name)
    strategy_key = desc.get('strategy', 'emulator')
    strategy_params = desc.get('strategy_params', {}) or {}

    curobo_uri = desc.get('curobo_config')
    if not curobo_uri:
        raise ValueError(f"Descriptor {desc_path} is missing required 'curobo_config'")
    curobo_path = _resolve_uri(curobo_uri, os.path.dirname(desc_path))

    # Resolve the two portability-sensitive paths (urdf_path/asset_root_path)
    # FIRST, emitting a resolved copy if either needed rewriting — see
    # resolve_curobo_config for why this must happen before anything builds
    # kinematics from this path. robot_cfg_dict is then loaded from the
    # resolved copy so it matches curobo_config_path (both absolute paths),
    # instead of from the original possibly-package://-or-relative source.
    resolved_path = resolve_curobo_config(curobo_path)

    cfg = load_yaml(resolved_path)
    robot_cfg = cfg.get('robot_cfg', cfg)
    kin = robot_cfg.get('kinematics', {})

    base_link = desc.get('base_link') or kin.get('base_link')

    # cspace joint_names is a pre-kinematics fallback (kin_model is canonical later).
    # In cuRobo's format cspace lives under kinematics (robot_cfg.kinematics.cspace).
    cspace = kin.get('cspace') or robot_cfg.get('cspace') or {}
    fallback_joints = list(cspace.get('joint_names', [])) or None

    return RobotDescription(
        name=name,
        display_name=display_name,
        curobo_config_path=resolved_path,
        robot_cfg_dict=robot_cfg,
        base_link=base_link,
        strategy_key=strategy_key,
        strategy_params=strategy_params,
        _joint_names=fallback_joints,
        _dof=len(fallback_joints) if fallback_joints else None,
    )
