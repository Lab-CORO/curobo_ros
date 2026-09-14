#!/usr/bin/env python3
"""
Shared MPC config helpers used by both LBFGSController (lbfgs_planner.py) and
MPPIController (mppi_planner.py). The cost/optimizer tuning values (see
config/mpc/{mppi,lbfgs}_params.yaml) are real ROS 2 parameters -- declared
per-leaf with declare_from_defaults() below, overridden at launch by the
node's own standard `parameters=[...]` YAML loading (no custom parsing) --
and reassembled into the nested dict cuRobo's optimizer_configs expects with
read_nested(). This replaced an earlier hand-rolled YAML loader
(_load_mpc_config) that read the same shape from an app-specific file the
node parsed itself; ROS 2's own parameter-file loading does that job now.
"""

import copy

from curobo.content import get_task_configs_path
from curobo._src.util.config_io import resolve_config, join_path


def _flatten(prefix: str, value, out: dict = None) -> dict:
    """Nested dict -> {dotted.path: leaf_value}, e.g.
    _flatten('', {'rollout': {'cost_cfg': {'weight': [1.0, 2.0]}}})
    -> {'rollout.cost_cfg.weight': [1.0, 2.0]}. Lists/scalars are leaves;
    only dicts recurse."""
    if out is None:
        out = {}
    if isinstance(value, dict):
        for k, v in value.items():
            _flatten(f"{prefix}.{k}" if prefix else str(k), v, out)
    else:
        out[prefix] = value
    return out


def _unflatten(flat: dict) -> dict:
    """Inverse of _flatten: {'rollout.cost_cfg.weight': [1.0, 2.0]}
    -> {'rollout': {'cost_cfg': {'weight': [1.0, 2.0]}}}."""
    out = {}
    for dotted_name, value in flat.items():
        node = out
        parts = dotted_name.split('.')
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = value
    return out


def declare_from_defaults(node, prefix: str, defaults: dict) -> None:
    """Declare one ROS parameter per leaf of `defaults` (a nested dict),
    named '<prefix>.<dotted path>' -- e.g. prefix='mpc_mppi' declares
    'mpc_mppi.rollout.cost_cfg.weight'. Values in a params YAML loaded at
    launch (see config/mpc/{mppi,lbfgs}_params.yaml, rooted at `/**:`)
    override these defaults; without one, the values below apply. Skips
    names already declared -- build_solver() can rerun across planner
    rebuilds (PlannerManager)."""
    for dotted, value in _flatten('', defaults).items():
        name = f"{prefix}.{dotted}"
        if not node.has_parameter(name):
            node.declare_parameter(name, value)


def read_nested(node, prefix: str) -> dict:
    """Inverse of declare_from_defaults(): reconstruct the nested dict from
    the node's current parameter values under `prefix`."""
    raw = node.get_parameters_by_prefix(prefix)
    return _unflatten({name: p.value for name, p in raw.items()})


def _build_metrics_rollout_cfg(cost_cfg_source: dict) -> dict:
    """metrics_base.yml (the default metrics_rollout) has NO cost_cfg — only
    constraint_cfg + convergence_cfg — so get_current_metrics() never exposes
    weighted COST magnitudes, only constraint violations. Mirror the ACTIVE
    branch's tool_pose_cfg/cspace_cfg into a copy of metrics_base.yml's own
    cost_cfg so the metrics rollout (fixed batch size, no cuda-graph rebatch)
    computes them too, safe to read via get_current_metrics() every solve.

    CRASH-SAFETY (cf. debug 2026-07-20): a prior version instead called
    compute_metrics_from_action() on the OPTIMIZATION rollout (use_cuda_graph=True,
    shared with the optimizer) to get these same magnitudes — its rebatch
    (num_particles -> 1) under a captured graph triggered a device-side assert
    that corrupted the whole CUDA context. This metrics-rollout approach avoids
    that entirely: validated in sandbox with use_cuda_graph=True, identical cost
    values to the removed dangerous path, zero CUDA errors."""
    metrics_cfg = copy.deepcopy(
        resolve_config(join_path(get_task_configs_path(), "metrics_base.yml"))
    )
    metrics_cfg["rollout"]["cost_cfg"] = {
        "tool_pose_cfg": copy.deepcopy(cost_cfg_source["tool_pose_cfg"]),
        "cspace_cfg": copy.deepcopy(cost_cfg_source["cspace_cfg"]),
    }
    return metrics_cfg


def _extract_cspace_reg_weights(cost_cfg_source: dict):
    """(w_vel, w_acc, w_jerk) from cost_cfg.cspace_cfg.squared_l2_regularization_weight
    (indices 0/1/2 -- see config/mpc/*.yaml's own comment on that field's
    ordering), or None if absent/malformed. Passed to MPCDiagnostics so
    cost_breakdown() can log a cost_cspace_vel/acc/jerk PROXY -- see that
    method's docstring for why it's a proxy, not cuRobo's actual fused cost."""
    try:
        w = cost_cfg_source["cspace_cfg"]["squared_l2_regularization_weight"]
        return (float(w[0]), float(w[1]), float(w[2]))
    except (KeyError, IndexError, TypeError):
        return None
