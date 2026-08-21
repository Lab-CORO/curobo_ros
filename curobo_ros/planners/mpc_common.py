#!/usr/bin/env python3
"""
Shared MPC config-loading helpers used by both LBFGSController
(lbfgs_planner.py) and MPPIController (mppi_planner.py).
"""

import copy

import yaml

from curobo.content import get_task_configs_path
from curobo._src.util.config_io import resolve_config, join_path
from curobo._src.util.config_io import Loader as _CUROBO_YAML_LOADER


def _load_mpc_config(config_path: str) -> dict:
    """Load an MPC cost/optimizer YAML (see config/mpc/{mppi,lbfgs}_mpc.yaml).

    NOT routed through cuRobo's resolve_config/get_task_configs_path: these
    are our own files (paths come from the mpc_mppi_config_file /
    mpc_lbfgs_config_file ROS params), not cuRobo package-relative ones.

    Loaded with cuRobo's own patched Loader (config_io.py), not plain
    yaml.safe_load -- PyYAML's default float resolver requires a decimal
    point in the mantissa, so exponent notation like "1e-3" (no dot) silently
    parses as the STRING '1e-3' instead of a float. That reached a CUDA line
    search kernel launch and crashed with "TypeError: the argument is of
    unsupported type: <class 'str'>" (debug 2026-08-14,
    lbfgs_mpc.yaml's line_search_wolfe_c_1). cuRobo's stock YAMLs use the same
    "1e-3" spelling and only work because config_io.py patches this loader's
    float regex at import time; reusing it here (rather than yaml.safe_load)
    keeps our hand-edited files forgiving of the same spelling."""
    with open(config_path, 'r') as f:
        cfg = yaml.load(f, Loader=_CUROBO_YAML_LOADER)
    if not isinstance(cfg, dict):
        raise ValueError(f"MPC config file did not parse to a dict: {config_path}")
    return cfg


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
