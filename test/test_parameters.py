"""Tests for the parameter schema (curobo_ros/core/parameters.py).

Static checks keep the schema the single place parameters are declared; the
rclpy checks declare everything on a real node. No GPU needed (the MPPI and
L-BFGS default trees import cuRobo, not CUDA).
"""

import pathlib
import re

import pytest
import yaml

from curobo_ros.core import parameters

ROOT = pathlib.Path(__file__).resolve().parents[1]
PKG = ROOT / 'curobo_ros'

# Separate nodes with their own parameters (not the unified planner's).
SEPARATE_NODES = {
    'core/robot_segmentation.py', 'core/marker_publisher.py',
    'interfaces/show_voxel_grid.py', 'tools/run_pose_matrix.py',
    'tools/benchmark_voxel_grid.py',
}


def _planner_sources():
    for path in sorted(PKG.rglob('*.py')):
        rel = path.relative_to(PKG).as_posix()
        if rel not in SEPARATE_NODES:
            yield rel, path.read_text()


def test_no_declaration_outside_the_schema():
    offenders = [rel for rel, text in _planner_sources()
                 if 'declare_parameter(' in text and rel != 'core/parameters.py']
    assert offenders == []


def test_every_read_parameter_is_in_the_schema():
    read = set()
    for _, text in _planner_sources():
        read |= set(re.findall(r"get_parameter\(\s*'([A-Za-z0-9_.]+)'", text))
        read |= set(re.findall(r"parameters\.declare\(\s*[\w.]+,\s*'([A-Za-z0-9_.]+)'", text))
    unknown = sorted(name for name in read if parameters.spec_of(name) is None)
    assert unknown == []


def test_every_parameter_is_documented():
    doc = (ROOT / 'docs' / 'concepts' / 'parameters.md').read_text()
    missing = [name for name in parameters.SPECS if f'`{name}`' not in doc]
    assert missing == []
    for name in parameters.REMOVED:
        assert f'| `{name}` |' not in doc, f'{name} is removed but documented'


def test_specs_are_consistent():
    for spec in parameters.SPECS.values():
        assert spec.kind in (parameters.STARTUP, parameters.BUILD,
                             parameters.LIVE, parameters.STATUS), spec.name
        assert spec.description, spec.name
        assert (spec.kind == parameters.BUILD) == bool(spec.solvers), spec.name
        if spec.bounds is not None and spec.default is not parameters.DESCRIPTOR:
            low, high = spec.bounds
            assert low <= spec.default <= high, spec.name
        assert spec.name not in parameters.REMOVED


# ---- rclpy -----------------------------------------------------------------

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter  # noqa: E402


def _yaml_overrides():
    out = []
    for name in ('mppi_params.yaml', 'lbfgs_params.yaml'):
        tree = yaml.safe_load((ROOT / 'config' / 'mpc' / name).read_text())
        flat = {}

        def walk(prefix, value):
            if isinstance(value, dict):
                for k, v in value.items():
                    walk(f'{prefix}.{k}' if prefix else k, v)
            else:
                flat[prefix] = value
        walk('', tree['/**']['ros__parameters'])
        out += [Parameter(k, value=v) for k, v in flat.items()]
    return out


@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node('test_parameters', parameter_overrides=_yaml_overrides()
                          + [Parameter('voxel_size', value=0.03),
                             Parameter('mpc_solver_type', value='lbfgs_bspline'),
                             Parameter('max_joint_accel_dps', value=1000.0)])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_declare_all_with_the_shipped_yaml(node):
    parameters.declare_all(node)
    assert node.get_parameter('voxel_size').value == 0.03
    leaf = node.describe_parameter('mpc_mppi.rollout.cost_cfg.cspace_cfg.weight')
    assert leaf.additional_constraints.startswith('build')
    assert node.get_parameters_by_prefix('lbfgs')
    d = node.describe_parameter('interpolation_dt')
    assert d.description and d.additional_constraints.startswith('build')


def test_startup_parameters_are_read_only(node):
    parameters.declare_all(node)
    parameters.install_change_hook(node)
    result = node.set_parameters([Parameter('voxel_size', value=0.1)])[0]
    assert not result.successful and 'read-only' in result.reason
    result = node.set_parameters([Parameter('max_attempts', value=3)])[0]
    assert result.successful
    # The node writes its own status parameters.
    assert node.set_parameters([Parameter('node_is_available', value=True)])[0].successful


def test_out_of_range_is_rejected(node):
    parameters.declare_all(node)
    assert not node.set_parameters([Parameter('max_attempts', value=0)])[0].successful


def test_descriptor_defaults_need_the_call_site(node):
    with pytest.raises(ValueError):
        parameters.declare(node, 'base_link')
    assert parameters.declare(node, 'base_link', 'base_0') == 'base_0'
    assert parameters.declare(node, 'base_link', 'ignored') == 'base_0'


def test_unused_overrides_are_reported(node, monkeypatch):
    parameters.declare_all(node)
    warns = []
    monkeypatch.setattr(node.get_logger(), 'warn', warns.append, raising=False)
    logger = node.get_logger()
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    parameters.warn_unused_overrides(node)
    assert any('mpc_solver_type' in w and 'no longer exists' in w for w in warns)
    assert not any('max_joint_accel_dps' in w for w in warns)
