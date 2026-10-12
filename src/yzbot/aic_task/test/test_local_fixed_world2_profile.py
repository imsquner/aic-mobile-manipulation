"""本机固定 World2 档位：覆盖冲突配置，并保持其它场景隔离。"""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from aic_task.navigation.object_docking.profile import apply_fixed_world2_profile


@pytest.mark.parametrize('path', ['/tmp/scenes/world2/points.yaml', '/tmp/config/world2_points.yaml'])
def test_world2_without_yaml_switch_uses_fixed_docking(path):
    host = SimpleNamespace(points_file=Path(path), points={'approach': {}}, object_docking_enabled=False)
    assert apply_fixed_world2_profile(host)
    assert host.object_docking_enabled is True
    assert host.object_docking_config.gate_min_distance_m == 0.35
    assert host.object_docking_config.gate_max_distance_m == 0.48
    assert host.object_docking_config.gate_yaw_tolerance_rad == 0.20
    assert host.object_docking_config.stable_hold_sec == 0.60
    assert host.object_docking_max_retries == 1
    assert host.object_docking_minimum_budget_sec == 4.0
    assert host.object_docking_velocity_chain_check is True
    assert host.object_docking_velocity_chain_nodes == ('velocity_smoother', 'collision_monitor')


def test_conflicting_yaml_cannot_disable_or_relax_fixed_world2_profile():
    host = SimpleNamespace(points_file=Path('/tmp/scenes/world2/points.yaml'),
                           points={'approach': {'object_docking_enabled': False, 'object_docking_gate_max_distance_m': 9.0}},
                           object_docking_config=None, object_docking_max_retries=99)
    assert apply_fixed_world2_profile(host)
    assert host.object_docking_enabled is True
    assert host.object_docking_config.gate_max_distance_m == 0.48
    assert host.object_docking_max_retries == 1


@pytest.mark.parametrize('path', ['/tmp/scenes/v1/points.yaml', '/tmp/scenes/world3/points.yaml', '/tmp/notworld2/points.yaml'])
def test_other_scenes_are_untouched(path):
    host = SimpleNamespace(points_file=Path(path), object_docking_enabled=False)
    before = vars(host).copy()
    assert apply_fixed_world2_profile(host) is False
    assert vars(host) == before


def test_node_applies_fixed_profile_before_yaml_and_mirrors_ros_parameters():
    source = Path(__file__).parents[1] / 'aic_task/task_orchestrator_node.py'
    tree = ast.parse(source.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_apply_object_docking_points')
    calls = [n.func.id for n in ast.walk(method) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)]
    assert 'apply_fixed_world2_profile' in calls
    assert any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'set_parameters' for n in ast.walk(method))


def test_fixed_ros_values_reject_later_parameter_drift():
    from aic_task.navigation.object_docking.profile import validate_fixed_world2_parameters
    assert validate_fixed_world2_parameters([SimpleNamespace(name='object_docking_enabled', value=False)])[0] is False
    assert validate_fixed_world2_parameters([SimpleNamespace(name='object_docking_gate_max_distance_m', value=0.60)])[0] is False
    assert validate_fixed_world2_parameters([SimpleNamespace(name='object_docking_enabled', value=True)])[0] is True
    assert validate_fixed_world2_parameters([SimpleNamespace(name='unrelated_parameter', value=99)])[0] is True


def test_node_runtime_and_parameter_readback_use_same_profile():
    from aic_task.navigation.object_docking.profile import fixed_world2_parameters, validate_fixed_world2_parameters
    source = Path(__file__).parents[1] / 'aic_task/task_orchestrator_node.py'
    method = next(n for n in ast.walk(ast.parse(source.read_text())) if isinstance(n, ast.FunctionDef) and n.name == '_apply_object_docking_points')
    namespace = {'apply_fixed_world2_profile': apply_fixed_world2_profile,
                 'fixed_world2_parameters': fixed_world2_parameters,
                 'validate_fixed_world2_parameters': validate_fixed_world2_parameters,
                 'Parameter': lambda name, value: SimpleNamespace(name=name, value=value),
                 'SetParametersResult': lambda **kw: SimpleNamespace(**kw)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    captured = {}
    host = SimpleNamespace(points_file=Path('/tmp/scenes/world2/points.yaml'), points={'approach': {}},
                           set_parameters=lambda params: captured.update({p.name:p.value for p in params}) or [SimpleNamespace(successful=True)],
                           get_logger=lambda: SimpleNamespace(info=lambda message: None),
                           add_on_set_parameters_callback=lambda callback: captured.update(callback=callback))
    namespace['_apply_object_docking_points'](host)
    assert captured['object_docking_enabled'] is host.object_docking_enabled is True
    assert captured['object_docking_gate_max_distance_m'] == host.object_docking_config.gate_max_distance_m == 0.48
    assert captured['callback']([SimpleNamespace(name='object_docking_enabled', value=False)]).successful is False
    assert host.points == {'approach': {}}
