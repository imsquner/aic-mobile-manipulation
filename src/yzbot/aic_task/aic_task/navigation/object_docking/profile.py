"""本机World2固定docking档位；不修改points或旧矩阵的来源身份。"""
from dataclasses import asdict
from pathlib import Path
from types import MappingProxyType

from .controller import DockingConfig

FIXED_WORLD2_CONFIG = DockingConfig()
FIXED_WORLD2_RUNTIME = MappingProxyType({
    'object_docking_enabled': True,
    'object_docking_timeout_sec': 12.0,
    'object_docking_frequency_hz': 50.0,
    'object_docking_max_retries': 1,
    'object_docking_handoff_min_m': 0.50,
    'object_docking_handoff_max_m': 0.85,
    'object_docking_handoff_zero_hold_sec': 0.10,
    'object_docking_target_max_age_sec': 0.20,
    'object_docking_minimum_budget_sec': 4.0,
    'object_docking_velocity_chain_check': True,
    'object_docking_velocity_chain_nodes': ('velocity_smoother', 'collision_monitor'),
})


def fixed_world2_parameters():
    """将唯一代码档位映射到现有ROS参数名称，供节点读回核对。"""
    values = {'object_docking_' + key: value for key, value in asdict(FIXED_WORLD2_CONFIG).items()}
    values.update(FIXED_WORLD2_RUNTIME)
    values['object_docking_velocity_chain_nodes'] = list(values['object_docking_velocity_chain_nodes'])
    return values


def apply_fixed_world2_profile(host):
    """固定World2运行属性；v1/world3继续使用原场景配置流程。"""
    path = Path(host.points_file)
    if not (path.parent.name == 'world2' and path.name == 'points.yaml') and path.name != 'world2_points.yaml':
        return False
    host.object_docking_config = FIXED_WORLD2_CONFIG
    for name, value in FIXED_WORLD2_RUNTIME.items():
        setattr(host, name, value)
    return True


def validate_fixed_world2_parameters(parameters):
    """拒绝运行时改变固定档位，避免ROS读回与真实控制口径分裂。"""
    fixed = fixed_world2_parameters()
    for parameter in parameters:
        if parameter.name in fixed and parameter.value != fixed[parameter.name]:
            return False, f"{parameter.name} is fixed by the World2 competition profile"
    return True, ""
