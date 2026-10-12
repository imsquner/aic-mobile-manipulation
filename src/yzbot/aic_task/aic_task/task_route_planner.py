"""Compatibility import for aic_task.planning.route_planner."""
from aic_task.planning.route_planner import *  # noqa: F401,F403
from aic_task.planning.route_planner import __name__ as _target_name
import importlib as _importlib
_impl = _importlib.import_module(_target_name)
def __getattr__(name):
    return getattr(_impl, name)
