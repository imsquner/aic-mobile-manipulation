"""Compatibility import for aic_task.navigation.client."""
from aic_task.navigation.client import *  # noqa: F401,F403
from aic_task.navigation.client import __name__ as _target_name
import importlib as _importlib
_impl = _importlib.import_module(_target_name)
def __getattr__(name):
    return getattr(_impl, name)
