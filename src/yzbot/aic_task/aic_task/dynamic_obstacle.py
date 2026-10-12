"""Compatibility import for aic_task.navigation.dynamic_obstacle."""
from aic_task.navigation.dynamic_obstacle import *  # noqa: F401,F403
from aic_task.navigation.dynamic_obstacle import __name__ as _target_name
import importlib as _importlib
_impl = _importlib.import_module(_target_name)
def __getattr__(name):
    return getattr(_impl, name)

if __name__ == "__main__":
    _impl.main()
