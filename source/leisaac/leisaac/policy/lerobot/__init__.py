import sys
import types

from . import helpers
from .helpers import *
from .helpers import (
    FeatureType,
    PolicyFeature,
    RemotePolicyConfig,
    TimedAction,
    TimedData,
    TimedObservation,
)


def _register_fake_module(path: str, attrs: dict):
    """Create a fake module hierarchy at `path` in sys.modules and populate it with attrs."""
    parts = path.split(".")
    for i in range(1, len(parts) + 1):
        sub_path = ".".join(parts[:i])
        if sub_path not in sys.modules:
            mod = types.ModuleType(sub_path)
            sys.modules[sub_path] = mod
        if i > 1:
            parent_path = ".".join(parts[: i - 1])
            setattr(sys.modules[parent_path], parts[i - 1], sys.modules[sub_path])
    sys.modules[path].__dict__.update(attrs)


# 服务器端实际的模块路径 — pickle 序列化时用这些路径
_register_fake_module(
    "lerobot.async_inference.helpers",
    {
        "RemotePolicyConfig": RemotePolicyConfig,
        "TimedObservation": TimedObservation,
        "TimedAction": TimedAction,
        "TimedData": TimedData,
    },
)

_register_fake_module(
    "lerobot.configs.types",
    {
        "FeatureType": FeatureType,
        "PolicyFeature": PolicyFeature,
    },
)

# 覆盖 __module__ 与服务器路径一致
RemotePolicyConfig.__module__ = "lerobot.async_inference.helpers"
TimedObservation.__module__ = "lerobot.async_inference.helpers"
TimedAction.__module__ = "lerobot.async_inference.helpers"
TimedData.__module__ = "lerobot.async_inference.helpers"
FeatureType.__module__ = "lerobot.configs.types"
PolicyFeature.__module__ = "lerobot.configs.types"
