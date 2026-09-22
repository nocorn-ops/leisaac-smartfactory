"""PhysX 配置的版本兼容层（IsaacLab 2.x vs 3.0+）。

- IsaacLab 2.x：PhysX 参数位于 ``simulation_cfg.physx``（``PhysxCfg`` 实例），逐项赋值。
- IsaacLab 3.0+：``simulation_cfg.physx`` 被移除，改为 ``simulation_cfg.physics = PhysxCfg(**params)``
  （引入物理后端抽象，支持 Isaac Sim PhysX / Newton 等）。
"""

from __future__ import annotations

from typing import Any


def apply_physx_settings(simulation_cfg: Any, **params: Any) -> None:
    """把 PhysX 参数写入 ``SimulationCfg``，兼容 IsaacLab 2.x 与 3.0+。

    Args:
        simulation_cfg: ``SimulationCfg`` 实例（``env_cfg.sim``）。
        **params: 要设置的 PhysX 参数键值对，例如 ``bounce_threshold_velocity=0.01``。
    """
    try:  # IsaacLab 3.0+：PhysX 后端配置改为 simulation_cfg.physics
        from isaaclab_physx.physics import PhysxCfg

        current = getattr(simulation_cfg, "physics", None)
        if isinstance(current, PhysxCfg):
            for key, value in params.items():
                setattr(current, key, value)
        else:
            simulation_cfg.physics = PhysxCfg(**params)
    except ImportError:  # IsaacLab 2.x：simulation_cfg.physx
        for key, value in params.items():
            setattr(simulation_cfg.physx, key, value)
