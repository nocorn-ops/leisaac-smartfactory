"""渲染设置的版本兼容层（IsaacLab 2.x vs 3.0+）。

- IsaacLab 2.x：渲染设置位于 ``simulation_cfg.render``（``RenderCfg`` 实例），可逐项赋值。
- IsaacLab 3.0+：``simulation_cfg.render`` 被移除，渲染设置改为**按传感器的**
  ``renderer_cfg``（参见 ``isaaclab_physx.renderers.set_isaac_rtx_global_settings``）。

本层在 3.0+ 下跳过这些外观设置（不影响环境构建与物理仿真）；如需在 3.0 下恢复，
应对每个相机传感器调用 ``set_isaac_rtx_global_settings(camera_cfg.renderer_cfg, ...)``。
"""

from __future__ import annotations

from typing import Any


def apply_render_settings(simulation_cfg: Any, **params: Any) -> None:
    """把渲染参数写入 ``SimulationCfg``（IsaacLab 2.x）；3.0+ 下为空操作。

    Args:
        simulation_cfg: ``SimulationCfg`` 实例（``env_cfg.sim``）。
        **params: 渲染参数键值对，例如 ``enable_translucency=True``。
    """
    render_cfg = getattr(simulation_cfg, "render", None)
    if render_cfg is None:  # IsaacLab 3.0+：渲染设置已迁移到按传感器的 renderer_cfg
        return
    for key, value in params.items():
        setattr(render_cfg, key, value)
