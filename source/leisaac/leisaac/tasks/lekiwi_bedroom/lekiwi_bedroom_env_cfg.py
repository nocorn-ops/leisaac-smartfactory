from isaaclab.assets import AssetBaseCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from leisaac.assets.scenes.bedroom import LIGHTWHEEL_BEDROOM_CFG

from ..template import (
    LeKiwiObservationsCfg,
    LeKiwiTaskEnvCfg,
    LeKiwiTaskSceneCfg,
    LeKiwiTerminationsCfg,
)


@configclass
class LeKiwiBedroomSceneCfg(LeKiwiTaskSceneCfg):
    """卧室场景 + LeKiwi 机器人."""

    scene: AssetBaseCfg = LIGHTWHEEL_BEDROOM_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")


@configclass
class LeKiwiBedroomTerminationsCfg(LeKiwiTerminationsCfg):
    """只有超时终止."""


@configclass
class LeKiwiBedroomEnvCfg(LeKiwiTaskEnvCfg):
    """卧室场景下操控 LeKiwi 移动机械臂."""

    scene: LeKiwiBedroomSceneCfg = LeKiwiBedroomSceneCfg(env_spacing=8.0)
    observations: LeKiwiObservationsCfg = LeKiwiObservationsCfg()
    terminations: LeKiwiBedroomTerminationsCfg = LeKiwiBedroomTerminationsCfg()
    task_description: str = "Explore the bedroom with LeKiwi mobile manipulator."

    def __post_init__(self) -> None:
        super().__post_init__()
        # 视角对准卧室区域（参考 fold_cloth 的坐标）
        self.viewer.eye = (2.0, 5.0, 3.0)
        self.viewer.lookat = (0.0, 7.0, 2.0)
        # LeKiwi 放到卧室中间
        self.scene.robot.init_state.pos = (0.0, 7.0, 3.0)
        self.scene.robot.init_state.rot = (1.0, 0.0, 0.0, 0.0)
