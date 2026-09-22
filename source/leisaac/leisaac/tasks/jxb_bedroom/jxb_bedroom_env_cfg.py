import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass
from leisaac.assets.scenes.bedroom import LIGHTWHEEL_BEDROOM_CFG
from leisaac.utils.constant import REPO_ROOT

from ..template import (
    SingleArmObservationsCfg,
    SingleArmTaskEnvCfg,
    SingleArmTaskSceneCfg,
    SingleArmTerminationsCfg,
)


@configclass
class JxbBedroomSceneCfg(SingleArmTaskSceneCfg):
    """卧室场景 + 单臂 SO101 — 降低相机分辨率节省 GPU 显存."""

    scene: AssetBaseCfg = LIGHTWHEEL_BEDROOM_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")

    def __post_init__(self):
        super().__post_init__()
        # 指向轻量版卧室（删了镜子、柜子、窗帘等）
        self.scene.spawn.usd_path = str(REPO_ROOT / "assets" / "scenes" / "lightwheel_bedroom" / "scene_light.usd")

    # 腕部相机：320×240
    wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/gripper/wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.001, 0.1, -0.04), rot=(-0.912179, -0.0451242, 0.0486914, -0.404379), convention="ros"
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=36.5, focus_distance=400.0, horizontal_aperture=36.83,
            clipping_range=(0.01, 50.0), lock_camera=True,
        ),
        width=320, height=240, update_period=1 / 30.0,
    )

    # 前置相机：320×240
    front: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base/front_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.0, -0.5, 0.6), rot=(-0.9862856, 0.0, 0.0, 0.1650476), convention="ros"
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=28.7, focus_distance=400.0, horizontal_aperture=38.11,
            clipping_range=(0.01, 50.0), lock_camera=True,
        ),
        width=320, height=240, update_period=1 / 30.0,
    )


@configclass
class JxbBedroomTerminationsCfg(SingleArmTerminationsCfg):
    """超时终止."""


@configclass
class JxbBedroomEnvCfg(SingleArmTaskEnvCfg):
    """卧室单臂遥操作 & 数据录制."""

    scene: JxbBedroomSceneCfg = JxbBedroomSceneCfg(env_spacing=8.0)
    observations: SingleArmObservationsCfg = SingleArmObservationsCfg()
    terminations: JxbBedroomTerminationsCfg = JxbBedroomTerminationsCfg()
    task_description: str = "Explore the bedroom and manipulate objects with the SO101 arm."

    def __post_init__(self) -> None:
        super().__post_init__()
        # 视角对准卧室
        self.viewer.eye = (0.0, 5.0, 4.0)
        self.viewer.lookat = (-0.9, 8.0, 3.25)
        # 机械臂放在卧室中间（和双臂左臂位置对齐）
        self.scene.robot.init_state.pos = (-0.9, 8.3, 3.25)
