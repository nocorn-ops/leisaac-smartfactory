import isaaclab.sim as sim_utils
import torch
from isaaclab.assets import AssetBaseCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass
from leisaac.assets.scenes.simple import TABLE_WITH_CUBE_CFG, TABLE_WITH_CUBE_USD_PATH
from leisaac.utils.domain_randomization import domain_randomization, randomize_camera_uniform, randomize_object_uniform
from leisaac.utils.env_utils import delete_attribute
from leisaac.utils.general_assets import parse_usd_and_create_subassets

from ..template import SingleArmObservationsCfg, SingleArmTaskEnvCfg, SingleArmTaskSceneCfg, SingleArmTerminationsCfg
from ..lift_cube import mdp


@configclass
class JxbLiftCubeSceneCfg(SingleArmTaskSceneCfg):
    """使用 jixiebi2022 的场景配置 — 修正 root_joint 路径."""

    scene: AssetBaseCfg = TABLE_WITH_CUBE_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")

    # 重写相机路径以适应 jixiebi2022 的 root_joint 层级
    wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/root_joint/gripper/wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.001, 0.1, -0.04), rot=(-0.912179, -0.0451242, 0.0486914, -0.404379), convention="ros"
            # XYZW（IsaacLab 3.0 起四元数顺序由 WXYZ 改为 XYZW，原值已重排）
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=36.5, focus_distance=400.0, horizontal_aperture=36.83,
            clipping_range=(0.01, 50.0), lock_camera=True,
        ),
        width=640, height=480, update_period=1 / 30.0,
    )

    front: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/root_joint/base/front_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.6, -0.75, 0.38), rot=(0.77337, 0.55078, -0.2374, -0.20537), convention="opengl"
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=40.6, focus_distance=400.0, horizontal_aperture=38.11,
            clipping_range=(0.01, 50.0), lock_camera=True,
        ),
        width=640, height=480, update_period=1 / 30.0,
    )

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=1000.0),
    )

    def __post_init__(self):
        super().__post_init__()
        self.ee_frame.prim_path = "{ENV_REGEX_NS}/Robot/root_joint/base"
        for tf in self.ee_frame.target_frames:
            tf.prim_path = tf.prim_path.replace("/Robot/", "/Robot/root_joint/")
        delete_attribute(self, "wrist")


@configclass
class ObservationsCfg(SingleArmObservationsCfg):

    @configclass
    class SubtaskCfg(ObsGroup):
        pick_cube = ObsTerm(func=mdp.object_grasped, params={
            "robot_cfg": SceneEntityCfg("robot"),
            "ee_frame_cfg": SceneEntityCfg("ee_frame"),
            "object_cfg": SceneEntityCfg("cube"),
        })
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    subtask_terms: SubtaskCfg = SubtaskCfg()

    def __post_init__(self):
        super().__post_init__()
        delete_attribute(self.policy, "wrist")


@configclass
class TerminationsCfg(SingleArmTerminationsCfg):
    success = DoneTerm(func=mdp.cube_height_above_base, params={
        "cube_cfg": SceneEntityCfg("cube"),
        "robot_cfg": SceneEntityCfg("robot"),
        "robot_base_name": "root_joint",
        "height_threshold": 0.20,
    })


@configclass
class JxbLiftCubeEnvCfg(SingleArmTaskEnvCfg):
    scene: JxbLiftCubeSceneCfg = JxbLiftCubeSceneCfg(env_spacing=8.0)
    observations: ObservationsCfg = ObservationsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    task_description: str = "Lift the red cube up."

    def __post_init__(self) -> None:
        super().__post_init__()
        self.viewer.eye = (-0.4, -0.6, 0.5)
        self.viewer.lookat = (0.9, 0.0, -0.3)
        self.scene.robot.init_state.pos = (0.35, -0.64, 0.01)
        parse_usd_and_create_subassets(TABLE_WITH_CUBE_USD_PATH, self)
        domain_randomization(self, random_options=[
            randomize_object_uniform("cube", pose_range={
                "x": (-0.075, 0.075), "y": (-0.075, 0.075),
                "z": (0.0, 0.0), "yaw": (-30 * torch.pi / 180, 30 * torch.pi / 180),
            }),
            randomize_camera_uniform("front", pose_range={
                "x": (-0.005, 0.005), "y": (-0.005, 0.005), "z": (-0.005, 0.005),
                "roll": (-0.05 * torch.pi / 180, 0.05 * torch.pi / 180),
                "pitch": (-0.05 * torch.pi / 180, 0.05 * torch.pi / 180),
                "yaw": (-0.05 * torch.pi / 180, 0.05 * torch.pi / 180),
            }, convention="opengl"),
        ])
