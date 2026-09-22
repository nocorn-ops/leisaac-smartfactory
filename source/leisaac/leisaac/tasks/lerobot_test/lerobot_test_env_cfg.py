"""LeRobot 移动双臂测试任务 — 最简单场景 + 双臂 + 腰部升降."""
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass
from leisaac.assets.robots.lerobot import LEROBOT_ROBOT_CFG
from leisaac.assets.scenes.simple import TABLE_WITH_CUBE_CFG

from ..template import (
    SingleArmObservationsCfg,
    SingleArmTaskEnvCfg,
    SingleArmTaskSceneCfg,
    SingleArmTerminationsCfg,
)


@configclass
class LeRobotTestSceneCfg(SingleArmTaskSceneCfg):
    """桌子+方块 场景 + LeRobot 移动双臂."""

    scene: AssetBaseCfg = TABLE_WITH_CUBE_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")

    # 用我们自己的机器人配置
    robot: ArticulationCfg = LEROBOT_ROBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=1000.0),
    )


@configclass
class LeRobotTestEnvCfg(SingleArmTaskEnvCfg):
    """LeRobot 移动双臂测试."""

    scene: LeRobotTestSceneCfg = LeRobotTestSceneCfg(env_spacing=8.0)
    observations: SingleArmObservationsCfg = SingleArmObservationsCfg()
    terminations: SingleArmTerminationsCfg = SingleArmTerminationsCfg()
    task_description: str = "Test the leRobot mobile bi-arm manipulator."
    def __post_init__(self) -> None:
        super().__post_init__()
        self.viewer.eye = (2.0, -2.0, 2.0)
        self.viewer.lookat = (0.35, -0.64, 0.5)
        self.dynamic_reset_gripper_effort_limit = False
        # 去掉冲突的硬件组件
        self.scene.ee_frame = None
        self.scene.wrist = None
        self.scene.front = None
        # 简化动作配置为纯查看
        from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
        self.actions.arm_action = JointPositionActionCfg(
            asset_name="robot",
            joint_names=["left_shoulder_pan", "left_shoulder_lift", "left_elbow_flex",
                         "left_wrist_flex", "left_wrist_roll", "left_gripper",
                         "right_shoulder_pan", "right_shoulder_lift", "right_elbow_flex",
                         "right_wrist_flex", "right_wrist_roll", "right_gripper",
                         "waist_lift"],
            scale=1.0,
        )
        delattr(self.actions, "gripper_action")
        # 去掉相机相关的观测项
        for term in ["wrist", "front", "ee_frame_state", "joint_pos_target"]:
            try:
                delattr(self.observations.policy, term)
            except Exception:
                pass
