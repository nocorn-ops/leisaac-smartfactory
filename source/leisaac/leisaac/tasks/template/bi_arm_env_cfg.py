from dataclasses import MISSING
from typing import Any

import isaaclab.sim as sim_utils
import numpy as np
import torch
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.recorders.recorders_cfg import (
    ActionStateRecorderManagerCfg as RecordTerm,
)
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import TiledCameraCfg
from isaaclab.utils import configclass
from isaaclab.utils.datasets.episode_data import EpisodeData
from leisaac.assets.robots.lerobot import SO101_FOLLOWER_CFG
from leisaac.devices.action_process import init_action_cfg, preprocess_device_action
from leisaac.enhance.datasets.lerobot_dataset_handler import LeRobotDatasetCfg
from leisaac.utils.constant import BI_ARM_JOINT_NAMES
from leisaac.utils.physx_compat import apply_physx_settings
from leisaac.utils.render_compat import apply_render_settings
from leisaac.utils.robot_utils import convert_leisaac_action_to_lerobot

from . import mdp


@configclass
class BiArmTaskSceneCfg(InteractiveSceneCfg):
    """Scene configuration for the bi arm task."""

    scene: AssetBaseCfg = MISSING

    left_arm: ArticulationCfg = SO101_FOLLOWER_CFG.replace(prim_path="{ENV_REGEX_NS}/Left_Robot")

    right_arm: ArticulationCfg = SO101_FOLLOWER_CFG.replace(prim_path="{ENV_REGEX_NS}/Right_Robot")

    left_wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Left_Robot/gripper/left_wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.001, 0.1, -0.04), rot=(-0.912179, -0.0451242, 0.0486914, -0.404379), convention="ros"
        ),  # XYZW（IsaacLab 3.0 起四元数顺序由 WXYZ 改为 XYZW，原值已重排）
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=36.5,
            focus_distance=400.0,
            horizontal_aperture=36.83,  # For a 75° FOV (assuming square image)
            clipping_range=(0.01, 50.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
        update_period=1 / 30.0,  # 30FPS
    )

    right_wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Right_Robot/gripper/right_wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.001, 0.1, -0.04), rot=(-0.912179, -0.0451242, 0.0486914, -0.404379), convention="ros"
        ),  # XYZW（IsaacLab 3.0 起四元数顺序由 WXYZ 改为 XYZW，原值已重排）
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=36.5,
            focus_distance=400.0,
            horizontal_aperture=36.83,  # For a 75° FOV (assuming square image)
            clipping_range=(0.01, 50.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
        update_period=1 / 30.0,  # 30FPS
    )

    top: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Right_Robot/base/top_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.225, -0.5, 0.6), rot=(-0.9862856, 0.0, 0.0, 0.1650476), convention="ros"
        ),  # XYZW（原 wxyz 值重排）
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=28.7,
            focus_distance=400.0,
            horizontal_aperture=38.11,  # For a 78° FOV (assuming square image)
            clipping_range=(0.01, 50.0),
            lock_camera=True,
        ),
        width=640,
        height=480,
        update_period=1 / 30.0,  # 30FPS
    )

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


@configclass
class BiArmActionsCfg:
    """Configuration for the actions."""

    left_arm_action: mdp.ActionTermCfg = MISSING
    left_gripper_action: mdp.ActionTermCfg = MISSING
    right_arm_action: mdp.ActionTermCfg = MISSING
    right_gripper_action: mdp.ActionTermCfg = MISSING


@configclass
class BiArmEventCfg:
    """Configuration for the events."""

    # reset to default scene
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


@configclass
class BiArmObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        left_joint_pos = ObsTerm(func=mdp.joint_pos, params={"asset_cfg": SceneEntityCfg("left_arm")})
        left_joint_vel = ObsTerm(func=mdp.joint_vel, params={"asset_cfg": SceneEntityCfg("left_arm")})
        left_joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("left_arm")})
        left_joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("left_arm")})
        left_joint_pos_target = ObsTerm(func=mdp.joint_pos_target, params={"asset_cfg": SceneEntityCfg("left_arm")})

        right_joint_pos = ObsTerm(func=mdp.joint_pos, params={"asset_cfg": SceneEntityCfg("right_arm")})
        right_joint_vel = ObsTerm(func=mdp.joint_vel, params={"asset_cfg": SceneEntityCfg("right_arm")})
        right_joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("right_arm")})
        right_joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("right_arm")})
        right_joint_pos_target = ObsTerm(func=mdp.joint_pos_target, params={"asset_cfg": SceneEntityCfg("right_arm")})

        actions = ObsTerm(func=mdp.last_action)
        left_wrist = ObsTerm(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("left_wrist"), "data_type": "rgb", "normalize": False}
        )
        right_wrist = ObsTerm(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("right_wrist"), "data_type": "rgb", "normalize": False}
        )
        top = ObsTerm(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("top"), "data_type": "rgb", "normalize": False}
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = False

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class BiArmRewardsCfg:
    """Configuration for the rewards"""


@configclass
class BiArmTerminationsCfg:
    """Configuration for the termination"""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class BiArmTaskEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the bi arm task template environment."""

    scene: BiArmTaskSceneCfg = MISSING

    observations: BiArmObservationsCfg = MISSING
    actions: BiArmActionsCfg = BiArmActionsCfg()
    events: BiArmEventCfg = BiArmEventCfg()

    rewards: BiArmRewardsCfg = BiArmRewardsCfg()
    terminations: BiArmTerminationsCfg = MISSING

    recorders: RecordTerm = RecordTerm()

    dynamic_reset_gripper_effort_limit: bool = True
    """Whether to dynamically reset the gripper effort limit."""

    robot_name: str = "bi_so101_follower"
    """Robot name for lerobot dataset export."""
    default_feature_joint_names: list[str] = MISSING
    """Default feature joint names for lerobot dataset export."""
    task_description: str = MISSING
    """Task description for lerobot dataset export."""

    def __post_init__(self) -> None:
        super().__post_init__()

        self.decimation = 1
        self.episode_length_s = 25.0
        self.viewer.eye = (2.5, -1.0, 1.3)
        self.viewer.lookat = (3.6, -0.4, 1.0)

        apply_physx_settings(self.sim, bounce_threshold_velocity=0.01, friction_correlation_distance=0.00625)
        apply_render_settings(self.sim, enable_translucency=True)

        self.scene.left_arm.init_state.pos = (3.4, -0.65, 0.89)
        self.scene.right_arm.init_state.pos = (3.8, -0.65, 0.89)

        self.default_feature_joint_names = [f"{joint_name}.pos" for joint_name in BI_ARM_JOINT_NAMES]

    def use_teleop_device(self, teleop_device) -> None:
        self.task_type = teleop_device
        self.actions = init_action_cfg(self.actions, device=teleop_device)
        # ★ 增量/IK 类设备（键盘、手柄、状态机）要**关掉机械臂的重力**。
        #
        #   为什么：这些设备走 `use_relative_mode=True` 的 IK，"什么键都不按" = 零位姿增量
        #   = 目标跟住**当前**位姿，而不是锁死在某个目标 —— 于是 PD 控制器误差恒为 0，
        #   整条臂在重力下**匀速往下塌**（实测默认 PD 下 1 秒掉 22.5 cm，加了 PD 补偿后
        #   仍有 15.4 cm/30 步）。表现就是"一打开遥操臂就往下塌、手感极差、数据也脏"。
        #   关掉臂的重力后零动作=真的不动（物体仍然有重力，抓取手感不变）。
        #
        #   `bi_so101_state_machine` 是上游原有的同类处理；`bi-keyboard`/`bi-gamepad` 是本项目
        #   新增的双臂 IK 设备，必须一并列入，否则双臂任务上键盘遥操会一直往下塌。
        #   单臂模板 `single_arm_env_cfg.py` 对 `keyboard`/`gamepad` 早就是这么做的。
        if teleop_device in ["bi_so101_state_machine", "bi-keyboard", "bi-gamepad"]:
            self.scene.left_arm.spawn.rigid_props.disable_gravity = True
            self.scene.right_arm.spawn.rigid_props.disable_gravity = True

    def preprocess_device_action(self, action: dict[str, Any], teleop_device) -> torch.Tensor:
        return preprocess_device_action(action, teleop_device)

    def build_lerobot_frame(self, episode_data: EpisodeData, dataset_cfg: LeRobotDatasetCfg) -> dict:
        obs_data = episode_data._data["obs"]
        action = episode_data._data["actions"][-1]
        if dataset_cfg.action_align:
            action = action.unsqueeze(0)
            left_arm_action = convert_leisaac_action_to_lerobot(action[:, :6]).squeeze(0)
            right_arm_action = convert_leisaac_action_to_lerobot(action[:, 6:]).squeeze(0)
            processed_action = np.concatenate([left_arm_action, right_arm_action], axis=0)
        else:
            processed_action = action.cpu().numpy()
        frame = {
            "action": processed_action,
            "observation.state": np.concatenate(
                [
                    convert_leisaac_action_to_lerobot(obs_data["left_joint_pos"][-1].unsqueeze(0)).squeeze(0),
                    convert_leisaac_action_to_lerobot(obs_data["right_joint_pos"][-1].unsqueeze(0)).squeeze(0),
                ],
                axis=0,
            ),
            "task": self.task_description,
        }
        for frame_key in dataset_cfg.features.keys():
            if not frame_key.startswith("observation.images"):
                continue
            camera_key = frame_key.split(".")[-1]
            frame[frame_key] = obs_data[camera_key][-1].cpu().numpy()

        return frame
