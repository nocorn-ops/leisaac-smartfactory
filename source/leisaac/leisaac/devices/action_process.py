from dataclasses import MISSING, fields
from typing import Any

import isaaclab.envs.mdp as mdp
import torch
from leisaac.assets.robots.lerobot import SO101_FOLLOWER_USD_JOINT_LIMLITS

try:  # IsaacLab 3.0：DifferentialIKControllerCfg 不再从 isaaclab.envs.mdp 转出，改由 isaaclab.controllers 提供
    from isaaclab.controllers import DifferentialIKControllerCfg
except ImportError:  # IsaacLab 2.x
    DifferentialIKControllerCfg = mdp.DifferentialIKControllerCfg


def init_action_cfg(action_cfg, device):
    """SO101 Follower action configuration: arm_action and gripper_action"""
    if device in ["so101leader", "lekiwi-leader"]:
        action_cfg.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            scale=1.0,
        )
        action_cfg.gripper_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["gripper"],
            scale=1.0,
        )
    elif device in ["keyboard", "gamepad", "lekiwi-keyboard", "lekiwi-gamepad"]:
        action_cfg.arm_action = mdp.DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            body_name="gripper",
            controller=DifferentialIKControllerCfg(command_type="pose", ik_method="dls", use_relative_mode=True),
        )
        action_cfg.gripper_action = mdp.RelativeJointPositionActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "gripper"],
            scale=1.0,
        )
    elif device in ["bi-so101leader", "so101leader-one"]:
        # so101leader-one：只有一条主手，但任务仍是双臂 12 维结构
        # （另一半在 preprocess_device_action 里被锁成"当前关节位置"）
        action_cfg.left_arm_action = mdp.JointPositionActionCfg(
            asset_name="left_arm",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            scale=1.0,
        )
        action_cfg.left_gripper_action = mdp.JointPositionActionCfg(
            asset_name="left_arm",
            joint_names=["gripper"],
            scale=1.0,
        )
        action_cfg.right_arm_action = mdp.JointPositionActionCfg(
            asset_name="right_arm",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            scale=1.0,
        )
        action_cfg.right_gripper_action = mdp.JointPositionActionCfg(
            asset_name="right_arm",
            joint_names=["gripper"],
            scale=1.0,
        )
    elif device in ["mimic_so101leader"]:
        action_cfg.arm_action = mdp.DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            body_name="gripper",
            controller=DifferentialIKControllerCfg(command_type="pose", ik_method="dls", use_relative_mode=False),
        )
        action_cfg.gripper_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["gripper"],
            scale=1.0,
        )
    elif device in ["mimic_keyboard", "mimic_gamepad"]:
        action_cfg.arm_action = mdp.DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            body_name="gripper",
            controller=DifferentialIKControllerCfg(command_type="pose", ik_method="dls", use_relative_mode=False),
        )
        action_cfg.gripper_action = mdp.RelativeJointPositionActionCfg(
            asset_name="robot",
            joint_names=["gripper"],
            scale=1.0,
        )
    elif device in ["so101_state_machine"]:  # IK-based: action = EE pose (7D) + binary gripper, not raw joint angles
        action_cfg.arm_action = mdp.DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            body_name="gripper",
            controller=DifferentialIKControllerCfg(
                command_type="pose", ik_method="dls", ik_params={"lambda_val": 0.04}
            ),
        )
        action_cfg.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["gripper"],
            open_command_expr={"gripper": 1.0},
            close_command_expr={"gripper": 0.4},
        )
    elif device in ["bi_so101_state_machine"]:  # IK-based: action = EE pose (7D) + binary gripper, not raw joint angles
        action_cfg.left_arm_action = mdp.DifferentialInverseKinematicsActionCfg(
            asset_name="left_arm",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            body_name="gripper",
            controller=DifferentialIKControllerCfg(
                command_type="pose", ik_method="dls", ik_params={"lambda_val": 0.04}
            ),
        )
        action_cfg.left_gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="left_arm",
            joint_names=["gripper"],
            open_command_expr={"gripper": 1.0},
            close_command_expr={"gripper": 0.01},
        )
        action_cfg.right_arm_action = mdp.DifferentialInverseKinematicsActionCfg(
            asset_name="right_arm",
            joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
            body_name="gripper",
            controller=DifferentialIKControllerCfg(
                command_type="pose", ik_method="dls", ik_params={"lambda_val": 0.04}
            ),
        )
        action_cfg.right_gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="right_arm",
            joint_names=["gripper"],
            open_command_expr={"gripper": 1.0},
            close_command_expr={"gripper": 0.01},
        )
    elif device in ["bi-keyboard", "bi-gamepad"]:
        # 双臂键盘/手柄：**每臂 8 维**（末端位姿增量 6 + shoulder_pan 1 + 夹爪 1），合计 16 维。
        # 结构与单臂的 `keyboard` 分支逐项一致，只是实体换成 left_arm / right_arm
        # —— 单臂那支写死了 `asset_name="robot"`，双臂场景里没有这个实体（实测 KeyError）。
        # 4 个 term 的顺序必须与 `BiArmActionsCfg` 的字段顺序一致：
        #   left_arm_action(6) / left_gripper_action(2) / right_arm_action(6) / right_gripper_action(2)
        for side in ("left", "right"):
            setattr(
                action_cfg,
                f"{side}_arm_action",
                mdp.DifferentialInverseKinematicsActionCfg(
                    asset_name=f"{side}_arm",
                    joint_names=["shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
                    body_name="gripper",
                    controller=DifferentialIKControllerCfg(
                        command_type="pose", ik_method="dls", use_relative_mode=True
                    ),
                ),
            )
            setattr(
                action_cfg,
                f"{side}_gripper_action",
                mdp.RelativeJointPositionActionCfg(
                    asset_name=f"{side}_arm",
                    joint_names=["shoulder_pan", "gripper"],
                    scale=1.0,
                ),
            )
    """LeKiwi action configuration"""
    if device in ["lekiwi-leader", "lekiwi-keyboard", "lekiwi-gamepad"]:
        action_cfg.wheel_action = mdp.JointVelocityActionCfg(
            asset_name="robot",
            joint_names=["base_x", "base_y", "base_theta"],
            scale=1.0,
        )

    """Check if all the action configurations are set"""
    for field in fields(action_cfg):
        value = getattr(action_cfg, field.name, None)
        if value is None or value is MISSING:
            raise ValueError(f"Action configuration '{field.name}' for {device} is not set")

    return action_cfg


joint_names_to_motor_ids = {
    "shoulder_pan": 0,
    "shoulder_lift": 1,
    "elbow_flex": 2,
    "wrist_flex": 3,
    "wrist_roll": 4,
    "gripper": 5,
}


def convert_action_from_so101_leader(
    joint_state: dict[str, float], motor_limits: dict[str, tuple[float, float]], teleop_device
) -> torch.Tensor:
    processed_action = torch.zeros(teleop_device.env.num_envs, 6, device=teleop_device.env.device)
    joint_limits = SO101_FOLLOWER_USD_JOINT_LIMLITS
    for joint_name, motor_id in joint_names_to_motor_ids.items():
        motor_limit_range = motor_limits[joint_name]
        joint_limit_range = joint_limits[joint_name]
        motor_range = motor_limit_range[1] - motor_limit_range[0]
        joint_range = joint_limit_range[1] - joint_limit_range[0]
        motor_degree = joint_state[joint_name] - motor_limit_range[0]
        processed_degree = motor_degree / motor_range * joint_range + joint_limit_range[0]
        processed_radius = processed_degree / 180.0 * torch.pi  # convert degree to radius
        processed_action[:, motor_id] = processed_radius
    return processed_action


def preprocess_device_action(action: dict[str, Any], teleop_device) -> torch.Tensor:
    if action.get("so101_leader") is not None:
        processed_action = convert_action_from_so101_leader(
            action["joint_state"], action["motor_limits"], teleop_device
        )
    elif action.get("keyboard") is not None or action.get("gamepad") is not None:
        processed_action = torch.zeros(teleop_device.env.num_envs, 8, device=teleop_device.env.device)
        processed_action[:, :] = action["joint_state"]
    elif action.get("bi_so101_keyboard") is not None:
        # 双臂键盘：设备侧已经拼好 16 维 = [左臂 8, 右臂 8]（每臂各自从 gripper 系转到了该臂基座系），
        # 并且与上面 `bi-keyboard` 的 4 个 term 顺序对齐，直接整块写进去即可。
        processed_action = torch.zeros(teleop_device.env.num_envs, 16, device=teleop_device.env.device)
        processed_action[:, :] = action["joint_state"]
    elif action.get("bi_so101_leader") is not None:
        processed_action = torch.zeros(teleop_device.env.num_envs, 12, device=teleop_device.env.device)
        processed_action[:, :6] = convert_action_from_so101_leader(
            action["joint_state"]["left_arm"], action["motor_limits"]["left_arm"], teleop_device
        )
        processed_action[:, 6:] = convert_action_from_so101_leader(
            action["joint_state"]["right_arm"], action["motor_limits"]["right_arm"], teleop_device
        )
    elif action.get("so101_leader_one") is not None:
        # 单臂主手驱动双臂任务：受控那半边来自主手，另一半锁定为从手当前关节位置（弧度），
        # 动作空间仍是 12 维，录下来的数据格式与双臂采集一致。
        processed_action = torch.zeros(teleop_device.env.num_envs, 12, device=teleop_device.env.device)
        # 设备侧已把软启动斜坡算好（action["controlled"]，弧度）；没有时退回直接转换
        controlled = action.get("controlled")
        if controlled is None:
            controlled = convert_action_from_so101_leader(
                action["joint_state"], action["motor_limits"], teleop_device
            )
        else:
            controlled = controlled.to(device=processed_action.device, dtype=processed_action.dtype)
        side = action["side"]
        # hold 是设备侧在 reset 时抓下来的固定目标（弧度，[5 臂关节, gripper]），
        # 每步都用它 —— 若改成"读当前值"，PD 误差恒为 0，那只手臂会自由下沉。
        hold = action["hold"]
        hold = hold.torch if hasattr(hold, "torch") else hold  # IsaacLab 3.0: ProxyArray
        hold = hold.to(device=processed_action.device, dtype=processed_action.dtype)
        if side == "left":
            processed_action[:, :6] = controlled
            processed_action[:, 6:] = hold
        else:
            processed_action[:, 6:] = controlled
            processed_action[:, :6] = hold
    elif action.get("lekiwi-leader") is not None:
        processed_action = torch.zeros(teleop_device.env.num_envs, 9, device=teleop_device.env.device)
        processed_action[:, :6] = convert_action_from_so101_leader(
            action["joint_state"]["arm_action"], action["motor_limits"], teleop_device
        )
        processed_action[:, 6:] = action["joint_state"]["wheel_action"]
    elif action.get("lekiwi-keyboard") is not None or action.get("lekiwi-gamepad") is not None:
        processed_action = torch.zeros(teleop_device.env.num_envs, 11, device=teleop_device.env.device)
        processed_action[:, :] = action["joint_state"]
    else:
        raise NotImplementedError(f"Not implemented for this device now: {teleop_device.device_type}")
    return processed_action
