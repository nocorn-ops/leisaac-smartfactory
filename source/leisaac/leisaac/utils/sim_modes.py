"""统一仿真入口的动作源构造器 —— 上肢动作从哪来。

背景：原来三个入口各管一段
    scripts/environments/ros2_chassis_teleop.py      底盘（ROS2 遥控）+ 桥接 + 相机 + HUD
    scripts/environments/teleoperation/teleop_se3_agent.py   上肢遥操 + 录制
    scripts/evaluation/policy_inference.py           上肢推理 + 动作安全层

那两个脚本**不能被 import**（模块顶层就 `AppLauncher(...)` + `argparse`，一 import 就会
再起一个 Isaac Sim）。所以这里把它们的"设备/策略构造"逻辑照搬过来，供统一入口按
``--mode`` 选择；原脚本保持不动，仍可作为单一功能工具使用。

    nav     上肢保持初始位姿（hold 在入口脚本里就地算，不需要本模块）
    teleop  build_device()   ← teleop_se3_agent.py 的 device if/elif 链
    policy  build_policy()   ← policy_inference.py 的 policy if/elif 链
"""

from __future__ import annotations

import base64
import io
import json
import socket

import numpy as np
import torch


# ─────────────────────────────────────────────────────────────────────────────
# 上肢遥操设备（照搬 teleop_se3_agent.py）
# ─────────────────────────────────────────────────────────────────────────────
def build_device(
    env,
    *,
    device: str,
    sensitivity: float = 1.0,
    port: str | None = None,
    remote_endpoint: str | None = None,
    recalibrate: bool = False,
    left_arm_port: str | None = None,
    right_arm_port: str | None = None,
    arm_side: str = "left",
    relative_one_arm: bool = False,
    engage_threshold: float | None = None,
):
    """按名字构建设备。返回的对象有 `reset()` / `advance()` / `add_callback(key, fn)`。"""
    if device == "keyboard":
        from leisaac.devices import SO101Keyboard

        return SO101Keyboard(env, sensitivity=sensitivity)
    if device == "gamepad":
        from leisaac.devices import SO101Gamepad

        return SO101Gamepad(env, sensitivity=sensitivity)
    if device == "so101leader":
        if remote_endpoint:
            from leisaac.devices import SO101LeaderRemote

            return SO101LeaderRemote(env, endpoint=remote_endpoint)
        from leisaac.devices import SO101Leader

        return SO101Leader(env, port=port, recalibrate=recalibrate)
    if device == "bi-so101leader":
        from leisaac.devices import BiSO101Leader

        return BiSO101Leader(
            env, left_port=left_arm_port, right_port=right_arm_port, recalibrate=recalibrate
        )
    if device == "bi-keyboard":
        # 双臂键盘遥操（给**没有真主手**的队伍）：T 切换左右臂，键位与单臂键盘逐项一致。
        # 动作空间 16 维（每臂 8 维增量），与真主手的 12 维关节角不同 —— 见操作指南 §5（发布包 `README.md` / 开发机 `README1.md`）。
        from leisaac.devices import BiSO101Keyboard

        return BiSO101Keyboard(env, sensitivity=sensitivity, start_side=arm_side)
    if device == "lekiwi-keyboard":
        from leisaac.devices import LeKiwiKeyboard

        return LeKiwiKeyboard(env, sensitivity=sensitivity)
    if device == "lekiwi-leader":
        from leisaac.devices import LeKiwiLeader

        return LeKiwiLeader(env, port=port, recalibrate=recalibrate)
    if device == "lekiwi-gamepad":
        from leisaac.devices import LeKiwiGamepad

        return LeKiwiGamepad(env, sensitivity=sensitivity)
    if device == "so101leader-one":
        from leisaac.devices import SO101LeaderOneArm

        return SO101LeaderOneArm(
            env,
            port=port,
            side=arm_side,
            recalibrate=recalibrate,
            relative=relative_one_arm,
            engage_threshold=engage_threshold,
        )
    raise ValueError(
        f"Invalid device interface '{device}'. Supported: 'bi-so101leader', 'so101leader-one',"
        " 'bi-keyboard'（双臂键盘）, 'keyboard'/'gamepad'（仅单臂任务）, 'so101leader',"
        " 'lekiwi-keyboard', 'lekiwi-leader', 'lekiwi-gamepad'."
    )


# ─────────────────────────────────────────────────────────────────────────────
# 策略（照搬 policy_inference.py）
# ─────────────────────────────────────────────────────────────────────────────
def preprocess_obs_dict(obs_dict: dict, model_type: str, language_instruction: str | None):
    """Preprocess the observation dictionary to the format expected by the policy."""
    if model_type in ["gr00tn1.5", "gr00tn1.6", "lerobot", "openpi", "local-act"]:
        obs_dict["task_description"] = language_instruction
        return obs_dict
    raise ValueError(f"Model type {model_type} not supported")


class TCPActPolicy:
    """直接 TCP 连接本地 ACT action server（不经过 gRPC）—— 照搬 policy_inference.py。"""

    def __init__(self, env, host: str, port: int):
        from isaaclab.sensors import Camera

        self._camera_keys = [k for k, s in env.scene.sensors.items() if isinstance(s, Camera)]
        self.sock = socket.socket()
        self.sock.settimeout(30)
        self.sock.connect((host, port))
        self._rfile = self.sock.makefile("rb")

    def get_action(self, obs_dict: dict) -> torch.Tensor:
        from PIL import Image

        from leisaac.utils.robot_utils import (
            convert_lerobot_action_to_leisaac,
            convert_leisaac_action_to_lerobot,
        )

        images_b64 = {}
        for key in self._camera_keys:
            img = obs_dict[key]
            if isinstance(img, torch.Tensor):
                img = img.cpu().numpy()
            if img.ndim == 4:
                img = img[0]
            pil_img = Image.fromarray(img.astype(np.uint8))
            buf = io.BytesIO()
            pil_img.save(buf, format="PNG")
            images_b64[key] = base64.b64encode(buf.getvalue()).decode("utf-8")

        # State: joint positions —— 弧度 → 电机值（与训练数据一致）。键名以观测组为准
        # （`left_joint_pos` / `right_joint_pos`），**不是** `joint_state`
        # （那个是设备侧 `device_base.py` 里的键，别搞混）。
        left = obs_dict.get("left_joint_pos", torch.zeros(1, 6))
        right = obs_dict.get("right_joint_pos", torch.zeros(1, 6))
        if isinstance(left, torch.Tensor):
            left = left.cpu().numpy()
        if isinstance(right, torch.Tensor):
            right = right.cpu().numpy()
        left = left.reshape(1, -1)[:, :6]
        right = right.reshape(1, -1)[:, :6]
        state = np.concatenate(
            [
                convert_leisaac_action_to_lerobot(left).reshape(-1),
                convert_leisaac_action_to_lerobot(right).reshape(-1),
            ]
        ).tolist()

        request = json.dumps({"images": images_b64, "state": state}) + "\n"
        self.sock.sendall(request.encode())
        result = json.loads(self._rfile.readline().decode().strip())
        if result.get("error"):
            raise RuntimeError(f"Policy server error: {result['error']}")
        actions = np.array(result["actions"])  # (chunk_size, 12)
        actions = convert_lerobot_action_to_leisaac(actions)
        return torch.from_numpy(actions[:, None, :])

    def close(self):
        self.sock.close()


def build_policy(
    env,
    *,
    policy_type: str,
    task_type: str,
    host: str = "localhost",
    port: int = 5555,
    timeout_ms: int = 15000,
    action_horizon: int = 16,
    language_instruction: str | None = None,
    checkpoint_path: str | None = None,
    policy_device: str = "cpu",
):
    """按名字构建策略客户端。返回对象有 `get_action(obs_dict) -> (horizon, n_env, act_dim)`。"""
    if policy_type == "local-act":
        return TCPActPolicy(env, host, port)

    if policy_type == "gr00tn1.5":
        from leisaac.policy import Gr00tServicePolicyClient

        if task_type != "so101leader":
            raise ValueError(f"Task type {task_type} not supported when using GR00T N1.5 policy yet.")
        from isaaclab.sensors import Camera

        return Gr00tServicePolicyClient(
            host=host,
            port=port,
            timeout_ms=timeout_ms,
            camera_keys=[k for k, s in env.scene.sensors.items() if isinstance(s, Camera)],
            modality_keys=["single_arm", "gripper"],
        )

    if policy_type == "gr00tn1.6":
        from leisaac.policy import Gr00t16ServicePolicyClient

        if task_type != "so101leader":
            raise ValueError(f"Task type {task_type} not supported when using GR00T N1.6 policy yet.")
        from isaaclab.sensors import Camera

        return Gr00t16ServicePolicyClient(
            host=host,
            port=port,
            timeout_ms=timeout_ms,
            camera_keys=[k for k, s in env.scene.sensors.items() if isinstance(s, Camera)],
            modality_keys=["single_arm", "gripper"],
        )

    if "lerobot" in policy_type:
        from isaaclab.sensors import Camera
        from leisaac.policy import LeRobotServicePolicyClient

        return LeRobotServicePolicyClient(
            host=host,
            port=port,
            timeout_ms=timeout_ms,
            camera_infos={
                k: s.image_shape for k, s in env.scene.sensors.items() if isinstance(s, Camera)
            },
            task_type=task_type,
            policy_type=policy_type.split("-")[1],
            pretrained_name_or_path=checkpoint_path,
            actions_per_chunk=action_horizon,
            device=policy_device,
        )

    if policy_type == "openpi":
        from isaaclab.sensors import Camera
        from leisaac.policy import OpenPIServicePolicyClient

        return OpenPIServicePolicyClient(
            host=host,
            port=port,
            camera_keys=[k for k, s in env.scene.sensors.items() if isinstance(s, Camera)],
            task_type=task_type,
        )

    raise ValueError(
        f"Invalid policy type '{policy_type}'. Supported: 'local-act', 'lerobot-<model>',"
        " 'gr00tn1.5', 'gr00tn1.6', 'openpi'."
    )


#: 这些策略类型的观测需要 `preprocess_obs_dict` 把语言指令塞进 `task_description`
OBS_NEEDS_LANGUAGE = ("gr00tn1.5", "gr00tn1.6", "lerobot", "openpi", "local-act")


# ─────────────────────────────────────────────────────────────────────────────
# 观测组注入（遥操录制 / 推理需要，场景任务本身没有）
# ─────────────────────────────────────────────────────────────────────────────
def ensure_observations(env_cfg) -> bool:
    """给"观测是空的"任务补一个双臂观测组；已有观测组的任务不动。返回是否补了。

    为什么需要：`LeIsaac-SmartFactory-v0` 的 `observations = EmptyObsCfg()`
    （M3 只做到"能开、能建图导航"）。**不带录制**的上肢遥操其实不需要观测
    （设备直接给出动作），但：
      · `--record` 要有观测可存；
      · `--mode policy` 要把 `obs_dict["policy"]["left_joint_pos"]` / 三路相机交给策略。

    ★ 采用**注入**而不是去改 `smart_factory_env_cfg.py`：这样 M1/M2/M3 那些场景脚本
      （`shot_scene.py` / `verify_smart_factory_*`）完全不受影响 —— 一旦任务自带
      图像观测项，那些没开相机渲染的脚本就会挂。

    观测项直接复用厨房那套定义：场地与厨房是同一台机器人，场景实体名
    (`left_arm` / `right_arm` / `left_wrist` / `right_wrist` / `front`) 完全一致
    （`smart_factory_env_cfg.py` 的注释里明确写了要保持一致）。
    """
    if hasattr(getattr(env_cfg, "observations", None), "policy"):
        return False  # 任务自带观测组，别动

    from leisaac.tasks.lerobot_kitchen.lerobot_kitchen_env_cfg import (
        LeRobotKitchenObservationsCfg,
    )

    env_cfg.observations = LeRobotKitchenObservationsCfg()
    return True
