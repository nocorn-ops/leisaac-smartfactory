"""Script to run a leisaac inference with leisaac in the simulation.

⚠️ **这是"内部实现 / 单一功能工具"，不是对外的仿真入口。**
   对外的**唯一仿真启动入口**是 `reproduce/ros2_chassis_teleop.sh`：
       建图/导航   → --mode nav
       遥操/数采   → --mode teleop [--record ...]
       验证推理    → --mode policy --policy_type ...
   本脚本保留是因为它多了 `--eval_rounds N`（无头批量跑 N 轮评测）这个统一入口没有的能力；
   只在需要批量评测时直接用，日常请走统一入口。
"""

"""Launch Isaac Sim Simulator first."""
import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse
import os
import sys

# ── 让本仓库的 `leisaac` 包可被 import（不依赖 editable 安装 / 外部 PYTHONPATH）──
# 打包给赛队时仓库可能被解压到任意目录、也不一定跑过 `pip install -e source/leisaac`；
# 这里从本文件往上找到含 `source/leisaac` 的仓库根，自己补 sys.path（不依赖 CWD）。
_p = os.path.dirname(os.path.abspath(__file__))
while _p != os.path.dirname(_p):
    if os.path.isdir(os.path.join(_p, "source", "leisaac")):
        sys.path.insert(0, os.path.join(_p, "source", "leisaac"))
        break
    _p = os.path.dirname(_p)

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="leisaac inference for leisaac in the simulation.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--step_hz", type=int, default=60, help="Environment stepping rate in Hz.")
parser.add_argument("--seed", type=int, default=None, help="Seed of the environment.")
parser.add_argument("--episode_length_s", type=float, default=60.0, help="Episode length in seconds.")
parser.add_argument(
    "--eval_rounds",
    type=int,
    default=0,
    help=(
        "Number of evaluation rounds. 0 means don't add time out termination, policy will run until success or manual"
        " reset."
    ),
)
parser.add_argument(
    "--policy_type",
    type=str,
    default="gr00tn1.5",
    help="Type of policy to use. support gr00tn1.5, gr00tn1.6, lerobot-<model_type>, openpi",
)
parser.add_argument("--policy_host", type=str, default="localhost", help="Host of the policy server.")
parser.add_argument("--policy_port", type=int, default=5555, help="Port of the policy server.")
parser.add_argument("--policy_timeout_ms", type=int, default=15000, help="Timeout of the policy server.")
parser.add_argument("--policy_action_horizon", type=int, default=16, help="Action horizon of the policy.")
parser.add_argument("--policy_language_instruction", type=str, default=None, help="Language instruction of the policy.")
parser.add_argument("--policy_checkpoint_path", type=str, default=None, help="Checkpoint path of the policy.")
parser.add_argument("--policy_device", type=str, default="cpu", help="Device for the policy server (cpu/cuda).")
parser.add_argument(
    "--action_safety",
    dest="action_safety",
    action="store_true",
    default=True,
    help="启用动作安全层（位置限幅 + 每步增量限幅 + NaN 守卫），默认开启（平台给 VLA 的地基之一）",
)
parser.add_argument(
    "--no_action_safety", dest="action_safety", action="store_false", help="关闭动作安全层（对比实验用）"
)
parser.add_argument("--action_safety_margin_deg", type=float, default=2.0, help="位置限幅边距（度），默认 2")
parser.add_argument(
    "--enable_task_success",
    action="store_true",
    help="（可选，默认关）挂上任务自带的**参考**成功判据，仅供队伍自测；比赛判定由裁判/队伍决定",
)
parser.add_argument(
    "--progress_every",
    type=int,
    default=30,
    help="每 N 步打印一行评测进度（success 终止项 + 任务自带指标，如货物到目标的距离）；0=关闭",
)
parser.add_argument(
    "--action_safety_max_delta", type=float, default=0.1, help="每步最大动作变化（弧度），默认 0.1 ≈ 3rad/s@30Hz"
)


# IsaacLab 3.0 起 --headless / --enable_cameras 不再是 CLI 参数（改成 AppLauncher 的配置键），
# 且这两个字段名不能再被 argparse 占用（AppLauncher 的冲突检查按 argparse 的 dest 判定）。
# 这里用别名 dest 保留原来的命令行写法，解析后再写回 AppLauncher 认识的键。
parser.add_argument(
    "--enable_cameras", dest="enable_cameras_flag", action="store_true", default=False, help="Enable camera sensors."
)
parser.add_argument(
    "--headless", dest="headless_flag", action="store_true", default=False, help="Run without a window."
)

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
args_cli.enable_cameras = bool(args_cli.enable_cameras_flag)
args_cli.headless = True if args_cli.headless_flag else None  # None = 交给 AppLauncher 按 HEADLESS 环境变量/默认值决定

app_launcher_args = vars(args_cli)

# launch omniverse app
app_launcher = AppLauncher(app_launcher_args)
simulation_app = app_launcher.app

import time

import carb
import gymnasium as gym
import omni
import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg
from leisaac.utils.env_utils import (
    dynamic_reset_gripper_effort_limit_sim,
    get_task_type,
)

import leisaac  # noqa: F401
from leisaac.utils.eval_metrics import format_progress  # noqa: E402


class RateLimiter:
    """Convenience class for enforcing rates in loops."""

    def __init__(self, hz):
        """
        Args:
            hz (int): frequency to enforce
        """
        self.hz = hz
        self.last_time = time.time()
        self.sleep_duration = 1.0 / hz
        self.render_period = min(0.0166, self.sleep_duration)

    def sleep(self, env):
        """Attempt to sleep at the specified rate in hz."""
        next_wakeup_time = self.last_time + self.sleep_duration
        while time.time() < next_wakeup_time:
            time.sleep(self.render_period)
            env.sim.render()

        self.last_time = self.last_time + self.sleep_duration

        # detect time jumping forwards (e.g. loop is too slow)
        if self.last_time < time.time():
            while self.last_time < time.time():
                self.last_time += self.sleep_duration


class Controller:
    def __init__(self):
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            self._on_keyboard_event,
        )
        self.reset_state = False

    def __del__(self):
        """Release the keyboard interface."""
        if hasattr(self, "_input") and hasattr(self, "_keyboard") and hasattr(self, "_keyboard_sub"):
            self._input.unsubscribe_from_keyboard_events(self._keyboard, self._keyboard_sub)
            self._keyboard_sub = None

    def reset(self):
        self.reset_state = False

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Handle keyboard events using carb."""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == "R":
                self.reset_state = True
        return True


def preprocess_obs_dict(obs_dict: dict, model_type: str, language_instruction: str):
    """Preprocess the observation dictionary to the format expected by the policy."""
    if model_type in ["gr00tn1.5", "gr00tn1.6", "lerobot", "openpi", "local-act"]:
        obs_dict["task_description"] = language_instruction
        return obs_dict
    else:
        raise ValueError(f"Model type {model_type} not supported")


def main():
    """Running lerobot teleoperation with leisaac manipulation environment."""

    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    task_type = get_task_type(args_cli.task)
    env_cfg.use_teleop_device(task_type)
    env_cfg.seed = args_cli.seed if args_cli.seed is not None else int(time.time())
    env_cfg.episode_length_s = args_cli.episode_length_s

    # modify configuration
    if args_cli.eval_rounds <= 0:
        if hasattr(env_cfg.terminations, "time_out"):
            env_cfg.terminations.time_out = None
    max_episode_count = args_cli.eval_rounds
    env_cfg.recorders = None
    # ★ "是否成功"由队伍/裁判决定，**平台不预设判据**（原项目也是这个设计：遥操脚本会把终止项全部
    #   删掉，成功与否靠人按 N）。所以这里默认**不挂** success 终止项；只有显式加
    #   `--enable_task_success` 时，才使用任务自己提供的参考判据（例如厨房任务的"橘子进盘子"），
    #   这仅用于队伍自测/消融，不代表比赛判定。
    if args_cli.enable_task_success:
        _enable_success = getattr(env_cfg, "enable_success", None)
        if callable(_enable_success):
            _enable_success()
            print("[Evaluation] 已按 --enable_task_success 挂上任务自带的**参考**判据（非比赛判定）")
        else:
            print("[Evaluation] 该任务没有提供 enable_success()，忽略 --enable_task_success")

    # create environment
    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    # create policy
    model_type = args_cli.policy_type
    if args_cli.policy_type == "gr00tn1.5":
        from isaaclab.sensors import Camera
        from leisaac.policy import Gr00tServicePolicyClient

        if task_type == "so101leader":
            modality_keys = ["single_arm", "gripper"]
        else:
            raise ValueError(f"Task type {task_type} not supported when using GR00T N1.5 policy yet.")

        policy = Gr00tServicePolicyClient(
            host=args_cli.policy_host,
            port=args_cli.policy_port,
            timeout_ms=args_cli.policy_timeout_ms,
            camera_keys=[key for key, sensor in env.scene.sensors.items() if isinstance(sensor, Camera)],
            modality_keys=modality_keys,
        )
    elif args_cli.policy_type == "gr00tn1.6":
        from isaaclab.sensors import Camera
        from leisaac.policy import Gr00t16ServicePolicyClient

        if task_type == "so101leader":
            modality_keys = ["single_arm", "gripper"]
        else:
            raise ValueError(f"Task type {task_type} not supported when using GR00T N1.5 policy yet.")

        policy = Gr00t16ServicePolicyClient(
            host=args_cli.policy_host,
            port=args_cli.policy_port,
            timeout_ms=args_cli.policy_timeout_ms,
            camera_keys=[key for key, sensor in env.scene.sensors.items() if isinstance(sensor, Camera)],
            modality_keys=modality_keys,
        )

    elif args_cli.policy_type == "local-act":
        # 直接 TCP 连接本地 ACT action server（不经过 gRPC）
        model_type = "local-act"
        import base64
        import io
        import json
        import socket

        import numpy as np
        from isaaclab.sensors import Camera
        from PIL import Image

        camera_keys = [key for key, sensor in env.scene.sensors.items() if isinstance(sensor, Camera)]
        policy_host = args_cli.policy_host
        policy_port = args_cli.policy_port

        class TCPActPolicy:
            def __init__(self):
                self.sock = socket.socket()
                self.sock.settimeout(30)
                self.sock.connect((policy_host, policy_port))
                self._rfile = self.sock.makefile("rb")

            def get_action(self, obs_dict: dict) -> torch.Tensor:
                # Encode images to base64
                images_b64 = {}
                for key in camera_keys:
                    img = obs_dict[key]  # shape: (H, W, 3), uint8 numpy
                    if isinstance(img, torch.Tensor):
                        img = img.cpu().numpy()
                    if img.ndim == 4:
                        img = img[0]
                    pil_img = Image.fromarray(img.astype(np.uint8))
                    buf = io.BytesIO()
                    pil_img.save(buf, format="PNG")
                    images_b64[key] = base64.b64encode(buf.getvalue()).decode()

                # State: joint positions — convert radians → motor degrees (matching training data)
                from leisaac.utils.robot_utils import convert_leisaac_action_to_lerobot
                left = obs_dict.get("left_joint_pos", torch.zeros(1, 6))
                right = obs_dict.get("right_joint_pos", torch.zeros(1, 6))
                if isinstance(left, torch.Tensor):
                    left = left.cpu().numpy()
                if isinstance(right, torch.Tensor):
                    right = right.cpu().numpy()
                left = left.reshape(1, -1)[:, :6]
                right = right.reshape(1, -1)[:, :6]
                state = np.concatenate([
                    convert_leisaac_action_to_lerobot(left).reshape(-1),
                    convert_leisaac_action_to_lerobot(right).reshape(-1),
                ]).tolist()

                request = json.dumps({"images": images_b64, "state": state}) + "\n"
                self.sock.sendall(request.encode())

                response_line = self._rfile.readline()
                result = json.loads(response_line.decode().strip())
                if result.get("error"):
                    raise RuntimeError(f"Policy server error: {result['error']}")
                actions = np.array(result["actions"])  # (chunk_size, 12)
                print(f"[TCP] Action range: [{actions.min():.3f}, {actions.max():.3f}], "
                      f"first: {[round(a,3) for a in actions[0][:6]]}")
                # Convert LeRobot motor values → IsaacLab radians
                from leisaac.utils.robot_utils import convert_lerobot_action_to_leisaac
                actions = convert_lerobot_action_to_leisaac(actions)
                print(f"[TCP] After convert: range [{actions.min():.3f}, {actions.max():.3f}], "
                      f"first: {[round(a,3) for a in actions[0][:6]]}")
                return torch.from_numpy(actions[:, None, :])

            def close(self):
                self.sock.close()

        policy = TCPActPolicy()

    elif "lerobot" in args_cli.policy_type:
        from isaaclab.sensors import Camera
        from leisaac.policy import LeRobotServicePolicyClient

        model_type = "lerobot"

        policy_type = args_cli.policy_type.split("-")[1]
        policy = LeRobotServicePolicyClient(
            host=args_cli.policy_host,
            port=args_cli.policy_port,
            timeout_ms=args_cli.policy_timeout_ms,
            camera_infos={
                key: sensor.image_shape for key, sensor in env.scene.sensors.items() if isinstance(sensor, Camera)
            },
            task_type=task_type,
            policy_type=policy_type,
            pretrained_name_or_path=args_cli.policy_checkpoint_path,
            actions_per_chunk=args_cli.policy_action_horizon,
            device=args_cli.policy_device,
        )
    elif args_cli.policy_type == "openpi":
        from isaaclab.sensors import Camera
        from leisaac.policy import OpenPIServicePolicyClient

        policy = OpenPIServicePolicyClient(
            host=args_cli.policy_host,
            port=args_cli.policy_port,
            camera_keys=[key for key, sensor in env.scene.sensors.items() if isinstance(sensor, Camera)],
            task_type=task_type,
        )

    rate_limiter = RateLimiter(args_cli.step_hz)
    controller = Controller()

    # ★ 动作安全层：把模型输出裁到仿真真实关节行程内 + 限制每步增量 + 拦 NaN。
    #   遥操路径不需要（主手行程本来就映射到从手限位区间），但策略/VLA 的输出是无约束的。
    action_limiter = None
    if getattr(args_cli, "action_safety", True):
        from leisaac.utils.action_safety import ActionSafetyLimiter, install_declarative_clip

        action_limiter = ActionSafetyLimiter(
            env,
            margin_deg=args_cli.action_safety_margin_deg,
            max_delta=args_cli.action_safety_max_delta,
        )
        # 声明式兜底：即使有人绕过包装器直接 env.step()，也越不出（内缩后的）关节限位
        install_declarative_clip(env, margin_deg=args_cli.action_safety_margin_deg)

    # reset environment
    obs_dict, _ = env.reset()
    controller.reset()
    if action_limiter is not None:
        action_limiter.reset()

    # record the results
    success_count, episode_count = 0, 1

    # simulate environment
    while max_episode_count <= 0 or episode_count <= max_episode_count:
        print(f"[Evaluation] Evaluating episode {episode_count}...")
        success, time_out = False, False
        step_in_episode = 0
        while simulation_app.is_running():
            # run everything in inference mode
            with torch.inference_mode():
                if controller.reset_state:
                    controller.reset()
                    obs_dict, _ = env.reset()
                    if action_limiter is not None:
                        action_limiter.reset()
                    episode_count += 1
                    break

                obs_dict = preprocess_obs_dict(obs_dict["policy"], model_type, args_cli.policy_language_instruction)
                actions = policy.get_action(obs_dict).to(env.device)
                for i in range(min(args_cli.policy_action_horizon, actions.shape[0])):
                    action = actions[i, :, :]
                    step_in_episode += 1
                    if action_limiter is not None:
                        action = action_limiter.filter(action)
                    if env.cfg.dynamic_reset_gripper_effort_limit:
                        dynamic_reset_gripper_effort_limit_sim(env, task_type)
                    obs_dict, _, reset_terminated, reset_time_outs, _ = env.step(action)
                    if args_cli.progress_every and step_in_episode % args_cli.progress_every == 0:
                        print(f"[progress] ep{episode_count} " + format_progress(env, step_in_episode), flush=True)
                    if reset_terminated[0]:
                        success = True
                        break
                    if reset_time_outs[0]:
                        time_out = True
                        break
                    if rate_limiter:
                        rate_limiter.sleep(env)
            if success:
                print(f"[Evaluation] Episode {episode_count} is successful! ({format_progress(env)})")
                episode_count += 1
                success_count += 1
                break
            if time_out:
                print(f"[Evaluation] Episode {episode_count} timed out! ({format_progress(env)})")
                episode_count += 1
                break
        print(
            f"[Evaluation] now success rate: {success_count / (episode_count - 1)} "
            f" [{success_count}/{episode_count - 1}]"
        )
    print(
        f"[Evaluation] Final success rate: {success_count / max_episode_count:.3f} "
        f" [{success_count}/{max_episode_count}]"
    )
    if action_limiter is not None:
        print(f"[Evaluation] 动作安全层累计拦截统计：{action_limiter.stats}")

    # close the simulator
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    # run the main function
    main()
