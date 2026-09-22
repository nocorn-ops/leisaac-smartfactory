# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run a leisaac teleoperation with leisaac manipulation environments.

⚠️ **这是"内部实现 / 单一功能工具"，不是对外的仿真入口。**
   对外的**唯一仿真启动入口**是 `reproduce/ros2_chassis_teleop.sh`：
       遥操/数采 → --mode teleop [--record --dataset_file ...] [--teleop_device ...]
       建图/导航 → --mode nav    验证推理 → --mode policy
   本脚本保留是为了能脱离统一入口单独调试遥操（例如只跑某个单臂任务的键盘遥操）；
   日常遥操/数采请走统一入口 —— 它还顺带起了 ROS2 桥接，方便边采边看底盘。
"""

"""Launch Isaac Sim Simulator first."""
import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse
import os
import signal
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
parser = argparse.ArgumentParser(description="leisaac teleoperation for leisaac environments.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument(
    "--teleop_device",
    type=str,
    default="keyboard",
    choices=[
        "keyboard",
        "gamepad",
        "so101leader",
        "so101leader-one",
        "bi-so101leader",
        "lekiwi-keyboard",
        "lekiwi-gamepad",
        "lekiwi-leader",
    ],
    help="Device for interacting with environment",
)
parser.add_argument(
    "--port", type=str, default="/dev/ttyACM0", help="Port for the teleop device:so101leader, default is /dev/ttyACM0"
)
parser.add_argument(
    "--remote_endpoint",
    type=str,
    default=None,
    help=(
        "ZMQ endpoint for remote so101leader (e.g. tcp://192.168.1.10:5556). Uses so101_joint_state_server.py on the"
        " remote machine."
    ),
)
parser.add_argument(
    "--left_arm_port",
    type=str,
    default="/dev/ttyACM0",
    help="Port for the left teleop device:bi-so101leader, default is /dev/ttyACM0",
)
parser.add_argument(
    "--right_arm_port",
    type=str,
    default="/dev/ttyACM1",
    help="Port for the right teleop device:bi-so101leader, default is /dev/ttyACM1",
)
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed for the environment.")
parser.add_argument("--sensitivity", type=float, default=1.0, help="Sensitivity factor.")

# recorder_parameter
parser.add_argument("--record", action="store_true", help="whether to enable record function")
parser.add_argument("--step_hz", type=int, default=60, help="Environment stepping rate in Hz.")
parser.add_argument(
    "--dataset_file", type=str, default="./datasets/dataset.hdf5", help="File path to export recorded demos."
)
parser.add_argument("--resume", action="store_true", help="whether to resume recording in the existing dataset file")
parser.add_argument(
    "--num_demos", type=int, default=0, help="Number of demonstrations to record. Set to 0 for infinite."
)

parser.add_argument("--recalibrate", action="store_true", help="recalibrate SO101-Leader or Bi-SO101Leader")
parser.add_argument(
    "--arm_side",
    type=str,
    default="left",
    choices=["left", "right"],
    help="so101leader-one 时：驱动哪一只从手（另一只保持不动），默认 left",
)
parser.add_argument(
    "--relative_one_arm",
    action="store_true",
    help="so101leader-one 用相对（增量）映射（默认是绝对位姿映射：主从姿势一一对应）",
)
parser.add_argument(
    "--engage_threshold",
    type=float,
    default=0.35,
    help="so101leader-one 绝对映射的接管门槛（弧度）：主手与从手差异大于它时不接管、只提示，默认 0.35",
)
parser.add_argument(
    "--settle_steps",
    type=int,
    default=150,
    help="开始遥控前的纯物理静置步数（让台面物体先落稳，避免按 B 瞬间集体弹跳），默认 150，设 0 关闭",
)
parser.add_argument(
    "--camera_view",
    type=str,
    default="panel",
    choices=["panel", "windows", "off"],
    help=(
        "遥操时怎么显示三路相机（left_wrist/right_wrist/front）："
        "panel=一个窗口里三张实时图（默认，复用传感器图像，开销小）；"
        "windows=每台相机各开一个 Kit 视口窗口（可拖动，但每台要多一次 RTX 渲染，实测 +29ms/步）；"
        "off=不显示（相机数据照常录进数据集）"
    ),
)
parser.add_argument(
    "--eye", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"), help="覆盖视口相机位置（默认用任务配置里的 viewer.eye）"
)
parser.add_argument(
    "--lookat",
    type=float,
    nargs=3,
    default=None,
    metavar=("X", "Y", "Z"),
    help="覆盖视口相机注视点（默认用任务配置里的 viewer.lookat）",
)
parser.add_argument("--quality", action="store_true", help="whether to enable quality render mode.")
parser.add_argument("--use_lerobot_recorder", action="store_true", help="whether to use lerobot recorder.")
parser.add_argument("--lerobot_dataset_repo_id", type=str, default=None, help="Lerobot Dataset repository ID.")
parser.add_argument("--lerobot_dataset_fps", type=int, default=30, help="Lerobot Dataset frames per second.")

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

import os
import time

import gymnasium as gym
import torch
from isaaclab.envs import DirectRLEnv, ManagerBasedRLEnv
from isaaclab.managers import DatasetExportMode, TerminationTermCfg
from isaaclab_tasks.utils import parse_env_cfg
from leisaac.enhance.managers import EnhanceDatasetExportMode, StreamingRecorderManager
from leisaac.utils.env_utils import dynamic_reset_gripper_effort_limit_sim
from leisaac.utils.camera_view import CameraImagePanel, create_camera_view_windows
from leisaac.utils.render_compat import apply_render_settings


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


def manual_terminate(env: ManagerBasedRLEnv | DirectRLEnv, success: bool):
    if hasattr(env, "termination_manager"):
        if success:
            env.termination_manager.set_term_cfg(
                "success",
                TerminationTermCfg(func=lambda env: torch.ones(env.num_envs, dtype=torch.bool, device=env.device)),
            )
        else:
            env.termination_manager.set_term_cfg(
                "success",
                TerminationTermCfg(func=lambda env: torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)),
            )
        env.termination_manager.compute()
    elif hasattr(env, "_get_dones"):
        env.cfg.return_success_status = success


def main():  # noqa: C901
    """Running lerobot teleoperation with leisaac manipulation environment."""

    # get directory path and file name (without extension) from cli arguments
    output_dir = os.path.dirname(args_cli.dataset_file)
    output_file_name = os.path.splitext(os.path.basename(args_cli.dataset_file))[0]
    # create directory if it does not exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.use_teleop_device(args_cli.teleop_device)
    env_cfg.seed = args_cli.seed if args_cli.seed is not None else int(time.time())
    task_name = args_cli.task

    # 视口相机覆盖（IsaacLab 3.0 已废弃 env_cfg.viewer，但仍会自动转发到 KitVisualizerCfg）
    if args_cli.eye is not None:
        env_cfg.viewer.eye = tuple(args_cli.eye)
        print(f"[teleop] 视口相机 eye = {env_cfg.viewer.eye}")
    if args_cli.lookat is not None:
        env_cfg.viewer.lookat = tuple(args_cli.lookat)
        print(f"[teleop] 视口相机 lookat = {env_cfg.viewer.lookat}")

    if args_cli.quality:
        # IsaacLab 3.0 已移除 sim.render，兼容层在 3.0 下为空操作（画质设置在 3.0 走按传感器的 renderer_cfg）
        apply_render_settings(env_cfg.sim, antialiasing_mode="FXAA", rendering_mode="quality")

    # precheck task and teleop device
    if "BiArm" in task_name:
        assert args_cli.teleop_device == "bi-so101leader", "only support bi-so101leader for bi-arm task"
    if "LeKiwi" in task_name:
        assert args_cli.teleop_device in [
            "lekiwi-leader",
            "lekiwi-keyboard",
            "lekiwi-gamepad",
        ], "only support lekiwi-leader, lekiwi-keyboard, lekiwi-gamepad for lekiwi task"
    is_direct_env = "Direct" in task_name
    if is_direct_env:
        assert args_cli.teleop_device in [
            "so101leader",
            "bi-so101leader",
        ], "only support so101leader or bi-so101leader for direct task"

    # timeout and terminate preprocess
    if is_direct_env:
        env_cfg.never_time_out = True
        env_cfg.manual_terminate = True
    else:
        # modify configuration
        if hasattr(env_cfg.terminations, "time_out"):
            env_cfg.terminations.time_out = None
        if hasattr(env_cfg.terminations, "success"):
            env_cfg.terminations.success = None
    # recorder preprocess & manual success terminate preprocess
    if args_cli.record:
        if args_cli.use_lerobot_recorder:
            if args_cli.resume:
                env_cfg.recorders.dataset_export_mode = EnhanceDatasetExportMode.EXPORT_SUCCEEDED_ONLY_RESUME
            else:
                env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_ONLY
        else:
            if args_cli.resume:
                env_cfg.recorders.dataset_export_mode = EnhanceDatasetExportMode.EXPORT_ALL_RESUME
                assert os.path.exists(
                    args_cli.dataset_file
                ), "the dataset file does not exist, please don't use '--resume' if you want to record a new dataset"
            else:
                env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_ALL
                assert not os.path.exists(
                    args_cli.dataset_file
                ), "the dataset file already exists, please use '--resume' to resume recording"
        env_cfg.recorders.dataset_export_dir_path = output_dir
        env_cfg.recorders.dataset_filename = output_file_name
        if is_direct_env:
            env_cfg.return_success_status = False
        else:
            if not hasattr(env_cfg.terminations, "success"):
                setattr(env_cfg.terminations, "success", None)
            env_cfg.terminations.success = TerminationTermCfg(
                func=lambda env: torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
            )
    else:
        env_cfg.recorders = None

    # create environment
    env: ManagerBasedRLEnv | DirectRLEnv = gym.make(task_name, cfg=env_cfg).unwrapped
    # replace the original recorder manager with the streaming recorder manager or lerobot recorder manager
    if args_cli.record:
        del env.recorder_manager
        if args_cli.use_lerobot_recorder:
            from leisaac.enhance.datasets.lerobot_dataset_handler import (
                LeRobotDatasetCfg,
            )
            from leisaac.enhance.managers.lerobot_recorder_manager import (
                LeRobotRecorderManager,
            )

            dataset_cfg = LeRobotDatasetCfg(
                repo_id=args_cli.lerobot_dataset_repo_id,
                fps=args_cli.lerobot_dataset_fps,
            )
            env.recorder_manager = LeRobotRecorderManager(env_cfg.recorders, dataset_cfg, env)
        else:
            env.recorder_manager = StreamingRecorderManager(env_cfg.recorders, env)
            env.recorder_manager.flush_steps = 100
            env.recorder_manager.compression = "lzf"

    # create controller
    if args_cli.teleop_device == "keyboard":
        from leisaac.devices import SO101Keyboard

        teleop_interface = SO101Keyboard(env, sensitivity=args_cli.sensitivity)
    elif args_cli.teleop_device == "gamepad":
        from leisaac.devices import SO101Gamepad

        teleop_interface = SO101Gamepad(env, sensitivity=args_cli.sensitivity)
    elif args_cli.teleop_device == "so101leader":
        if args_cli.remote_endpoint:
            from leisaac.devices import SO101LeaderRemote

            teleop_interface = SO101LeaderRemote(env, endpoint=args_cli.remote_endpoint)
        else:
            from leisaac.devices import SO101Leader

            teleop_interface = SO101Leader(env, port=args_cli.port, recalibrate=args_cli.recalibrate)
    elif args_cli.teleop_device == "bi-so101leader":
        from leisaac.devices import BiSO101Leader

        teleop_interface = BiSO101Leader(
            env, left_port=args_cli.left_arm_port, right_port=args_cli.right_arm_port, recalibrate=args_cli.recalibrate
        )
    elif args_cli.teleop_device == "bi-keyboard":
        # 双臂键盘遥操（给没有真主手的队伍）：T 切换左右臂。
        # 动作空间 16 维（每臂 8 维增量），与 bi-so101leader 的 12 维关节角不同。
        from leisaac.devices import BiSO101Keyboard

        teleop_interface = BiSO101Keyboard(
            env, sensitivity=args_cli.sensitivity, start_side=args_cli.arm_side
        )
    elif args_cli.teleop_device == "lekiwi-keyboard":
        from leisaac.devices import LeKiwiKeyboard

        teleop_interface = LeKiwiKeyboard(env, sensitivity=args_cli.sensitivity)
    elif args_cli.teleop_device == "lekiwi-leader":
        from leisaac.devices import LeKiwiLeader

        teleop_interface = LeKiwiLeader(env, port=args_cli.port, recalibrate=args_cli.recalibrate)
    elif args_cli.teleop_device == "lekiwi-gamepad":
        from leisaac.devices import LeKiwiGamepad

        teleop_interface = LeKiwiGamepad(env, sensitivity=args_cli.sensitivity)
    elif args_cli.teleop_device == "so101leader-one":
        # 单臂主手驱动双臂任务（如厨房）里的一只从手，另一只保持当前位姿
        from leisaac.devices import SO101LeaderOneArm

        teleop_interface = SO101LeaderOneArm(
            env,
            port=args_cli.port,
            side=args_cli.arm_side,
            recalibrate=args_cli.recalibrate,
            relative=args_cli.relative_one_arm,
            engage_threshold=args_cli.engage_threshold,
        )
    else:
        raise ValueError(
            f"Invalid device interface '{args_cli.teleop_device}'. Supported: 'bi-so101leader',"
            " 'so101leader-one', 'bi-keyboard'（双臂键盘）, 'keyboard'/'gamepad'（仅单臂任务）,"
            " 'so101leader', 'lekiwi-keyboard', 'lekiwi-leader', 'lekiwi-gamepad'."
        )

    # add teleoperation key for env reset
    should_reset_recording_instance = False

    def reset_recording_instance():
        nonlocal should_reset_recording_instance
        should_reset_recording_instance = True

    # add teleoperation key for task success
    should_reset_task_success = False

    def reset_task_success():
        nonlocal should_reset_task_success
        should_reset_task_success = True
        reset_recording_instance()

    teleop_interface.add_callback("R", reset_recording_instance)
    teleop_interface.add_callback("N", reset_task_success)
    teleop_interface.display_controls()
    rate_limiter = RateLimiter(args_cli.step_hz)

    # reset environment
    if hasattr(env, "initialize"):
        env.initialize()
    env.reset()
    # ★ 开遥控前先让物理空跑一会儿：按 B 之前主循环只调 env.render()，不推进物理，
    #   台面上的物体会一直停在"出生姿态"。这里先走一段纯物理步进（不走 env.step，
    #   所以不会污染录制），把它们落稳，避免按 B 的第一步才集体去穿透。
    if args_cli.settle_steps > 0:
        print(f"[INFO] 物理静置 {args_cli.settle_steps} 步，让台面上的物体先落稳...")
        for _ in range(args_cli.settle_steps):
            # render=True 是必须的：不泵 Kit/RTX 主循环的话，腕部/前视相机的 render product
            # 不会更新，传感器输出会变成全白图（实测 mean=245），于是静置后第一帧观测是白的
            # （会进数据集/影响推理首帧）。
            env.sim.step(render=True)
            env.scene.update(env.physics_dt)
        print("[INFO] 静置完成，物体已稳定（此时再按 B 就不会弹跳了）")
    # ★ 三路相机显示（遥操时同时看到两个腕部相机 + 一个头部相机）
    camera_view_update = None
    env_camera_view = None
    if args_cli.camera_view != "off" and not args_cli.headless:
        if args_cli.camera_view == "windows":
            env_camera_view = create_camera_view_windows(env)  # 保持引用，别被 GC
        else:
            env_camera_view = CameraImagePanel(env)
            camera_view_update = env_camera_view.update
    teleop_interface.reset()

    resume_recorded_demo_count = 0
    if args_cli.record and args_cli.resume:
        resume_recorded_demo_count = env.recorder_manager._dataset_file_handler.get_num_episodes()
        print(f"Resume recording from existing dataset file with {resume_recorded_demo_count} demonstrations.")
    current_recorded_demo_count = resume_recorded_demo_count

    start_record_state = False

    interrupted = False

    def signal_handler(signum, frame):
        """Handle SIGINT (Ctrl+C) signal."""
        nonlocal interrupted
        interrupted = True
        print("\n[INFO] KeyboardInterrupt (Ctrl+C) detected. Cleaning up resources...")

    original_sigint_handler = signal.signal(signal.SIGINT, signal_handler)

    try:
        while simulation_app.is_running() and not interrupted:
            # run everything in inference mode
            with torch.inference_mode():
                if camera_view_update is not None:
                    camera_view_update()  # 相机图像面板（限频刷新，见 CameraImagePanel.update_every）
                if env.cfg.dynamic_reset_gripper_effort_limit:
                    dynamic_reset_gripper_effort_limit_sim(env, args_cli.teleop_device)
                actions = teleop_interface.advance()
                if should_reset_task_success:
                    print("Task Success!!!")
                    should_reset_task_success = False
                    if args_cli.record:
                        manual_terminate(env, True)
                if should_reset_recording_instance:
                    env.reset()
                    should_reset_recording_instance = False
                    if start_record_state:
                        if args_cli.record:
                            print("Stop Recording!!!")
                        start_record_state = False
                    if args_cli.record:
                        manual_terminate(env, False)
                    # print out the current demo count if it has changed
                    if (
                        args_cli.record
                        and env.recorder_manager.exported_successful_episode_count + resume_recorded_demo_count
                        > current_recorded_demo_count
                    ):
                        current_recorded_demo_count = (
                            env.recorder_manager.exported_successful_episode_count + resume_recorded_demo_count
                        )
                        print(f"Recorded {current_recorded_demo_count} successful demonstrations.")
                    if (
                        args_cli.record
                        and args_cli.num_demos > 0
                        and env.recorder_manager.exported_successful_episode_count + resume_recorded_demo_count
                        >= args_cli.num_demos
                    ):
                        print(f"All {args_cli.num_demos} demonstrations recorded. Exiting the app.")
                        break

                elif actions is None:
                    # 设备没"启动"（按 B 前）：不推进物理，但要泵 Kit 事件循环。
                    # ⚠️ 不能用 `env.render()` —— IsaacLab 3.0 里开了相机
                    # （has_rtx_sensors=True）时它是**空操作**，视口不再重绘 → 窗口"未响应"。
                    # 见 ros2_chassis_teleop.py 同一处注释。
                    env.sim.render()
                # apply actions
                else:
                    if not start_record_state:
                        if args_cli.record:
                            print("Start Recording!!!")
                        start_record_state = True
                    env.step(actions)
                if rate_limiter:
                    rate_limiter.sleep(env)
            if interrupted:
                break
    except Exception as e:
        import traceback

        print(f"\n[ERROR] An error occurred: {e}\n")
        traceback.print_exc()
        print("[INFO] Cleaning up resources...")
    finally:
        # Restore original signal handler
        signal.signal(signal.SIGINT, original_sigint_handler)
        # finalize the recorder manager
        if args_cli.record and hasattr(env.recorder_manager, "finalize"):
            env.recorder_manager.finalize()
        # close the simulator
        env.close()
        simulation_app.close()


if __name__ == "__main__":
    # run the main function
    main()
