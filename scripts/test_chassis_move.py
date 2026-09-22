#!/usr/bin/env python3
"""测试脚本：键盘控制底盘在厨房场景中移动，验证机械臂跟随。

用法:
    export LD_LIBRARY_PATH=<conda-env>/lib:$LD_LIBRARY_PATH   # 见 HANDOFF.md §2
    python scripts/test_chassis_move.py --task LeIsaac-SmartFactory-v0

键盘控制:
    W/S: 前进/后退 (vx)
    A/D: 左/右平移 (vy)
    Q/E: 逆时针/顺时针旋转 (wz)
    R:   重置环境
    Esc: 退出
"""

import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse
import time

import numpy as np
from isaaclab.app import AppLauncher

# ── argparse（必须在 AppLauncher 之后、SimulationApp 之前）──
parser = argparse.ArgumentParser(description="Chassis movement test with keyboard control.")
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0", help="Name of the task.")
parser.add_argument("--step_hz", type=int, default=60, help="Environment stepping rate in Hz.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# ── 启动 Isaac Sim ──
# 注意：所有 omni/carb/isaaclab.* 导入必须在 SimulationApp 实例化之后
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

# ── Isaac Sim / Isaac Lab 模块（SimulationApp 之后才能 import）──
import carb
import gymnasium as gym
import omni
import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg
from leisaac.utils.env_utils import get_task_type
from leisaac.utils.sim_bridge_server import SimBridgeServer
from pxr import Gf, UsdGeom

import leisaac  # noqa: F401


class KeyboardChassisController:
    """读取键盘输入，产生底盘速度指令."""

    def __init__(self):
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_key)
        self._keys_pressed = set()
        self.reset_requested = False
        # 速度档位
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 1.0  # rad/s
        # 倍率键
        self._speed_multiplier = 1.0

    def _on_key(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._keys_pressed.add(event.input.name)
            if event.input.name == "R":
                self.reset_requested = True
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._keys_pressed.discard(event.input.name)

    def get_cmd_vel(self):
        """返回 (vx, vy, wz) 速度指令."""
        if "LEFT_SHIFT" in self._keys_pressed or "RIGHT_SHIFT" in self._keys_pressed:
            self._speed_multiplier = 2.0
        else:
            self._speed_multiplier = 1.0

        vx, vy, wz = 0.0, 0.0, 0.0
        lin = self.linear_speed * self._speed_multiplier
        ang = self.angular_speed * self._speed_multiplier

        if "W" in self._keys_pressed:
            vx = lin
        if "S" in self._keys_pressed:
            vx = -lin
        if "A" in self._keys_pressed:
            vy = lin
        if "D" in self._keys_pressed:
            vy = -lin
        if "Q" in self._keys_pressed:
            wz = ang
        if "E" in self._keys_pressed:
            wz = -ang

        return vx, vy, wz

    def __del__(self):
        if hasattr(self, "_sub") and self._sub:
            self._input.unsubscribe_from_keyboard_events(self._keyboard, self._sub)
            self._sub = None


def quaternion_to_yaw(q_wxyz):
    """从 (w, x, y, z) 四元数提取 yaw 角."""
    w, x, y, z = q_wxyz
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw):
    """将 yaw 角转换为 (w, x, y, z) 四元数（绕 Z 轴旋转）."""
    return np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])


class ChassisTracker:
    """跟踪底盘位姿（x, y, yaw），移动 Body prim + 同步机械臂."""

    def __init__(self, env):
        self._env = env
        cfg = env.cfg
        # 初始位姿从 config 读取
        self.pos = np.array(cfg.scene.body.init_state.pos, dtype=np.float64)
        init_rot = np.array(cfg.scene.body.init_state.rot)  # quaternion wxyz
        self._init_yaw = quaternion_to_yaw(init_rot)  # body 初始朝向
        self.yaw = 0.0  # 相对于初始朝向的 yaw 增量
        # 机械臂局部偏移（body 初始朝向下的偏移）
        self._left_offset = cfg._left_arm_offset.copy()
        self._right_offset = cfg._right_arm_offset.copy()
        # 获取 Body USD prim
        self._stage = omni.usd.get_context().get_stage()
        self._body_prim = self._stage.GetPrimAtPath("/World/envs/env_0/Body")
        # 获取机械臂 articulation 对象
        self._left_arm = env.scene["left_arm"]
        self._right_arm = env.scene["right_arm"]
        # 臂初始位姿（世界坐标）
        self._left_init_pos = np.array(cfg.scene.left_arm.init_state.pos, dtype=np.float64)
        self._right_init_pos = np.array(cfg.scene.right_arm.init_state.pos, dtype=np.float64)
        self._body_init_pos = self.pos.copy()

    def step(self, vx, vy, wz, dt):
        """根据局部速度 (vx, vy, wz) 更新底盘和机械臂位姿."""
        total_yaw = self._init_yaw + self.yaw
        cos_y, sin_y = np.cos(total_yaw), np.sin(total_yaw)
        # 局部速度 → 世界速度
        vx_w = vx * cos_y - vy * sin_y
        vy_w = vx * sin_y + vy * cos_y
        # 积分
        self.pos[0] += vx_w * dt
        self.pos[1] += vy_w * dt
        self.yaw += wz * dt
        new_total_yaw = self._init_yaw + self.yaw

        # ── 更新 Body prim (USD API: 直接操作 translate/rotate 属性) ──
        xform = UsdGeom.Xformable(self._body_prim)
        # 清除旧 op → 重建，避免顺序混乱
        xform.ClearXformOpOrder()
        translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat, "xformOp:translate")
        translate_op.Set(Gf.Vec3d(float(self.pos[0]), float(self.pos[1]), float(self.pos[2])))
        # 旋转：body 原始旋转 + yaw 增量绕 Z
        # body 原始四元数分解为 Z 旋转 + 可能的其他分量
        body_init_rot = np.array(self._env.cfg.scene.body.init_state.rot)
        body_init_yaw = quaternion_to_yaw(body_init_rot)
        # 用 body 初始朝向 × yaw 增量
        body_q = yaw_to_quaternion(body_init_yaw + self.yaw)
        rotate_op = xform.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat, "xformOp:rotateXYZ")
        rotate_op.Set(Gf.Vec3f(0.0, 0.0, np.degrees(body_init_yaw + self.yaw)))

        # ── 更新机械臂 root pose ──
        # 臂初始偏移（世界坐标系），随 yaw 增量旋转
        delta_left_pos = self._left_init_pos - self._body_init_pos
        delta_right_pos = self._right_init_pos - self._body_init_pos
        cos_d, sin_d = np.cos(self.yaw), np.sin(self.yaw)
        left_world = np.array([
            cos_d * delta_left_pos[0] - sin_d * delta_left_pos[1],
            sin_d * delta_left_pos[0] + cos_d * delta_left_pos[1],
            delta_left_pos[2],
        ])
        right_world = np.array([
            cos_d * delta_right_pos[0] - sin_d * delta_right_pos[1],
            sin_d * delta_right_pos[0] + cos_d * delta_right_pos[1],
            delta_right_pos[2],
        ])
        dev = self._env.device
        # 臂朝向：臂在躯干上朝后安装（+X 是躯干前方，臂朝 -X），加 180° 偏移
        arm_q = yaw_to_quaternion(self.yaw + np.pi)
        self._left_arm.write_root_pose_to_sim(
            torch.tensor([[* (self.pos + left_world), *arm_q]], device=dev, dtype=torch.float32)
        )
        self._right_arm.write_root_pose_to_sim(
            torch.tensor([[* (self.pos + right_world), *arm_q]], device=dev, dtype=torch.float32)
        )
        return self.pos.copy(), new_total_yaw


def _send_lidar_scan(env, bridge):
    """读取 RayCaster LiDAR 数据并通过 bridge 发送."""
    try:
        lidar = env.scene["lidar"]
        # 传感器世界位姿
        sensor_pos = lidar.data.pos_w[0]  # (3,)
        ray_hits = lidar.data.ray_hits_w[0]  # (N_rays, 3)
        # 计算距离
        diffs = ray_hits - sensor_pos.unsqueeze(0)
        ranges = torch.norm(diffs, dim=-1)
        # 未命中 → max_distance
        ranges = torch.nan_to_num(ranges, nan=8.0, posinf=8.0)
        ranges = torch.clamp(ranges, min=0.15, max=8.0)
        # 构造 scan 消息
        n = len(ranges)
        scan_data = {
            "ranges": ranges.cpu().tolist(),
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 2.0 * 3.14159 / n if n > 0 else 0.01745,
            "range_min": 0.15,
            "range_max": 8.0,
        }
        bridge.send_scan(scan_data)
    except Exception:
        pass  # LiDAR 未配置或未就绪时静默跳过


def print_status(pos, yaw, vx, vy, wz, speed_mult):
    """打印底盘状态."""
    print(
        f"\r[Chassis] pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) "
        f"yaw={np.degrees(yaw):.1f}° "
        f"cmd=({vx:+.2f}, {vy:+.2f}, {wz:+.2f}) "
        f"×{speed_mult:.0f}  ",
        end="",
    )


class RateLimiter:
    """限速器，保持固定步频."""

    def __init__(self, hz):
        self.hz = hz
        self.last_time = time.time()
        self.sleep_duration = 1.0 / hz
        self.render_period = min(0.0166, self.sleep_duration)

    def sleep(self, env):
        next_wakeup = self.last_time + self.sleep_duration
        while time.time() < next_wakeup:
            time.sleep(self.render_period)
            env.sim.render()
        self.last_time = self.last_time + self.sleep_duration
        if self.last_time < time.time():
            while self.last_time < time.time():
                self.last_time += self.sleep_duration


def main():
    """主循环."""
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
    task_type = get_task_type(args_cli.task)
    env_cfg.use_teleop_device(task_type)
    env_cfg.recorders = None

    # 创建环境
    env: ManagerBasedRLEnv = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    # 底盘跟踪器（在 env 创建之后，因为需要访问 USD stage 和 articulation 对象）
    chassis = ChassisTracker(env)

    # ── TCP 桥接服务（供 ROS2 客户端连接）──
    bridge = SimBridgeServer(host="127.0.0.1", port=5560)
    bridge.start()

    # 键盘控制器
    controller = KeyboardChassisController()
    rate_limiter = RateLimiter(args_cli.step_hz)

    print("\n" + "=" * 60)
    print("  底盘移动测试 — 键盘 + ROS2 桥接")
    print("  W/S: 前进/后退  A/D: 左/右平移")
    print("  Q/E: 左/右旋转  Shift: 加速")
    print("  R: 重置场景  Esc: 退出")
    print(f"  Bridge: tcp://127.0.0.1:5560 (ros2 bridge 可连接)")
    print("=" * 60 + "\n")

    # 重置
    obs_dict, _ = env.reset()
    dt = 1.0 / args_cli.step_hz

    step_count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            if controller.reset_requested:
                controller.reset_requested = False
                obs_dict, _ = env.reset()
                # 重置底盘位姿
                chassis.pos = chassis._body_init_pos.copy()
                chassis.yaw = 0.0
                print("\n[Reset] Environment reset.")
                continue

            # 键盘优先；无键盘输入时用 ROS2 bridge 的 cmd_vel
            vx, vy, wz = controller.get_cmd_vel()
            if abs(vx) < 1e-4 and abs(vy) < 1e-4 and abs(wz) < 1e-4:
                vx, vy, wz = bridge.get_cmd_vel()

            # 底盘移动
            if abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4:
                pos, yaw = chassis.step(vx, vy, wz, dt)
                if step_count % 30 == 0:
                    print_status(pos, yaw, vx, vy, wz, controller._speed_multiplier)
            elif step_count % 300 == 0:
                print(f"\n[Chassis] Idle at pos=({chassis.pos[0]:.3f}, {chassis.pos[1]:.3f}, {chassis.pos[2]:.3f})")

            # 定期发送位姿 + LiDAR 给 ROS2（即使静止也发送，SLAM 需要）
            if step_count % 3 == 0:
                total_yaw = chassis._init_yaw + chassis.yaw
                bridge.send_pose(float(chassis.pos[0]), float(chassis.pos[1]), float(total_yaw))
                _send_lidar_scan(env, bridge)

            step_count += 1
            rate_limiter.sleep(env)

    bridge.stop()
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
