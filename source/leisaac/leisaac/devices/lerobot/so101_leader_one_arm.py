"""单臂 SO101 主手驱动「双臂任务」中的一只手臂，另一只手臂保持当前位姿。

用途：像厨房（`LeIsaac-LeRobot-Kitchen-v0`）这种双臂任务，只有一条主手可用时，
仍然按 12 维动作空间跑（左/右各 6），未被操控的那半边由
``preprocess_device_action`` 填成"从手当前关节位置" —— 所以录下来的数据格式与
双臂采集完全一致，只是另一只手臂全程静止。

用法（由 teleop 脚本按设备名选择）::

    --teleop_device so101leader-one --arm_side left --port /dev/ttyACM0
"""

import torch

from ..action_process import convert_action_from_so101_leader
from ..device_base import Device
from .so101_leader import SO101Leader

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class SO101LeaderOneArm(Device):
    """一条 SO101 主手 → 双臂任务中指定的一只从手。

    Args:
        env: 遥操环境。
        port: 主手串口。
        side: 被驱动的是哪只从手（``"left"`` / ``"right"``）；另一只保持当前位姿。
        recalibrate: 是否重新标定主手（需要人工把手臂摆到量程中点）。
        calibration_file_name: 标定文件名（默认与 ``side`` 对应）。
    """

    def __init__(
        self,
        env,
        port: str = "/dev/ttyACM0",
        side: str = "left",
        recalibrate: bool = False,
        calibration_file_name: str | None = None,
        ramp_steps: int = 60,
        relative: bool = False,
        engage_threshold: float = 0.35,
    ):
        if side not in ("left", "right"):
            raise ValueError(f"side 必须是 'left' 或 'right'，收到 {side!r}")
        # 必须在 super().__init__ 之前赋值：基类构造里会调用 _add_device_control_description
        self._side = side
        super().__init__(env, "so101_leader_one")
        if calibration_file_name is None:
            calibration_file_name = f"{side}_so101_leader.json"
        print(f"Connecting to so101_leader on {port} (drives the {side} follower arm)...")
        self._leader = SO101Leader(env, port, recalibrate, calibration_file_name)
        # 键盘回调（含 B/R/N 的"开始控制"门槛）只保留本设备这一层，
        # 否则内部 SO101Leader 自己也会响应按键。
        self._leader._stop_keyboard_listener()
        # 另一只手臂的锁定目标：环境已 reset 过，这里读到的就是初始位姿
        self._capture_hold_pose()
        # 软启动：按下 B 之后，用 ramp_steps 步从"从手当前位姿"平滑过渡到主手位姿。
        # 不做的话会一步跳过去（主手和从手初始位姿差多少就跳多少），速度极大，
        # 会把台面上的东西撞飞、甚至看起来像"桌子翻了"。
        self._ramp_steps = max(int(ramp_steps), 1)
        self._ramp_i = self._ramp_steps
        self._was_started = False
        # relative=True：按 B 时从手原地不动，只跟随主手的**变化量**（主从对应关系与绝对模式不同）
        self._relative = bool(relative)
        self._leader_ref = None
        self._follower_ref = None
        # 绝对模式的对齐门槛（弧度）：按 B 时若主手与从手差异超过它，先保持不动并提示对齐，
        # 避免从手一步扫过台面（橙子/盘子就在手臂工作区里，一碰就翻）。
        self._engage_threshold = float(engage_threshold)
        self._engaged = False
        self._warn_i = 0
        # 等待对齐期间，受控臂要锁定"按下 B 那一刻"的固定位姿（不是每帧读当前值，
        # 否则 PD 误差恒为 0，手臂会在重力下慢慢下垂）。
        self._engage_lock = None

    def _controlled_joint_pos(self):
        """受控那只手臂当前的关节位置（弧度，(N,6)，顺序 [5 臂关节, gripper]）。"""
        arm = self.env.scene["left_arm" if self._side == "left" else "right_arm"]
        joint_ids, _ = arm.find_joints(JOINT_NAMES)
        pos = arm.data.joint_pos
        pos = pos.torch if hasattr(pos, "torch") else pos  # IsaacLab 3.0: ProxyArray
        return pos[:, joint_ids]

    @property
    def side(self) -> str:
        """被驱动的从手（``"left"`` / ``"right"``）。"""
        return self._side

    def _capture_hold_pose(self) -> None:
        """记录"另一只手臂"的当前关节位置，作为它接下来要锁定的固定目标（弧度）。

        必须锁成**固定目标**：如果每步都命令"当前值"，PD 控制器误差恒为 0，
        那只手臂会在重力下慢慢下沉。这里在初始化与每次 reset 时抓一次。
        """
        other = self.env.scene["right_arm" if self._side == "left" else "left_arm"]
        joint_ids, _ = other.find_joints(JOINT_NAMES)
        pos = other.data.joint_pos
        pos = pos.torch if hasattr(pos, "torch") else pos  # IsaacLab 3.0: ProxyArray
        self._hold = pos[:, joint_ids].detach().clone()
        print(f"[one-arm] 另一只手臂锁定目标 = {[round(float(v), 3) for v in self._hold[0]]}")

    def _add_device_control_description(self):
        self._display_controls_table.add_row([
            f"so101-leader(one,{self._side})",
            f"move so101-leader to control the {self._side} follower arm（the other arm holds still）",
        ])

    def reset(self):
        self._leader.reset()
        self._capture_hold_pose()  # 每次 reset 重新锁定当时的位姿
        self._leader_ref = None    # 相对模式的参考点也重新对齐
        self._follower_ref = None
        self._engaged = False      # 绝对模式重新走一次对齐门槛
        self._warn_i = 0
        self._engage_lock = None
        super().reset()

    def get_device_state(self):
        return self._leader.get_device_state()

    def input2action(self):
        ac_dict = super().input2action()
        if "joint_state" not in ac_dict:
            return ac_dict  # reset 分支：这一帧不需要动作
        ac_dict["motor_limits"] = self._leader.motor_limits
        ac_dict["side"] = self._side
        ac_dict["hold"] = self._hold

        target = convert_action_from_so101_leader(ac_dict["joint_state"], self._leader.motor_limits, self)
        started_now = bool(ac_dict.get("started"))
        just_started = started_now and not self._was_started
        self._was_started = started_now

        if self._relative:
            # 相对（增量）模式：按下 B 的那一刻把"主手当前位姿"和"从手当前位姿"作为参考点，
            # 之后只把主手的**变化量**叠加到从手上。
            # 好处：无论主手此刻是什么姿势（趴着、举着、标定有偏差），按 B 都不会让从手瞬移/扫过台面。
            if just_started or self._leader_ref is None:
                self._leader_ref = target.clone()
                self._follower_ref = self._controlled_joint_pos()
                print("[one-arm] 相对模式：已在当前位姿对齐（从手不会因按 B 而移动）")
            raw = self._follower_ref + (target - self._leader_ref)
            # 限制在从手各关节行程内，避免顶到限位
            arm = self.env.scene["left_arm" if self._side == "left" else "right_arm"]
            joint_ids, _ = arm.find_joints(JOINT_NAMES)
            limits = arm.data.joint_pos_limits
            limits = limits.torch if hasattr(limits, "torch") else limits
            limits = limits[0][joint_ids]
            ac_dict["controlled"] = torch.clamp(raw.reshape(1, -1), limits[:, 0], limits[:, 1])
            return ac_dict

        if started_now and just_started:
            self._ramp_i = 0      # 绝对模式：刚按下 B 时重新开始软启动
            self._engaged = False  # 并重新走一次对齐门槛
            self._warn_i = 0
            # 门槛通过之前锁住"按下 B 那一刻"的固定位姿：如果每帧都命令当前值，
            # PD 误差恒为 0，受控臂会在重力下慢慢下垂。
            self._engage_lock = self._controlled_joint_pos().detach().clone()

        # 对齐门槛：主手与从手差异太大时先不接管，只保持从手原位并提示。
        # 橙子/盘子就在手臂工作区里，一旦从手从远处一步扫过去就会把它们撞翻，
        # 所以要求操作者先把主手摆到与从手相近的姿势（差异 < engage_threshold）。
        if started_now and not self._engaged:
            current = self._controlled_joint_pos()
            diff = (target - current).abs()
            worst = float(diff.max())
            if worst <= self._engage_threshold:
                self._engaged = True
                self._engage_lock = None  # 已接管，释放锁定
                print(f"[one-arm] 已接管（主手与从手最大差异 {worst:.2f} rad），现在主从姿势一一对应")
            else:
                if self._warn_i % 60 == 0:
                    order = torch.argsort(diff[0], descending=True)
                    hints = ", ".join(
                        f"{JOINT_NAMES[int(j)]} {float(diff[0, j]) * 180 / torch.pi:.0f}°" for j in order[:2]
                    )
                    print(
                        f"[one-arm] 还未接管：请把主手摆成与仿真里那只手臂相近的姿势"
                        f"（差最多的是 {hints}，门槛 {self._engage_threshold * 180 / torch.pi:.0f}°），"
                        "从手此刻保持不动"
                    )
                self._warn_i += 1
                if self._engage_lock is not None:
                    ac_dict["controlled"] = self._engage_lock
                else:
                    ac_dict["controlled"] = current
                return ac_dict

        if started_now and self._ramp_i < self._ramp_steps:
            self._ramp_i += 1
            alpha = self._ramp_i / self._ramp_steps
            current = self._controlled_joint_pos()
            ac_dict["controlled"] = current + alpha * (target - current)
            if self._ramp_i == 1 or self._ramp_i % 20 == 0 or self._ramp_i == self._ramp_steps:
                print(f"[one-arm] 软启动 {self._ramp_i}/{self._ramp_steps}")
        else:
            ac_dict["controlled"] = target
        return ac_dict
