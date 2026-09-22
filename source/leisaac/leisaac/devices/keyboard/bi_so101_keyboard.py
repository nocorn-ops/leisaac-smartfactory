"""双臂 SO101 键盘遥操 —— 给**双臂任务**（智慧工厂场地 / 厨房）用的键盘设备。

为什么需要新写一个，不能直接用 `SO101Keyboard`：

    单臂的 `SO101Keyboard` 里写死了 `asset_name = "robot"`，那是**单臂任务**模板
    （`single_arm_env_cfg.py` 的 `robot: ArticulationCfg`）里的实体名。
    双臂场景（`BiArmTaskSceneCfg`）里只有 `left_arm` / `right_arm`，**没有 `robot`**
    → 实测会抛 `KeyError: "Scene entity with key 'robot' not found"`。

和单臂键盘的关系：

    · 键位、每键步长、末端笛卡尔增量控制方式**完全一致**（照搬，手感一样）
    · 区别只在：实体换成 left_arm/right_arm；按键只作用在**当前激活的那条臂**上；
      `T` 在两臂之间切换；动作空间从 8 维变成 **16 维 = [左臂 8, 右臂 8]**

动作空间（每臂 8 维，与单臂键盘的同名结构一致）::

    [dx, dy, dz, droll, dpitch, dyaw, d_shoulder_pan, d_gripper]
      └──── 末端位姿增量（gripper 自身系）────┘  └─ 关节增量 ─┘

⚠️ 与真主手（`bi-so101leader`）的 **12 维关节角**动作空间**不同**（16 vs 12）：
    `hdf5_to_lerobot_v3.py` 会自动识别（16 维 → `dim_i` 命名 + 原样存，不做数值转换），
    所以键盘数据**能正常转成 LeRobot 数据集并训练**；但**一份数据集里不要混两种设备**
    （动作语义不同）。详见操作指南 §5 / §7。

按键（`T` 是本设备新增的）::

    ============================== ================= =================
    Description                    Key               Key
    ============================== ================= =================
    Forward / Backward              W                 S
    Left / Right (肩部 pan)         A                 D
    Up / Down                       Q                 E
    Rotate (Yaw) Left / Right       J                 L
    Rotate (Pitch) Up / Down        K                 I
    Gripper Open / Close            U                 O
    **切换左/右臂**                  T
    ============================== ================= =================
"""

import carb
import numpy as np

from ..device_base import Device

#: 每臂 8 维：末端位姿增量 6 + shoulder_pan 1 + 夹爪 1
_ARM_DIM = 8
#: 双臂合计 16 维，顺序 = [左臂 8, 右臂 8]（与 BiArmActionsCfg 的 4 个 term 顺序一致）
TOTAL_DIM = 2 * _ARM_DIM


class BiSO101Keyboard(Device):
    """双臂键盘遥操：`T` 切换左右臂，其余键位与单臂键盘一致。"""

    #: 两条臂对应的场景实体名（`device_type` 用下划线，与 BiSO101Leader 的命名约定一致）
    ARM_ENTITIES = {"left": "left_arm", "right": "right_arm"}

    def __init__(self, env, sensitivity: float = 1.0, start_side: str = "left"):
        super().__init__(env, "bi_so101_keyboard")

        if start_side not in self.ARM_ENTITIES:
            raise ValueError(f"start_side 只能是 'left' / 'right'，收到 {start_side!r}")

        self.pos_sensitivity = 0.01 * sensitivity
        self.joint_sensitivity = 0.15 * sensitivity
        self.rot_sensitivity = 0.15 * sensitivity

        self._create_key_bindings()

        # 每条臂一个增量缓冲；**只累加到当前激活的臂**，另一条保持全 0（= 保持当前位姿）
        self._delta = {"left": np.zeros(_ARM_DIM), "right": np.zeros(_ARM_DIM)}
        #: key → 它被按下时作用在哪条臂。**必须记**：否则"按住 W → 按 T 切臂 → 松开 W"
        #: 会把减量算到新臂上，把那条臂拽走。
        self._pressed: dict[str, str] = {}

        # 每条臂各自的 articulation 与末端 body 下标（`_convert_delta_from_frame` 要按臂传）
        self._arms: dict[str, tuple[object, int]] = {}
        for side, entity in self.ARM_ENTITIES.items():
            try:
                asset = self.env.scene[entity]
            except KeyError as exc:
                raise KeyError(
                    f"场景里找不到 '{entity}' —— 本设备只能用在**双臂**任务上"
                    f"（单臂任务请用 --teleop_device keyboard）。可用实体: "
                    f"{list(self.env.scene.keys())}"
                ) from exc
            body_idxs, _ = asset.find_bodies("gripper")
            self._arms[side] = (asset, body_idxs[0])

        self._active_side = start_side
        print(
            f"[bi-keyboard] 双臂键盘遥操已就绪：当前控制 **{self._active_side}** 臂"
            f"（按 T 切换；按 B 开始，R 重置本集，N 标记成功）"
        )

    # ── 控制说明表 ────────────────────────────────────────────────────────────
    def _add_device_control_description(self):
        """注意：本方法是在 `Device.__init__` 里被调用的（早于子类初始化），
        所以只能加**静态**行，不能读 self._active_side 之类还没设好的属性。"""
        self._display_controls_table.add_row(["W", "forward"])
        self._display_controls_table.add_row(["S", "backward"])
        self._display_controls_table.add_row(["A", "left (shoulder_pan)"])
        self._display_controls_table.add_row(["D", "right (shoulder_pan)"])
        self._display_controls_table.add_row(["Q", "up"])
        self._display_controls_table.add_row(["E", "down"])
        self._display_controls_table.add_row(["J", "rotate_left"])
        self._display_controls_table.add_row(["L", "rotate_right"])
        self._display_controls_table.add_row(["K", "rotate_up"])
        self._display_controls_table.add_row(["I", "rotate_down"])
        self._display_controls_table.add_row(["U", "gripper_open"])
        self._display_controls_table.add_row(["O", "gripper_close"])
        self._display_controls_table.add_row(["==========", "=========="])
        self._display_controls_table.add_row(["T", "switch LEFT / RIGHT arm"])

    # ── 状态读出 ──────────────────────────────────────────────────────────────
    @property
    def active_side(self) -> str:
        return self._active_side

    def get_device_state(self):
        """返回 16 维增量 [左臂 8, 右臂 8]（各臂已从 gripper 系转到该臂基座系）。"""
        left_asset, left_idx = self._arms["left"]
        right_asset, right_idx = self._arms["right"]
        left = self._convert_delta_from_frame(self._delta["left"], left_asset, left_idx)
        right = self._convert_delta_from_frame(self._delta["right"], right_asset, right_idx)
        return np.concatenate([left, right])

    def reset(self):
        self._delta["left"][:] = 0.0
        self._delta["right"][:] = 0.0
        self._pressed.clear()

    # ── 按键 ──────────────────────────────────────────────────────────────────
    def _toggle_side(self):
        self._active_side = "right" if self._active_side == "left" else "left"
        # 切臂时**不清零**已有缓冲：另一条臂的残留增量靠"松键减量"自然归零
        # （`_pressed` 记了每条 key 属于哪条臂，所以不会串到新臂上）。
        print(f"[bi-keyboard] 现在控制 **{self._active_side}** 臂")

    def _on_keyboard_event(self, event, *args, **kwargs):
        # 先交给基类：它负责 B（开始）/ R / N 以及注册的附加回调
        super()._on_keyboard_event(event, *args, **kwargs)

        name = event.input.name
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if name == "T":
                self._toggle_side()
                return
            if name in self._INPUT_KEY_MAPPING:
                side = self._active_side
                self._pressed[name] = side
                self._delta[side] += self._ACTION_DELTA_MAPPING[self._INPUT_KEY_MAPPING[name]]

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # ★ 从"按下时的那条臂"上减，而不是从当前激活臂减
            side = self._pressed.pop(name, None)
            if side is not None and name in self._INPUT_KEY_MAPPING:
                self._delta[side] -= self._ACTION_DELTA_MAPPING[self._INPUT_KEY_MAPPING[name]]

    def _create_key_bindings(self):
        """与单臂 `SO101Keyboard` **逐行一致**，手感保持不变。"""
        self._ACTION_DELTA_MAPPING = {
            "forward": np.asarray([0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]) * self.pos_sensitivity,
            "backward": np.asarray([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]) * self.pos_sensitivity,
            "left": np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0]) * self.joint_sensitivity,
            "right": np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]) * self.joint_sensitivity,
            "up": np.asarray([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) * self.pos_sensitivity,
            "down": np.asarray([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) * self.pos_sensitivity,
            "rotate_up": np.asarray([0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0]) * self.rot_sensitivity,
            "rotate_down": np.asarray([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]) * self.rot_sensitivity,
            "rotate_left": np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]) * self.rot_sensitivity,
            "rotate_right": np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0]) * self.rot_sensitivity,
            "gripper_open": np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]) * self.joint_sensitivity,
            "gripper_close": np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]) * self.joint_sensitivity,
        }
        self._INPUT_KEY_MAPPING = {
            "W": "forward",
            "S": "backward",
            "A": "left",
            "D": "right",
            "Q": "up",
            "E": "down",
            "K": "rotate_up",
            "I": "rotate_down",
            "J": "rotate_left",
            "L": "rotate_right",
            "U": "gripper_open",
            "O": "gripper_close",
        }
