# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Base class for teleoperation interface."""

import weakref
from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import Any

import carb
import isaaclab.utils.math as math_utils
import numpy as np
import omni
import torch
from leisaac.utils.math_utils import rotvec_to_euler
from prettytable import PrettyTable


class DeviceBase(ABC):
    """An interface class for teleoperation devices."""

    def __init__(self):
        """Initialize the teleoperation interface."""
        pass

    def __str__(self) -> str:
        """Returns: A string containing the information of joystick."""
        return f"{self.__class__.__name__}"

    """
    Operations
    """

    @abstractmethod
    def reset(self):
        """Reset the internals."""
        raise NotImplementedError

    @abstractmethod
    def add_callback(self, key: Any, func: Callable):
        """Add additional functions to bind keyboard.

        Args:
            key: The button to check against.
            func: The function to call when key is pressed. The callback function should not
                take any arguments.
        """
        raise NotImplementedError

    @abstractmethod
    def advance(self) -> Any:
        """Provides the joystick event state.

        Returns:
            The processed output form the joystick.
        """
        raise NotImplementedError


class Device(DeviceBase):
    def __init__(self, env, device_type: str):
        """
        Args:
            env (RobotEnv): The environment which contains the robot(s) to control
                            using this device.
            device_type: The type of the device.
        """
        self.env = env
        self.device_type = device_type

        # functional keyboard setup
        # 注意：omni.appwindow 是随 GUI 使能的扩展，headless app（例如纯标定脚本）里拿不到。
        # 拿不到时降级为"无键盘控制"，设备本身仍可构建/读取（B/R/N 按键在 headless 下本来也没用）。
        try:
            self._appwindow = omni.appwindow.get_default_app_window()
            self._input = carb.input.acquire_input_interface()
            self._keyboard = self._appwindow.get_keyboard()
            # note: Use weakref on callbacks to ensure that this object can be deleted when its destructor is called.
            self._keyboard_sub = self._input.subscribe_to_keyboard_events(
                self._keyboard,
                lambda event, *args, obj=weakref.proxy(self): obj._on_keyboard_event(event, *args),
            )
        except AttributeError as _exc:
            print(f"[device] 无 appwindow（{_exc}）：键盘控制（B/R/N）不可用，仅能构建/读取设备")
            self._appwindow = None
            self._input = None
            self._keyboard = None
            self._keyboard_sub = None

        # some flags and callbacks
        self._started = False
        self._reset_state = False
        self._additional_callbacks = {}

        # display control table
        self._display_controls_table = PrettyTable()
        self._display_controls_table.title = f"Teleoperation Controls for {self.device_type}"
        self._display_controls_table.field_names = ["Key", "Description"]
        self._display_controls_table.align["Description"] = "l"
        # basic controls
        self._display_controls_table.add_row(["B", "start control"])
        self._display_controls_table.add_row(["R", "reset simulation and set task success to False"])
        self._display_controls_table.add_row(["N", "reset simulation and set task success to True"])
        self._display_controls_table.add_row(["Control+C", "quit"])
        self._display_controls_table.add_row(["==========", "=========="])
        self._add_device_control_description()

    def __del__(self):
        """Release the keyboard interface."""
        self._stop_keyboard_listener()

    def get_device_state(self):
        raise NotImplementedError

    def input2action(self):
        state = {}
        reset = state["reset"] = self._reset_state
        state["started"] = self.started
        if reset:
            self._reset_state = False
            return state
        state["joint_state"] = self.get_device_state()

        ac_dict = {
            "reset": reset,
            "started": self.started,
            self.device_type: True,
        }
        if reset:
            return ac_dict
        ac_dict["joint_state"] = state["joint_state"]
        return ac_dict

    def advance(self):
        """
        Returns:
            Can be:
                - torch.Tensor: The action to be applied to the robot.
                - dict: state of the scene and the task, and the task need to reset.
                - None: the scene is not started
        """
        action = self.input2action()
        if action is None:
            return self.env.action_manager.action
        if not action["started"]:
            return None
        if action["reset"]:
            return action
        for key, value in action.items():
            if isinstance(value, np.ndarray):
                action[key] = torch.tensor(value, device=self.env.device, dtype=torch.float32)
        return self.env.cfg.preprocess_device_action(action, self)

    def reset(self):
        pass

    def add_callback(self, key: str, func: Callable):
        self._additional_callbacks[key] = func

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Handle keyboard events using carb."""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == "B":
                self._started = True
                self._reset_state = False
            elif event.input.name == "R":
                self._started = False
                self._reset_state = True
                if "R" in self._additional_callbacks:
                    self._additional_callbacks["R"]()
            elif event.input.name == "N":
                self._started = False
                self._reset_state = True
                if "N" in self._additional_callbacks:
                    self._additional_callbacks["N"]()
            else:
                # ★ 其它键交给注册的附加回调（`add_callback("P", ...)` 之类）。
                #   统一入口用它在运行中切换上肢驱动源（P = 在 auto / policy 间切），
                #   于是"仿真一直开着、开到柜子前再开推理"不需要重启仿真。
                cb = self._additional_callbacks.get(event.input.name)
                if cb is not None:
                    cb()

    def _stop_keyboard_listener(self):
        # headless 下降级时这些属性是 None，跳过即可
        if getattr(self, "_input", None) is not None and getattr(self, "_keyboard_sub", None) is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._keyboard_sub)
            self._keyboard_sub = None

    def _add_device_control_description(self):
        """
        Add device control description to the display control table.
        """
        raise NotImplementedError

    def display_controls(self):
        print(self._display_controls_table)

    def _convert_delta_from_frame(
        self, delta_action: np.ndarray, robot_asset=None, target_frame_idx: int | None = None
    ) -> np.ndarray:
        """
        Convert delta action from target frame to robot base frame.
        target_frame -> root_frame

        双臂设备可以对**每条臂各调一次**：显式传 ``robot_asset`` / ``target_frame_idx``
        （默认仍用 ``self.robot_asset`` / ``self.target_frame_idx``，单臂路径行为不变）。

        Args:
            delta_action: Delta action in target frame.
            robot_asset: 该臂的 articulation（默认 self.robot_asset）
            target_frame_idx: 该臂末端 body 的下标（默认 self.target_frame_idx）
        Returns:
            Delta action in robot base frame.
        """
        robot_asset = self.robot_asset if robot_asset is None else robot_asset
        if target_frame_idx is None:
            target_frame_idx = self.target_frame_idx
        if np.allclose(delta_action[:3], 0.0) and np.allclose(delta_action[3:6], 0.0):
            return delta_action
        is_delta_rot = not np.allclose(delta_action[3:6], 0.0)

        torch_delta_action = torch.tensor(delta_action, device=self.env.device, dtype=torch.float32)

        delta_pos_f = torch_delta_action[:3].repeat(self.env.num_envs, 1)
        delta_rot_f = torch_delta_action[3:6].repeat(self.env.num_envs, 1)
        delta_quat_f = math_utils.quat_from_euler_xyz(delta_rot_f[:, 0], delta_rot_f[:, 1], delta_rot_f[:, 2])
        delta_rotvec_f = math_utils.axis_angle_from_quat(delta_quat_f)

        # ⚠️ IsaacLab 3.0 起 `data.root_pos_w` / `root_quat_w` / `body_quat_w` 是 warp 的
        #    **ProxyArray**，而 math_utils 要 torch.Tensor —— 直接传会抛
        #    `RuntimeError: quat_inv() Expected a value of type 'Tensor' ... found type 'ProxyArray'`
        #    （实测；和 env_utils.py 里修 joint_effort_limits 是同一类坑）。
        #    用 `.torch` 取回 tensor；2.x 的普通 tensor 没有该属性，`hasattr` 兜住。
        def _as_torch(x):
            return x.torch if hasattr(x, "torch") else x

        frame_pos = _as_torch(robot_asset.data.root_pos_w)
        frame_quat = _as_torch(robot_asset.data.body_quat_w)[:, target_frame_idx]
        root_pos = _as_torch(robot_asset.data.root_pos_w)
        root_quat = _as_torch(robot_asset.data.root_quat_w)
        _, frame2root = math_utils.subtract_frame_transforms(root_pos, root_quat, frame_pos, frame_quat)
        frame2root_quat = math_utils.quat_unique(frame2root)

        delta_pos_r = math_utils.quat_apply(frame2root_quat, delta_pos_f)
        delta_rotvec_r = math_utils.quat_apply(frame2root_quat, delta_rotvec_f)
        delta_rot_r = rotvec_to_euler(delta_rotvec_r) if is_delta_rot else torch.zeros(3, device=self.env.device)

        delta_action_r = torch.cat([delta_pos_r.squeeze(0), delta_rot_r, torch_delta_action[6:]], dim=0)

        return delta_action_r.cpu().numpy()

    @property
    def started(self) -> bool:
        return self._started

    @property
    def reset_state(self) -> bool:
        return self._reset_state

    @reset_state.setter
    def reset_state(self, reset_state: bool):
        self._reset_state = reset_state
