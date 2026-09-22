"""`scripts/nav2_config/ops/*.launch.py` 共用的小工具。

设计约定（为什么这么写）:
- ops 下的入口 launch 用 ``OpaqueFunction``，在里面把**所有 launch 参数 perform 成普通 Python 值**
  再往下传。这样拼 ``--ros-args -p k:=v`` 时不会踩"f-string 把 LaunchConfiguration 的 repr
  当成参数值"的坑，条件判断（要不要起桥接 / 要不要自动存图）也能直接用 if 写，
  不用堆 IfCondition / PythonExpression。
- 节点都是仓库里的普通脚本（不是 ament 包），所以统一用 ``ExecuteProcess`` +
  ``python3 <脚本> --ros-args -p ...`` 启动。
"""

from __future__ import annotations

import os
import sys

from launch.actions import ExecuteProcess

_TRUTHY = ("1", "true", "yes", "on", "y")


def as_bool(text) -> bool:
    """launch 参数字符串 → bool（'true'/'1'/'yes'/'on'/'y' 都算真）。"""
    if text is None:
        return False
    if isinstance(text, bool):
        return text
    return str(text).strip().lower() in _TRUTHY


def python_script(script_path: str, params: dict | None = None, name: str | None = None,
                  emulate_tty: bool = True) -> ExecuteProcess:
    """用 ``<ROS 的 python> <脚本> --ros-args -p k:=v ...`` 起一个仓库里的普通 ROS2 脚本。

    ★ 用 ``sys.executable`` 而**不是字符串 "python3"**：本函数是在 `ros2 launch`
    进程里执行的，``sys.executable`` 就是 ROS 自己那个 python（一定有 rclpy）；
    而 ``"python3"`` 会走 PATH —— **赛队机器上 conda 常驻时，python3 会解析到 conda 的
    python，它没有 rclpy，桥接/节点直接 ImportError 起不来**。
    容器里因为只有系统 python 所以一直没暴露这个问题。
    """
    cmd = [sys.executable, script_path]
    if params:
        cmd.append("--ros-args")
        for key, value in params.items():
            cmd += ["-p", f"{key}:={value}"]
    return ExecuteProcess(
        cmd=cmd,
        name=name or os.path.basename(script_path),
        output="screen",
        emulate_tty=emulate_tty,
    )


#: 键盘操作的提示语（teleop 入口与建图入口共用）
KEYBOARD_HELP = "键盘：i 前进 / , 后退 / j 左转 / l 右转 / k 停 / q·z 调速 / Ctrl+C 退出"


def keyboard_process(speed: str = "0.5", turn: str = "1.0") -> ExecuteProcess:
    """键盘遥控进程（teleop_twist_keyboard）。

    ★ 为什么是 ``bash -c "... < /dev/tty"`` 而不是 ``Node(package="teleop_twist_keyboard")``：
      ROS2 Humble 的 launch 用 asyncio 的 subprocess 起子进程，**stdin 默认是新建的 pipe**
      （``osrf_pycommon/.../async_execute_process_asyncio/impl.py`` 里
      ``_async_execute_process_pty = _async_execute_process_nopty``，而 nopty 不传 stdin，
      于是吃 asyncio 的默认 ``stdin=PIPE``）。所以 ``emulate_tty`` 设 True/False 都一样，
      子进程的 ``sys.stdin.isatty()`` 恒为 **False**，而 teleop_twist_keyboard 会
      ``termios.tcgetattr(sys.stdin)`` → 直接 ``termios.error: (25, ...)`` 挂掉（实测两种都挂）。
      绕法是让子进程自己重开控制终端：``< /dev/tty`` → 实测 ``isatty() = True``。
      ⚠️ 因此**容器必须用 ``docker run -it``**（``-i`` 不够，``-t`` 才会给容器分配 tty）。
    """
    cmd = (
        "exec ros2 run teleop_twist_keyboard teleop_twist_keyboard "
        f"--ros-args -p speed:={speed} -p turn:={turn} < /dev/tty"
    )
    return ExecuteProcess(
        cmd=["bash", "-c", cmd],
        name="teleop_twist_keyboard",
        output="screen",
        emulate_tty=True,
    )


def paths(launch_file: str) -> tuple[str, str, str, str]:
    """从 ops 下的 launch 文件路径推出 (config_dir, repo_root, nodes_dir, maps_dir)。

    ops/xxx.launch.py → config_dir = scripts/nav2_config, repo_root = 仓库根
    """
    ops_dir = os.path.dirname(os.path.realpath(launch_file))
    config_dir = os.path.dirname(ops_dir)
    repo_root = os.path.dirname(os.path.dirname(config_dir))
    return (
        config_dir,
        repo_root,
        os.path.join(config_dir, "nodes"),
        os.path.join(config_dir, "maps"),
    )
