#!/usr/bin/env python
"""数采会话驱动（宿主机，纯标准库）—— 按"定时分集"把一条条 demo 录下来。

**不需要 GUI**：所有参数由 `reproduce/collect.env` + 命令行给定；本脚本只做三件事：
  1. 通过 TCP 桥接给仿真发分集指令（`record=begin` / `success` / `fail`）
  2. 计时：每条录 EPISODE_TIME 秒 → 保存 → 停 RESET_TIME 秒 → 下一条
  3. 读终端热键（借鉴 A1Z 的录制热键）：
       →（右箭头）提前结束当前阶段     ←（左箭头）丢弃当前这条并重录     q 结束会话

由 `reproduce/collect.sh` 调用（它会先保证仿真在跑）。
"""
from __future__ import annotations

import argparse
import json
import os
import select
import socket
import sys
import time

# ── 终端热键（raw 模式 + 非阻塞读）────────────────────────────────────────────
try:
    import termios
    import tty

    _HAS_TTY = True
except ImportError:  # pragma: no cover - 非 POSIX
    _HAS_TTY = False


class Hotkeys:
    """把终端切成 raw 模式，非阻塞读取按键。退出时恢复。"""

    def __init__(self, enabled: bool):
        self.enabled = bool(enabled) and _HAS_TTY and sys.stdin.isatty()
        self._saved = None

    def __enter__(self):
        if self.enabled:
            self._saved = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *exc):
        if self.enabled and self._saved is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._saved)
        return False

    def poll(self, timeout: float) -> str | None:
        """最多等 timeout 秒；返回 'skip' / 'discard' / 'quit' / None。"""
        if not self.enabled:
            time.sleep(timeout)
            return None
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return None
        ch = os.read(sys.stdin.fileno(), 1).decode("utf-8", "ignore")
        if ch == "\x1b":  # 方向键：ESC [ A/B/C/D
            seq = os.read(sys.stdin.fileno(), 2).decode("utf-8", "ignore")
            if seq.endswith("C"):
                return "skip"
            if seq.endswith("D"):
                return "discard"
            return None
        if ch in ("q", "Q", "\x03"):  # q 或 Ctrl+C
            return "quit"
        if ch in ("\r", "\n", " "):
            return "skip"
        return None


class Bridge:
    """和仿真侧 SimBridgeServer 的极简客户端（JSON over TCP）。"""

    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(5)

    def send(self, **kw) -> None:
        self.sock.sendall((json.dumps(kw) + "\n").encode())

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


def _bar(done: float, total: float, width: int = 24) -> str:
    frac = 0.0 if total <= 0 else max(0.0, min(1.0, done / total))
    filled = int(frac * width)
    return "[" + "#" * filled + "." * (width - filled) + f"] {done:5.1f}/{total:g}s"


def wait_phase(hk: Hotkeys, seconds: float, label: str, index: int, total: int) -> str:
    """等一个阶段（录制/重置）。返回 ''（正常结束）/ 'skip'（提前结束）/ 'discard' / 'quit'。"""
    t0 = time.time()
    last_len = 0
    interactive = sys.stdout.isatty()
    last_log = -99.0
    while True:
        el = time.time() - t0
        if el >= seconds:
            if interactive:
                print("\r" + " " * last_len, end="")
            return ""
        if not interactive:
            # 输出被重定向（无人值守 / 写日志）→ 每 5 秒一行就够了，别刷屏
            if el - last_log >= 5.0:
                last_log = el
                print(f"[collect] 第 {index}/{total} 条 | {label} {el:5.1f}/{seconds:g}s", flush=True)
            key = hk.poll(min(0.2, max(0.0, seconds - el)))
            if key in ("skip", "quit", "discard"):
                return key
            continue
        msg = (
            f"  第 {index}/{total} 条 | {label} {_bar(el, seconds)}"
            f" | → 提前结束  ← 丢弃重录  q 结束"
        )
        print("\r" + msg.ljust(last_len), end="", flush=True)
        last_len = len(msg)
        key = hk.poll(min(0.2, max(0.0, seconds - el)))
        if key in ("skip", "quit"):
            print("\r" + " " * last_len, end="")
            return key
        if key == "discard":
            print("\r" + " " * last_len, end="")
            return "discard"


def main() -> int:
    ap = argparse.ArgumentParser(description="数采会话驱动（定时分集 + 热键）")
    ap.add_argument("--episodes", type=int, required=True)
    ap.add_argument("--episode_time", type=float, required=True)
    ap.add_argument("--reset_time", type=float, default=0.0)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5560)
    ap.add_argument("--no_hotkeys", action="store_true", help="不读终端按键（脚本/无人值守用）")
    ap.add_argument(
        "--shutdown",
        action="store_true",
        help="会话结束后让仿真**优雅退出**（flush HDF5 再关）。硬杀会丢最后一条的元数据",
    )
    args = ap.parse_args()

    print("=" * 72)
    print(f"  数采会话：{args.episodes} 条 × {args.episode_time:g}s（条间停 {args.reset_time:g}s）")
    print("  热键：→ 提前结束保存   ← 丢弃重录   q 结束会话")
    print("  ── 怎么操作机械臂（重要）──────────────────────────────────────────")
    print("   ① 用鼠标**点一下仿真窗口**（按键事件只发给聚焦的窗口，不点窗口就收不到键盘）")
    print("   ② 在仿真窗口按 **B** 开始控制；双臂键盘按 **T** 切换左/右臂")
    print("   ③ 每条到时间会自动保存；想提前结束就按 →，想丢弃重录就按 ←")
    print("=" * 72)

    try:
        br = Bridge(args.host, args.port)
    except OSError as exc:
        print(f"[collect] ❌ 连不上仿真桥接 {args.host}:{args.port}（{exc}）\n"
              f"          仿真起来了吗？看 {os.environ.get('SIM_LOG', 'reproduce/out/collect_sim.log')}")
        return 2

    # ★ 开始前先把上肢驱动源切到**遥操设备**：collect.sh 可能挂到一个"上次留在 policy/hold"
    #   的仿真上（比如刚做完推理验证），不切的话按 B 也不受控、录出来的是保持位姿的废数据。
    try:
        br.send(arm_source="device")
        time.sleep(0.5)
        print("[collect] 已把上肢驱动源切到遥操设备（arm_source=device）")
    except OSError:
        pass

    done = 0
    idx = 1
    try:
        with Hotkeys(not args.no_hotkeys) as hk:
            while idx <= args.episodes:
                br.send(record="begin")
                time.sleep(0.6)  # 等仿真复位场景并开始累计
                key = wait_phase(hk, args.episode_time, "录制中", idx, args.episodes)
                if key == "quit":
                    br.send(record="fail")  # 丢掉这条没做完的
                    print("\n[collect] 收到 q → 结束会话")
                    break
                if key == "discard":
                    br.send(record="fail")
                    print(f"\n[collect] 第 {idx} 条已丢弃，重录同一条")
                    time.sleep(1.0)
                    continue
                br.send(record="success")
                done += 1
                print(f"\n[collect] ✅ 第 {idx} 条已保存（累计 {done} 条）")
                idx += 1
                if idx <= args.episodes and args.reset_time > 0:
                    key = wait_phase(hk, args.reset_time, "重置中", idx, args.episodes)
                    if key == "quit":
                        print("\n[collect] 收到 q → 结束会话")
                        break
                    if key == "discard":
                        idx -= 1  # 退回上一条重录
                        done -= 1
                        print("\n[collect] 回到上一条重录")
                        continue
    finally:
        if args.shutdown:
            # ★ 必须优雅退出：流式 HDF5 是异步写，只有 env.close() 才 flush + 等写入完成。
            #   直接 SIGTERM/SIGKILL 会让最后一条 episode 变成 success=False / 帧数被截断（实测）。
            time.sleep(1.5)  # 先给仿真一点时间把最后一条导出/落盘（双保险）
            print("[collect] 让仿真优雅退出（刷盘）...")
            try:
                br.send(shutdown=True)
            except OSError:
                pass
            deadline = time.time() + 40
            while time.time() < deadline:
                try:
                    probe = socket.create_connection((args.host, args.port), timeout=0.5)
                    probe.close()
                    time.sleep(1.0)
                except OSError:
                    print("[collect] 仿真已退出 ✓（数据已刷盘）")
                    break
            else:
                print("[collect] ⚠ 40s 没等到仿真退出；稍后 collect.sh 会强杀（最后一条可能不完整）")
        br.close()

    print("=" * 72)
    print(f"  会话结束：成功保存 {done} 条")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
