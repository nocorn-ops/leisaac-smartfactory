"""Isaac Sim 侧 TCP 桥接服务 — 将底盘位姿和 LiDAR 数据发送给 ROS2，接收 cmd_vel。

协议 (JSON over TCP, 换行符分隔):
    ← 发送: {"type": "pose", "data": [x, y, yaw]}
    ← 发送: {"type": "scan", "data": {"ranges": [...], "angle_min": -3.14, "angle_max": 3.14, ...}}
    → 接收: {"cmd_vel": [vx, vy, wz]}
    → 接收: {"reset": true}            # 底盘瞬移回初始位姿（建图完 → 导航前归位）

用法:
    bridge = SimBridgeServer(host="127.0.0.1", port=5560)
    bridge.start()
    ...
    bridge.send_pose(x, y, yaw)
    cmd = bridge.get_cmd_vel()  # (vx, vy, wz) or (0, 0, 0)
    ...
    bridge.stop()
"""

import json
import select
import socket
import threading
import time


class SimBridgeServer:
    """TCP 桥接服务端，运行在 Isaac Sim 进程中."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5560):
        self._host = host
        self._port = port
        self._server_sock = None
        self._client_sock = None
        self._client_file = None
        # ★ 自己维护接收缓冲：`socket.makefile()` 的缓冲 + `select` 混用会**丢行**
        #   （一次 readline 可能把后面几行也读进用户态缓冲，之后 select 再也不报可读，
        #    那几行就永远卡在缓冲里 —— 实测：cmd_vel 和控制指令连着发，控制指令丢失）
        self._recv_buf = b""
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        # 最新的 cmd_vel 指令
        self._cmd_vel = (0.0, 0.0, 0.0)
        self._new_cmd = False
        # ★ 指令保持时长（deadman）：ROS2 键盘 ~10Hz 发、仿真主循环 ~10-15Hz 取，
        #   如果"取一次就清零"，大部分循环帧拿到的是 0 → 实际速度只有指令的 1/3（实测
        #   0.2 m/s 只走出 0.067 m/s）。保持 0.3s：既平滑，又能在客户端挂掉后 0.3s 内停下。
        self._cmd_hold_s = 0.3
        self._cmd_t = 0.0
        # 复位请求（ROS2 侧发 {"reset": true} 时置位，由仿真主循环消费）
        self._reset_requested = False
        # 优雅退出请求（ROS2/数采脚本发 {"shutdown": true}）：
        # ★ 必须走"干净退出"——流式 HDF5 写入是**异步 executor**，只有 env.close() 才会
        #   flush + shutdown。直接 SIGTERM 掉仿真会丢掉最后一条 episode 的元数据
        #   （实测：最后一条变成 success=False 且帧数被截断）。
        self._shutdown_requested = False
        # 运行时控制指令队列（ROS2 侧发 {"arm_source": ...} / {"record": ...}）：
        # 让"仿真一直开着、容器里按需切换数采/推理"成为可能（见操作指南 §2.1）
        self._control: list[dict] = []
        # LiDAR 数据缓存
        self._latest_scan = None

    def start(self):
        """启动服务线程."""
        self._running = True
        self._thread = threading.Thread(target=self._serve, name="SimBridge", daemon=True)
        self._thread.start()

    def stop(self):
        """停止服务."""
        self._running = False
        if self._client_sock:
            try:
                self._client_sock.close()
            except Exception:
                pass
        if self._server_sock:
            try:
                self._server_sock.close()
            except Exception:
                pass

    def _serve(self):
        """服务线程主循环 — 同一时刻只服务**一个**客户端，第二个会被明确拒绝并告知原因。

        ⚠️ 2026-09-18 改：以前 accept 一个客户端后就一直阻塞在它的 readline 上，
        第二个客户端连上来只会**静静排队**（TCP backlog），表现为"键盘能按、车不动"，
        极难排查。现在用 select 同时盯 listen socket，第二个客户端会被立刻拒掉并收到
        一行 JSON 说明（终端也能看到），用户马上知道要关掉多余的桥接客户端。
        """
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.settimeout(1.0)
        try:
            self._server_sock.bind((self._host, self._port))
            self._server_sock.listen(4)
            print(f"[SimBridge] Listening on {self._host}:{self._port}")
        except OSError as e:
            print(f"[SimBridge] Failed to bind: {e}")
            self._running = False
            return

        while self._running:
            # ① 当前没有客户端 → 等着来一个
            if self._client_sock is None:
                try:
                    conn, addr = self._server_sock.accept()
                except socket.timeout:
                    continue
                except Exception as e:  # noqa: BLE001
                    if self._running:
                        print(f"[SimBridge] Accept error: {e}")
                    continue
                self._client_sock = conn
                self._client_file = conn.makefile("r", buffering=1)  # 仅为兼容保留（不再用它读）
                self._recv_buf = b""
                print(f"[SimBridge] Client connected: {addr}")
                continue

            # ② 已有客户端 → 同时盯着 listen socket（检测"第二个客户端"）
            try:
                readable, _, _ = select.select([self._server_sock, self._client_sock], [], [], 0.2)
            except (OSError, ValueError):
                time.sleep(0.05)
                continue

            if self._server_sock in readable:
                try:
                    extra, addr2 = self._server_sock.accept()
                except Exception:  # noqa: BLE001
                    extra = None
                if extra is not None:
                    print(
                        f"[SimBridge] ⛔ 拒绝第二个客户端 {addr2}：**同一时刻只服务一个**"
                        f"（已有客户端在连）。\n"
                        f"           → 关掉多余的键盘/建图/导航/复位进程，一个容器里只留一个桥接客户端；\n"
                        f"           → 自查：bash reproduce/stop_all.sh --status",
                        flush=True,
                    )
                    try:
                        extra.sendall(
                            (
                                json.dumps(
                                    {
                                        "error": "another client is already connected",
                                        "hint": "同一时刻只允许一个桥接客户端；请关掉多余的 "
                                        "ros2_leisaac_bridge/键盘/建图/导航进程",
                                    },
                                    ensure_ascii=False,
                                )
                                + "\n"
                            ).encode()
                        )
                    except Exception:  # noqa: BLE001
                        pass
                    try:
                        extra.close()
                    except Exception:  # noqa: BLE001
                        pass
                continue

            if self._client_sock in readable:
                try:
                    chunk = self._client_sock.recv(65536)
                except Exception:  # noqa: BLE001
                    chunk = b""
                if not chunk:  # 客户端断开
                    self._drop_client()
                    print("[SimBridge] Client disconnected")
                    continue
                self._recv_buf += chunk
                # ★ 一次 recv 里可能有好几行（cmd_vel + 控制指令连着发）——**全部**处理掉，
                #   绝不能只 readline 一行（见 __init__ 里 _recv_buf 的注释）
                while b"\n" in self._recv_buf:
                    raw_line, self._recv_buf = self._recv_buf.split(b"\n", 1)
                    self._handle_line(raw_line)

    def _drop_client(self) -> None:
        """客户端断开后清引用并把指令归零（否则 client_connected 会一直显示已连接）。"""
        with self._lock:
            self._client_sock = None
            self._client_file = None
            self._recv_buf = b""
            self._new_cmd = False
            self._cmd_vel = (0.0, 0.0, 0.0)
            self._cmd_t = 0.0

    def _handle_line(self, line) -> None:
        """解析一行 JSON 指令（cmd_vel / reset）。"""
        try:
            if isinstance(line, bytes):
                line = line.decode("utf-8", "ignore")
            line = line.strip()
            if not line:
                return
            msg = json.loads(line)
            if "cmd_vel" in msg:
                with self._lock:
                    cv = msg["cmd_vel"]
                    self._cmd_vel = (float(cv[0]), float(cv[1]), float(cv[2]))
                    self._new_cmd = True
                    self._cmd_t = time.monotonic()
            if msg.get("reset"):
                with self._lock:
                    self._reset_requested = True
            # ★ 运行时控制：{"arm_source": "device|policy|hold|auto"(+可选 policy_port/policy_type)}
            #              {"record": "begin|success|fail|stop"}
            if msg.get("shutdown"):
                print("[SimBridge] ← 收到控制指令 {'shutdown': True}", flush=True)
                with self._lock:
                    self._shutdown_requested = True
            if "arm_source" in msg or "record" in msg:
                print(f"[SimBridge] ← 收到控制指令 {msg}", flush=True)  # 控制指令很少，值得留痕
                with self._lock:
                    self._control.append(msg)
                    if len(self._control) > 64:  # 防呆：客户端刷屏时不留垃圾
                        self._control = self._control[-64:]
        except (json.JSONDecodeError, ValueError, KeyError, TypeError):
            return

    def consume_shutdown(self) -> bool:
        """取出并清除"请干净退出"请求（仿真主循环看到就跳出 → 正常收尾刷盘）。"""
        with self._lock:
            want, self._shutdown_requested = self._shutdown_requested, False
            return want

    def has_control(self) -> bool:
        """队列里还有没有待处理的控制指令（只看不取）。

        用途：收到 shutdown 时若还有没处理的指令（典型：`record=success` 和 `shutdown`
        在同一个 TCP 批里到达），**先别退** —— 让下一帧把它们处理完（含导出这一条 episode），
        否则最后一条会变成 success=False / 帧数被截断（实测踩到）。
        """
        with self._lock:
            return bool(self._control)

    def consume_control(self) -> list:
        """取出并清空运行中控制指令（由仿真主循环每帧调一次）。"""
        with self._lock:
            cmds, self._control = self._control, []
            return cmds

    @property
    def client_connected(self) -> bool:
        """ROS2 侧的桥接客户端是否已连上。"""
        return self._client_sock is not None

    def get_cmd_vel(self) -> tuple[float, float, float]:
        """取当前生效的 cmd_vel —— **最近 `_cmd_hold_s` 秒内收到过就一直有效**（deadman）。

        超时没收到新指令就返回 0（客户端死了/断线 → 机器人自己停）。
        """
        with self._lock:
            if self._new_cmd:
                self._new_cmd = False
                self._cmd_t = time.monotonic()
            if time.monotonic() - self._cmd_t <= self._cmd_hold_s:
                return self._cmd_vel
            return (0.0, 0.0, 0.0)

    def consume_reset(self) -> bool:
        """取出并清除"复位"请求（ROS2 侧发了 {"reset": true}）。"""
        with self._lock:
            if self._reset_requested:
                self._reset_requested = False
                return True
            return False

    def send_pose(self, x: float, y: float, yaw: float):
        """发送底盘位姿."""
        self._send({"type": "pose", "data": [x, y, yaw]})

    def send_scan(self, scan_data: dict):
        """发送 LiDAR 扫描数据."""
        self._send({"type": "scan", "data": scan_data})

    def _send(self, msg: dict):
        """发送 JSON 消息 + 换行符."""
        if self._client_sock is None:
            return
        try:
            data = json.dumps(msg) + "\n"
            self._client_sock.sendall(data.encode("utf-8"))
        except Exception:
            pass

    @property
    def connected(self) -> bool:
        return self._client_sock is not None
