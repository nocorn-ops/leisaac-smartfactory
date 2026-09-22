#!/usr/bin/env python
"""验收：TCP 桥接的**客户端冲突检测**（纯 Python + socket，秒级，不需要仿真/GPU）。

背景（真实踩坑）：仿真侧桥接以前"accept 一个客户端后就阻塞在它的 readline 上"，
第二个 ROS2 客户端连上来只会**静静排队**，表现是"键盘能按、车不动"，极难排查。
现在第二个客户端会被**明确拒绝**并收到一行 JSON 说明。

检查项：
  ① 第一个客户端能正常发 cmd_vel，服务端 get_cmd_vel() 收得到
  ② 第二个客户端连上后**立刻收到** {"error": ...} 说明，然后被断开
  ③ 第二个客户端被拒后，**第一个客户端仍然能继续发指令**（不被踢掉）
  ④ 第一个客户端断开后，新客户端可以正常接手（不是一次性的）

用法（仓库根目录，任何 python 都能跑，不依赖 isaaclab）::

    python3 reproduce/verify_bridge_conflicts.py

通过标准：结尾 `✅ 桥接客户端冲突检测 全部通过`。
"""
import json
import os
import socket
import sys
import threading
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "source", "leisaac"))

from leisaac.utils.sim_bridge_server import SimBridgeServer  # noqa: E402

HOST = "127.0.0.1"
PORT = 5599  # 故意避开 5560，免得撞上正在跑的仿真
failures: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  {'✅' if ok else '❌'} {name}" + (f"  （{detail}）" if detail else ""))
    if not ok:
        failures.append(name)


def connect(timeout: float = 3.0):
    s = socket.create_connection((HOST, PORT), timeout=timeout)
    return s, s.makefile("r", buffering=1)


def send(sock: socket.socket, vx: float) -> None:
    sock.sendall((json.dumps({"cmd_vel": [vx, 0.0, 0.0]}) + "\n").encode())


bridge = SimBridgeServer(host=HOST, port=PORT)
bridge.start()
time.sleep(0.4)  # 等服务线程 bind/listen

print("\n===== ① 第一个客户端能发指令 =====")
a_sock, a_file = connect()
time.sleep(0.3)
send(a_sock, 0.25)
time.sleep(0.4)
cmd = bridge.get_cmd_vel()
check("服务端收到第一个客户端的 cmd_vel", abs(cmd[0] - 0.25) < 1e-6, f"cmd_vel={cmd}")
check("client_connected = True", bridge.client_connected)

print("\n===== ② 第二个客户端应被明确拒绝 =====")
b_sock, b_file = connect()
b_sock.settimeout(3.0)
try:
    line = b_file.readline()
except Exception as exc:  # noqa: BLE001
    line = f"<读失败 {type(exc).__name__}: {exc}>"
print(f"  第二个客户端读到: {line.strip()!r}")
ok_reject = "error" in str(line)
check("第二个客户端收到拒绝说明（含 error 字段）", ok_reject)
b_file.close()
b_sock.close()

print("\n===== ③ 第一个客户端不受影响（还能继续发）=====")
bridge.get_cmd_vel()  # 清掉上一条
send(a_sock, -0.4)
time.sleep(0.4)
cmd2 = bridge.get_cmd_vel()
check("第一个客户端仍能发指令", abs(cmd2[0] + 0.4) < 1e-6, f"cmd_vel={cmd2}")
check("服务端仍认为它连着", bridge.client_connected)

print("\n===== ④ 运行中控制指令（arm_source / record）能透传 =====")
bridge.get_cmd_vel()
send(a_sock, 0.1)  # 占位，确认通道还活着
c_sock2 = a_sock
c_sock2.sendall((json.dumps({"arm_source": "policy"}) + "\n").encode())
c_sock2.sendall((json.dumps({"record": "begin"}) + "\n").encode())
time.sleep(0.5)
cmds = bridge.consume_control()
kinds = [("arm_source" if "arm_source" in c else "record") for c in cmds]
check("arm_source / record 指令都收到了", kinds == ["arm_source", "record"], f"{cmds}")
check("consume_control() 取完即清空（不会重复执行）", bridge.consume_control() == [])

print("\n===== ⑤ 第一个断开后，新客户端可以接手 =====")
# ⚠️ socket.makefile() 会**多持有一个 fd 引用**：只 close(sock) 不会真的断开，
#    必须连 file 对象一起关（第一版测试就漏了这一步，结果误判"服务端没释放"）。
a_file.close()
a_sock.close()
time.sleep(0.8)
check("断开后 client_connected = False", not bridge.client_connected)
c_sock, _c_file = connect()
time.sleep(0.3)
bridge.get_cmd_vel()
send(c_sock, 0.77)
time.sleep(0.4)
cmd3 = bridge.get_cmd_vel()
check("新客户端能正常接手", abs(cmd3[0] - 0.77) < 1e-6, f"cmd_vel={cmd3}")
_c_file.close()
c_sock.close()

bridge.stop()
time.sleep(0.2)

print("\n" + "=" * 64)
if failures:
    print("❌ 未通过：" + "、".join(failures))
    sys.exit(1)
print("✅ 桥接客户端冲突检测 全部通过")
print("=" * 64)
sys.exit(0)
