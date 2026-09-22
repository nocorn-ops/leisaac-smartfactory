#!/usr/bin/env bash
# ⚠️ 旧入口（保留可用）。推荐等价的 `ros2 launch scripts/nav2_config/ops/teleop.launch.py`
#    （自带桥接+键盘，一条命令）。对照见 scripts/nav2_config/ops/README.md §5。
# ROS2 键盘遥控底盘 —— **容器侧**（在 ros2-humble-dev 容器里运行）
#
# 作用：① 起桥接客户端（连宿主机的 Isaac Sim）② 起 teleop_twist_keyboard 让你用键盘遥控。
#
# 起容器（宿主机执行，注意 --network host，否则连不到宿主机 127.0.0.1:5560）:
#   docker run -it --rm --network host \
#     -v $REPO:/work -w /work \
#     ros2-humble-dev:latest \
#     bash -lc "source /opt/ros/humble/setup.bash && bash /work/reproduce/ros2_keyboard_container.sh"
#
# ⚠️ 本脚本**自己会起桥接**（ros2_leisaac_bridge.py），所以：
#   · 单独用键盘遥控 → 直接用这个脚本，不要再另起桥接；
#   · 已经在用 ros2_mapping.sh / ros2_navigation.sh（它们也起桥接）→
#     这里只跑 `ros2 run teleop_twist_keyboard teleop_twist_keyboard` 就行，**别再起第二个桥接**；
#   · 仿真侧 SimBridgeServer 同一时刻只服务一个客户端：多起的那个 TCP 能连上但数据不通，
#     表现就是"键盘能按、机器人不动"。
#   · **跨容器**（键盘和桥接在两个容器里）必须给两边都加 `--ipc=host`，否则 ROS2 共享内存传输
#     跨容器失效，同样是"能发现话题但指令传不过去"。
#
# 参数:
#   --sim_host HOST   宿主机地址（默认 127.0.0.1，配 --network host）
#   --sim_port PORT   桥接端口（默认 5560）
#   --check           只测连通性：起桥接 → 等连上 → 打印 OK → 退出（不开键盘）
#
# teleop_twist_keyboard 按键（默认档 speed=0.5 turn=1.0）:
#   i 前进   , 后退   j 左转   l 右转   u/o/m/. 斜向
#   k 停     q/z 加减速度     w/x 只调线速度     e/c 只调角速度
#   Ctrl+C 退出
set -uo pipefail

SIM_HOST="127.0.0.1"
SIM_PORT="5560"
CHECK_ONLY=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim_host) SIM_HOST="$2"; shift 2 ;;
        --sim_port) SIM_PORT="$2"; shift 2 ;;
        --check)    CHECK_ONLY=1; shift ;;
        -h|--help)  sed -n '2,22p' "$0"; exit 0 ;;
        *)          echo "[ERROR] 未知参数 $1" >&2; exit 2 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
BRIDGE="$REPO/scripts/ros2_leisaac_bridge.py"

if [ ! -f "$BRIDGE" ]; then
    echo "[ERROR] 找不到 $BRIDGE（容器里请把仓库挂到 /work 并 -w /work）" >&2
    exit 1
fi
if ! command -v ros2 >/dev/null 2>&1; then
    echo "[ERROR] 没找到 ros2，请先 source /opt/ros/humble/setup.bash" >&2
    exit 1
fi

echo "[ros2] 连接 Isaac Sim: $SIM_HOST:$SIM_PORT"
python3 "$BRIDGE" --sim_host "$SIM_HOST" --sim_port "$SIM_PORT" > /tmp/leisaac_bridge.log 2>&1 &
BRIDGE_PID=$!
trap 'kill $BRIDGE_PID 2>/dev/null' EXIT

# 等桥接连上（宿主机那边要先跑 reproduce/ros2_chassis_teleop.sh）
for _ in $(seq 1 20); do
    if grep -q "Connected" /tmp/leisaac_bridge.log 2>/dev/null; then
        echo "[ros2] ✅ 已连上 Isaac Sim（日志 /tmp/leisaac_bridge.log）"
        break
    fi
    if ! kill -0 $BRIDGE_PID 2>/dev/null; then
        echo "[ros2] ❌ 桥接进程退出，日志：" >&2
        tail -5 /tmp/leisaac_bridge.log >&2
        exit 1
    fi
    sleep 1
done
if ! grep -q "Connected" /tmp/leisaac_bridge.log 2>/dev/null; then
    echo "[ros2] ⚠️ 20 秒内没连上；宿主机那边是否已启动 ros2_chassis_teleop.sh？" >&2
fi

if [ "$CHECK_ONLY" = "1" ]; then
    echo "[ros2] --check 通过（只验证连通性）；/odom 与 /scan 正在发布"
    timeout 5 ros2 topic echo /odom --once 2>/dev/null | grep -A 4 "position" | head -5 || true
    exit 0
fi

echo "[ros2] 键盘遥控启动：i 前进 / , 后退 / j 左转 / l 右转 / k 停 / q·z 调速 / Ctrl+C 退出"
echo "[ros2] 注意：ROS2 侧停发指令（或断开）时，仿真里的机器人会**立即停车**（deadman 行为）"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
