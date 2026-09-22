#!/usr/bin/env bash
# ⚠️ 旧入口（保留可用）。推荐等价的 `ros2 launch scripts/nav2_config/ops/mapping.launch.py`
#    —— 自带桥接，一条命令 = 桥接+SLAM+RViz+键盘。对照见 scripts/nav2_config/ops/README.md §5。
# 【容器侧 · 第一步：建图】起桥接 + slam_toolbox，然后开着机器人把场地走一遍。
#
# 起容器（宿主机，注意 --network host 与挂载）:
#   docker run -it --rm --network host \
#     -v $REPO:/work -w /work \
#     ros2-humble-dev:latest \
#     bash -lc "source /opt/ros/humble/setup.bash && bash /work/reproduce/ros2_mapping.sh"
#
# 参数:
#   --sim_host HOST      宿主机地址（默认 127.0.0.1）
#   --sim_port PORT      桥接端口（默认 5560）
#   --rviz               同时开 RViz 看建图过程（需要 X11；容器要 -e DISPLAY -v /tmp/.X11-unix）
#   --file NAME          地图名字（默认 kitchen，存在 scripts/nav2_config/maps/）
#   --auto               无人值守：自动探索扫场 → 自动存图 → 自动归位 → 退出（用于验证/演示）
#   （不加 --auto 是**交互模式**：后台起 slam，你遥控走一圈，回本终端按回车即存图+归位）
#   --pattern explore|map  --auto 时的行驶方式（默认 explore：自动挑最开阔方向探索）
#
# 手动流程：本脚本只负责"起建图"。另开一个终端遥控机器人把场地走一圈：
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard     # 手动
#   或 bash /work/reproduce/ros2_auto_drive.sh --pattern map   # 自动扫一圈
# 走完后另开终端存图：
#   bash /work/reproduce/ros2_save_map.sh kitchen
set -uo pipefail

SIM_HOST="127.0.0.1"
SIM_PORT="5560"
WITH_RVIZ=0
MAP_NAME="kitchen"
AUTO=0
DRIVE_PATTERN="explore"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim_host) SIM_HOST="$2"; shift 2 ;;
        --sim_port) SIM_PORT="$2"; shift 2 ;;
        --rviz)     WITH_RVIZ=1; shift ;;
        --file)     MAP_NAME="$2"; shift 2 ;;
        --auto)     AUTO=1; shift ;;
        --pattern)  DRIVE_PATTERN="$2"; shift 2 ;;
        -h|--help)  sed -n '2,28p' "$0"; exit 0 ;;
        *)          echo "[ERROR] 未知参数 $1" >&2; exit 2 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
BRIDGE="$REPO/scripts/ros2_leisaac_bridge.py"
SLAM_PARAMS="$REPO/scripts/nav2_config/slam_toolbox.yaml"
MAP_OUT="$REPO/scripts/nav2_config/maps/$MAP_NAME"

command -v ros2 >/dev/null 2>&1 || { echo "[ERROR] 先 source /opt/ros/humble/setup.bash" >&2; exit 1; }
[ -f "$BRIDGE" ] || { echo "[ERROR] 找不到 $BRIDGE（容器里把仓库挂到 /work 并 -w /work）" >&2; exit 1; }

echo "[map] ① 连接 Isaac Sim $SIM_HOST:$SIM_PORT ..."
python3 "$BRIDGE" --sim_host "$SIM_HOST" --sim_port "$SIM_PORT" > /tmp/leisaac_bridge.log 2>&1 &
BRIDGE_PID=$!
trap 'kill $BRIDGE_PID 2>/dev/null' EXIT
for _ in $(seq 1 20); do
    grep -q "Connected" /tmp/leisaac_bridge.log 2>/dev/null && break
    kill -0 $BRIDGE_PID 2>/dev/null || { echo "[map] ❌ 桥接退出：" >&2; tail -5 /tmp/leisaac_bridge.log >&2; exit 1; }
    sleep 1
done
grep -q "Connected" /tmp/leisaac_bridge.log || echo "[map] ⚠️ 还没连上（宿主机那边起了 ros2_chassis_teleop.sh 吗？）"

echo "[map] ② 话题自检（应看到 /scan 与 /odom）"
timeout 5 ros2 topic list 2>/dev/null | grep -E "^/(scan|odom|tf)$" | sed 's/^/       /' || true

echo "[map] ③ 启动 slam_toolbox + RViz（use_sim_time=false，参数 $SLAM_PARAMS）"
if [ "$WITH_RVIZ" = "1" ]; then
    RVIZ_ARG="rviz:=true"
    echo "[map]    RViz 要容器能连 X11：宿主机先跑一次 xhost +SI:localuser:\$USER，"
    echo "         容器加 --gpus all -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"
else
    RVIZ_ARG="rviz:=false"
fi

# 非交互（没有 TTY）又不自动 → 保持老行为：前台跑，自己 Ctrl+C
if [ "$AUTO" = "0" ] && [ ! -t 0 ]; then
    echo "[map] ④ 没有 TTY（没加 -it）→ 前台起 slam，请你另开终端遥控；Ctrl+C 结束（结束后记得手动存图）"
    exec ros2 launch "$REPO/scripts/nav2_config/mapping.launch.py" \
        use_sim_time:=false "$RVIZ_ARG" slam_params_file:="$SLAM_PARAMS"
fi

# ---------- 起 slam（后台），手动或自动都走这里 ----------
echo "[map] ④ 后台启动 slam_toolbox ..."
ros2 launch "$REPO/scripts/nav2_config/mapping.launch.py" \
    use_sim_time:=false "$RVIZ_ARG" slam_params_file:="$SLAM_PARAMS" \
    > /tmp/slam_toolbox.log 2>&1 &
SLAM_PID=$!
cleanup() { kill -INT $SLAM_PID 2>/dev/null; kill $BRIDGE_PID 2>/dev/null; }
trap cleanup EXIT

echo "[map] ⑤ 等 slam_toolbox 上线 ..."
OK=0
for i in $(seq 1 40); do
    if timeout 3 ros2 topic list 2>/dev/null | grep -qx "/map"; then OK=1; break; fi
    kill -0 $SLAM_PID 2>/dev/null || break
    sleep 1
done
if [ "$OK" = "1" ]; then echo "[map] ✅ /map 已发布（${i}s）"; else
    echo "[map] ⚠️ /map 未出现，日志末尾：" >&2; tail -12 /tmp/slam_toolbox.log >&2
fi

if [ "$AUTO" = "1" ]; then
    echo "[map] ⑥ 自动行驶扫场（$DRIVE_PATTERN）..."
    bash "$SCRIPT_DIR/ros2_auto_drive.sh" --pattern "$DRIVE_PATTERN"
else
    echo ""
    echo "======================================================================"
    echo " 现在去【另一个终端】遥控机器人把场地走一圈（边走 RViz 里边长地图）："
    echo "   ★ 推荐：用 docker exec 进**本容器**开键盘（同一容器最稳，不用管 --ipc）"
    echo "     docker exec -it <本容器名> bash -lc \"source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard\""
    echo "   ○ 或者另开一个容器（两边都必须加 --ipc=host，否则 /cmd_vel 传不过来）："
    echo "     docker run -it --rm --network host --ipc=host -v $REPO:/work -w /work ros2-humble-dev:latest \\"
    echo "       bash -lc \"source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard\""
    echo "   ⚠️ 这两种都**只跑键盘、不要再起桥接**（本脚本已经起了；仿真同一时刻只接一个桥接）"
    echo "   ○ 不想手动走也可以让脚本自动逛：bash $SCRIPT_DIR/ros2_auto_drive.sh --pattern explore"
    echo ""
    echo " 按键：i 前进 / , 后退 / j 左转 / l 右转 / k 停 / q 减速 / z 加速"
    echo " 走完回到**本终端**按回车 → 自动存图 + 自动归位"
    echo "======================================================================"
    echo ""
    read -r -p "[map] 走完了吗？按回车存图（Ctrl+C 放弃）..." _ || true
fi

echo "[map] ⑥ 等地图收敛（3 秒）..."
sleep 3
if timeout 3 ros2 topic list 2>/dev/null | grep -qx "/map"; then
    bash "$SCRIPT_DIR/ros2_save_map.sh" "$MAP_NAME"
    RC=$?
else
    echo "[map] ❌ 没有 /map，跳过存图" >&2; RC=1
fi

echo "[map] ⑦ 存完图再让机器人归位（导航要从同一起点开始，省得重启仿真）"
bash "$SCRIPT_DIR/ros2_reset_sim.sh" || echo "[map] ⚠️ 归位失败，导航前请手动重启仿真或用 ros2_auto_drive.sh 开回起点"

echo "[map] 结束（地图前缀 $MAP_OUT）"
exit ${RC:-1}
