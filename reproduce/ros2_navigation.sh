#!/usr/bin/env bash
# ⚠️ 旧入口（保留可用）。推荐等价的 `ros2 launch scripts/nav2_config/ops/navigation.launch.py`
#    —— 自带桥接与归位，`goal:="X Y YAW"` 可无人值守。对照见 ops/README.md §5。
# 【容器侧 · 第三步：导航】起桥接 + Nav2（定位 + 全局规划 + 局部控制）
#
# 用法:
#   bash /work/reproduce/ros2_navigation.sh                       # 用默认地图，前台跑，Ctrl+C 结束
#   bash /work/reproduce/ros2_navigation.sh /work/xxx/map.yaml    # 指定地图
#   bash /work/reproduce/ros2_navigation.sh --map maps/kitchen.yaml --rviz   # 开 RViz 自己点 2D Goal Pose 发目标
#   bash /work/reproduce/ros2_navigation.sh --auto -0.8 0.0 0     # 无人值守：起导航→发目标点→报告结果→退出
#                                                                 （会先把机器人送回起点，用 --no-reset 可跳过）
#   bash /work/reproduce/ros2_navigation.sh --localization odom   # 定位用"固定 map→odom"（里程计即真值，定位零误差）
#
# 前台模式下，另开一个终端发目标点:
#   bash /work/reproduce/ros2_send_goal.sh -0.8 0.0 0
#   bash /work/reproduce/ros2_send_goal.sh --check          # 看定位/导航状态
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
MAP_YAML="$REPO/scripts/nav2_config/maps/kitchen.yaml"
SIM_HOST="${SIM_HOST:-127.0.0.1}"
SIM_PORT="${SIM_PORT:-5560}"
WITH_RVIZ=0
AUTO=0
LOCALIZATION="amcl"
GOAL=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --map)      MAP_YAML="$2"; [[ "$MAP_YAML" = /* ]] || MAP_YAML="$REPO/scripts/nav2_config/$MAP_YAML"; shift 2 ;;
        --sim_host) SIM_HOST="$2"; shift 2 ;;
        --sim_port) SIM_PORT="$2"; shift 2 ;;
        --rviz)     WITH_RVIZ=1; shift ;;
        --no-reset) NO_RESET=1; shift ;;
        --localization) LOCALIZATION="$2"; shift 2 ;;
        --auto)     AUTO=1; shift ;;
        -[0-9]*)    GOAL+=("$1"); shift ;;   # 目标点坐标（负数也算参数，不能当未知选项）
        -h|--help)  sed -n '2,16p' "$0"; exit 0 ;;
        -*)         echo "[ERROR] 未知参数 $1" >&2; exit 2 ;;
        *)          GOAL+=("$1"); shift ;;   # 位置参数 = 地图路径 或 目标点坐标
    esac
done

# 位置参数里第一个不是数字的当地图路径
if [ "${#GOAL[@]}" -gt 0 ] && [[ ! "${GOAL[0]}" =~ ^-?[0-9] ]]; then
    MAP_YAML="${GOAL[0]}"
    [[ "$MAP_YAML" = /* ]] || MAP_YAML="$REPO/scripts/nav2_config/$MAP_YAML"
    GOAL=("${GOAL[@]:1}")
fi

command -v ros2 >/dev/null 2>&1 || { echo "[ERROR] 先 source /opt/ros/humble/setup.bash" >&2; exit 1; }
[ -s "$MAP_YAML" ] || { echo "[ERROR] 找不到地图 $MAP_YAML（先建图：ros2_mapping.sh --auto）" >&2; exit 1; }
if [ "$WITH_RVIZ" = "1" ]; then RVIZ_ARG="rviz:=true"; else RVIZ_ARG="rviz:=false"; fi

echo "[nav] ① 连接 Isaac Sim $SIM_HOST:$SIM_PORT ..."
python3 "$REPO/scripts/ros2_leisaac_bridge.py" --sim_host "$SIM_HOST" --sim_port "$SIM_PORT" > /tmp/leisaac_bridge.log 2>&1 &
BRIDGE_PID=$!
trap 'kill $BRIDGE_PID 2>/dev/null' EXIT
for _ in $(seq 1 20); do
    grep -q "Connected" /tmp/leisaac_bridge.log 2>/dev/null && break
    kill -0 $BRIDGE_PID 2>/dev/null || { echo "[nav] ❌ 桥接退出：" >&2; tail -5 /tmp/leisaac_bridge.log >&2; exit 1; }
    sleep 1
done
grep -q "Connected" /tmp/leisaac_bridge.log && echo "[nav] ✅ 桥接已连上" \
    || echo "[nav] ⚠️ 桥接未连上（宿主机起了 ros2_chassis_teleop.sh 吗？）"

echo "[nav] ② 启动 Nav2（地图 $MAP_YAML，定位=$LOCALIZATION，use_sim_time=false）"
LAUNCH_CMD=(ros2 launch "$REPO/scripts/nav2_config/navigation.launch.py"
            map:="$MAP_YAML" use_sim_time:=false "localization:=$LOCALIZATION" "$RVIZ_ARG")

if [ "$AUTO" = "0" ]; then
    echo "[nav]    另开终端发目标点: bash $SCRIPT_DIR/ros2_send_goal.sh -0.8 0.0 0"
    echo "[nav]    Ctrl+C 结束导航"
    exec "${LAUNCH_CMD[@]}"
fi

# ---------- --auto 无人值守模式 ----------
if [ "${#GOAL[@]}" -lt 2 ]; then
    echo "[ERROR] --auto 需要目标点: --auto X Y [YAW角度]" >&2; exit 2
fi
echo "[nav] ③ 先让底盘回初始位姿（地图是从这个起点建的，定位也按 (0,0,0) 起步）"
echo '        如果确实想从机器人现在的位置开始导航，加 --no-reset'
if [ "${NO_RESET:-0}" = "0" ]; then
    bash "$SCRIPT_DIR/ros2_reset_sim.sh" || echo "[nav] ⚠️ 归位失败，继续（结果可能不准）"
fi

ros2 launch "$REPO/scripts/nav2_config/navigation.launch.py" \
    map:="$MAP_YAML" use_sim_time:=false "localization:=$LOCALIZATION" rviz:=false > /tmp/nav2.log 2>&1 &
NAV_PID=$!
cleanup() { kill -INT $NAV_PID 2>/dev/null; kill $BRIDGE_PID 2>/dev/null; }
trap cleanup EXIT

echo "[nav] ④ 等 Nav2 生命周期激活 ..."
ACTIVE=0
for i in $(seq 1 60); do
    ST="$(timeout 3 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1)"
    if [ "${ST%% *}" = "active" ]; then ACTIVE=1; echo "[nav] ✅ bt_navigator active（${i}s）"; break; fi
    kill -0 $NAV_PID 2>/dev/null || break
    sleep 1
done
if [ "$ACTIVE" != "1" ]; then
    echo "[nav] ❌ Nav2 没激活，日志末尾：" >&2; tail -25 /tmp/nav2.log >&2; exit 1
fi

echo "[nav] ⑤ 定位自检（map→base_footprint 是否已给出）"
bash "$SCRIPT_DIR/ros2_send_goal.sh" --check

echo "[nav] ⑤ 发送目标点 ..."
bash "$SCRIPT_DIR/ros2_send_goal.sh" "${GOAL[@]}"
RC=$?

echo "[nav] ⑥ 到达后定位复核"
bash "$SCRIPT_DIR/ros2_send_goal.sh" --check
if [ -s /tmp/nav2.log ]; then
    echo "[nav] Nav2 日志里的报错行（若有）:"
    grep -iE "error|fail|abort|timeout" /tmp/nav2.log | tail -8 | sed 's/^/       /' || true
fi
echo "[nav] 结束，退出码 $RC（0 = 已到达）"
exit $RC
