#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════════
# **数采一条命令** —— 读配置 → （必要时）起仿真 → 定时分集录完 → 给出转换指令
#
#   bash reproduce/collect.sh                    # 用 reproduce/collect.env 里的参数
#   bash reproduce/collect.sh --episodes 10      # 命令行覆盖任意参数
#   bash reproduce/collect.sh --no_sim           # 仿真已经在跑 → 直接挂上去录
#
# 借鉴 A1Z-LinkerHand 的数采方式：**参数提前在配置文件里设好，一条命令把整场录完**，
# 定时分集（每条 N 秒自动保存）+ 热键（→ 提前结束、← 丢弃重录、q 结束），不做 GUI。
# ═══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

CONFIG="$REPO/reproduce/collect.env"
NO_SIM=0
DRIVER_ARGS=()
declare -A OV=()          # 命令行覆盖项（在 source 配置之后再生效 → 命令行优先）

# ── 解析命令行：`--config` + 覆盖配置里的任意 key + 少量透传 ─────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)       CONFIG="$2"; shift 2 ;;
        --no_sim)       NO_SIM=1; shift ;;
        --no_hotkeys)   DRIVER_ARGS+=(--no_hotkeys); shift ;;
        --episodes)     OV[EPISODES]="$2"; shift 2 ;;
        --episode_time) OV[EPISODE_TIME]="$2"; shift 2 ;;
        --reset_time)   OV[RESET_TIME]="$2"; shift 2 ;;
        --dataset)      OV[DATASET]="$2"; shift 2 ;;
        --task)         OV[TASK]="$2"; shift 2 ;;
        --task_desc)    OV[TASK_DESC]="$2"; shift 2 ;;
        --start_at)     OV[START_AT]="$2"; shift 2 ;;
        --teleop_device) OV[TELEOP_DEVICE]="$2"; shift 2 ;;
        --lidar_stop)   OV[LIDAR_STOP]="$2"; shift 2 ;;
        -h|--help)      sed -n '2,11p' "$0"; exit 0 ;;
        *)              echo "[collect] 未知参数: $1（配置项请用 --KEY value，或直接改 $CONFIG）" >&2; exit 2 ;;
    esac
done

if [[ ! -f "$CONFIG" ]]; then
    echo "[collect] [ERROR] 找不到配置文件: $CONFIG" >&2
    exit 2
fi

# 读配置（KEY=VALUE 形式，直接 source，安全前提是这是本仓库自己的文件）
set -a
# shellcheck disable=SC1090
source "$CONFIG"
set +a

# 命令行覆盖（在 source 之后 → 优先级更高）
for _k in "${!OV[@]}"; do
    declare -g "$_k=${OV[$_k]}"
done

: "${EPISODES:=30}"; : "${EPISODE_TIME:=60}"; : "${RESET_TIME:=20}"
: "${TASK:=LeIsaac-SmartFactory-v0}"; : "${DATASET:=datasets/session.hdf5}"
: "${TASK_DESC:=}"; : "${START_AT:=}"; : "${TELEOP_DEVICE:=}"
: "${LEFT_ARM_PORT:=}"; : "${RIGHT_ARM_PORT:=}"
: "${LIDAR_STOP:=0}"; : "${EXTRA_SIM_ARGS:=}"   # 急停默认关（0.45 在场地角落/柜子前会挡路）
: "${BRIDGE_PORT:=5560}"; : "${SIM_LOG:=reproduce/out/collect_sim.log}"
: "${STOP_SIM_AT_END:=1}"

mkdir -p "$(dirname "$SIM_LOG")"

echo "════════════════════════════════════════════════════════════════════"
echo "  数采会话配置（$CONFIG）"
echo "    任务        : $TASK"
echo "    数据集      : $DATASET   （$( [[ -e "$DATASET" ]] && echo '⚠ 已存在，会被拒绝，请改名或加 --resume' || echo '不存在，可写' )）"
echo "    条数/时长   : $EPISODES 条 × ${EPISODE_TIME}s（条间停 ${RESET_TIME}s）"
echo "    作业位      : ${START_AT:-场景初始位姿}"
echo "    遥操设备    : ${TELEOP_DEVICE:-自动探测}"
echo "    任务描述    : ${TASK_DESC:-（未填，转换时要用）}"
echo "════════════════════════════════════════════════════════════════════"

if [[ -e "$DATASET" ]]; then
    echo "[collect] [ERROR] $DATASET 已存在 —— 换个名字，或确认要续录时加 --resume（本脚本暂不代管续录）" >&2
    exit 2
fi

bridge_up() {
    python3 - "$BRIDGE_PORT" <<'PY' 2>/dev/null
import socket, sys
s = socket.socket(); s.settimeout(0.3)
sys.exit(0 if s.connect_ex(("127.0.0.1", int(sys.argv[1]))) == 0 else 1)
PY
}

SIM_PID=""
SIM_OWNED=0
if bridge_up; then
    echo "[collect] 检测到仿真已在跑（桥接 :$BRIDGE_PORT 通）→ 直接挂上去录（不重启、车不动）"
else
    if [[ "$NO_SIM" == "1" ]]; then
        echo "[collect] [ERROR] --no_sim 但桥接 :$BRIDGE_PORT 连不上 —— 先把仿真起起来" >&2
        exit 2
    fi
    SIM_ARGS=(
        --task "$TASK"
        --dataset_file "$DATASET"
        --max_seconds 0
        --lidar_stop_distance "$LIDAR_STOP"
        --bridge_port "$BRIDGE_PORT"
    )
    [[ -n "$START_AT" ]]        && SIM_ARGS+=(--start_at "$START_AT")
    [[ -n "$TELEOP_DEVICE" ]]   && SIM_ARGS+=(--teleop_device "$TELEOP_DEVICE")
    [[ -n "$LEFT_ARM_PORT" ]]   && SIM_ARGS+=(--left_arm_port "$LEFT_ARM_PORT")
    [[ -n "$RIGHT_ARM_PORT" ]]  && SIM_ARGS+=(--right_arm_port "$RIGHT_ARM_PORT")
    # shellcheck disable=SC2206
    [[ -n "$EXTRA_SIM_ARGS" ]]  && SIM_ARGS+=($EXTRA_SIM_ARGS)

    echo "[collect] 启动仿真（日志：$SIM_LOG）"
    echo "[collect]   bash reproduce/ros2_chassis_teleop.sh ${SIM_ARGS[*]}"
    # ★ 必须 setsid：`ros2_chassis_teleop.sh` 只是包装，真正的 Isaac Sim 是它的子进程
    #   （python.sh → kit/python）。只 kill 包装进程的话**仿真会活下来占着 GPU 和端口**（踩过）。
    #   起在独立进程组里，收尾时 kill 整个组。
    setsid bash reproduce/ros2_chassis_teleop.sh "${SIM_ARGS[@]}" >"$SIM_LOG" 2>&1 &
    SIM_PID=$!
    SIM_OWNED=1

    echo -n "[collect] 等仿真就绪"
    for _ in $(seq 1 120); do
        if bridge_up; then break; fi
        if ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo; echo "[collect] [ERROR] 仿真进程退出了，日志尾部：" >&2
            tail -25 "$SIM_LOG" >&2
            exit 3
        fi
        echo -n "."; sleep 1
    done
    echo
    if ! bridge_up; then
        echo "[collect] [ERROR] 等 120s 还没通，日志尾部：" >&2
        tail -25 "$SIM_LOG" >&2
        kill "$SIM_PID" 2>/dev/null || true
        exit 3
    fi
    echo "[collect] 仿真就绪 ✓（先看一眼窗口里的机器人位置对不对）"
    sleep 3
fi

# 自己起的仿真 → 让 driver 在录完后请求它**优雅退出**（刷盘）；别人的仿真绝不动
SHUTDOWN_ARG=()
if [[ "$SIM_OWNED" == "1" && "$STOP_SIM_AT_END" == "1" ]]; then
    SHUTDOWN_ARG+=(--shutdown)
fi

RC=0
set +e
python3 "$REPO/reproduce/collect_driver.py" \
    --episodes "$EPISODES" --episode_time "$EPISODE_TIME" --reset_time "$RESET_TIME" \
    --port "$BRIDGE_PORT" "${SHUTDOWN_ARG[@]+"${SHUTDOWN_ARG[@]}"}" \
    "${DRIVER_ARGS[@]+"${DRIVER_ARGS[@]}"}"
RC=$?
set -e

if [[ "$SIM_OWNED" == "1" && "$STOP_SIM_AT_END" == "1" ]]; then
    if ! bridge_up; then
        echo "[collect] 仿真已优雅退出 ✓（HDF5 已刷盘）"
        rm -f /tmp/leisaac_sim.lock
    else
    echo "[collect] 仿真还在（可能是 driver 的优雅退出没等到）→ 关掉它（进程组 pgid=$SIM_PID）"
    kill -TERM -"$SIM_PID" 2>/dev/null || kill -TERM "$SIM_PID" 2>/dev/null || true
    for _ in $(seq 1 15); do
        kill -0 "$SIM_PID" 2>/dev/null || break
        sleep 1
    done
    if kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[collect] 还活着 → 强杀"
        kill -KILL -"$SIM_PID" 2>/dev/null || kill -KILL "$SIM_PID" 2>/dev/null || true
        sleep 2
    fi
    rm -f /tmp/leisaac_sim.lock
    fi
fi

echo
echo "════════════════════════════════════════════════════════════════════"
if [[ "$SIM_OWNED" == "0" ]]; then
    # ★ 挂载模式：数据写的是**仿真自己**启动时那个 HDF5（看仿真终端横幅里 `录制:` 那一行），
    #   本脚本的 --dataset 只在"需要自己起仿真"时才生效 —— 别拿它去判断成功与否（会误报）。
    echo "  挂载模式：数据写进**那个正在跑的仿真**自己的录制文件（看仿真终端横幅的 录制: 一行）"
    echo "            本脚本的 --dataset/--task_desc 只在需要自己起仿真时生效"
    echo "  下一步（转 LeRobot v3 → 训练）见 README §7/§8"
elif [[ -f "$DATASET" ]]; then
    SIZE=$(du -h "$DATASET" | cut -f1)
    echo "  ✅ 数据集：$DATASET  ($SIZE)"
    echo
    echo "  下一步（转 LeRobot v3 → 训练）："
    echo "    LR=/home/vedal/miniconda3/envs/leisaac-lerobot-sim/bin/python"
    echo "    \$LR scripts/convert/hdf5_to_lerobot_v3.py \\"
    echo "        --hdf5 $DATASET --repo_id $(basename "${DATASET%.hdf5}") \\"
    echo "        --root $REPO/datasets/lerobot --task \"$TASK_DESC\" --fps 30"
else
    echo "  ⚠ 没找到 $DATASET —— 一条都没保存成功？"
fi
echo "════════════════════════════════════════════════════════════════════"
exit $RC
