#!/usr/bin/env bash
# LeIsaac 一键停止 —— 把仿真 / 桥接 / ROS2 / 容器 相关进程全清掉。
#
# 覆盖（按"谁起的"分类）：
#   ① Isaac Sim 本体      isaac-sim-6.0.1 下的 python.sh / kit/python / omni.telemetry / carb
#   ② 仿真入口脚本        ros2_chassis_teleop.py / view_task*.py / teleop_se3_agent.py /
#                         policy_inference.py / shot_scene.py / verify_*.py / export_*.py ...
#   ③ ROS2 侧（宿主机）    ros2 launch / ros2 run / rviz2 / slam_toolbox / nav2 各节点 /
#                         teleop_twist_keyboard / ros2_leisaac_bridge.py
#   ④ 策略服务端          act_action_server.py / run_policy_server.sh / lerobot policy_server
#   ⑤ 容器                ros2-humble-dev:latest 的**运行中**容器
#                         （加 --containers-stopped 连"已退出"的也删掉）
#   ⑥ ros2 daemon         （可用 --keep-daemon 保留）
#
# 用法:
#   bash reproduce/stop_all.sh              # 列出要停谁 → SIGTERM → 等 5s → SIGKILL 残留
#   bash reproduce/stop_all.sh --status      # 只看现在有什么在跑，什么都不停
#   bash reproduce/stop_all.sh -n            # dry-run：只列不杀
#   bash reproduce/stop_all.sh -y            # 不问直接停（脚本化用）
#   bash reproduce/stop_all.sh --keep-daemon
#   bash reproduce/stop_all.sh --containers-stopped   # 连"已退出"的容器也删
#
# ★ 安全设计（这条脚本会 kill，必须保守）：
#   1) 只按**明确的脚本名 / 路径特征**匹配，绝不用 "python"、"ros2" 这种宽泛词；
#   2) 显式排除本脚本自身 + **整条父进程链**（避免把调用它的终端 / DSH harness 一起带走）；
#   3) 杀完**再扫一轮**，并把目标的**全部后代**一并收进来
#      （否则 `python.sh → kit/python`、包装脚本 → 子进程 这类会留孤儿）。
set -uo pipefail   # 故意不用 -e：pgrep 无匹配返回 1，不该让脚本中断

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 路径解析：自动探测 Isaac Sim / lerobot 环境；要覆盖就 export ISAACSIM_DIR 或写 reproduce/local.env
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/leisaac_env.sh"
leisaac_require_isaacsim || exit 1

STATUS_ONLY=0
DRY_RUN=0
ASSUME_YES=0
KEEP_DAEMON=0
STOPPED_CONTAINERS=0
TERM_WAIT=5

while [[ $# -gt 0 ]]; do
    case "$1" in
        -s|--status)            STATUS_ONLY=1; shift ;;
        -n|--dry-run)           DRY_RUN=1; shift ;;
        -y|--yes)               ASSUME_YES=1; shift ;;
        --keep-daemon)          KEEP_DAEMON=1; shift ;;
        --containers-stopped)   STOPPED_CONTAINERS=1; shift ;;
        -h|--help)              sed -n '2,27p' "$0"; exit 0 ;;
        *) echo "[ERROR] 未知参数 $1（-h 看用法）" >&2; exit 2 ;;
    esac
done

# ── 匹配模式：全部是"明确的脚本名 / 路径特征" ────────────────────────────────
PATTERNS=(
    # ① Isaac Sim 本体
    "/isaac-sim-6\\.0\\.1/"
    "omni\\.telemetry"
    # ② 仿真入口脚本
    "ros2_chassis_teleop\\.py"
    "view_task\\.py"
    "view_task_nocams\\.py"
    "teleop_se3_agent\\.py"
    "policy_inference\\.py"
    "smoke_test_env\\.py"
    "shot_scene\\.py"
    "open_scene_gui\\.py"
    "record_synthetic_episode\\.py"
    "check_capabilities\\.py"
    "measure_geometry\\.py"
    "capture_startup\\.py"
    "bench_camera_view\\.py"
    "sweep_lighting\\.py"
    "calibrate_camera_exposure\\.py"
    "calibrate_leader\\.py"
    "verify_chassis"
    "verify_smart_factory"
    "verify_contact_noise\\.py"
    "verify_success_metric\\.py"
    "verify_action_safety\\.py"
    "verify_camera_windows\\.py"
    "export_arena_usd\\.py"
    "export_props_usd\\.py"
    "test_chassis_move\\.py"
    # ③ ROS2 侧
    "ros2_leisaac_bridge\\.py"
    "teleop_twist_keyboard"
    "slam_toolbox"
    "rviz2"
    "ros2 launch"
    "ros2 run"
    "controller_server|planner_server|bt_navigator|behavior_server|map_server|amcl|lifecycle_manager|velocity_smoother"
    # ④ 策略服务端
    "act_action_server\\.py"
    "run_policy_server\\.sh"
    "lerobot\\.async_inference\\.policy_server"
)

# ── 自我保护：本进程 + 整条祖先链，绝不杀 ───────────────────────────────────
declare -A SELF_PIDS=()
_p=$$
while [[ -n "$_p" && "$_p" != "0" && "$_p" != "1" ]]; do
    SELF_PIDS["$_p"]=1
    _p="$(ps -o ppid= -p "$_p" 2>/dev/null | tr -d ' ')"
done

# ── 收集目标 = 模式匹配 ∪ 这些进程的全部后代 ────────────────────────────────
# 后代也要收：`python.sh` 会拉起 `kit/python`；包装脚本会拉起真正干活的进程。
collect_targets() {
    declare -A M=()
    local pid pat k kids cur
    for pat in "${PATTERNS[@]}"; do
        while IFS= read -r pid; do
            [[ -z "$pid" ]] && continue
            [[ -n "${SELF_PIDS[$pid]:-}" ]] && continue
            kill -0 "$pid" 2>/dev/null || continue
            M["$pid"]=1
        done < <(pgrep -f -- "$pat" 2>/dev/null || true)
    done

    local queue=("${!M[@]}")
    while [[ ${#queue[@]} -gt 0 ]]; do
        cur="${queue[0]}"; queue=("${queue[@]:1}")
        kids="$(pgrep -P "$cur" 2>/dev/null || true)"
        for k in $kids; do
            [[ -n "${SELF_PIDS[$k]:-}" ]] && continue
            kill -0 "$k" 2>/dev/null || continue
            [[ -n "${M[$k]:-}" ]] && continue
            M["$k"]=1
            queue+=("$k")
        done
    done

    [[ ${#M[@]} -eq 0 ]] && return 0
    printf '%s\n' "${!M[@]}" | sort -n
}

cmdline_of() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | cut -c1-150; }

# ── 容器 ───────────────────────────────────────────────────────────────────
mapfile -t RUNNING_C < <(docker ps -q --filter "ancestor=ros2-humble-dev:latest" 2>/dev/null || true)
mapfile -t STOPPED_C < <(docker ps -aq --filter "ancestor=ros2-humble-dev:latest" --filter "status=exited" 2>/dev/null || true)

# ── 报告 ───────────────────────────────────────────────────────────────────
mapfile -t TARGETS < <(collect_targets)

echo "════════ LeIsaac 相关进程 ════════"
if [[ ${#TARGETS[@]} -eq 0 ]]; then
    echo "  （无）"
else
    for pid in "${TARGETS[@]}"; do printf "  %-7s %s\n" "$pid" "$(cmdline_of "$pid")"; done
fi
echo
echo "════════ docker 容器（ros2-humble-dev:latest）════════"
if [[ ${#RUNNING_C[@]} -eq 0 && ${#STOPPED_C[@]} -eq 0 ]]; then
    echo "  （无）"
else
    docker ps -a --filter "ancestor=ros2-humble-dev:latest" \
        --format '  {{.ID}}  {{.Names}}  {{.Status}}  {{.Command}}' 2>/dev/null | cut -c1-130
fi
echo
echo "════════ 5560 端口（仿真↔ROS2 桥接）════════"
if ss -ltn 2>/dev/null | grep -q ':5560'; then ss -ltnp 2>/dev/null | grep ':5560'; else echo "  （空闲）"; fi
echo

[[ "$STATUS_ONLY" == "1" ]] && { echo "[status] 只看不动（-h 看用法）。"; exit 0; }
[[ "$DRY_RUN" == "1" ]] && { echo "[dry-run] 上面这些**会被停掉**（实际没动）。去掉 -n 就真停。"; exit 0; }

if [[ "$ASSUME_YES" != "1" ]]; then
    if [[ ! -t 0 ]]; then
        echo "[ERROR] 没有 TTY 可确认。加 -y 表示确认停止，或加 -n 只看。" >&2
        exit 2
    fi
    read -r -p "确认停止以上全部？（y/N）" ans || true
    [[ "${ans:-}" =~ ^[Yy]$ ]] || { echo "已取消。"; exit 0; }
fi

# ── 停：SIGTERM → 等 → 重扫 → SIGKILL ──────────────────────────────────────
if [[ ${#TARGETS[@]} -gt 0 ]]; then
    echo "[stop] SIGTERM ${#TARGETS[@]} 个进程 ..."
    kill -TERM "${TARGETS[@]}" 2>/dev/null || true
fi

if [[ ${#RUNNING_C[@]} -gt 0 ]]; then
    echo "[stop] docker stop ${#RUNNING_C[@]} 个运行中容器 ..."
    docker stop "${RUNNING_C[@]}" >/dev/null 2>&1 || true
fi
if [[ "$STOPPED_CONTAINERS" == "1" && ${#STOPPED_C[@]} -gt 0 ]]; then
    echo "[stop] docker rm -f ${#STOPPED_C[@]} 个已退出容器 ..."
    docker rm -f "${STOPPED_C[@]}" >/dev/null 2>&1 || true
fi

for _ in $(seq 1 "$TERM_WAIT"); do
    sleep 1
    left=0
    for pid in "${TARGETS[@]}"; do kill -0 "$pid" 2>/dev/null && left=$((left+1)); done
    [[ "$left" -eq 0 ]] && break
done

mapfile -t TARGETS2 < <(collect_targets)
if [[ ${#TARGETS2[@]} -gt 0 ]]; then
    echo "[stop] 第一轮后还剩 ${#TARGETS2[@]} 个，SIGKILL ..."
    for pid in "${TARGETS2[@]}"; do printf "  %-7s %s\n" "$pid" "$(cmdline_of "$pid")"; done
    kill -KILL "${TARGETS2[@]}" 2>/dev/null || true
    sleep 1
fi

if [[ "$KEEP_DAEMON" != "1" ]] && command -v ros2 >/dev/null 2>&1; then
    echo "[stop] ros2 daemon stop"
    ros2 daemon stop >/dev/null 2>&1 || true
fi

# ── 收尾 ───────────────────────────────────────────────────────────────────
sleep 1
mapfile -t LEFT < <(collect_targets)
echo
if [[ ${#LEFT[@]} -eq 0 ]]; then
    echo "✅ 已全部停止（残留 0 个）。"
else
    echo "⚠️ 还有 ${#LEFT[@]} 个没退，手动看一下：" >&2
    for pid in "${LEFT[@]}"; do printf "  %-7s %s\n" "$pid" "$(cmdline_of "$pid")" >&2; done
    echo "   （可用 kill -9 <pid> 补刀）" >&2
    exit 1
fi
