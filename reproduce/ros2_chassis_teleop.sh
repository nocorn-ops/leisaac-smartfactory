#!/usr/bin/env bash
# LeIsaac **统一仿真入口** —— 宿主机侧（Isaac Sim 开窗 + TCP 桥接 + 底盘运动学）。
#
# ★ **只有一个模式**：一次仿真里所有操控通道**同时在线**（2026-09-18 起）：
#
#   底盘  ROS2 /cmd_vel —— 容器里的键控 / Nav2 / 任何发布者，随时能开（除非 --lock_chassis）
#   上肢  遥操设备 —— 一"启动"就接管（主手使能 / 键盘按 B）；没启动时保持位姿
#   上肢  策略 —— 配了 --mode policy / --enable_policy 就跑；**人一按 B 就交给人**，
#                 R / N 之后交回策略
#   录制  --record 随时可用（R = 失败重录 / N = 成功保存）
#
#   所以"打开底盘遥操就开底盘、打开机械臂遥操就操作机械臂"，两者互不干扰。
#   `--mode nav|teleop` 仍然接受，但只是**兼容别名**（行为完全一样，会打印提示）。
#
# ★ 常用就**一条命令**，其余全是默认值（2026-09-19 起）:
#     bash reproduce/ros2_chassis_teleop.sh
#   默认已经包含：开窗 + 三路相机 + 录制就绪(自动命名 datasets/session_<月日>_<时分>.hdf5)
#   + LiDAR 前向急停 0.45m + 遥操设备**自动探测**（两个主手串口在就用双臂主手，否则 bi-keyboard）
#   + 底盘自由 + 容器侧可运行中切设备/策略/录制。
#
# 用法（在仓库根目录）:
#   bash reproduce/ros2_chassis_teleop.sh                    # 就要这一条
#   bash reproduce/ros2_chassis_teleop.sh --headless --max_seconds 600 --lidar_stop_distance 0.45
#   bash reproduce/ros2_chassis_teleop.sh --lidar_vis         # 显示 LiDAR 射线
#
#   # 数采：直接从收纳架作业位起仿真（不用先开车过去），机器人留在原地录完所有 demo
#   bash reproduce/ros2_chassis_teleop.sh --start_at shelf --teleop_device bi-keyboard \
#        --record --dataset_file ./datasets/sf_biarm.hdf5 --num_demos 30 --enable_cameras
#
#   # 上肢遥操（真主手；先插好两个 SO101 主手）
#   bash reproduce/ros2_chassis_teleop.sh \
#        --left_arm_port /dev/ttyACM0 --right_arm_port /dev/ttyACM1 --enable_cameras
#
#   # 上肢推理（先在另一个终端起策略服务端，见 scripts/evaluation/act_action_server.py）
#   bash reproduce/ros2_chassis_teleop.sh --enable_policy \
#        --policy_type local-act --policy_port 5556 --enable_cameras
#
# 冲突检测（脚本之间互相感知，不用人猜）:
#   · 同一台机器只允许一个仿真 —— 第二个会被锁文件/端口检查拦住并给出处理办法
#   · 桥接同一时刻只服务一个 ROS2 客户端 —— 第二个会被明确拒绝并打印原因
#   · /cmd_vel 上多于一个发布者（键控 + Nav2 同时开）—— 桥接客户端会警告并列出节点名
#
# 然后开容器（另开一个终端，见 reproduce/ros2_keyboard_container.sh）:
#   docker run -it --rm --network host -v "$PWD":/work -w /work ros2-humble-dev:latest \
#     bash -lc "source /opt/ros/humble/setup.bash && bash /work/reproduce/ros2_keyboard_container.sh"
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 路径解析：自动探测 Isaac Sim / lerobot 环境；要覆盖就 export ISAACSIM_DIR 或写 reproduce/local.env
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/leisaac_env.sh"
leisaac_require_isaacsim || exit 1
TASK="LeIsaac-SmartFactory-v0"
PORT=5560
EXTRA=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --task)          TASK="$2"; shift 2 ;;
        --bridge_port)   PORT="$2"; shift 2 ;;
        -h|--help)       sed -n '2,30p' "$0"; exit 0 ;;
        *)               EXTRA+=("$1"); shift ;;
    esac
done

if [ ! -x "$ISAACSIM_DIR/python.sh" ]; then
    echo "[ERROR] 找不到 Isaac Sim: $ISAACSIM_DIR" >&2
    exit 1
fi

export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
export DISPLAY="${DISPLAY:-:0}"
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE VIRTUAL_ENV || true
# 新版 Ubuntu（26.04 等）需要这套旧 ABI 库（libxml2/icu74）；22.04/24.04 自带、无需也兼容。
# 只在**确实有库文件**时才加（打包给别人时可能是空目录/死链，加了也不该报错）
if [ -f "$REPO/reproduce/compat_libs_isaac6/libxml2.so.2" ]; then
    export LD_LIBRARY_PATH="$REPO/reproduce/compat_libs_isaac6:${LD_LIBRARY_PATH:-}"
fi
export ISAAC_LAB_ENABLE_ISAAC_RTX_PER_ENV_SCENE_PARTITION=0

ARGS=(
    "$REPO/scripts/environments/ros2_chassis_teleop.py"
    --task "$TASK"
    --bridge_port "$PORT"
)
# 没显式说 headless 就开窗口（IsaacLab 3.0 不选可视化器会强制 headless）
if [[ " ${EXTRA[*]:-} " != *" --headless "* ]]; then
    ARGS+=(--visualizer kit)
fi
echo "[ros2] 任务=$TASK  桥接端口=$PORT"
echo "[ros2] 窗口出现后（约 10 秒），在容器里跑 ros2_keyboard_container.sh，然后按 i/,/j/l 等键遥控"
echo "[ros2] 键盘在 ROS2 侧（teleop_twist_keyboard），Isaac Sim 这边只负责显示和执行"
exec "$ISAACSIM_DIR/python.sh" -u "${ARGS[@]}" ${EXTRA[@]+"${EXTRA[@]}"}
