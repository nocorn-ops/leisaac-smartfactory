#!/usr/bin/env bash
# 单臂遥操（一条 SO101 主手驱动双臂任务里的左/右臂）
#
# ★ 已收敛：本脚本只是**统一仿真入口**的薄封装，内部转调
#     reproduce/ros2_chassis_teleop.sh --teleop_device so101leader-one ...
#   对外的仿真启动入口**只有 `ros2_chassis_teleop.sh` 一个**，而且**没有模式之分**
#   （2026-09-18 起：底盘随时可被 ROS2 驱动、上肢一启动就接管）。留这个脚本是为了把
#   "单臂主手 + 双臂任务 + 设备名自动选择"收成一行，参数名向后兼容（--port/--dataset）。
#
# 等价的手写命令（推荐直接记这个）:
#   bash reproduce/ros2_chassis_teleop.sh \
#        --teleop_device so101leader-one --port /dev/ttyACM0 --arm_side left \
#        --enable_cameras [--record --dataset_file ./datasets/x.hdf5]
#
# 只用一条主手驱动双臂任务里的左臂：另一只手臂锁定在当前位姿不动，
# 动作空间仍是 12 维（与双臂采集的数据格式一致）。
#
# 用法（在仓库根目录执行）:
#   bash reproduce/teleop_left_arm.sh                       # 默认场景（智慧工厂场地）+ 左臂（推荐先试这个）
#   bash reproduce/teleop_left_arm.sh --record              # 上面 + 录制 HDF5
#   bash reproduce/teleop_left_arm.sh --arm_side right      # 改成驱动右臂（用右臂标定）
#   bash reproduce/teleop_left_arm.sh --task LeIsaac-LeRobot-Kitchen-v0      # 换双臂任务
#   bash reproduce/teleop_left_arm.sh --task LeIsaac-SO101-CleanToyTable-v0  # 换单臂任务
#   bash reproduce/teleop_left_arm.sh --port /dev/ttyACM1   # 换串口
#   bash reproduce/teleop_left_arm.sh --headless            # 不开窗（自检用）
#   bash reproduce/teleop_left_arm.sh --camera_view windows # 其它参数原样透传给统一入口
#
# 操作键（在 Isaac Sim 窗口里按，先点一下窗口让它获得键盘焦点）:
#   B           ★ 开始控制（必须按！不按的话从手不动，看起来像"卡住"）
#   N           本次演示成功 → 保存这一条并重置（之后要再按一次 B 继续）
#   R           放弃/重置当前这条（之后要再按一次 B 继续）
#   Ctrl+C      在终端里按，退出
#
# ★ 按 B 之前先把主手摆成与仿真里那只手臂**相近**的姿势（都是"伸直朝前"）。
#   若差得太多（默认 >20°），从手不会动，终端会提示差最多的关节；
#   摆近后会自动接管，不需要再按一次 B。这是绝对位姿映射：接管后主从姿势一一对应。
#   觉得门槛太严可以加 --engage_threshold 0.6（单位弧度，约 34°）。
#
# ★ 相机画面：默认在一个窗口里并排显示 left_wrist / right_wrist / front 三张实时图
#   （就是录进数据集的那三台相机）。换其它方式：--camera_view windows（每台一个视口窗口，
#   可拖动但更吃性能）/ --camera_view off（不显示，相机数据照常录）。
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 路径解析：自动探测 Isaac Sim / lerobot 环境；要覆盖就 export ISAACSIM_DIR 或写 reproduce/local.env
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/leisaac_env.sh"
leisaac_require_isaacsim || exit 1
PORT="/dev/ttyACM0"
# 默认任务：智慧工厂场地（双臂任务），本脚本只用一条主手驱动其中一只手臂
# （so101leader-one：受控那半边跟随主手，另一只手臂锁定在当前位姿，动作空间仍是 12 维）。
# 其它可选：LeIsaac-LeRobot-Kitchen-v0（厨房）、LeIsaac-SO101-CleanToyTable-v0（玩具房整理）。
# ⚠️ 别用 LiftCube：它的 table_with_cube 场景资产是残缺的（一块 0.7×0.65×0.07 的平板，
#    方块还嵌在台面里），视口里看不到东西。
TASK="LeIsaac-SmartFactory-v0"
ARM_SIDE="left"
DATASET="./datasets/one_arm_teleop.hdf5"
RECORD=0
EXTRA=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --port)     PORT="$2"; shift 2 ;;
        --task)     TASK="$2"; shift 2 ;;
        --arm_side) ARM_SIDE="$2"; shift 2 ;;
        --dataset)  DATASET="$2"; shift 2 ;;
        --record)   RECORD=1; shift ;;
        -h|--help)  sed -n '2,40p' "$0"; exit 0 ;;
        *)          EXTRA+=("$1"); shift ;;
    esac
done

# 双臂任务（名字里带 LeRobot / BiArm / SmartFactory）用「单臂驱动双臂任务」的设备；
# 单臂任务用普通 so101leader。
# ⚠️ 判据与 `leisaac/utils/env_utils.py::get_task_type()` 保持一致 —— 少写一个就会
#    把双臂任务当成单臂任务，报 `Scene entity with key 'robot' not found`（踩过）。
if [[ "$TASK" == *LeRobot* || "$TASK" == *BiArm* || "$TASK" == *SmartFactory* ]]; then
    DEVICE="so101leader-one"
else
    DEVICE="so101leader"
fi

if [ ! -x "$ISAACSIM_DIR/python.sh" ]; then
    echo "[ERROR] 找不到 Isaac Sim: $ISAACSIM_DIR" >&2
    exit 1
fi
if [ ! -e "$PORT" ]; then
    echo "[ERROR] 串口不存在: $PORT" >&2
    echo "        可用串口：$(ls /dev/ttyACM* 2>/dev/null || echo '（一个都没有，检查主手 USB）')" >&2
    exit 1
fi
if [ "$RECORD" = "1" ] && [ -e "$DATASET" ]; then
    echo "[ERROR] 数据集已存在: $DATASET" >&2
    echo "        换一个 --dataset 路径，或对已有文件用 --resume（本脚本不自动加）" >&2
    exit 1
fi

# Isaac Sim EULA + 图形显示 + 不要被 conda 干扰
export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
export DISPLAY="${DISPLAY:-:0}"
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE VIRTUAL_ENV || true
# Ubuntu 26.04 的旧 ABI 兼容库（libxml2.so.2 / libicu 74）
# 新版 Ubuntu（26.04 等）需要这套旧 ABI 库（libxml2/icu74）；22.04/24.04 自带、无需也兼容。
# 只在**确实有库文件**时才加（打包给别人时可能是空目录/死链，加了也不该报错）
if [ -f "$REPO/reproduce/compat_libs_isaac6/libxml2.so.2" ]; then
    export LD_LIBRARY_PATH="$REPO/reproduce/compat_libs_isaac6:${LD_LIBRARY_PATH:-}"
fi
# ★ 关键：开了相机（--enable_cameras）时，IsaacLab 会启用 env 级 RTX 场景分区，
#   而它与 Kit 视口的渲染产物冲突 —— 现象就是"窗口出来了但视口一片空白"。
#   我们只用 1 个 env，分区没有意义，直接关掉（官方提供的开关）。
export ISAAC_LAB_ENABLE_ISAAC_RTX_PER_ENV_SCENE_PARTITION=0

# ── 转调统一仿真入口（唯一对外入口）─────────────────────────────────────────
# 统一入口自己会处理：EULA、unset conda、兼容库、--visualizer kit、TCP 桥接。
UNIFIED="$REPO/reproduce/ros2_chassis_teleop.sh"
if [ ! -x "$UNIFIED" ]; then
    echo "[teleop] [ERROR] 找不到统一入口: $UNIFIED" >&2
    exit 1
fi

ARGS=(
    --task "$TASK"
    --teleop_device "$DEVICE"
    --port "$PORT"
    --enable_cameras            # 这些任务的场景里带相机，必须显式打开，否则会报 "camera was spawned without --enable_cameras"
)
if [ "$DEVICE" = "so101leader-one" ]; then
    ARGS+=(--arm_side "$ARM_SIDE")
fi
if [ "$RECORD" = "1" ]; then
    ARGS+=(--record --dataset_file "$DATASET")
    echo "[teleop] 录制模式：数据将写入 $DATASET"
fi

echo "[teleop] 转调统一入口: $UNIFIED"
echo "[teleop] 任务=$TASK  设备=$DEVICE  主手=$PORT  臂=$ARM_SIDE  录制=$RECORD"
echo "[teleop] 窗口出现后：点一下窗口 → 把主手摆到与仿真里那只手臂相近的姿势 → 按 B 开始控制"
echo "[teleop] 提示：按 B 后终端若显示「还未接管」，照着提示把差得多的关节摆近，会自动接管"
if [ "$RECORD" = "1" ]; then
    echo "[teleop] 每条做完按 N 保存（再按 B 继续下一条）；放弃当前这条按 R"
fi

cd "$REPO"
exec bash "$UNIFIED" "${ARGS[@]}" ${EXTRA[@]+"${EXTRA[@]}"}
