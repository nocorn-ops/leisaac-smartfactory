#!/usr/bin/env bash
# LeIsaac 打开仿真 GUI 界面
# 路线：Isaac Sim 6.0.1 standalone + IsaacLab 3.0
#   （RTX 5070 Ti / Blackwell 上唯一能打开 GUI 的组合；5.1.0 的 RTX 渲染器在该卡上会崩）
# 用法（宿主机终端，需图形界面 + NVIDIA 显卡）:
#   bash reproduce/launch_gui.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 路径解析：自动探测 Isaac Sim / lerobot 环境；要覆盖就 export ISAACSIM_DIR 或写 reproduce/local.env
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/leisaac_env.sh"
leisaac_require_isaacsim || exit 1

if [ ! -x "$ISAACSIM_DIR/python.sh" ]; then
    echo "[ERROR] 未找到 Isaac Sim standalone: $ISAACSIM_DIR" >&2
    echo "        请先把 isaac-sim-standalone-6.0.1-linux-x86_64.zip 解压到该目录" >&2
    exit 1
fi

# 接受 Isaac Sim EULA（首次运行必需，否则会卡在交互确认）
export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
# IsaacLab 3.0 要求使用 standalone 自带 Python，须避免 conda 环境干扰
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE

# Ubuntu 26.04 只提供 libxml2.so.16 等新 ABI；Isaac Sim 6.0.1 的扩展需要旧 ABI（libxml2.so.2 / libicu 74）
# 新版 Ubuntu（26.04 等）需要这套旧 ABI 库（libxml2/icu74）；22.04/24.04 自带、无需也兼容。
# 只在**确实有库文件**时才加（打包给别人时可能是空目录/死链，加了也不该报错）
if [ -f "$REPO/reproduce/compat_libs_isaac6/libxml2.so.2" ]; then
    export LD_LIBRARY_PATH="$REPO/reproduce/compat_libs_isaac6:${LD_LIBRARY_PATH:-}"
fi

cd "$REPO"
echo "[launch] 启动任务 LeIsaac-SmartFactory-v0（Isaac Sim 6.0.1）..."
exec "$ISAACSIM_DIR/python.sh" reproduce/view_task_nocams.py --task LeIsaac-SmartFactory-v0 --visualizer kit "$@"
