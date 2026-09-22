#!/usr/bin/env bash
# 路线 A：用 Isaac Sim 6.0.1 standalone 直接打开厨房场景 USD（不依赖 IsaacLab）。
#
# 只显示场景资产（厨房 + 橙子 + 盘子），不含机器人；机器人由 IsaacLab 配置在路线 B 中生成。
# 已经验证：Isaac Sim 6.0.1 原生 GUI 在本机（RTX 5070 Ti / 驱动 580.173.02）可以正常显示该场景。
#
# 用法（宿主机终端，需图形界面）:
#   bash reproduce/open_kitchen_gui.sh                 # 开窗，手动关窗退出
#   bash reproduce/open_kitchen_gui.sh --seconds 60    # 60 秒后自动退出（无人值守测试用）
#   bash reproduce/open_kitchen_gui.sh --headless --seconds 20   # 无窗口冒烟
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 路径解析：自动探测 Isaac Sim / lerobot 环境；要覆盖就 export ISAACSIM_DIR 或写 reproduce/local.env
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/leisaac_env.sh"
leisaac_require_isaacsim || exit 1

if [ ! -x "$ISAACSIM_DIR/python.sh" ]; then
    echo "[ERROR] 未找到 Isaac Sim standalone: $ISAACSIM_DIR" >&2
    exit 1
fi

export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
# 避免 conda 环境污染 Isaac Sim 自带的 Python 3.12
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE VIRTUAL_ENV
# Ubuntu 26.04 只提供 libxml2.so.16；Isaac Sim 6.0.1 的部分扩展需要旧 ABI
# 新版 Ubuntu（26.04 等）需要这套旧 ABI 库（libxml2/icu74）；22.04/24.04 自带、无需也兼容。
# 只在**确实有库文件**时才加（打包给别人时可能是空目录/死链，加了也不该报错）
if [ -f "$REPO/reproduce/compat_libs_isaac6/libxml2.so.2" ]; then
    export LD_LIBRARY_PATH="$REPO/reproduce/compat_libs_isaac6:${LD_LIBRARY_PATH:-}"
fi

cd "$REPO"
echo "[open] 用 Isaac Sim 6.0.1 打开厨房场景 ..."
exec "$ISAACSIM_DIR/python.sh" reproduce/open_scene_gui.py "$@"
