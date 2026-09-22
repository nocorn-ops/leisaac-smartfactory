#!/usr/bin/env bash
# LeIsaac 环境冒烟测试（headless，验证环境能否构建任务并稳定跑 N 步）
# 用法（宿主机终端，需 NVIDIA 显卡；无需图形界面）:
#   bash reproduce/verify_env.sh               # 60 步，无相机
#   bash reproduce/verify_env.sh --steps 200   # 自定义步数
#   bash reproduce/verify_env.sh --cameras     # 连相机一起验证
# 退出码 0 = 环境复现成功。
#
# ★ 2026-09 重写：原来走的是已废弃的 conda `leisaac` 环境 + 写死 Ubuntu 26.04 的
#   /snap/mesa-2404 路径（在 22.04 上必坏）。现在统一走 `leisaac_env.sh` 自动探测
#   Isaac Sim 6.0.1 standalone，并用它自带的 python.sh。
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 路径解析：自动探测 Isaac Sim；要覆盖就 export ISAACSIM_DIR 或写 reproduce/local.env
source "$REPO/reproduce/leisaac_env.sh"
leisaac_require_isaacsim || exit 1

export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
# IsaacLab 3.0 不允许在虚拟环境里用下载版 Isaac Sim
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE VIRTUAL_ENV || true
# 旧 ABI 兼容库：**只在库文件真实存在时**才加（Ubuntu 26.04 需要 libxml2.so.2/icu74；
# 22.04/24.04 自带，这里会直接跳过，不会把悬空软链塞进 LD_LIBRARY_PATH）
if [ -f "$REPO/reproduce/compat_libs_isaac6/libxml2.so.2" ]; then
    export LD_LIBRARY_PATH="$REPO/reproduce/compat_libs_isaac6:${LD_LIBRARY_PATH:-}"
fi

cd "$REPO"
echo "[verify] 冒烟测试 LeIsaac-SmartFactory-v0 (headless)"
echo "[verify] Isaac Sim: $ISAACSIM_DIR"
# 注意：smoke_test_env.py 自己固定 headless；IsaacLab 3.0 起 --headless/--enable_cameras
# 不再是 CLI 参数（传了会报 unrecognized arguments），相机用 --cameras。
exec "$ISAACSIM_DIR/python.sh" -u reproduce/smoke_test_env.py \
    --task LeIsaac-SmartFactory-v0 "$@"
