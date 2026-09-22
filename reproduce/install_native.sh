#!/usr/bin/env bash
# ⚠️ 已过时（历史脚本）：这是 **Isaac Sim 5.1.0 + IsaacLab 2.3.0** 时代的 conda+pip 安装路线，
#   现在跑它**装不出可用栈**（本仓库已切到 6.0.1 standalone + IsaacLab 3.0.0）。
#   保留仅作参考。当前安装步骤见 ../HANDOFF.md §2 与 ../环境依赖查证_Ubuntu22.04_Humble.md。
# LeIsaac 原生安装脚本（Ubuntu 22.04 / 24.04 / 26.04 均可；路径自动探测，不用改源码）
# 按已归档的 pip 路线：
#   Miniconda -> conda env leisaac (py3.11) -> isaacsim[all,extscache]==5.1.0 -> torch 2.7.0 cu128
#   -> isaaclab 2.3.0 (源码可编辑) -> leisaac (源码可编辑)
set -uo pipefail

# 仓库目录 = 本脚本所在目录的上一级（跟着压缩包解压到哪就在哪）
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# miniconda 目录：优先环境变量/已有安装，否则装到 $HOME/miniconda3
source "$REPO/reproduce/leisaac_env.sh"
ENV_DIR="$CONDA_DIR/envs/leisaac"
PY="$ENV_DIR/bin/python"

# 大文件临时区与 pip 缓存（放工作区，可断点复用、避免 tmpfs 16G 溢出）
export TMPDIR="$REPO/.tmp"
export PIP_CACHE_DIR="$REPO/.pip-cache"
mkdir -p "$TMPDIR" "$PIP_CACHE_DIR"

step() { echo; echo "############################################################"; echo "===== $* @ $(date '+%H:%M:%S') ====="; echo "############################################################"; }

step "STEP 1/7 安装 Miniconda -> $CONDA_DIR"
if [ -d "$CONDA_DIR" ] && [ -z "$(ls -A "$CONDA_DIR" 2>/dev/null)" ]; then
    echo "[info] $CONDA_DIR 是空目录，清掉以避免安装器报错"
    rmdir "$CONDA_DIR"
fi
if [ -x "$CONDA_DIR/bin/conda" ]; then
    echo "[skip] conda 已存在"
else
    wget -q --show-progress -O "$TMPDIR/miniconda.sh" https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
    bash "$TMPDIR/miniconda.sh" -b -p "$CONDA_DIR"
fi
"$CONDA_DIR/bin/conda" --version

step "STEP 2/7 创建 conda 环境 leisaac (python=3.11)"
# 加大超时，规避弱网导致的读超时
"$CONDA_DIR/bin/conda" config --set remote_connect_timeout_secs 60
"$CONDA_DIR/bin/conda" config --set remote_read_timeout_secs 600
if "$CONDA_DIR/bin/conda" env list | grep -qw leisaac; then
    echo "[skip] 环境 leisaac 已存在"
else
    for attempt in $(seq 1 12); do
        echo "----- conda create 尝试 $attempt/12 @ $(date '+%H:%M:%S') -----"
        if "$CONDA_DIR/bin/conda" create -y -n leisaac -c conda-forge --override-channels python=3.11 pip; then
            echo "[ok] conda env leisaac 创建成功"
            break
        fi
        echo "[warn] 尝试 $attempt 失败，5 秒后重试（已下完的包会缓存复用）"
        sleep 5
    done
fi
[ -x "$ENV_DIR/bin/python" ] || { echo "[FATAL] leisaac 环境仍无 python，退出"; exit 1; }
"$PY" --version
"$PY" -m pip install --upgrade pip setuptools wheel

step "STEP 3/7 安装 Isaac Sim 5.1.0（大下载，约15-20GB，来源 pypi.nvidia.com）"
"$PY" -m pip install --retries 10 --timeout 300 \
    "isaacsim[all,extscache]==5.1.0" \
    --extra-index-url https://pypi.nvidia.com

step "STEP 4/7 安装 PyTorch 2.7.0 (CUDA 12.8)"
"$PY" -m pip install --retries 10 --timeout 300 \
    -U torch==2.7.0 torchvision==0.22.0 \
    --index-url https://download.pytorch.org/whl/cu128

step "STEP 5/7 安装 IsaacLab 2.3.0 扩展（源码可编辑）"
"$PY" -m pip install --retries 10 -e "$REPO/dependencies/IsaacLab/source/isaaclab"
"$PY" -m pip install --retries 10 -e "$REPO/dependencies/IsaacLab/source/isaaclab_assets"
"$PY" -m pip install --retries 10 -e "$REPO/dependencies/IsaacLab/source/isaaclab_tasks"

step "STEP 6/7 安装 LeIsaac（源码可编辑）"
"$PY" -m pip install --retries 10 -e "$REPO/source/leisaac"

step "STEP 7/7 校验导入（仅包级，GPU 需在宿主机验证）"
"$PY" -c "import isaacsim; print('isaacsim import OK')" 2>&1 | tail -3 || echo "isaacsim import FAILED"
"$PY" -c "import torch; print('torch', torch.__version__)" 2>&1 | tail -3 || echo "torch import FAILED"
"$PY" -c "import isaaclab; print('isaaclab', isaaclab.__version__)" 2>&1 | tail -3 || echo "isaaclab import FAILED"
"$PY" -c "import leisaac; print('leisaac import OK')" 2>&1 | tail -3 || echo "leisaac import FAILED"

echo; echo "################# INSTALL FINISHED #################"
