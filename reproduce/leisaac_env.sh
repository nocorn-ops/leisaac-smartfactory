#!/usr/bin/env bash
# LeIsaac 路径解析 —— **换台机器 / 换目录都不用改源码**。
#
# 用法（在别的脚本里第一行 source 它）:
#     source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/leisaac_env.sh"
#
# 它会导出:
#     REPO         仓库根目录（本文件所在目录的上一级，**自动算出来**）
#     ISAACSIM_DIR Isaac Sim 安装目录（含 python.sh）
#     ISAAC_PY     "$ISAACSIM_DIR/python.sh"
#     LR_PY        lerobot 环境的 python（装了 lerobot 的那个解释器）
#     CONDA_DIR    miniconda 安装目录
#
# 覆盖优先级（从高到低，**不需要动源码**）:
#     ① 直接 export 环境变量：ISAACSIM_DIR=...  LEISAAC_LR_PY=...  CONDA_DIR=...
#     ② 写进 `reproduce/local.env`（模板见 local.env.example，不进 git）
#     ③ 自动探测（下面那串候选路径）
#
# 注意：本文件故意**不** `set -e`，避免影响调用方的错误处理。

# shellcheck disable=SC2034
_leisaac_env_self="${BASH_SOURCE[0]}"
REPO="$(cd "$(dirname "$_leisaac_env_self")/.." && pwd)"
export REPO

# ★ 总是使用**本仓库**里的 leisaac 包：机器上可能装过别处的 editable 版本（例如解压出来的
#   精简包和原仓库同时在），不加这一条会悄悄 import 到另一份代码。放最前面即可覆盖。
export PYTHONPATH="$REPO/source/leisaac${PYTHONPATH:+:$PYTHONPATH}"

# ── ② local.env（机器相关的私人配置，不进 git）─────────────────────────────
if [ -f "$REPO/reproduce/local.env" ]; then
    # shellcheck disable=SC1091
    . "$REPO/reproduce/local.env"
fi

# ── ③ Isaac Sim 自动探测 ───────────────────────────────────────────────────
if [ -z "${ISAACSIM_DIR:-}" ] || [ ! -x "${ISAACSIM_DIR:-}/python.sh" ]; then
    for _c in \
        "$REPO/../isaac-sim-"* \
        "$REPO/../"*/isaac-sim-* \
        "$HOME/isaac-sim-"* \
        "$HOME/WorkStation/isaac-sim-"* \
        "$HOME/Downloads/isaac-sim-"* \
        "$HOME/opt/isaac-sim-"* \
        /opt/isaac-sim-* \
        /isaac-sim-*; do
        if [ -x "$_c/python.sh" ]; then
            ISAACSIM_DIR="$_c"
            break
        fi
    done
fi
export ISAACSIM_DIR="${ISAACSIM_DIR:-}"
ISAAC_PY="${ISAACSIM_DIR:+$ISAACSIM_DIR/python.sh}"
export ISAAC_PY

# ── ③ lerobot 环境（装了 lerobot 的解释器）自动探测 ────────────────────────
if [ -z "${LEISAAC_LR_PY:-}" ]; then
    for _p in \
        "$REPO/.venv-lerobot/bin/python" \
        "${CONDA_PREFIX:-}/bin/python" \
        "$HOME/miniconda3/envs/leisaac-lerobot-sim/bin/python" \
        "$HOME/miniconda3/envs/leisaac-lerobot/bin/python" \
        "$HOME/miniconda3/envs/lerobot/bin/python" \
        "$(command -v python3 2>/dev/null || true)"; do
        if [ -n "$_p" ] && [ -x "$_p" ] && "$_p" -c "import lerobot" >/dev/null 2>&1; then
            LEISAAC_LR_PY="$_p"
            break
        fi
    done
fi
export LEISAAC_LR_PY="${LEISAAC_LR_PY:-}"
LR_PY="${LEISAAC_LR_PY:-python3}"
export LR_PY

# ── ③ miniconda 自动探测（install_native.sh / verify_env.sh 用）────────────
if [ -z "${CONDA_DIR:-}" ]; then
    for _d in "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" "$HOME/mambaforge" /opt/conda; do
        if [ -x "$_d/bin/conda" ]; then
            CONDA_DIR="$_d"
            break
        fi
    done
fi
export CONDA_DIR="${CONDA_DIR:-$HOME/miniconda3}"

# ── 统一的"找不到就报错"小工具（脚本里可以调）──────────────────────────────
leisaac_require_isaacsim() {
    if [ -z "${ISAACSIM_DIR:-}" ] || [ ! -x "$ISAACSIM_DIR/python.sh" ]; then
        cat >&2 <<EOF
[ERROR] 没找到 Isaac Sim（需要含 python.sh 的目录）。
        三种解决方式（**都不用改源码**）：
          1) 临时指定:  ISAACSIM_DIR=/你的/isaac-sim-6.0.1 bash <本脚本>
          2) 写进配置文件: 把下面这行加进 $REPO/reproduce/local.env
                 ISAACSIM_DIR=/你的/isaac-sim-6.0.1
          3) 放到常见位置（$HOME/isaac-sim-*、$REPO/../isaac-sim-*、/opt/isaac-sim-*）会自动找到
EOF
        return 1
    fi
    return 0
}

leisaac_require_lerobot_py() {
    if ! "$LR_PY" -c "import lerobot" >/dev/null 2>&1; then
        cat >&2 <<EOF
[ERROR] 没找到装了 lerobot 的 python（当前 LR_PY=$LR_PY）。
        同样不用改源码：
          · LEISAAC_LR_PY=~/miniconda3/envs/你的环境/bin/python bash <本脚本>
          · 或写进 $REPO/reproduce/local.env 的 LEISAAC_LR_PY=...
          · 环境搭建步骤见 HANDOFF.md §2 / 环境依赖查证_Ubuntu22.04_Humble.md
EOF
        return 1
    fi
    return 0
}

# 打印解析结果（脚本里想显示就调；LEISAAC_VERBOSE=1 时 source 完也会打一行）
leisaac_print_paths() {
    echo "[env] 仓库     REPO=$REPO"
    echo "[env] Isaac Sim ISAACSIM_DIR=${ISAACSIM_DIR:-（未找到）}"
    echo "[env] lerobot   LR_PY=${LEISAAC_LR_PY:-（未找到，将回退 python3）}"
}
if [ "${LEISAAC_VERBOSE:-0}" = "1" ]; then
    leisaac_print_paths
fi
