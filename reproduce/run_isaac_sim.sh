#!/usr/bin/env bash
# ============================================================================
# run_isaac_sim.sh — 用 docker + GPU CDI 启动 Isaac Sim 5.1.0 容器跑 LeIsaac
#
# 特性:
#   * 仓库以"绝对路径"挂载到容器内同一路径 —— 与代码中 REPO_ROOT 解析一致，
#     原生 conda 跑法和容器跑法通用
#   * 自动 pip install -e source/leisaac 与 IsaacLab(--no-deps, 幂等, 可跳过)
#   * GPU: --device nvidia.com/gpu=all（CDI）
#   * 缓存目录用 docker named volume 持久化
#
# 用法:
#   reproduce/run_isaac_sim.sh                 # 默认: 无界面冒烟测试
#   reproduce/run_isaac_sim.sh --gui           # 默认改为 GUI 查看厨房场景
#   reproduce/run_isaac_sim.sh -c 'python scripts/view_task.py --task LeIsaac-SmartFactory-v0'
#   reproduce/run_isaac_sim.sh -c 'python reproduce/smoke_test_env.py --task LeIsaac-SmartFactory-v0 --headless --enable_cameras'
#   reproduce/run_isaac_sim.sh --no-install    # 跳过 editable 安装(首次后加速)
#   reproduce/run_isaac_sim.sh --no-pull       # 镜像缺失时不自动拉取
# ============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${LEISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}"
CONTAINER_NAME="leisaac-sim"
CACHE_ROOT="${LEISAAC_CACHE_ROOT:-leisaac-cache}"

GUI=0; DO_PULL=1; DO_INSTALL=1; USE_X=1
CMD_ARGS=()

usage() {
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--cmd)          shift; CMD_ARGS=("$@"); break ;;
        --gui)             GUI=1 ;;
        --no-pull)         DO_PULL=0 ;;
        --no-install)      DO_INSTALL=0 ;;
        --no-x)            USE_X=0 ;;
        -h|--help)         usage ;;
        *) echo "未知参数: $1" >&2; usage ;;
    esac
    shift
done

echo "===== LeIsaac Isaac Sim 容器 ====="
echo "仓库 : $REPO_DIR"
echo "镜像 : $IMAGE"

# 1) 镜像检查/拉取
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    if [[ $DO_PULL -eq 0 ]]; then
        echo "镜像不存在且 --no-pull 指定，请先手动执行: docker pull $IMAGE" >&2
        exit 1
    fi
    echo "镜像不存在，开始拉取（几十 GB，视带宽可能需要很长时间）..."
    docker pull "$IMAGE"
fi

# 2) 默认命令
#    GUI 模式 → 打开厨房场景窗口(关闭窗口即退出);  无 GUI → headless 冒烟测试
if [[ ${#CMD_ARGS[@]} -eq 0 ]]; then
    if [[ $GUI -eq 1 ]]; then
        CMD_ARGS=(python scripts/view_task.py --task LeIsaac-SmartFactory-v0)
    else
        CMD_ARGS=(python reproduce/smoke_test_env.py --task LeIsaac-SmartFactory-v0 --headless --steps 60)
    fi
fi

# 3) X11（可选，--no-x 关闭；宿主机有 /tmp/.X11-unix 时挂载才有效）
X_ARGS=()
if [[ $USE_X -eq 1 ]]; then
    X_ARGS=(-v /tmp/.X11-unix:/tmp/.X11-unix -e "DISPLAY=${DISPLAY:-:0}")
    echo "X11: 挂载 /tmp/.X11-unix, DISPLAY=${DISPLAY:-:0}  (若黑屏/报错: 在宿主机执行 xhost +local:docker)"
else
    echo "X11: 已关闭(--no-x)"
fi

# 4) 容器内一次性 editable 安装
INSTALL_CMD=""
if [[ $DO_INSTALL -eq 1 ]]; then
    INSTALL_CMD="pip install -q -e source/leisaac && pip install -q -e dependencies/IsaacLab/source/isaaclab --no-deps && "
fi

echo "命令 : ${CMD_ARGS[*]}"
echo
echo ">>> 启动容器（首次含 editable 安装，稍等）..."
# shellcheck disable=SC2086
exec docker run --rm -it \
    --name "$CONTAINER_NAME" \
    --network=host \
    --device nvidia.com/gpu=all \
    -e ACCEPT_EULA=Y \
    -e PRIVACY_CONSENT=Y \
    "${X_ARGS[@]}" \
    -v "$REPO_DIR:$REPO_DIR" \
    -v "$CACHE_ROOT-ov:/root/.cache/ov" \
    -v "$CACHE_ROOT-kit:/root/.cache/kit" \
    -v "$CACHE_ROOT-isaacsim:/root/.cache/isaac-sim" \
    -v "$CACHE_ROOT-hf:/root/.cache/huggingface" \
    -w "$REPO_DIR" \
    "$IMAGE" \
    bash -lc "${INSTALL_CMD}exec \"\$@\"" _ "${CMD_ARGS[@]}"
