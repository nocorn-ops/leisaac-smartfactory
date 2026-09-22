#!/usr/bin/env bash
# ============================================================================
# run_policy_server.sh — 启动 ACT 策略推理服务（lerobot 0.4.2 环境, 与 Isaac Sim 分离）
#
# 默认以 "TCP 直连" 模式启动仓库自带的 act_action_server.py(CPU),
# 供 Isaac Sim 容器内 policy_inference.py --policy_type local-act 连接。
# 两个容器都用 --network=host，因此 127.0.0.1:<port> 互通。
#
# 用法:
#   reproduce/run_policy_server.sh                # 默认: CPU 跑现成 300k checkpoint, 端口 5556
#   reproduce/run_policy_server.sh --device cuda  # 有 GPU 时用 cuda
#   reproduce/run_policy_server.sh python -m lerobot.async_inference.policy_server --host=127.0.0.1 --port=5555 --fps=30
#                                                 # 自定义命令（gRPC 模式示例）
# ============================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${LEISAAC_POLICY_IMAGE:-leisaac-policy:lerobot0.4.2}"
DOCKERFILE_DIR="$REPO_DIR/reproduce"
CKPT="$REPO_DIR/outputs/train/kitchen_biarm_act_300k/checkpoints/300000/pretrained_model"

# 1) 镜像不存在则构建
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "构建策略镜像 $IMAGE (首次会下载 torch/lerobot, 约 1-3 GB)..."
    docker build -t "$IMAGE" -f "$DOCKERFILE_DIR/Dockerfile.policy" "$DOCKERFILE_DIR"
fi

# 2) 默认命令（TCP 直连, CPU 推理）
CMD_ARGS=("$@")
if [[ ${#CMD_ARGS[@]} -eq 0 ]]; then
    if [[ ! -f "$CKPT/model.safetensors" ]]; then
        echo "未找到现成 checkpoint: $CKPT" >&2
        echo "请先确认模型文件存在，或改用 -c 指定其他命令。" >&2
        exit 1
    fi
    echo "使用现成 300k checkpoint (CPU 推理, 端口 5556):"
    CMD_ARGS=(python scripts/evaluation/act_action_server.py \
        --checkpoint_path "$CKPT" --host 127.0.0.1 --port 5556 --device cpu)
fi

echo "命令 : ${CMD_ARGS[*]}"
echo ">>> 启动策略容器 ..."
exec docker run --rm -it \
    --name leisaac-policy \
    --network=host \
    -v "$REPO_DIR:$REPO_DIR" \
    -v leisaac-cache-hf:/root/.cache/huggingface \
    -w "$REPO_DIR" \
    "$IMAGE" \
    bash -lc 'exec "$@"' _ "${CMD_ARGS[@]}"
