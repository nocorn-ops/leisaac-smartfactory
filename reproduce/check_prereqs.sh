#!/usr/bin/env bash
# ============================================================================
# check_prereqs.sh — 复现 LeIsaac 前检查本机条件（docker / GPU / 镜像 / 网络 / 磁盘）
# 用法:  reproduce/check_prereqs.sh [--probe]
#       --probe  额外做一次真实的 docker GPU 探测（docker run nvidia-smi）
# ============================================================================
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${LEISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}"
PROBE_IMAGE="${LEISAAC_PROBE_IMAGE:-ros2-humble-dev:latest}"   # 本机已有的轻量镜像，用于 GPU 探测
PROBE=0
[[ "${1:-}" == "--probe" ]] && PROBE=1

PASS=0; FAIL=0; WARN=0
say()  { printf '\033[1;32m[PASS]\033[0m %s\n' "$1"; PASS=$((PASS+1)); }
fail() { printf '\033[1;31m[FAIL]\033[0m %s\n' "$1"; FAIL=$((FAIL+1)); }
warn() { printf '\033[1;33m[WARN]\033[0m %s\n' "$1"; WARN=$((WARN+1)); }

echo "===== LeIsaac 复现前置检查 ====="
echo "仓库目录: $REPO_DIR"
echo "Isaac Sim 镜像: $IMAGE"
echo

# --- docker 可用 ---
if command -v docker >/dev/null 2>&1; then
    say "已找到 docker: $(docker --version 2>/dev/null | cut -d' ' -f1-3)"
else
    fail "未找到 docker 命令"; docker=0
fi
if ! docker info >/dev/null 2>&1; then
    fail "无法连接 docker daemon（docker info 失败）——需要可用的 docker 服务"
else
    say "docker daemon 可连接"
fi

# --- GPU（CDI 设备）---
if command -v nvidia-ctk >/dev/null 2>&1 && nvidia-ctk cdi list 2>/dev/null | grep -q 'nvidia.com/gpu'; then
    say "NVIDIA CDI 设备可用: $(nvidia-ctk cdi list 2>/dev/null | tr '\n' ' ')"
else
    warn "未发现 nvidia-ctk 或 CDI 设备列表为空（GPU 容器需要 CDI 或 nvidia runtime）"
fi

# --- 真实 GPU 探测（可选）---
if [[ $PROBE -eq 1 ]]; then
    if docker image inspect "$PROBE_IMAGE" >/dev/null 2>&1; then
        echo "--- 用 $PROBE_IMAGE 做 GPU 探测 ---"
        if docker run --rm --device nvidia.com/gpu=all \
               -v /usr/bin/nvidia-smi:/usr/local/bin/nvidia-smi:ro \
               "$PROBE_IMAGE" nvidia-smi >/dev/null 2>&1; then
            say "GPU 容器内 nvidia-smi 运行成功（显卡可用）"
        else
            fail "GPU 容器内 nvidia-smi 失败——检查 CDI 与驱动"
        fi
    else
        warn "本地无 $PROBE_IMAGE，跳过容器内 nvidia-smi 探测（可用 --probe 前置拉取）"
    fi
fi

# --- Isaac Sim 镜像 ---
if docker image inspect "$IMAGE" >/dev/null 2>&1; then
    say "Isaac Sim 镜像已在本地: $(docker image inspect "$IMAGE" --format '{{.Size}}' | awk '{printf "%.1f GB", $1/1e9}')"
else
    warn "Isaac Sim 镜像尚未拉取。下一步:"
    echo "      docker pull $IMAGE"
    echo "    （几十 GB，耗时取决于带宽，可后台执行）"
fi

# --- nvcr.io 网络可达性 ---
if timeout 60 docker manifest inspect "$IMAGE" >/dev/null 2>&1; then
    say "nvcr.io 可访问，镜像清单存在（linux/amd64 可用）"
else
    fail "无法访问 nvcr.io 镜像清单——检查网络/代理（注意 registry 加速镜像仅对 Docker Hub 生效）"
fi

# --- 磁盘 ---
FREE_GB=$(df -BG --output=avail / | tail -1 | tr -dc '0-9')
if [[ ${FREE_GB:-0} -ge 150 ]]; then
    say "磁盘可用空间充足: ${FREE_GB} GB（>150GB）"
else
    fail "磁盘可用空间不足: ${FREE_GB} GB（Isaac Sim 镜像 + 缓存建议 ≥150GB）"
fi

# --- 仓库内残留旧绝对路径检查（运行时致命的两处已修复，这里做体检）---
LEFT=$(grep -rl --include='*.py' '/home/anno' "$REPO_DIR/source" "$REPO_DIR/scripts" 2>/dev/null || true)
if [[ -z "$LEFT" ]]; then
    say "source/scripts 下已无 /home/anno 绝对路径残留"
else
    warn "以下 .py 仍引用 /home/anno（非主任务/辅助脚本，如需运行请自行替换）:"
    echo "$LEFT" | sed 's/^/      /'
fi

echo
echo "===== 结果: PASS=$PASS FAIL=$FAIL WARN=$WARN ====="
echo "失败项为硬性条件，需解决后才能跑 Isaac Sim 容器。"
[[ $FAIL -eq 0 ]]
