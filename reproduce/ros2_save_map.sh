#!/usr/bin/env bash
# ⚠️ 旧入口（保留可用）。推荐等价的 `ros2 launch scripts/nav2_config/ops/save_map.launch.py`
#    （走 slam_toolbox 自带服务，不用猜 QoS）。对照见 ops/README.md §5。
# 【容器侧 · 存图】把 slam_toolbox 建好的地图存成 .pgm + .yaml
#
# 用法（建图进程还在跑的时候执行）:
#   bash /work/reproduce/ros2_save_map.sh [名字]      # 默认 kitchen
# 输出: scripts/nav2_config/maps/<名字>.pgm / .yaml
set -uo pipefail

NAME="${1:-kitchen}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT_DIR="$REPO/scripts/nav2_config/maps"
mkdir -p "$OUT_DIR"

command -v ros2 >/dev/null 2>&1 || { echo "[ERROR] 先 source /opt/ros/humble/setup.bash" >&2; exit 1; }
ros2 topic list 2>/dev/null | grep -qx "/map" || {
    echo "[ERROR] 没看到 /map 话题 —— slam_toolbox 起了吗？机器人走过一圈了吗？" >&2; exit 1; }

echo "[map] 保存地图到 $OUT_DIR/$NAME.{pgm,yaml} ..."
ros2 run nav2_map_server map_saver_cli -f "$OUT_DIR/$NAME" --ros-args -p use_sim_time:=false 2>&1 | tail -6

for f in "$OUT_DIR/$NAME.pgm" "$OUT_DIR/$NAME.yaml"; do
    [ -s "$f" ] && echo "  ✅ $f ($(stat -c%s "$f") 字节)" || echo "  ❌ $f 缺失/为空"
done
if [ -s "$OUT_DIR/$NAME.pgm" ]; then
    echo "[map] 地图统计（像素值：0=障碍 205=未知 254=空闲）:"
    python3 - "$OUT_DIR/$NAME.pgm" <<'PY'
import sys, collections
with open(sys.argv[1], "rb") as f:
    data = f.read()
# 跳过 PGM 头（P5 + 空白分隔的 3 个数字）
toks, i = [], 0
while len(toks) < 4:
    while data[i:i+1].isspace(): i += 1
    if data[i:i+1] == b"#":
        while data[i:i+1] not in (b"\n", b""): i += 1
        continue
    j = i
    while not data[j:j+1].isspace(): j += 1
    toks.append(data[i:j]); i = j
w, h = int(toks[1]), int(toks[2])
px = data[i+1:]
cnt = collections.Counter(px)
occ = sum(v for k, v in cnt.items() if k < 100)
free = sum(v for k, v in cnt.items() if k > 240)
unk = sum(v for k, v in cnt.items() if 100 <= k <= 240)
print(f"      尺寸 {w}x{h} = {w*h} 像素 | 障碍 {occ} ({occ/max(w*h,1)*100:.1f}%) "
      f"空闲 {free} ({free/max(w*h,1)*100:.1f}%) 未知 {unk}")
PY
fi
