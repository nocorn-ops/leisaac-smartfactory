#!/usr/bin/env bash
# 打包"竞赛精简包" —— 生成一个可以直接发给赛队的 zip。
#
#   bash reproduce/make_release_zip.sh                 # 精简包（约 200 MB）
#   bash reproduce/make_release_zip.sh --with-kitchen  # 额外带上厨房场景（+104 MB）
#   bash reproduce/make_release_zip.sh --with-checkpoint   # 额外带上 ACT checkpoint（+197 MB）
#   bash reproduce/make_release_zip.sh --verify        # 打完后**解压到临时目录跑一遍自检**
#   bash reproduce/make_release_zip.sh --out /tmp/x.zip
#   bash reproduce/make_release_zip.sh --stage ../leisaac_release   # **只落盘不打包**（直接跑这个目录）
#
# 包内结构（解压得到一个 leisaac/ 目录，路径与仓库一致，**不需要改任何源码**）:
#   leisaac/README.md            ← ★ 队伍上手（**原生 ROS2 Humble 版**：建图/导航/遥操/数采/训练/推理）
#                                  = 仓库根的 `README.md` 本身，**不需要改名/拷贝**
#                                  开发机自己那份容器版指南 `README1.md` **不进包**
#   leisaac/{source,scripts,reproduce,dependencies/IsaacLab,...}
#   leisaac/assets/scenes/smart_factory/    场地 + 货物（文本 USD）
#   leisaac/assets/robots/so101_follower.usd
#   leisaac/lerobot_robot_1/                躯干模型
#   leisaac/datasets|outputs/               空目录（放录制/训练产物）
#
# ★ 排除规则**必须锚定**（`/datasets/` 而不是 `datasets`）：
#   仓库里有 `source/leisaac/leisaac/enhance/datasets/`，不锚定会把它一起排掉，
#   解压后直接 `ModuleNotFoundError: leisaac.enhance.datasets`（实测踩过）。
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT="${REPO%/}/../leisaac_sim_release.zip"
WITH_KITCHEN=0
WITH_CHECKPOINT=0
VERIFY=0
STAGE_OUT=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --with-kitchen)    WITH_KITCHEN=1; shift ;;
        --with-checkpoint) WITH_CHECKPOINT=1; shift ;;
        --verify)          VERIFY=1; shift ;;
        --out)             OUT="$2"; shift 2 ;;
        --stage)           STAGE_OUT="$2"; shift 2 ;;
        -h|--help)         sed -n '2,24p' "$0"; exit 0 ;;
        *)                 echo "[ERROR] 未知参数 $1（-h 看用法）" >&2; exit 2 ;;
    esac
done
if [ -n "$STAGE_OUT" ]; then
    # --stage：直接落到指定目录（**不打包**），方便就地跑
    PKG="$(cd "$(dirname "$STAGE_OUT")" && pwd)/$(basename "$STAGE_OUT")"
    STAGE="$(dirname "$PKG")"
    if [ -e "$PKG" ]; then
        echo "[ERROR] $PKG 已存在；先移走或换名字（本脚本不会覆盖）" >&2; exit 2
    fi
    OUT=""
    trap - EXIT
else
    OUT="$(cd "$(dirname "$OUT")" 2>/dev/null && pwd)/$(basename "$OUT")" || { echo "[ERROR] 输出目录不存在" >&2; exit 2; }
    STAGE="$(mktemp -d /tmp/leisaac_release.XXXXXX)"
    PKG="$STAGE/leisaac"
    trap 'rm -rf "$STAGE"' EXIT
fi

echo "[pack] 组装到 $PKG ..."

# ── 1) 代码与脚本（用 rsync 锚定排除，把数据集/环境/缓存/用不到的资产挡掉）──
EXCLUDES=(
    --exclude='/.git/'
    --exclude='/datasets/*'
    --exclude='/outputs/*'
    --exclude='/leisaac_env/'
    --exclude='/.pip-cache/'
    --exclude='/.tmp/'
    --exclude='/logs/*'
    --exclude='/docs_archive/'          # 过时文档归档：4.4 MB，队伍不需要
    --exclude='__pycache__/'
    --exclude='*.pyc'
    --exclude='/reproduce/out/'
    --exclude='/reproduce/*.log'
    --exclude='/assets/lerobot_robot(1)/'
    --exclude='/assets/lerobot_robot_no_arms/'
    --exclude='/assets/scenes/lightwheel_*'
    --exclude='/assets/scenes/kitchen_with_cloth.usd'
    --exclude='/assets/scenes/kitchen_with_orange.zip'
    --exclude='/assets/scenes/table_with_cube/'
    --exclude='/assets/robots/lekiwi.usd'
    --exclude='/assets/robots/SubUSDs/'
    --exclude='/assets/robots/SubUSDs_body/'
    --exclude='/assets/robots/configuration/'
    --exclude='/assets/robots/lerobot_robot_usdz/'
    --exclude='/assets/robots/test/'
    # ★ 必须排除：`dependencies/IsaacLab/_isaac_sim` 是**指向本机 Isaac Sim 安装（38 GB）的软链**，
    #   `zip` 默认**跟随软链接**，不排除就会把整个 Isaac Sim 打进包里（实测：5 分钟写了 3.9 GB
    #   还没完，最后会是个几十 GB 的废包）。赛队按 INSTALL.md 自己建这个软链即可。
    --exclude='/dependencies/IsaacLab/_isaac_sim'
    --exclude='/CLAUDE1.md'          # AI 写作风格人设文档，与赛队无关
    --exclude='/.claude/'            # 本机 agent 配置
    --exclude='/last'                # 历史遗留的坏软链
    # ★ 躯干模型有**三份**，只有 `lerobot_robot_1/` 被出货任务用到：
    #   smart_factory / lerobot_kitchen 都用 `lerobot_robot_1/SubUSDs/lerobot_robot_no_arms_base.usd`
    #   （自包含 USD crate，`strings` 里没有任何 @外部引用@）。另两份（各 86 MB）没被引用，
    #   排除后包小 172 MB（实测 --verify 仍然全绿）。
    --exclude='/lerobot_robot/'
    --exclude='/lerobot_robot.usdz'
    --exclude='/README1.md'          # 开发机（宿主机+Docker ROS2）版指南；包里只有一个 README.md（原生 Humble）
)
if [[ "$WITH_KITCHEN" = "0" ]]; then
    EXCLUDES+=(--exclude='/assets/scenes/kitchen_with_orange/')
fi
mkdir -p "$PKG"
rsync -a "${EXCLUDES[@]}" "$REPO/" "$PKG/"

# compat_libs_isaac6 在本机是**指向 /snap/... 的符号链接** → 拷成实体文件，
# 否则发给别人就是一堆死链（Ubuntu 26.04 的机器需要它；22.04/24.04 用不到但不碍事）
if [ -d "$REPO/reproduce/compat_libs_isaac6" ]; then
    rm -rf "$PKG/reproduce/compat_libs_isaac6"
    rsync -aL "$REPO/reproduce/compat_libs_isaac6/" "$PKG/reproduce/compat_libs_isaac6/" 2>/dev/null \
        && echo "[pack] compat_libs_isaac6 已按实体文件打包（$(du -sh "$PKG/reproduce/compat_libs_isaac6" | cut -f1)）"
fi

# ── 2) 空目录（放录制/训练产物，避免脚本运行时才创建）─────────────────────
for d in datasets outputs logs; do mkdir -p "$PKG/$d"; touch "$PKG/$d/.gitkeep"; done

# ── 3) 合规文件 + 上层文档（仓库里被删掉的 LICENSE 从 git 取回；HANDOFF 在上一层）──
( cd "$REPO" && git show HEAD:LICENSE > "$PKG/LICENSE" 2>/dev/null ) \
    || echo "[pack] ⚠️ 没能从 git 取回 LICENSE（请手动放一份授权说明）"
for f in "$REPO/../HANDOFF.md" "$REPO/../智慧工厂任务挑战赛（线下）.docx" "$REPO/../环境依赖查证_Ubuntu22.04_Humble.md"; do
    [ -f "$f" ] && cp -a "$f" "$PKG/" && echo "[pack] + $(basename "$f")"
done

# ── 4) 可选：ACT checkpoint（开箱能推理）───────────────────────────────────
if [[ "$WITH_CHECKPOINT" = "1" ]]; then
    CKPT="$REPO/outputs/train/kitchen_biarm_act_300k/checkpoints/300000/pretrained_model"
    if [ -d "$CKPT" ]; then
        mkdir -p "$PKG/outputs/train/kitchen_biarm_act_300k/checkpoints/300000"
        rsync -a "$CKPT" "$PKG/outputs/train/kitchen_biarm_act_300k/checkpoints/300000/"
        echo "[pack] + ACT checkpoint（pretrained_model）"
    else
        echo "[pack] ⚠️ 找不到 checkpoint: $CKPT"
    fi
fi

# ── 4.5) 打包前自检：暂存目录不该有超大内容（软链被跟随的典型症状）─────────
_BIG="$(du -sm "$PKG" 2>/dev/null | cut -f1)"
if [ "${_BIG:-0}" -gt 1200 ]; then
    echo "[pack] ⚠️ 暂存目录 ${_BIG} MB —— 远超预期（~400MB），多半又有软链被跟随了："
    du -sh "$PKG"/* 2>/dev/null | sort -rh | head -6 | sed 's/^/         /'
    echo "[pack]    检查 dependencies/IsaacLab/_isaac_sim 之类的软链是否被排除"
fi

# ── 5) 打 zip（--stage 模式跳过）───────────────────────────────────────────
if [ -z "$OUT" ]; then
    echo
    echo "════════ 已落盘（未打包）════════"
    du -sh "$PKG"/* 2>/dev/null | sort -rh | sed 's/^/  /'
    echo
    echo "  目录: $PKG"
    echo "  大小: $(du -sh "$PKG" | cut -f1)"
    echo
    echo "跑法（PYTHONPATH 由 reproduce/leisaac_env.sh 自动加，普通脚本直接跑就行）："
    echo "  cd $PKG"
    echo "  bash reproduce/ros2_chassis_teleop.sh --headless --max_seconds 300 --lidar_stop_distance 0.45"
    exit 0
fi
rm -f "$OUT"
# `-y` = 把符号链接**按链接存**，不要跟随它指向的内容（第二层防护，见上面的 _isaac_sim 注释）
( cd "$STAGE" && zip -q -r -y "$OUT" leisaac ) || { echo "[ERROR] zip 失败" >&2; exit 1; }

echo
echo "════════ 包内容（顶层）════════"
du -sh "$PKG"/* 2>/dev/null | sort -rh | sed 's/^/  /'
echo
echo "════════ 产物 ════════"
printf "  %s\n" "$OUT"
printf "  大小: %s\n" "$(du -h "$OUT" | cut -f1)"
printf "  文件数: %s\n" "$(unzip -l "$OUT" | tail -1 | awk '{print $2}')"

# ── 6) 可选：解压到临时目录真跑一遍（"换台机器能跑吗"的硬证据）────────────
if [[ "$VERIFY" = "1" ]]; then
    VT="$(mktemp -d /tmp/leisaac_verify.XXXXXX)"
    echo
    echo "[verify] 解压到 $VT 并跑自检 ..."
    unzip -q "$OUT" -d "$VT"
    ISAACSIM_DIR="${ISAACSIM_DIR:-}" \
    PYTHONPATH="$VT/leisaac/source/leisaac" \
    OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
    LD_LIBRARY_PATH="$VT/leisaac/reproduce/compat_libs_isaac6" \
    ISAAC_LAB_ENABLE_ISAAC_RTX_PER_ENV_SCENE_PARTITION=0 \
        timeout 600 "${ISAACSIM_DIR:-/home/vedal/WorkStation/isaac-sim-6.0.1}/python.sh" -u \
        "$VT/leisaac/reproduce/verify_smart_factory_robot.py" --steps 20 2>&1 \
        | grep -E "①|②|③|④|✅|❌|命中距离|底盘世界位置|结论|Traceback|Error" | tail -14
    echo "[verify] 临时目录保留在 $VT（确认完可删）"
    # ★ 这里以前直接 `trap - EXIT`，导致**打包暂存目录**（~400MB）被留在 /tmp 里（实测留了好几个）。
    #   自检目录故意保留给人看，但暂存目录必须清掉。
    rm -rf "$STAGE"
    trap - EXIT
fi
