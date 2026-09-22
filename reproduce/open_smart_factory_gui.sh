#!/usr/bin/env bash
# 在 Isaac Sim GUI 里打开**智慧工厂场地**（可编辑）。
#
#   bash reproduce/open_smart_factory_gui.sh            # 没有 USD 就先自动导出，然后开窗口
#   bash reproduce/open_smart_factory_gui.sh --regen    # 强制从 Python 布局重新导出（会覆盖你的手工修改！）
#   bash reproduce/open_smart_factory_gui.sh --seconds 60   # 60 秒后自动关窗（无人值守测试）
#
# 打开的是 assets/scenes/smart_factory/smart_factory.usda（**文本 USD**）：
#   · 随便拖/加/删，Ctrl+S 保存 → 仿真任务加载的就是你改过的版本
#   · 想加碰撞体：选中 prim → Property 面板右下 Add → Physics → Collision
#   · 新加的东西请放在 /Arena 下面（未来激光雷达按这个子树扫，任务也按它算场景）
#   · 单位：1 = 1 米；场地坐标 X∈[0,4]、Y∈[0,3]、Z 向上
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 路径解析：自动探测 Isaac Sim / lerobot 环境；要覆盖就 export ISAACSIM_DIR 或写 reproduce/local.env
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/leisaac_env.sh"
leisaac_require_isaacsim || exit 1
USD="$REPO/assets/scenes/smart_factory/smart_factory.usda"
REGEN=0
ARGS=()

for a in "$@"; do
    case "$a" in
        --regen) REGEN=1 ;;
        *) ARGS+=("$a") ;;
    esac
done

[ -x "$ISAACSIM_DIR/python.sh" ] || { echo "[ERROR] 找不到 Isaac Sim: $ISAACSIM_DIR" >&2; exit 1; }

export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE VIRTUAL_ENV || true
# Ubuntu 26.04 只有新 ABI，Isaac Sim 6.0.1 的部分扩展要旧 ABI（libxml2.so.2 / libicu 74）
# 新版 Ubuntu（26.04 等）需要这套旧 ABI 库（libxml2/icu74）；22.04/24.04 自带、无需也兼容。
# 只在**确实有库文件**时才加（打包给别人时可能是空目录/死链，加了也不该报错）
if [ -f "$REPO/reproduce/compat_libs_isaac6/libxml2.so.2" ]; then
    export LD_LIBRARY_PATH="$REPO/reproduce/compat_libs_isaac6:${LD_LIBRARY_PATH:-}"
fi

cd "$REPO"

if [ "$REGEN" = "1" ] || [ ! -f "$USD" ]; then
    if [ "$REGEN" = "1" ]; then
        echo "[gui] --regen：从 Python 布局重新导出（会覆盖 USD 里的手工修改）"
    else
        echo "[gui] 还没有场地 USD，先导出一次 ..."
    fi
    "$ISAACSIM_DIR/python.sh" -u reproduce/export_arena_usd.py
fi

echo "[gui] 打开 $USD（在窗口里改完按 Ctrl+S 保存）"
# 初始视角：斜俯视全场（X∈[0,4] Y∈[0,3] 的中心）
exec "$ISAACSIM_DIR/python.sh" -u reproduce/open_scene_gui.py \
    --usd "$USD" \
    --eye 5.6 -2.6 2.8 --lookat 2.0 1.5 0.35 \
    ${ARGS[@]+"${ARGS[@]}"}
