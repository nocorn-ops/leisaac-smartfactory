#!/usr/bin/env bash
# =============================================================================
# install_all.sh —— 一键装好「除 ROS2 以外」的全部环境与依赖
#
# 装什么：
#   ① 前置自检（系统 / GPU / 驱动 / 磁盘 / 工具）
#   ② Isaac Sim 6.0.1 standalone（用现成的 / 用本地 zip / 自动下载 12 GB）
#   ③ IsaacLab 3.0.0（git submodule + `isaaclab.sh --install`）+ 本项目 `leisaac`（editable）
#   ④ lerobot 环境（conda）：lerobot 0.4.2 + torch 2.7.1+cu128 + h5py + PyAV
#   ⑤ 写 `reproduce/local.env`（把探测到的路径记下来，之后所有脚本自动用）
#   ⑥ 验收：headless 冒烟测试 + lerobot 依赖自检
#
# **不装 ROS2**（Humble 请按 `环境依赖查证_Ubuntu22.04_Humble.md` 用 apt 装）。
#
# 用法（在仓库根目录）::
#
#   bash reproduce/install_all.sh                 # 全自动（没有 Isaac Sim 就自动下载）
#   bash reproduce/install_all.sh --dry-run       # 只打印将要做什么，不动任何东西
#   bash reproduce/install_all.sh --isaacsim-zip ~/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64.zip
#   bash reproduce/install_all.sh --isaacsim-dir /opt/isaac-sim-6.0.1      # 已经装好了
#   bash reproduce/install_all.sh --skip-isaacsim --skip-isaaclab          # 只建 lerobot 环境
#   bash reproduce/install_all.sh --torch-cu cu126                         # 40 系想用 cu126
#
# 设计原则：
#   · **只增不删** —— 脚本里没有任何针对用户数据的删除动作；
#   · **幂等** —— 每一步先检测，已完成就跳过，可以随时 Ctrl+C 再重跑；
#   · **可续传** —— Isaac Sim 那个 12 GB 的包用 `curl -C -` 断点续传；
#   · **不碰系统** —— 驱动只检查并打印命令；要真装得自己加 `--install-driver`（会要 sudo）。
# =============================================================================
set -uo pipefail

# ── 路径 ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── 颜色 ─────────────────────────────────────────────────────────────────────
if [ -t 1 ]; then
    C_R=$'\033[31m'; C_G=$'\033[32m'; C_Y=$'\033[33m'; C_B=$'\033[36m'; C_D=$'\033[2m'; C_0=$'\033[0m'
else
    C_R=""; C_G=""; C_Y=""; C_B=""; C_D=""; C_0=""
fi
say()  { printf '%s\n' "$*"; }
ok()   { printf '%s✅ %s%s\n' "$C_G" "$*" "$C_0"; }
warn() { printf '%s⚠️  %s%s\n' "$C_Y" "$*" "$C_0" >&2; }
err()  { printf '%s❌ %s%s\n' "$C_R" "$*" "$C_0" >&2; }
step() { printf '\n%s──── [%s/6] %s %s\n' "$C_B" "$1" "$2" "$C_0"; }
die()  { err "$*"; exit 1; }

# ── 默认参数 ─────────────────────────────────────────────────────────────────
ISAACSIM_DIR_OPT=""
ISAACSIM_ZIP_OPT=""
ISAACSIM_DEST_OPT=""
ISAACSIM_URL="https://downloads.isaacsim.nvidia.com/isaac-sim-standalone-6.0.1-linux-x86_64.zip"
ISAACSIM_EXPECT_BYTES=13018774928          # 官方 6.0.1 zip 的字节数（用于校验下载完整性）
NO_DOWNLOAD=0
LEROBOT_ENV="leisaac-lerobot-sim"
CONDA_DIR_OPT=""
TORCH_CU="cu128"
TORCH_VER="2.7.1"
TV_VER="0.22.1"
SKIP_ISAACSIM=0
SKIP_ISAACLAB=0
SKIP_LEROBOT=0
SKIP_SMOKE=0
INSTALL_DRIVER=0
DRY_RUN=0
ASSUME_YES=0
MIN_FREE_GB=60

usage() { sed -n '2,29p' "$0"; exit 0; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --isaacsim-dir)   ISAACSIM_DIR_OPT="$2"; shift 2 ;;
        --isaacsim-zip)   ISAACSIM_ZIP_OPT="$2"; shift 2 ;;
        --isaacsim-dest)  ISAACSIM_DEST_OPT="$2"; shift 2 ;;
        --isaacsim-url)   ISAACSIM_URL="$2"; shift 2 ;;
        --no-download)    NO_DOWNLOAD=1; shift ;;
        --lerobot-env)    LEROBOT_ENV="$2"; shift 2 ;;
        --conda-dir)      CONDA_DIR_OPT="$2"; shift 2 ;;
        --torch-cu)       TORCH_CU="$2"; shift 2 ;;
        --skip-isaacsim)  SKIP_ISAACSIM=1; shift ;;
        --skip-isaaclab)  SKIP_ISAACLAB=1; shift ;;
        --skip-lerobot)   SKIP_LEROBOT=1; shift ;;
        --skip-smoke)     SKIP_SMOKE=1; shift ;;
        --install-driver) INSTALL_DRIVER=1; shift ;;
        --min-free-gb)    MIN_FREE_GB="$2"; shift 2 ;;
        -n|--dry-run)     DRY_RUN=1; shift ;;
        -y|--yes)         ASSUME_YES=1; shift ;;
        -h|--help)        usage ;;
        *) err "未知参数: $1（-h 看用法）"; exit 2 ;;
    esac
done

# 这个脚本**不能**在 conda/venv 激活状态下跑（IsaacLab 3.0 会拒绝下载版 Isaac Sim + 虚拟环境）
if [ -n "${CONDA_PREFIX:-}" ] || [ -n "${VIRTUAL_ENV:-}" ]; then
    warn "检测到 conda/venv 处于激活状态（CONDA_PREFIX=${CONDA_PREFIX:-} VIRTUAL_ENV=${VIRTUAL_ENV:-}）"
    warn "本脚本的子步骤会自动 unset 这些变量，但建议你先 conda deactivate，少踩别的坑。"
fi

have() { command -v "$1" >/dev/null 2>&1; }

# 在「无 conda」环境下执行命令（IsaacLab / Isaac Sim 的 Python 必须这样跑）
in_isaac_env() {
    env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u CONDA_EXE -u VIRTUAL_ENV \
        OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y "${@}"
}

# 统一的"打印 + 执行"（--dry-run 时只打印）
# 打印时只给「含空格/引号」的参数加单引号 —— 既好读、又基本可以直接复制粘贴
_quote() {
    case "$1" in
        *[[:space:]\'\"]*) printf "'%s'" "$1" ;;
        *)                 printf '%s' "$1" ;;
    esac
}
run() {
    printf '  %s$ ' "$C_D"
    for _a in "$@"; do printf '%s ' "$(_quote "$_a")"; done
    printf '%s\n' "$C_0"
    if [ "$DRY_RUN" = "1" ]; then
        return 0
    fi
    "$@"
}

# 和 run 一样，但**显示**用第 1 个参数（给人看的干净写法），**执行**用剩下的参数。
# 用于那些真正执行时需要 bash -c "..." 包一层的复杂命令（否则打印出来一堆嵌套引号）。
run_disp() {
    local _disp="$1"; shift
    printf '  %s$ %s%s\n' "$C_D" "$_disp" "$C_0"
    if [ "$DRY_RUN" = "1" ]; then
        return 0
    fi
    "$@"
}

# 和 run / run_disp 一样，但**失败就停**（关键步骤用；--dry-run 下不执行、也就不会失败）
run_ck() {
    local why="$1"; shift
    run "$@" || die "$why"
}
run_disp_ck() {
    local disp="$1" why="$2"; shift 2
    run_disp "$disp" "$@" || die "$why"
}

# 下载/续传 Isaac Sim 的 zip，直到**字节数与官方完全一致**才返回 0。
# ★ 关键：只写 --retry 5 是**管不到** curl error 56/18 这类"传输被 CDN 掐断"的错误的，
#   必须加 --retry-all-errors（curl ≥ 7.71；Ubuntu 22.04 自带 7.81）。老 curl 上先探测再决定加不加。
fetch_isaacsim_zip() {   # $1=目标 zip 路径
    local zip="$1" attempt=0 max=5 sz
    local extra=()
    if have curl && curl --help 2>&1 | grep -q -- '--retry-all-errors'; then
        extra=(--retry-all-errors)
    fi
    while [ "$attempt" -lt "$max" ]; do
        attempt=$((attempt + 1))
        sz="$(stat -c%s "$zip" 2>/dev/null || echo 0)"
        [ "$sz" = "$ISAACSIM_EXPECT_BYTES" ] && return 0
        if [ "$sz" -gt 0 ]; then
            say "    已有 $sz / $ISAACSIM_EXPECT_BYTES 字节 → 从断点续传（第 $attempt/$max 次）"
        fi
        if [ "$DRY_RUN" = "1" ]; then
            say "    [dry-run] curl -L -C - --fail --retry 5 --retry-all-errors -o $zip $ISAACSIM_URL"
            say "    [dry-run] （真实运行会一直续传，直到字节数与官方完全一致才继续下一步）"
            return 0
        fi
        if have curl; then
            curl -L -C - --fail --retry 5 --retry-delay 5 ${extra[@]+"${extra[@]}"} \
                 -o "$zip" "$ISAACSIM_URL" || warn "    curl 退出码 $? —— 继续尝试续传"
        else
            wget -c -O "$zip" "$ISAACSIM_URL" || warn "    wget 退出码 $? —— 继续尝试续传"
        fi
    done
    return 1
}

# 判断一个 zip 能不能直接解压（0=可以；1=残缺，可作为续传起点）
#   大小正好等于官方值 → 认定完整（快路径，不必读那 12 GB）
#   小于官方值       → 再问 unzip -t：残缺包没有中央目录，会**立刻**失败（不会真读 12 GB）
zip_usable() {   # $1=path
    local p="$1" sz
    [ -f "$p" ] || return 1
    sz="$(stat -c%s "$p" 2>/dev/null || echo 0)"
    [ "$sz" -gt 0 ] || return 1
    [ "$sz" = "$ISAACSIM_EXPECT_BYTES" ] && return 0
    unzip -tqq "$p" >/dev/null 2>&1
}

confirm() {
    [ "$ASSUME_YES" = "1" ] && return 0
    [ "$DRY_RUN" = "1" ] && return 0
    if [ -t 0 ]; then
        read -r -p "$1 [y/N] " a
        [[ "$a" =~ ^[Yy]$ ]]
        return
    fi
    return 1        # 非交互（无人值守）且没给 -y：不自作主张（尤其是 12 GB 的下载）
}

printf '%s\n' "======================================================================"
printf '%s\n' " LeIsaac 智慧工厂仿真平台 —— 一键安装（除 ROS2）"
printf '%s\n' " 仓库: $REPO"
[ "$DRY_RUN" = "1" ] && printf '%s\n' " ${C_Y}模式: --dry-run（只看不做）${C_0}"
printf '%s\n' "======================================================================"

# =============================================================================
step 1 "前置自检"
# =============================================================================
FAIL=0

# --- 系统 ---
if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    say "  系统: ${PRETTY_NAME:-unknown}"
    case "${VERSION_ID:-}" in
        22.04|24.04) ok "  在 Isaac Sim 6.0 官方支持范围内" ;;
        *) warn "  官方支持 Ubuntu 22.04 / 24.04；你的版本是 ${VERSION_ID:-?}。" \
                "    非 22.04 的系统可能需要 reproduce/compat_libs_isaac6/ 里的旧 ABI 库（仓库已带）。" ;;
    esac
fi

# --- GPU / 驱动 ---
if have nvidia-smi; then
    GPU_LINE="$(nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader 2>/dev/null | head -1)"
    if [ -n "$GPU_LINE" ]; then
        ok "  GPU: $GPU_LINE"
        DRV="$(printf '%s' "$GPU_LINE" | awk -F', *' '{print $3}' | cut -d. -f1)"
        if [ -n "$DRV" ] && [ "$DRV" -lt 580 ] 2>/dev/null; then
            warn "  驱动大版本 < 580（Isaac Sim 6.0 要求 ≥ 580.95.05）。"
            warn "    装法: sudo apt install -y nvidia-driver-580-open && sudo reboot"
            if [ "$INSTALL_DRIVER" = "1" ]; then
                run sudo apt install -y nvidia-driver-580-open
                warn "  装完驱动**必须重启**，然后重跑本脚本。"
                exit 0
            fi
        fi
    else
        warn "  nvidia-smi 有命令但查询失败（NVML/驱动问题）。运行时不要 sudo，直接 nvidia-smi 复测。"
    fi
else
    err "  找不到 nvidia-smi —— 没有 NVIDIA 驱动，Isaac Sim 跑不起来。"
    say "    装法: sudo apt install -y nvidia-driver-580-open && sudo reboot"
    FAIL=1
fi

# --- 磁盘 ---
TARGET_FS="$REPO"
[ -n "$ISAACSIM_DEST_OPT" ] && TARGET_FS="$(dirname "$ISAACSIM_DEST_OPT")"
FREE_GB="$(df -Pk "$TARGET_FS" 2>/dev/null | awk 'NR==2{printf "%d", $4/1024/1024}')"
if [ -n "${FREE_GB:-}" ]; then
    if [ "$FREE_GB" -lt "$MIN_FREE_GB" ]; then
        err "  可用空间 ${FREE_GB} GB < ${MIN_FREE_GB} GB（Isaac Sim 解压后 38 GB + 依赖缓存）"
        FAIL=1
    else
        ok "  磁盘可用: ${FREE_GB} GB（需要 ≥ ${MIN_FREE_GB} GB）"
    fi
fi

# --- 工具 ---
for c in git unzip; do
    have "$c" || { err "  缺少命令: $c（sudo apt install -y $c）"; FAIL=1; }
done
if ! have curl && ! have wget; then
    err "  缺少 curl / wget（至少要有一个，用于下载 Isaac Sim）"; FAIL=1
fi

# --- conda（只有要装 lerobot 时才必须）---
CONDA_BIN=""
find_conda() {
    if [ -n "$CONDA_DIR_OPT" ] && [ -x "$CONDA_DIR_OPT/bin/conda" ]; then CONDA_BIN="$CONDA_DIR_OPT/bin/conda"; return; fi
    for d in "${CONDA_PREFIX%/envs/*}" "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" "$HOME/mambaforge" /opt/conda; do
        [ -n "$d" ] && [ -x "$d/bin/conda" ] && { CONDA_BIN="$d/bin/conda"; return; }
    done
    have conda && CONDA_BIN="$(command -v conda)"
}
if [ "$SKIP_LEROBOT" = "0" ]; then
    find_conda
    if [ -n "$CONDA_BIN" ]; then ok "  conda: $CONDA_BIN"; else
        err "  找不到 conda（要用 --skip-lerobot 跳过 lerobot 环境，或 --conda-dir 指定）"; FAIL=1
    fi
fi

[ "$FAIL" = "1" ] && die "前置自检未通过，先解决上面标 ❌ 的项。"
ok "前置自检通过"

# =============================================================================
step 2 "Isaac Sim 6.0.1 standalone"
# =============================================================================
# 先把用户环境里可能已有的 ISAACSIM_DIR 存下来（后面会重新赋值为"探测结果"）
ISAACSIM_DIR_ENV="${ISAACSIM_DIR:-}"
ISAACSIM_DIR=""

verify_isaacsim_dir() {   # $1=dir
    [ -n "$1" ] && [ -x "$1/python.sh" ] && [ -f "$1/VERSION" ]
}

if [ "$SKIP_ISAACSIM" = "1" ]; then
    say "  按 --skip-isaacsim 跳过"
else
    # ① 显式指定 / 环境变量 / 自动探测
    for cand in "$ISAACSIM_DIR_OPT" "$ISAACSIM_DIR_ENV" \
                "$REPO/../isaac-sim-6.0.1" "$REPO/../isaac-sim-"* \
                "$HOME/isaac-sim-6.0.1" "$HOME/isaac-sim-"* "$HOME/WorkStation/isaac-sim-"* \
                "$HOME/Downloads/isaac-sim-"* "/opt/isaac-sim-"*; do
        [ -n "$cand" ] || continue
        if verify_isaacsim_dir "$cand"; then ISAACSIM_DIR="${cand%/}"; break; fi
    done

    if [ -n "$ISAACSIM_DIR" ]; then
        ok "  已找到 Isaac Sim: $ISAACSIM_DIR （$(cat "$ISAACSIM_DIR/VERSION" 2>/dev/null | head -1)）"
    else
        DEST="$ISAACSIM_DEST_OPT"
        [ -n "$DEST" ] || DEST="$(cd "$REPO/.." && pwd)/isaac-sim-6.0.1"

        # ② 找本地 zip —— **只认"完整可用"的**；残缺的记下来当"续传起点"
        #    ★ 踩过的坑：只判 [ -f ] 会把"下了一半的 zip"当成"已经下好了"，
        #      于是下载块被整个跳过 → 解压失败 → 重跑还是同一个坑，永远卡死。
        ZIP=""
        PARTIAL=""
        consider_zip() {   # $1=候选路径
            [ -f "$1" ] || return 0
            if zip_usable "$1"; then
                ZIP="$1"
            elif [ -z "$PARTIAL" ]; then
                PARTIAL="$1"
            fi
        }
        if [ -n "$ISAACSIM_ZIP_OPT" ]; then
            [ -f "$ISAACSIM_ZIP_OPT" ] || die "--isaacsim-zip 指定的文件不存在: $ISAACSIM_ZIP_OPT"
            consider_zip "$ISAACSIM_ZIP_OPT"
            [ -z "$ZIP" ] && warn "  指定的 zip 不完整（$(stat -c%s "$ISAACSIM_ZIP_OPT" 2>/dev/null || echo 0) / $ISAACSIM_EXPECT_BYTES 字节）→ 将尝试续传它"
        else
            for z in "$HOME/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64.zip" \
                     "$HOME/Downloads/isaac-sim"*.zip "$REPO/../isaac-sim"*.zip; do
                consider_zip "$z"
                [ -n "$ZIP" ] && break
            done
        fi

        # ③ 没有可用的 zip 就下载/续传（**必须下到字节数与官方完全一致才算完**）
        if [ -n "$ZIP" ]; then
            if [ "$(stat -c%s "$ZIP" 2>/dev/null || echo 0)" = "$ISAACSIM_EXPECT_BYTES" ]; then
                ok "  用现成的 zip: $ZIP（$ISAACSIM_EXPECT_BYTES 字节，与官方一致）"
            else
                ok "  用现成的 zip: $ZIP（非官方字节数，已通过 unzip -t 完整性校验）"
            fi
        else
            # 续传目标：显式指定 > glob 扫到的残缺包 > 默认路径
            if [ -n "${PARTIAL:-}" ]; then
                ZIP="$PARTIAL"
            else
                ZIP="$(dirname "$DEST")/$(basename "$ISAACSIM_URL")"
            fi
            if [ "$NO_DOWNLOAD" = "1" ]; then
                err "  没有可用的 Isaac Sim 安装包，且指定了 --no-download。"
                say "    手动下载: $ISAACSIM_URL   （约 12.1 GB）"
                say "    放到:     $ZIP"
                say "    解压到:   $DEST（zip 顶层**没有**文件夹，直接解压进去即可）"
                say "    或重跑:   bash reproduce/install_all.sh --isaacsim-dir $DEST"
                exit 1
            fi
            if [ -n "${PARTIAL:-}" ]; then
                say "  发现一个**不完整**的 zip（$(stat -c%s "$PARTIAL" 2>/dev/null || echo 0) / $ISAACSIM_EXPECT_BYTES 字节），从断点续传："
            else
                say "  没有可用的 Isaac Sim 安装包，准备下载（约 12.1 GB，支持断点续传）："
            fi
            say "    → $ZIP"
            if [ "$DRY_RUN" = "0" ] && ! confirm "  开始下载？"; then
                say "  已取消（无人值守时请在命令行加 -y 表示同意下载）。"
                say "    想手动下载就把 zip 放到上面那个路径；或重跑并加 --isaacsim-zip <你的zip路径>"
                exit 0
            fi
            fetch_isaacsim_zip "$ZIP" || die "下载未完成（$(stat -c%s "$ZIP" 2>/dev/null || echo 0) / $ISAACSIM_EXPECT_BYTES 字节）。
     网络恢复后**原样重跑本脚本即可自动续传**，不会重新下 12 GB。也可手动续传：
       curl -L -C - -o $ZIP $ISAACSIM_URL
     若你确认这个文件已损坏、想从头下，请自行删掉它再重跑（本脚本不会替你删）。"
        fi

        # ④ 解压（顶层没有文件夹 → 直接解压到 DEST；DEST 非空则拒绝，绝不覆盖）
        if [ "$DRY_RUN" = "0" ]; then
            if [ -d "$DEST" ] && [ -n "$(ls -A "$DEST" 2>/dev/null)" ]; then
                die "$DEST 已存在且非空（可能是上次解压/下载到一半留下的）。
     二选一：① 换个目录：--isaacsim-dest /新的/目录；② 确认里面不是你要留的东西后自己删掉它再重跑。"
            fi
        fi
        say "  先校验 zip 完整性（unzip -t）..."
        run_ck "zip 完整性校验未通过 —— 这个包是残缺的。
     重跑本脚本会自动续传；若要从头下载，请自行删掉 $ZIP 再重跑。" unzip -tqq "$ZIP"
        say "  解压到: $DEST"
        run mkdir -p "$DEST"
        run_ck "解压失败。若是磁盘满 / 权限问题请先解决；若 $DEST 里已留下半份内容，请自己删掉它再重跑。" \
            unzip -q -n "$ZIP" -d "$DEST"

        if [ "$DRY_RUN" = "0" ]; then
            if verify_isaacsim_dir "$DEST"; then
                ISAACSIM_DIR="$DEST"
                ok "  Isaac Sim 就绪: $ISAACSIM_DIR"
            else
                die "解压完成了，但没找到 $DEST/python.sh —— 包内容不像官方 standalone zip。
     自查: unzip -l $ZIP | head    应该能看到 isaac-sim.sh / python.sh 这些顶层文件。"
            fi
        else
            ISAACSIM_DIR="$DEST"
        fi
    fi
fi

# 兼容库（Ubuntu 26.04 等新系统需要旧 ABI 的 libxml2/libicu74；22.04 自带，这里会跳过）
COMPAT_DIR="$REPO/reproduce/compat_libs_isaac6"
if [ -f "$COMPAT_DIR/libxml2.so.2" ]; then
    say "  兼容库: $COMPAT_DIR（存在，运行时会被前置进 LD_LIBRARY_PATH）"
else
    say "  兼容库: 不需要（系统自带 libxml2.so.2）"
fi

# =============================================================================
step 3 "IsaacLab 3.0.0 + 本项目 leisaac（editable）"
# =============================================================================
if [ "$SKIP_ISAACLAB" = "1" ]; then
    say "  按 --skip-isaaclab 跳过"
else
    [ -n "${ISAACSIM_DIR:-}" ] || die "跳过/未找到 Isaac Sim，无法装 IsaacLab（用 --isaacsim-dir 指定）"

    # ① 子模块
    if [ -f "$REPO/dependencies/IsaacLab/VERSION" ]; then
        ok "  子模块已拉取（IsaacLab VERSION=$(cat "$REPO/dependencies/IsaacLab/VERSION" | head -1)）"
    else
        say "  拉取 git 子模块（dependencies/IsaacLab）..."
        run git -C "$REPO" submodule update --init --recursive
    fi
    if [ "$DRY_RUN" = "0" ]; then
        V="$(cat "$REPO/dependencies/IsaacLab/VERSION" 2>/dev/null | head -1)"
        [ "$V" = "3.0.0" ] || warn "  IsaacLab VERSION=$V（本项目验证过的是 3.0.0，版本不对可能失败）"
    fi

    # ② _isaac_sim 软链
    LINK="$REPO/dependencies/IsaacLab/_isaac_sim"
    if [ -L "$LINK" ]; then
        say "  更新软链: $LINK -> $ISAACSIM_DIR"
        run ln -sfn "$ISAACSIM_DIR" "$LINK"
    elif [ -d "$LINK" ]; then
        warn "  $LINK 是一个**真实目录**（不是软链），跳过 —— 请手动确认它是不是 Isaac Sim。"
    else
        run ln -sfn "$ISAACSIM_DIR" "$LINK"
    fi

    # ③ 装 IsaacLab（核心 12 个子包 + mimic/teleop；用 IsaacLab 自带 CLI，带重试）
    say "  安装 IsaacLab 子包（这一步比较久，会 pip 装一堆通用科学包）..."
    run_disp_ck "cd dependencies/IsaacLab && ./isaaclab.sh --install" \
        "IsaacLab 安装失败（CLI 内部已自动重试 3 次）。
     常见原因：网络不稳 / conda 没 unset / 磁盘满。修好后**原样重跑本脚本**即可（幂等，已完成的会跳过）。" \
        bash -c "cd '$REPO/dependencies/IsaacLab' && env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u CONDA_EXE -u VIRTUAL_ENV OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y ./isaaclab.sh --install"

    # ④ 装本项目
    say "  安装本项目 leisaac（editable）..."
    run_disp_ck "<IsaacSim>/python.sh -m pip install -e source/leisaac" \
        "安装 leisaac 失败。修好后原样重跑本脚本即可。" \
        bash -c "cd '$REPO' && env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u CONDA_EXE -u VIRTUAL_ENV OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y '$ISAACSIM_DIR/python.sh' -m pip install -e '$REPO/source/leisaac'"

    # ⑤ 立刻验一下（这是"上一步真的成功了"的判据）
    say "  验证 import ..."
    run_disp_ck "<IsaacSim>/python.sh -c 'import isaaclab, isaaclab_physx, leisaac; ...'" \
        "import 失败 —— 说明 IsaacLab 或 leisaac 没装好。
     最常见原因：跑脚本时 conda/venv 还激活着（本脚本子步骤会 unset，但外层环境可能已污染），
     请先 conda deactivate 再原样重跑本脚本。" \
        bash -c "env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u CONDA_EXE -u VIRTUAL_ENV OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y '$ISAACSIM_DIR/python.sh' -c 'import isaaclab, isaaclab_physx, leisaac; print(\"isaaclab\", isaaclab.__version__, \"| leisaac ok\")'"
fi

# =============================================================================
step 4 "lerobot 环境（conda: $LEROBOT_ENV）"
# =============================================================================
LR_PY=""
if [ "$SKIP_LEROBOT" = "1" ]; then
    say "  按 --skip-lerobot 跳过"
else
    [ -n "$CONDA_BIN" ] || die "没有 conda，无法建 lerobot 环境"
    CONDA_ROOT="$(dirname "$(dirname "$CONDA_BIN")")"
    ENV_DIR="$CONDA_ROOT/envs/$LEROBOT_ENV"
    LR_PY="$ENV_DIR/bin/python"

    if [ -x "$LR_PY" ]; then
        ok "  环境已存在: $ENV_DIR（跳过创建）"
    else
        say "  创建 conda 环境（python 3.12）..."
        # 坑：直接 conda create 会先要求接受 Anaconda 默认频道的 ToS → 用 --override-channels 绕开
        run_ck "conda create 失败。常见原因：没网 / 频道不可达 / 磁盘满。
     修好后原样重跑本脚本即可。" \
            "$CONDA_BIN" create -n "$LEROBOT_ENV" python=3.12 pip -y --override-channels -c conda-forge
    fi

    # lerobot 0.4.2（[async] 会带上 grpcio，lerobot.async_inference 需要）
    say "  安装 lerobot[async]==0.4.2 ..."
    run_ck "安装 lerobot 失败。检查网络后原样重跑本脚本（pip 会续着装）。" \
        "$LR_PY" -m pip install "lerobot[async]==0.4.2"

    # 坑：pip 装来的 torch 是 +cu126，**不含 sm_120 内核**（50 系会报 no kernel image）。
    #     必须显式写本地版本号，否则 pip 认为已满足、不换。
    say "  覆盖安装 torch ${TORCH_VER}+${TORCH_CU} / torchvision ${TV_VER}+${TORCH_CU} ..."
    run_ck "安装 torch 失败。检查网络（download.pytorch.org 可达？）后原样重跑。" \
        "$LR_PY" -m pip install --index-url "https://download.pytorch.org/whl/${TORCH_CU}" \
        "torch==${TORCH_VER}+${TORCH_CU}" "torchvision==${TV_VER}+${TORCH_CU}"

    # h5py：转换/读取 HDF5 用；av：lerobot 的视频解码后端（--dataset.video_backend=pyav）
    say "  安装 h5py / av ..."
    run_ck "安装 h5py / av 失败。检查网络后原样重跑。" "$LR_PY" -m pip install h5py av

    # 自检（复用仓库里的自检脚本，--deps-only = 只查依赖，不需要 checkpoint）
    say "  自检（--deps-only）..."
    run_ck "lerobot 环境依赖自检未通过（上面列出了缺哪几项）。
     补装: <你的 lerobot python> -m pip install <缺的包>；或原样重跑本脚本。" \
        "$LR_PY" "$REPO/reproduce/check_lerobot_env.py" --deps-only
fi

# =============================================================================
step 5 "写 reproduce/local.env（记下路径）"
# =============================================================================
LOCAL_ENV="$REPO/reproduce/local.env"
if [ -f "$LOCAL_ENV" ]; then
    warn "  $LOCAL_ENV 已存在 —— **不覆盖**（里面可能有你自己的设置）。"
    say "    本次探测到的路径："
    say "      ISAACSIM_DIR=${ISAACSIM_DIR:-（跳过）}"
    say "      LEISAAC_LR_PY=${LR_PY:-（跳过）}"
    say "      CONDA_DIR=${CONDA_ROOT:-（跳过）}"
    say "    需要的话手动把上面三行填进 $LOCAL_ENV。"
else
    if [ "$DRY_RUN" = "1" ]; then
        say "  [dry-run] 将创建 $LOCAL_ENV"
    else
        cat > "$LOCAL_ENV" <<EOF
# 本机路径配置 —— 由 reproduce/install_all.sh 自动生成（$(date '+%F %T')）
# 这个文件不进 git、不进发布包。改完直接生效，不用动任何源码。
# 优先级：export 的环境变量 > 本文件 > 自动探测
ISAACSIM_DIR=${ISAACSIM_DIR:-}
LEISAAC_LR_PY=${LR_PY:-}
CONDA_DIR=${CONDA_ROOT:-}
EOF
        ok "  已写入 $LOCAL_ENV"
    fi
fi

# =============================================================================
step 6 "验收"
# =============================================================================
if [ "$SKIP_SMOKE" = "1" ]; then
    say "  按 --skip-smoke 跳过"
else
    if [ -z "${ISAACSIM_DIR:-}" ]; then
        warn "  没有 Isaac Sim 路径，跳过冒烟测试"
    else
        say "  ① headless 冒烟测试（约 1 分钟，不弹窗）..."
        run_disp_ck "<IsaacSim>/python.sh reproduce/smoke_test_env.py --task LeIsaac-SmartFactory-v0 --steps 60" \
            "冒烟测试没通过 —— 环境还没真正就绪。
     先看上面最后几十行报错。最常见的两种：
       · import 相关 → conda/venv 污染，conda deactivate 后重跑；
       · libxml2.so.2 缺失 → 你的系统需要 reproduce/compat_libs_isaac6（本仓库自带）。
     排查手册：INSTALL.md §8 / README*.md §12" \
            bash -c "cd '$REPO' && env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u CONDA_EXE -u VIRTUAL_ENV \
                OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
                LD_LIBRARY_PATH='$COMPAT_DIR:\${LD_LIBRARY_PATH:-}' \
                '$ISAACSIM_DIR/python.sh' -u reproduce/smoke_test_env.py --task LeIsaac-SmartFactory-v0 --steps 60"
        if [ "$DRY_RUN" = "0" ]; then
            say "    （通过判据：上面出现 [smoke] OK ... 环境复现通过。）"
        fi
    fi
fi

# =============================================================================
printf '\n%s\n' "======================================================================"
if [ "$DRY_RUN" = "1" ]; then
    printf '%s\n' " ${C_Y}--dry-run 结束：什么都没改。去掉 --dry-run 再跑一次才会真正安装。${C_0}"
else
    printf '%s\n' " ${C_G}安装流程结束。${C_0}"
fi
printf '%s\n' "======================================================================"
if [ -n "${ISAACSIM_DIR:-}" ]; then say "  Isaac Sim : $ISAACSIM_DIR"; fi
if [ -n "${LR_PY:-}" ];        then say "  lerobot   : $LR_PY"; fi
cat <<'EOF'

下一步：
  1) 起仿真（headless，省显存）：
       bash reproduce/ros2_chassis_teleop.sh --headless --max_seconds 900 --lidar_stop_distance 0.45
  2) 完整操作指南（建图 / 2D Goal Pose 导航 / 遥操 / 数采 / 训练 / 推理）：
       Ubuntu 22.04 原生 Humble  →  README.md（发给赛队的那份）
       宿主机 + Docker ROS2      →  README1.md
  3) 装 ROS2 Humble（本脚本**不含**）：
       sudo apt install -y ros-humble-ros-base ros-humble-navigation2 ros-humble-nav2-bringup \
                           ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard ros-humble-rviz2

提示：脚本是幂等的，任何一步失败都可以修好之后原样重跑（已完成的步骤会自动跳过）。
EOF
