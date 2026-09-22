import os
from pathlib import Path


def _detect_git_root() -> Path:
    """Locate repository root; fallback to current file ancestor."""
    try:
        from git import Repo

        repo = Repo(os.getcwd(), search_parent_directories=True)
        return Path(repo.git.rev_parse("--show-toplevel"))
    except Exception:
        return Path(__file__).resolve().parents[4]


def _resolve_assets_root() -> str:
    """资产根目录：① 环境变量 ② **代码所在仓库**的 assets ③ 退回 cwd 的 git root。

    ⚠️ 顺序很重要（2026-09-16 为"打包给赛队"改）：原来只用 `git.Repo(os.getcwd())`，
    如果队伍把压缩包解压到**他们自己的 git 仓库里**再运行，就会把他们的
    `<他们的仓库>/assets` 当成资产目录 → 找不到 so101_follower.usd。
    现在优先用"本文件往上 4 层的 assets"（跟着代码走，解压到哪都对），
    这个目录不存在时才退回 git root。
    """
    env_root = os.environ.get("LEISAAC_ASSETS_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve().as_posix()

    from_code = Path(__file__).resolve().parents[4] / "assets"
    if from_code.is_dir():
        return from_code.resolve().as_posix()

    return (_detect_git_root() / "assets").resolve().as_posix()


ASSETS_ROOT = _resolve_assets_root()


def resolve_repo_root() -> Path:
    """仓库根目录：由本文件相对位置推导，不依赖 cwd / git / 环境变量。

    本文件位于 <repo>/source/leisaac/leisaac/utils/constant.py，
    parents[4] 即仓库根目录。仓库整体移动或更换容器挂载点时依然成立。
    """
    return Path(__file__).resolve().parents[4]


REPO_ROOT = resolve_repo_root()

SINGLE_ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
BI_ARM_JOINT_NAMES = [
    "left_shoulder_pan",
    "left_shoulder_lift",
    "left_elbow_flex",
    "left_wrist_flex",
    "left_wrist_roll",
    "left_gripper",
    "right_shoulder_pan",
    "right_shoulder_lift",
    "right_elbow_flex",
    "right_wrist_flex",
    "right_wrist_roll",
    "right_gripper",
]
LEROBOT_ROBOT_JOINT_NAMES = [
    "waist_lift",
    "left_shoulder_pan", "left_shoulder_lift", "left_elbow_flex",
    "left_wrist_flex", "left_wrist_roll", "left_gripper",
    "right_shoulder_pan", "right_shoulder_lift", "right_elbow_flex",
    "right_wrist_flex", "right_wrist_roll", "right_gripper",
]

LEKIWI_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
    "x",
    "y",
    "theta",
]
