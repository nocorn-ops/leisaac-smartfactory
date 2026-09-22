"""厨房任务的终止条件 / 评测指标。

原模板 `template/bi_arm_env_cfg.py::BiArmTerminationsCfg` **只有 `time_out`、没有 `success`** ——
它本来就是个"采数据"任务（成功与否靠人按 N 标记）。所以 `policy_inference.py` 报的"成功率"
以前永远是 0（其实是"跑满时间超时"）。这里补上一个**位置判据**，让评测有客观指标。

判据（与 `pick_orange` 任务的 `task_done` 同思路，但适配厨房场景）：
- **每颗橘子**都在盘子的水平邻域内（|Δx|, |Δy| <= `xy_tol`），且高度与盘子接近（`z_range`）；
- 可选：机械臂回到静止姿态（默认关闭 —— 评测"有没有放进去"时不该被回到原位卡住）。

`oranges_plate_distance()` 是给评测脚本用的**进度指标**（不参与终止），
把每颗橘子到盘子的水平距离打出来，这样即使没成功也能看出策略有没有"往盘子里送"。
"""
from __future__ import annotations

import torch
from isaaclab.assets import RigidObject
from isaaclab.envs import DirectRLEnv, ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg, TerminationTermCfg as DoneTerm

try:  # 静止姿态判据（可选）
    from leisaac.utils.robot_utils import is_so101_at_rest_pose
except Exception:  # noqa: BLE001
    is_so101_at_rest_pose = None


def _xy_dist_to_plate(env, oranges_cfg: list[SceneEntityCfg], plate_cfg: SceneEntityCfg) -> torch.Tensor:
    """每颗橘子到盘子的水平距离（米），形状 (num_envs, n_oranges)。"""
    plate: RigidObject = env.scene[plate_cfg.name]
    plate_xy = plate.data.root_pos_w[:, :2]
    dists = []
    for orange_cfg in oranges_cfg:
        orange: RigidObject = env.scene[orange_cfg.name]
        dists.append(torch.linalg.norm(orange.data.root_pos_w[:, :2] - plate_xy, dim=1))
    return torch.stack(dists, dim=1)


def oranges_plate_distance(
    env: ManagerBasedRLEnv | DirectRLEnv,
    oranges_cfg: list[SceneEntityCfg],
    plate_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """进度指标：每颗橘子到盘子中心的水平距离 (num_envs, n_oranges)，米。"""
    return _xy_dist_to_plate(env, oranges_cfg, plate_cfg)


def oranges_in_plate(
    env: ManagerBasedRLEnv | DirectRLEnv,
    oranges_cfg: list[SceneEntityCfg],
    plate_cfg: SceneEntityCfg,
    xy_tol: float = 0.08,
    z_range: tuple[float, float] = (-0.07, 0.07),
    require_rest: bool = False,
    rest_arm_names: tuple[str, ...] = ("left_arm", "right_arm"),
) -> torch.Tensor:
    """所有橘子都在盘子里 → 成功。返回 (num_envs,) bool。

    Args:
        env: 环境。
        oranges_cfg: 货物（橘子）实体列表。
        plate_cfg: 盘子实体。
        xy_tol: 与盘子中心的水平距离阈值（米）。盘子直径约 0.20 m、橘子直径约 0.05 m，
            0.08 大致对应"落在盘内偏外一点也算"。
        z_range: 相对盘子的高度容差（米），防止"橘子从台面掉到地上但水平上恰好对齐"。
        require_rest: 是否要求机械臂回到静止姿态（默认 False）。
        rest_arm_names: 需要检查静止姿态的机械臂名字。
    """
    plate: RigidObject = env.scene[plate_cfg.name]
    plate_pos = plate.data.root_pos_w

    done = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
    for orange_cfg in oranges_cfg:
        orange: RigidObject = env.scene[orange_cfg.name]
        rel = orange.data.root_pos_w - plate_pos
        inside_xy = (rel[:, 0].abs() <= xy_tol) & (rel[:, 1].abs() <= xy_tol)
        inside_z = (rel[:, 2] >= z_range[0]) & (rel[:, 2] <= z_range[1])
        done = done & inside_xy & inside_z

    if require_rest and is_so101_at_rest_pose is not None:
        for arm_name in rest_arm_names:
            arm = env.scene[arm_name]
            done = done & is_so101_at_rest_pose(arm.data.joint_pos, arm.data.joint_names)

    return done


def make_success_termination_cfg(xy_tol: float = 0.08, z_range: tuple[float, float] = (-0.07, 0.07)) -> DoneTerm:
    """构造厨房任务的 success 终止项（橘子都进盘子）。"""
    return DoneTerm(
        func=oranges_in_plate,
        params={
            "oranges_cfg": [SceneEntityCfg("Orange002"), SceneEntityCfg("Orange003")],
            "plate_cfg": SceneEntityCfg("Plate"),
            "xy_tol": xy_tol,
            "z_range": z_range,
            "require_rest": False,
        },
    )
