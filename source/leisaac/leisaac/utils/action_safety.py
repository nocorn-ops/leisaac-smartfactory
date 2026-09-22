"""动作安全层（限幅）—— 给"模型输出 → 仿真关节目标"这一步加保护。

为什么需要：遥操路径天然不会越界（`convert_action_from_so101_leader` 把主手的归一化行程
**线性映射到从手的关节限位区间**上），但**策略/VLA 的输出是无约束的** —— 模型可能给出
超出关节行程、甚至 NaN 的动作，直接把仿真搞崩或把台面扫飞。

三层保护（**限位来源是仿真自己**，不是主手标定文件）：

1. **位置限幅**：`env.scene[asset].data.joint_pos_limits`（官方 SO101 follower USD 的真实行程），
   两侧各内缩 ``margin``，避免 PD 顶死在硬限位上与 PhysX 打架。
2. **每步增量限幅**（slew rate）：``|a_t - a_{t-1}| <= max_delta``，防止"一步甩 180°"这种
   危险动作。IsaacLab 自带的 ``ActionTermCfg.clip`` **做不到**这一条，所以必须自己加。
3. **NaN / 维度守卫**：形状必须是 ``(num_envs, action_dim)``；出现非有限值就**保持上一步**动作
   （而不是把 NaN 灌进仿真，那会让整集数据作废）。

用法（策略推理侧）::

    limiter = ActionSafetyLimiter(env)          # 默认 2% 行程边距、0.1 rad/步
    limiter.reset()                             # 每集 reset 后调用
    action = limiter.filter(model_action)       # env.step(action) 之前过一遍

另外提供 :func:`build_declarative_clip` / :func:`install_declarative_clip`：
把同样的限位写进 IsaacLab 的 ``ActionTermCfg.clip``（作用在**处理后**的动作上，
本项目的 scale=1、default_joint_pos=0，所以就是弧度），作为"绕过包装器直接 env.step()"时的兜底。
"""

from __future__ import annotations

import math

import torch

#: 12 维动作的关节顺序（与 `default_feature_joint_names` 一致，仅用于打印/自检）
ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class ActionSafetyLimiter:
    """把策略输出的动作裁剪到"仿真真实关节行程"内，并限制每步增量、拦截 NaN。

    Args:
        env: 已构建好的 IsaacLab 环境（ManagerBasedRLEnv / DirectRLEnv）。
        margin_deg: 位置限幅的边距（度），默认 2.0；最终取 ``max(margin_deg, 2% 行程)``。
        max_delta: 每步最大变化量（弧度）。默认 0.1（30 Hz 下约 3 rad/s）。
            设为 ``None`` 或 ``<=0`` 关闭增量限制。
        clamp_position: 是否做位置限幅（默认 True）。
        verbose: 是否在拦截时打印（首次 + 每 N 次），默认 True。
    """

    def __init__(
        self,
        env,
        margin_deg: float = 2.0,
        max_delta: float | None = 0.1,
        clamp_position: bool = True,
        verbose: bool = True,
    ):
        self.env = env
        self.margin_deg = float(margin_deg)
        self.max_delta = None if max_delta is None or max_delta <= 0 else float(max_delta)
        self.clamp_position = bool(clamp_position)
        self.verbose = bool(verbose)

        self._lo, self._hi, self._layout = self._collect_limits(env)
        self._prev: torch.Tensor | None = None
        self._counts = {"pos": 0, "rate": 0, "nan": 0, "shape": 0}

    # ── 限位采集 ────────────────────────────────────────────────────────────
    def _collect_limits(self, env) -> tuple[torch.Tensor, torch.Tensor, list[tuple[str, list[str]]]]:
        """按 action term 顺序拼出每个动作维度对应的关节限位（弧度）。"""
        am = env.action_manager
        dim = am.total_action_dim
        lo = torch.full((dim,), -math.inf, device=env.device)
        hi = torch.full((dim,), math.inf, device=env.device)
        layout: list[tuple[str, list[str]]] = []

        term_names = list(getattr(am, "active_terms", []) or list(am._terms.keys()))
        offset = 0
        for name in term_names:
            term = am.get_term(name)
            term_dim = term.action_dim
            cfg = getattr(term, "cfg", None)
            asset_name = getattr(cfg, "asset_name", None)
            joint_names = list(getattr(term, "_joint_names", None) or getattr(cfg, "joint_names", []) or [])
            resolved = False
            if asset_name is not None and len(joint_names) == term_dim:
                try:
                    asset = env.scene[asset_name]
                    joint_ids, _ = asset.find_joints(joint_names)
                    limits = asset.data.joint_pos_limits
                    limits = limits.torch if hasattr(limits, "torch") else limits  # IsaacLab 3.0: ProxyArray
                    limits = limits[0][joint_ids].detach().to(device=self.env.device, dtype=torch.float32)
                    span = (limits[:, 1] - limits[:, 0]).clamp(min=0.0)
                    margin = torch.clamp(
                        torch.full_like(span, math.radians(self.margin_deg)), min=0.0
                    )
                    margin = torch.maximum(margin, 0.02 * span)  # 至少 2% 行程
                    lo[offset : offset + term_dim] = limits[:, 0] + margin
                    hi[offset : offset + term_dim] = limits[:, 1] - margin
                    layout.append((f"{name}({asset_name})", joint_names))
                    resolved = True
                except Exception as exc:  # noqa: BLE001
                    if self.verbose:
                        print(f"[safety] 动作项 {name} 的限位读取失败（该段不裁剪）：{exc}")
            if not resolved:
                layout.append((f"{name}(未解析，不裁剪)", []))
            offset += term_dim
        if self.verbose:
            print(
                f"[safety] 动作安全层就绪：dim={dim} 位置限幅={self.clamp_position} "
                f"边距={self.margin_deg}° 每步上限={self.max_delta} rad | 段={[n for n, _ in layout]}"
            )
        return lo, hi, layout

    # ── 运行 ────────────────────────────────────────────────────────────────
    def reset(self, init_action: torch.Tensor | None = None) -> None:
        """每集开始时调用。

        Args:
            init_action: 上一步动作的初值（默认取当前关节位置，避免第一帧就允许跳变）。
        """
        if init_action is not None:
            self._prev = init_action.detach().clone()
        else:
            self._prev = self.current_joint_action()

    def current_joint_action(self) -> torch.Tensor:
        """用仿真当前关节位置拼一个 (num_envs, dim) 的动作初值。"""
        am = self.env.action_manager
        dim = am.total_action_dim
        out = torch.zeros((self.env.num_envs, dim), device=self.env.device)
        offset = 0
        for name in list(getattr(am, "active_terms", []) or list(am._terms.keys())):
            term = am.get_term(name)
            term_dim = term.action_dim
            cfg = getattr(term, "cfg", None)
            asset_name = getattr(cfg, "asset_name", None)
            joint_names = list(getattr(term, "_joint_names", None) or getattr(cfg, "joint_names", []) or [])
            if asset_name is not None and len(joint_names) == term_dim:
                try:
                    asset = self.env.scene[asset_name]
                    joint_ids, _ = asset.find_joints(joint_names)
                    pos = asset.data.joint_pos
                    pos = pos.torch if hasattr(pos, "torch") else pos
                    out[:, offset : offset + term_dim] = pos[:, joint_ids].to(out.dtype)
                except Exception:  # noqa: BLE001
                    pass
            offset += term_dim
        return out

    def filter(self, action: torch.Tensor) -> torch.Tensor:  # noqa: A003
        """把模型输出裁剪成安全动作（返回新张量，不原地修改输入）。"""
        a = torch.as_tensor(action, device=self.env.device, dtype=torch.float32)
        if a.dim() == 1:
            a = a.unsqueeze(0)
        if a.shape != (self.env.num_envs, self._lo.numel()):
            self._counts["shape"] += 1
            if self.verbose and self._counts["shape"] <= 3:
                print(
                    f"[safety] ⚠ 动作形状 {tuple(a.shape)} != 期望 "
                    f"{(self.env.num_envs, self._lo.numel())} → 本次保持上一步"
                )
            return self._prev.clone() if self._prev is not None else torch.zeros_like(a)

        prev = self._prev if self._prev is not None else a.clone()
        bad = ~torch.isfinite(a)
        if bad.any():
            self._counts["nan"] += 1
            if self.verbose and self._counts["nan"] <= 3:
                idx = bad[0].nonzero().flatten().tolist()
                print(f"[safety] ⚠ 动作里有非有限值（维度 {idx}）→ 这些维度保持上一步")
            a = torch.where(bad, prev, a)

        if self.clamp_position:
            clamped = torch.clamp(a, min=self._lo, max=self._hi)
            if not torch.equal(clamped, a):
                self._counts["pos"] += 1
                if self.verbose and self._counts["pos"] <= 3:
                    over = (a - clamped).abs().max().item()
                    print(f"[safety] 位置限幅生效（最大超出 {over:.3f} rad）")
            a = clamped

        if self.max_delta is not None:
            delta = torch.clamp(a - prev, -self.max_delta, self.max_delta)
            if not torch.equal(delta, a - prev):
                self._counts["rate"] += 1
                if self.verbose and self._counts["rate"] <= 3:
                    big = (a - prev).abs().max().item()
                    print(f"[safety] 增量限幅生效（请求 {big:.3f} rad/步 → 限到 {self.max_delta:.3f}）")
            a = prev + delta

        self._prev = a.detach().clone()
        return a

    @property
    def stats(self) -> dict:
        """累计拦截次数（位置/增量/NaN/形状）。"""
        return dict(self._counts)


def build_declarative_clip(margin_deg: float = 2.0, include_gripper: bool = True) -> dict:
    """按官方 SO101 关节行程生成 IsaacLab ``ActionTermCfg.clip`` 用的字典。

    注意：``clip`` 作用在**处理后**的动作上（arm/gripper 的 ``scale=1.0``、
    ``use_default_offset=True`` 且默认关节角为 0，所以就是弧度）。
    数值来自 `leisaac.assets.robots.lerobot.SO101_FOLLOWER_USD_JOINT_LIMLITS`（度），
    与仿真里 ``joint_pos_limits`` 实测一致。
    """
    # 复制自 leisaac/assets/robots/lerobot.py（那个模块会 import isaaclab，这里不依赖它）
    usd_limits_deg = {
        "shoulder_pan": (-110.0, 110.0),
        "shoulder_lift": (-100.0, 100.0),
        "elbow_flex": (-100.0, 90.0),
        "wrist_flex": (-95.0, 95.0),
        "wrist_roll": (-160.0, 160.0),
        "gripper": (-10.0, 100.0),
    }
    clip = {}
    for joint, (lo_deg, hi_deg) in usd_limits_deg.items():
        if joint == "gripper" and not include_gripper:
            continue
        span = hi_deg - lo_deg
        margin = max(margin_deg, 0.02 * span)
        clip[joint] = (math.radians(lo_deg + margin), math.radians(hi_deg - margin))
    return clip


def install_declarative_clip(env, margin_deg: float = 2.0, verbose: bool = True) -> None:
    """把限位写进已经建好的 action term（运行时兜底，防止绕过包装器直接 env.step()）。"""
    clip = build_declarative_clip(margin_deg=margin_deg)
    am = env.action_manager
    installed = []
    for name in list(getattr(am, "active_terms", []) or list(am._terms.keys())):
        term = am.get_term(name)
        cfg = getattr(term, "cfg", None)
        joint_names = list(getattr(term, "_joint_names", None) or getattr(cfg, "joint_names", []) or [])
        if not joint_names:
            continue
        sub = {j: clip[j] for j in joint_names if j in clip}
        if not sub:
            continue
        term._clip = torch.tensor(
            [[sub.get(j, (-math.inf, math.inf)) for j in joint_names]], device=env.device, dtype=torch.float32
        ).repeat(env.num_envs, 1, 1)
        # IsaacLab 的 process_actions 是 `if self.cfg.clip is not None:` 才裁剪 → 同时把 cfg 设上
        try:
            cfg.clip = sub
        except Exception:  # noqa: BLE001
            pass
        installed.append(name)
    if verbose:
        print(f"[safety] 已安装声明式 clip 到：{installed}")
