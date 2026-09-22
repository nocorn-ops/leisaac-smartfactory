"""评测辅助：把"成功终止项"和"任务自带的进度指标"格式化成一行，供评测脚本周期性打印。

平台约定（任务侧可选实现）：
- 终止项：名字为 ``success`` 的 ``TerminationTermCfg``（`policy_inference.py` 用它判定成功）；
- 进度指标：``env.cfg.success_metric_fn(env, **env.cfg.success_metric_cfg)``
  —— 返回形状 (num_envs, k) 的张量（例如每颗货物到目标的距离），用于"没成功也能看出有没有在靠近"。
"""

from __future__ import annotations


def get_success(env):
    """取成功终止项的值（(num_envs,) bool），没有该终止项则返回 None。"""
    tm = getattr(env, "termination_manager", None)
    if tm is None:
        return None
    names = list(getattr(tm, "active_terms", []) or [])
    if "success" not in names:
        return None
    try:
        term_cfg = tm.get_term_cfg("success")
        if getattr(term_cfg, "time_out", False):
            return None  # 名字叫 success 但其实是超时项，不算
        return tm.get_term("success")
    except Exception:  # noqa: BLE001
        return None


def get_metric(env):
    """取任务自带的进度指标，没有则返回 None。"""
    cfg = getattr(env, "cfg", None)
    fn = getattr(cfg, "success_metric_fn", None)
    params = getattr(cfg, "success_metric_cfg", None) or {}
    if fn is None:
        return None
    try:
        return fn(env, **params)
    except Exception as exc:  # noqa: BLE001
        return f"metric 调用失败: {exc}"


def format_progress(env, step: int | None = None) -> str:
    """一行进度字符串：success 状态 + 指标数值。"""
    parts = []
    if step is not None:
        parts.append(f"step={step}")
    success = get_success(env)
    if success is not None:
        n_ok = int(success.sum().item())
        parts.append(f"success={n_ok}/{success.numel()}")
    metric = get_metric(env)
    if metric is None:
        parts.append("metric=无")
    elif isinstance(metric, str):
        parts.append(metric)
    else:
        vals = metric[0].detach().cpu().tolist() if hasattr(metric, "detach") else list(metric[0])
        parts.append("到目标距离(m)=[" + ", ".join(f"{float(v):.3f}" for v in vals) + "]")
    return " | ".join(parts)
