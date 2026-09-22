#!/usr/bin/env python
"""量几何：物体世界包围盒（判断平放/立放）+ 台面顶面高度 + 机械臂基座高度。

用法: reproduce/measure_geometry.py [--settle 150]
"""
import argparse
import dataclasses

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
parser.add_argument("--settle", type=int, default=150)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = False
args_cli.headless = True
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab.sensors import TiledCameraCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.env_utils import get_task_type  # noqa: E402


def _is_image_term(term) -> bool:
    func = getattr(term, "func", None)
    if func is not None and "image" in str(func):
        return True
    params = getattr(term, "params", None)
    return isinstance(params, dict) and "data_type" in params


def _strip(cfg_obj) -> int:
    n = 0
    if cfg_obj is None or not dataclasses.is_dataclass(cfg_obj):
        return 0
    for f in dataclasses.fields(cfg_obj):
        try:
            val = getattr(cfg_obj, f.name)
        except Exception:  # noqa: BLE001
            continue
        if val is None:
            continue
        if _is_image_term(val):
            setattr(cfg_obj, f.name, None)
            n += 1
        elif dataclasses.is_dataclass(val):
            n += _strip(val)
    return n


env_cfg = parse_env_cfg(args_cli.task, device="cuda:0", num_envs=1)
for _attr in ("observations", "events"):
    if hasattr(env_cfg, _attr):
        _strip(getattr(env_cfg, _attr))
for _name in list(vars(env_cfg.scene).keys()):
    if isinstance(getattr(env_cfg.scene, _name), TiledCameraCfg):
        setattr(env_cfg.scene, _name, None)
env_cfg.use_teleop_device(get_task_type(args_cli.task))
env_cfg.recorders = None

env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()

import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

stage = omni.usd.get_context().get_stage()
def _new_cache():
    return UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])


cache = _new_cache()


def world_aabb(path: str):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    b = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if b.IsEmpty():
        return None
    return b


def report(tag: str) -> None:
    global cache
    cache = _new_cache()  # 包围盒有缓存，每次重新建
    print(f"\n===== {tag}")
    print("  （quat 是 IsaacLab 3.0 的 XYZW；单位四元数 = (0,0,0,1)）")
    for name in ("Plate", "Orange002", "Orange003"):
        # parse_usd_and_create_subassets 建的 prim 路径是 {ENV_REGEX_NS}/Scene/<原路径去掉第一级>
        path = f"/World/envs/env_0/Scene/Scene/{name}"
        try:
            obj = env.scene[name]
        except Exception:  # noqa: BLE001
            obj = getattr(env.scene, name, None)
        if obj is None:
            print(f"  {name}: 场景里没有这个对象")
            continue
        pos = obj.data.root_pos_w
        quat = obj.data.root_quat_w
        pos = pos.torch[0] if hasattr(pos, "torch") else pos[0]
        quat = quat.torch[0] if hasattr(quat, "torch") else quat[0]
        # 真实 prim 路径从对象本身取（比手拼可靠）
        real_path = getattr(obj, "prim_path", None) or getattr(getattr(obj, "cfg", None), "prim_path", None) or path
        # 对象 prim_path 里带 {ENV_REGEX_NS} 正则，换成实际 env_0 才能查 USD
        real_path = real_path.replace("{ENV_REGEX_NS}", "/World/envs/env_0")
        import re as _re

        real_path = _re.sub(r"/World/envs/env_\[\^/\]\+", "/World/envs/env_0", real_path)
        r = world_aabb(real_path)
        aabb_txt = "AABB 取不到"
        if r is not None:
            mn, mx = r.GetMin(), r.GetMax()
            aabb_txt = (
                f"AABB x[{mn[0]:.3f},{mx[0]:.3f}] y[{mn[1]:.3f},{mx[1]:.3f}] z[{mn[2]:.3f},{mx[2]:.3f}] "
                f"尺寸=({mx[0] - mn[0]:.3f},{mx[1] - mn[1]:.3f},{mx[2] - mn[2]:.3f})"
            )
        print(
            f"  {name:10s} root=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}) "
            f"quat_xyzw=({quat[0]:.4f},{quat[1]:.4f},{quat[2]:.4f},{quat[3]:.4f}) {aabb_txt} [{real_path}]"
        )
    for label, path in (
        ("台面 counter_main_main_group", "/World/envs/env_0/Scene/Scene/counter_main_main_group"),
        ("台面 geometry_1", "/World/envs/env_0/Scene/Scene/counter_main_main_group/geometry_1"),
        ("左臂左基座", "/World/envs/env_0/Left_Robot/base"),
        ("左臂第一个 link", "/World/envs/env_0/Left_Robot/shoulder"),
    ):
        r = world_aabb(path)
        if r is None:
            print(f"  {label}: 取不到 ({path})")
        else:
            mn, mx = r.GetMin(), r.GetMax()
            print(f"  {label}: z[{mn[2]:.4f},{mx[2]:.4f}] x[{mn[0]:.3f},{mx[0]:.3f}] y[{mn[1]:.3f},{mx[1]:.3f}]")


report("reset 后（未推物理）")
for _ in range(args_cli.settle):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)
report(f"静置 {args_cli.settle} 步后")

for _ in range(300):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)
report("再静置 300 步后")

env.close()
simulation_app.close()
