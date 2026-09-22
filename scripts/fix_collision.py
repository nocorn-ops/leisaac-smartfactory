#!/usr/bin/env python
"""给 LeRobot USD 添加不可见碰撞球体——纯物理碰撞，无渲染，100% 可靠。"""
import multiprocessing
if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
from isaaclab.app import AppLauncher
import argparse, os
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, UsdPhysics

# 找到所有 base 子层
base_files = []
for root in ['assets/robots', 'lerobot_robot']:
    for sub in ['SubUSDs', 'configuration']:
        sd = os.path.join(root, sub)
        if os.path.isdir(sd):
            for f in os.listdir(sd):
                if 'base' in f.lower() and f.endswith('.usd'):
                    base_files.append(os.path.join(sd, f))

# 在 /colliders/ 下添加碰撞球体（不可见，仅供物理碰撞）
for base_path in base_files:
    stage = Usd.Stage.Open(base_path)
    print(f"Editing: {base_path}")

    added = 0
    for link, x, y, z, r in [
        ('left_gripper_link', 0.05, 0.015, -0.03, 0.025),
        ('right_gripper_link', 0.05, -0.015, -0.03, 0.025),
        ('left_moving_jaw_link', 0.055, 0.02, -0.038, 0.02),
        ('right_moving_jaw_link', 0.055, -0.02, -0.038, 0.02),
    ]:
        sphere_path = f'/colliders/{link}/gripper_sphere'
        if stage.GetPrimAtPath(sphere_path):
            continue
        prim = stage.GetPrimAtPath(f'/colliders/{link}')
        if not prim or not prim.IsValid():
            print(f"  SKIP: /colliders/{link} not found")
            continue

        sphere = UsdGeom.Sphere.Define(stage, sphere_path)
        sphere.CreateRadiusAttr(r)
        UsdGeom.XformCommonAPI(sphere).SetTranslate((x, y, z))
        # 标记为不可见（仅碰撞）
        sphere.CreatePurposeAttr().Set("guide")
        sp = stage.GetPrimAtPath(sphere_path)
        UsdPhysics.CollisionAPI.Apply(sp)
        api = UsdPhysics.MeshCollisionAPI.Apply(sp)
        api.CreateApproximationAttr().Set("convexHull")
        added += 1
        print(f"  ADDED: {sphere_path}")

    if added > 0:
        stage.GetRootLayer().Save()
        print(f"  Saved ({added} spheres)")

simulation_app.close()
print("Done")
