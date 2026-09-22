#!/usr/bin/env python
"""深度对比关节世界空间方向 — 遍历父链累积变换。"""
import multiprocessing
if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics, UsdGeom, Gf
import os

def compute_world_axis(stage, joint_prim):
    """Traverse from JOINT itself up through parent chain, accumulating transforms."""
    local_to_world = Gf.Matrix4d(1.0)

    # Walk from the JOINT up to the root
    current = joint_prim
    while current and current.IsValid():
        xform = UsdGeom.Xformable(current)
        local_m = Gf.Matrix4d(1.0)
        for op in xform.GetOrderedXformOps():
            name = op.GetOpName()
            val = op.Get()
            if 'translate' in name:
                t = Gf.Vec3d(val[0], val[1], val[2]) if len(val) >= 3 else Gf.Vec3d(val)
                local_m = Gf.Matrix4d(1.0).SetTranslate(t) * local_m
            elif 'rotate' in name:
                if len(val) == 3:
                    r = Gf.Rotation(Gf.Vec3d(val[0], val[1], val[2]))
                elif len(val) == 4:
                    r = Gf.Rotation(Gf.Quatd(val[0], val[1], val[2], val[3]))
                local_m = Gf.Matrix4d(1.0).SetRotate(r) * local_m
        local_to_world = local_m * local_to_world
        current = current.GetParent()

    # Joint axis is local Z
    world_axis = local_to_world.TransformDir(Gf.Vec3d(0, 0, 1))
    return world_axis, local_to_world

def analyze(path, label):
    stage = Usd.Stage.Open(path)
    print(f"\n{'='*70}")
    print(f"=== {label}: {path}")
    dp = stage.GetDefaultPrim()
    print(f"    defaultPrim: {dp.GetPath() if dp else 'NOT SET'}")

    for prim in stage.TraverseAll():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        name = prim.GetName()
        if not ('shoulder_pan' in name or 'shoulder_lift' in name):
            continue

        world_axis, _ = compute_world_axis(stage, prim)
        print(f"\n  {name}:")
        print(f"    world-space axis: ({world_axis[0]:.4f}, {world_axis[1]:.4f}, {world_axis[2]:.4f})")
        # Print JOINT ITSELF transforms first
        xform = UsdGeom.Xformable(prim)
        ops = []
        for op in xform.GetOrderedXformOps():
            ops.append(f"{op.GetOpName()}={[round(v,4) for v in op.Get()]}")
        print(f"    joint own: {'; '.join(ops) if ops else '(no xform)'}")
        # Then parent chain
        print(f"    parent chain:")
        current = prim.GetParent()
        depth = 0
        while current and current.IsValid() and depth < 10:
            xform = UsdGeom.Xformable(current)
            ops = []
            for op in xform.GetOrderedXformOps():
                ops.append(f"{op.GetOpName()}={[round(v,4) for v in op.Get()]}")
            if ops:
                print(f"      {current.GetName()}: {'; '.join(ops)}")
            current = current.GetParent()
            depth += 1

analyze("assets/robots/so101_follower.usd", "OFFICIAL SO101")
analyze("lerobot_robot/main.usdc", "LeRobot NEW")

simulation_app.close()
