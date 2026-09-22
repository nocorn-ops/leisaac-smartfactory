#!/usr/bin/env python
"""对比官方SO101和LeRobot的USD关节属性。"""
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

def check_usd(path, robot_name):
    stage = Usd.Stage.Open(path)
    print(f"\n{'='*60}")
    print(f"=== {robot_name}: {path} ===")

    for prim in stage.TraverseAll():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        name = prim.GetName()
        if "shoulder" not in name.lower() or "mount" in name.lower():
            continue

        joint = UsdPhysics.RevoluteJoint(prim)
        axis_attr = prim.GetAttribute('physics:axis')
        low_attr = prim.GetAttribute('physics:lowerLimit')
        high_attr = prim.GetAttribute('physics:upperLimit')

        axis = axis_attr.Get() if axis_attr else "N/A"
        low = low_attr.Get() if low_attr else "N/A"
        high = high_attr.Get() if high_attr else "N/A"

        print(f"\n  {name}:")
        print(f"    axis: {axis}")
        print(f"    limits: [{low}, {high}]")

        # Check parent
        rels = prim.GetRelationships()

        # Try to get DriveAPI
        drive_api = UsdPhysics.DriveAPI.Apply(joint)
        if drive_api and drive_api.GetDampingAttr().Get():
            print(f"    damping: {drive_api.GetDampingAttr().Get()}")
            print(f"    stiffness: {drive_api.GetStiffnessAttr().Get()}")
            print(f"    maxForce: {drive_api.GetMaxForceAttr().Get()}")

        # Try to get world-space transform
        xform = UsdGeom.Xformable(prim)
        local_ops = []
        for op in xform.GetOrderedXformOps():
            if 'translate' in op.GetOpName() or 'rotate' in op.GetOpName():
                local_ops.append(f"{op.GetOpName()}: {[round(v,4) for v in op.Get()]}")
        if local_ops:
            print(f"    local xform: {'; '.join(local_ops)}")

# Check official SO101
check_usd("assets/robots/so101_follower.usd", "SO101 Official")

# Check user's LeRobot
check_usd("assets/robots/lerobot_robot.usd", "LeRobot")

# Also check the raw USD from the usdz directory
import os
for f in os.listdir("assets/robots/configuration/"):
    if f.endswith(".usd") and "lerobot" in f:
        check_usd(f"assets/robots/configuration/{f}", f"LeRobot config: {f}")

simulation_app.close()
