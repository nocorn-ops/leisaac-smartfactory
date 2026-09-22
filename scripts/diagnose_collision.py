#!/usr/bin/env python
"""诊断：检查 robot 的每个 link 是否有 PhysX collision shape。"""
import multiprocessing
if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse, torch
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium, leisaac
from isaaclab_tasks.utils import parse_env_cfg

env_cfg = parse_env_cfg("LeIsaac-LeRobot-Kitchen-v0", device=args_cli.device, num_envs=1)
env = gymnasium.make("LeIsaac-LeRobot-Kitchen-v0", cfg=env_cfg).unwrapped
env.reset()

robot = env.scene["robot"]

# Check PhysX collision shapes via the physics scene API
from pxr import UsdPhysics, PhysxSchema
stage = env.sim.stage

# Find robot prims and check their collision setup
robot_path = "/World/envs/env_0/Robot"
for prim in stage.TraverseAll():
    path = str(prim.GetPath())
    if not path.startswith(robot_path):
        continue
    if prim.GetTypeName() != "Xform":
        continue
    name = prim.GetName()
    if "link" not in name.lower() and "jaw" not in name.lower() and "gripper" not in name.lower():
        continue

    # Check collision children
    coll = prim.GetChild('collisions')
    has_coll = coll and coll.IsValid()
    has_rigid = UsdPhysics.RigidBodyAPI.Apply(prim)

    # Count collision meshes
    mesh_count = 0
    if has_coll:
        for c in coll.GetChildren():
            mesh_count += 1

    # Check if contact occurs with an orange
    contact_count = 0
    if has_rigid:
        # Check contact report
        pass

    print(f"  {name:30s} collision={has_coll} meshes={mesh_count} rigid_body={has_rigid is not None}")

# Check oranges in the scene
print(f"\n=== Orange/Plate objects ===")
scene_path = "/World/envs/env_0/Scene/Scene"
for prim in stage.TraverseAll():
    path = str(prim.GetPath())
    if not path.startswith(scene_path):
        continue
    name = prim.GetName()
    if "Orange" not in name and "Plate" not in name:
        continue
    has_coll = prim.GetChild('collisions') is not None and prim.GetChild('collisions').IsValid()
    coll_api = UsdPhysics.CollisionAPI.Apply(prim)
    print(f"  {name}: collision_child={has_coll}")

# Check if there's collision filtering between robot and scene objects
print(f"\n=== Collision filtering ===")
for prim in stage.TraverseAll():
    if prim.IsA(PhysxSchema.PhysxCollisionGroup):
        print(f"  Group: {prim.GetPath()}")
        # Check filtered pairs
        fp = prim.GetRelationship('physxCollision:filteredGroups')
        if fp:
            print(f"    filtered: {fp.GetTargets()}")

env.close()
simulation_app.close()
