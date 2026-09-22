#!/usr/bin/env python
"""调试：打印 articulation 的关节 DOF 和 actuator 列表。"""
import multiprocessing
if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)

from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium, torch, leisaac
from isaaclab_tasks.utils import parse_env_cfg

env_cfg = parse_env_cfg("LeIsaac-SmartFactory-v0", device=args_cli.device, num_envs=1)
env = gymnasium.make("LeIsaac-SmartFactory-v0", cfg=env_cfg).unwrapped
obs, _ = env.reset()

robot = env.scene["robot"]

# Check what the actual action term uses
print("=== Action Manager state ===")
print(f"active_terms: {env.action_manager.active_terms}")
# Try various attribute names
for attr in ['_terms', '_action_terms', '_action_term_dict', 'active_terms']:
    val = getattr(env.action_manager, attr, None)
    if val is not None:
        print(f"{attr}: {type(val).__name__}")
        if isinstance(val, dict):
            for k, v in val.items():
                print(f"  {k}: {type(v).__name__}")
                for a in ['_joint_ids', '_joint_names', '_offset', 'cfg']:
                    av = getattr(v, a, None) if hasattr(v, a) else 'N/A'
                    if a == 'cfg' and hasattr(v, 'cfg'):
                        print(f"    cfg.preserve_order: {v.cfg.preserve_order}")
                    print(f"    {a}: {av}")
        elif isinstance(val, list):
            for i, v in enumerate(val):
                print(f"  [{i}]: {type(v).__name__}")
                for a in ['_joint_ids', '_joint_names', '_offset', 'cfg']:
                    if hasattr(v, a):
                        av = getattr(v, a)
                        if a == 'cfg':
                            print(f"    cfg.preserve_order: {av.preserve_order}")
                        print(f"    {a}: {av}")

print(f"\nExpected with preserve_order=True:")
ids, names = robot.find_joints(
    ["left_shoulder_pan","left_shoulder_lift","left_elbow_flex","left_wrist_flex","left_wrist_roll","left_gripper",
     "right_shoulder_pan","right_shoulder_lift","right_elbow_flex","right_wrist_flex","right_wrist_roll","right_gripper"],
    preserve_order=True
)
print(f"  ids={ids}")

env.close()
simulation_app.close()

env.close()
simulation_app.close()

env.close()
simulation_app.close()
