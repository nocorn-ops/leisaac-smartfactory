#!/usr/bin/env python
"""设置新导出的 USD：设 defaultPrim + 复制到 assets/robots/ + 检查关节。"""
import multiprocessing
if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
from isaaclab.app import AppLauncher
import argparse, os, shutil
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

from pxr import Usd, UsdPhysics

# 1. 设 defaultPrim
src = 'lerobot_robot/main.usdc'
stage = Usd.Stage.Open(src)
stage.SetDefaultPrim(stage.GetPrimAtPath('/lerobot_robot'))
stage.GetRootLayer().Save()
print('1. defaultPrim set to /lerobot_robot')

# 2. 复制到 assets/robots/
dst = 'assets/robots/lerobot_robot_new.usdc'
shutil.copy(src, dst)
if os.path.exists('assets/robots/SubUSDs'):
    shutil.rmtree('assets/robots/SubUSDs')
shutil.copytree('lerobot_robot/SubUSDs', 'assets/robots/SubUSDs')
print(f'2. Copied to {dst}')

# 3. 对复制的文件也设 defaultPrim
stage2 = Usd.Stage.Open(dst)
stage2.SetDefaultPrim(stage2.GetPrimAtPath('/lerobot_robot'))
stage2.GetRootLayer().Save()

# 4. 检查关节名称
print('\n3. Joints:')
for prim in stage2.TraverseAll():
    if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
        print(f'  {prim.GetName()}')
print('\nDone.')
simulation_app.close()
