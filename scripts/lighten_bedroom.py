"""创建轻量版卧室场景：移除镜子、柜子、装饰品等非核心物体。"""
import shutil
from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
from pxr import Usd

REPO_ROOT = Path(__file__).resolve().parents[1]
src = str(REPO_ROOT / "assets/scenes/lightwheel_bedroom/scene.usd")
dst = str(REPO_ROOT / "assets/scenes/lightwheel_bedroom/scene_light.usd")

# 复制
shutil.copy2(src, dst)
stage = Usd.Stage.Open(dst)

# 要禁用的物体
to_deactivate = [
    # 镜子
    'Mirror010', '_1_M_Mirror010a', '_2_m_Mirror010b',
    # 大衣柜 (16门16拉手)
    'StorageFurniture131',
    # 抽屉柜
    'StorageFurniture135',
    # 衣架
    'ClotheRack001',
    'FoldingRack022',
    # 窗帘
    'Curtain001', 'Curtain001_01',
    # 地毯
    'Carpet001', 'Carpet008',
    # 植物
    'Plant027', 'Plant027_01',
    # 挂画
    'WallArt015', 'WallArt015_01',
    # 纸巾
    'Tissue004',
    # 开关
    'Switch031', 'Switch031_01', 'Switch032', 'Switch032_01',
    # 钟
    'Clock001',
]

count = 0
for name in to_deactivate:
    for prefix in ['', '/Loft/', '/root/']:
        path = f'{prefix}{name}'
        prim = stage.GetPrimAtPath(path)
        if prim:
            prim.SetActive(False)
            count += 1

stage.GetRootLayer().Save()
print(f'已禁用 {count} 个物体，轻量场景: {dst}')
app.close()
