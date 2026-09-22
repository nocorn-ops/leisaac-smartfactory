"""把 Plate.usd 加到卧室场景里。"""
from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
from pxr import Usd, UsdGeom

REPO_ROOT = Path(__file__).resolve().parents[1]
bedroom = str(REPO_ROOT / "assets/scenes/lightwheel_bedroom/scene.usd")
plate   = str(REPO_ROOT / "assets/scenes/lightwheel_bedroom/Plate/Plate.usd")

stage = Usd.Stage.Open(bedroom)

# 创建盘子 Prim，引用外部 Plate.usd
prim = stage.DefinePrim('/Plate', 'Xform')
prim.GetReferences().AddReference(plate)

# 放到卧室桌面上
UsdGeom.XformCommonAPI(prim).SetTranslate((1.0, 8.0, 3.5))

stage.GetRootLayer().Save()
print('盘子已添加到卧室场景 (1.0, 8.0, 3.5)')
app.close()
