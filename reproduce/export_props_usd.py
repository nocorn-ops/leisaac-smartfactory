#!/usr/bin/env python
"""生成 M2 的三件货物（**文本 USD 资产**，可在 Isaac Sim GUI 里直接改）。

    assets/scenes/smart_factory/storage_box.usda   收纳盒（灰塑料周转箱，5 块板 → 内部真的是空的）
    assets/scenes/smart_factory/eggplant.usda      茄子（胶囊果身 + 圆柱果梗）
    香蕉：直接用 NVIDIA 官方 YCB 资产 011_banana（在线引用，不落库）

为什么自己生成而不是直接拿网上的资产：
- **收纳盒必须是"真空腔"**：单个网格资产的碰撞通常是凸包/整体网格，货物放进去会卡在"顶面"上；
  用 5 块薄板拼出来，PhysX 里内部就是真正空心的，抓放才有意义。
- 尺寸要精确可控（后面"按层高抓取/放盒"都依赖这个），也方便你按官方实物改。

实现同 `export_arena_usd.py`：只用 USD（pxr），**不起 Kit / SimulationApp**，几秒跑完。

用法:
    reproduce/export_props_usd.py            # 生成两个 .usda
    reproduce/export_props_usd.py --list     # 只打印尺寸参数
"""

import argparse
import importlib.util
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO, "assets", "scenes", "smart_factory")
LAYOUT_PY = os.path.join(REPO, "source", "leisaac", "leisaac", "assets", "scenes", "smart_factory_layout.py")


def load_layout():
    """按文件路径加载布局模块（不能 import leisaac 包，原因见 export_arena_usd.py）。"""
    spec = importlib.util.spec_from_file_location("smart_factory_layout", LAYOUT_PY)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod  # dataclass + from __future__ annotations 需要它
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    return mod


layout = load_layout()

# ── 收纳盒（外部尺寸 / 板厚，单位 m）─────────────────────────────────────────
BOX_X = layout.CRATE_SIZE[0]
BOX_Y = layout.CRATE_SIZE[1]
BOX_Z = layout.CRATE_SIZE[2]
BOX_T = 0.008  # 板厚
BOX_COLOR = (0.62, 0.64, 0.66)  # 灰色塑料
#: 顶部翻边（沿口外伸）：给夹爪一个"夹住边缘"的抓法（四面都能夹）
BOX_RIM_W = 0.012  # 外伸宽度
BOX_RIM_T = 0.010  # 厚度
#: 两端提手：横杆 + 两个立柱。夹爪可以从端面伸进去夹住横杆（真实周转箱就是这么拎的）
BOX_HANDLE_R = 0.006  # 横杆半径
BOX_HANDLE_LEN = 0.10  # 横杆长度（沿 Y）
BOX_HANDLE_OUT = 0.015  # 横杆相对端面外移多少（= 夹爪伸进去的空间）
BOX_HANDLE_Z = 0.020  # 横杆高度（盒子局部坐标，中心为 0）

# ── 茄子（胶囊果身 + 圆柱果梗，长轴沿 Z，摆到架子上时再整体转成横放）──────────
EGG_R = 0.035  # 果身半径
EGG_LEN = 0.13  # 胶囊圆柱段长度（总长 = 0.13 + 2*0.035 = 0.20）
EGG_STEM_R = 0.007
EGG_STEM_H = 0.035
EGG_COLOR = (0.18, 0.07, 0.26)  # 深紫
EGG_STEM_COLOR = (0.30, 0.42, 0.16)  # 绿梗

parser = argparse.ArgumentParser(description="生成货物 USD")
parser.add_argument("--list", action="store_true")
args_cli = parser.parse_args()

if args_cli.list:
    print(f"收纳盒: 外 {BOX_X} x {BOX_Y} x {BOX_Z} m, 板厚 {BOX_T}")
    print(f"  顶部翻边外伸 {BOX_RIM_W}（总宽 {BOX_X + 2 * BOX_RIM_W:.3f} x {BOX_Y + 2 * BOX_RIM_W:.3f}）")
    print(f"  端面提手: 横杆 r={BOX_HANDLE_R} 长 {BOX_HANDLE_LEN}，离端面 {BOX_HANDLE_OUT} "
          f"（总 X 外形 {BOX_X + 2 * (BOX_HANDLE_OUT + BOX_HANDLE_R):.3f}）")
    print(f"  内部空间约 {(BOX_X - 2 * BOX_T):.3f} x {(BOX_Y - 2 * BOX_T):.3f} x {(BOX_Z - BOX_T):.3f} m")
    print(f"茄子:   果身 半径 {EGG_R} 长 {EGG_LEN}（总长 {EGG_LEN + 2 * EGG_R:.3f} m）+ 梗 {EGG_STEM_H}")
    print(f"香蕉:   YCB 011_banana（在线资产，约 0.20 m 长）")
    print(f"落位（来自 layout）: 香蕉 {layout.BANANA_XY} z={layout.TIER_HIGH_Z}；"
          f"茄子 {layout.EGGPLANT_XY} z={layout.TIER_LOW_Z}；盒 {layout.CRATE_XY} z={layout.TIER_LOW_Z}")
    sys.exit(0)

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402


def new_stage(path: str, root_name: str):
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, f"/{root_name}")
    stage.SetDefaultPrim(root.GetPrim())
    # 动态刚体：根 prim 挂 RigidBodyAPI（质心/质量后面在任务里也可以覆盖）
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    return stage, root


def material(stage, prim_path: str, color, roughness=0.6):
    mat = UsdShade.Material.Define(stage, prim_path)
    sh = UsdShade.Shader.Define(stage, prim_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def add_cube(stage, path, size, pos, color, rot_deg=(0, 0, 0), t=None):
    """一块板/一个立方体（单位 Cube + scale），带碰撞 + 材质。"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    x = UsdGeom.Xformable(cube.GetPrim())
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if any(rot_deg):
        x.AddRotateXYZOp().Set(Gf.Vec3f(*rot_deg))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    cube.GetPrim().GetAttribute("physics:collisionEnabled").Set(True)
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material(stage, f"{path}/Looks/M", color))
    return cube


def add_capsule(stage, path, radius, height, pos, color, rot_deg=(0, 0, 0)):
    cap = UsdGeom.Capsule.Define(stage, path)
    cap.CreateAxisAttr().Set("Z")
    cap.CreateRadiusAttr().Set(radius)
    cap.CreateHeightAttr().Set(height)
    x = UsdGeom.Xformable(cap.GetPrim())
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if any(rot_deg):
        x.AddRotateXYZOp().Set(Gf.Vec3f(*rot_deg))
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
    cap.GetPrim().GetAttribute("physics:collisionEnabled").Set(True)
    UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(material(stage, f"{path}/Looks/M", color))
    return cap


def add_cylinder(stage, path, radius, height, pos, color, rot_deg=(0, 0, 0)):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateAxisAttr().Set("Z")
    cyl.CreateRadiusAttr().Set(radius)
    cyl.CreateHeightAttr().Set(height)
    x = UsdGeom.Xformable(cyl.GetPrim())
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if any(rot_deg):
        x.AddRotateXYZOp().Set(Gf.Vec3f(*rot_deg))
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    cyl.GetPrim().GetAttribute("physics:collisionEnabled").Set(True)
    UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(material(stage, f"{path}/Looks/M", color))
    return cyl


# ── 收纳盒：底 + 四壁（内部真空）─────────────────────────────────────────────
box_path = os.path.join(OUT_DIR, "storage_box.usda")
os.makedirs(OUT_DIR, exist_ok=True)
stage, root = new_stage(box_path, "StorageBox")
# 局部坐标：原点在盒子**几何中心**
add_cube(stage, "/StorageBox/Bottom", (BOX_X, BOX_Y, BOX_T), (0, 0, -BOX_Z / 2 + BOX_T / 2), BOX_COLOR)
add_cube(stage, "/StorageBox/WallXPos", (BOX_T, BOX_Y, BOX_Z), (+(BOX_X - BOX_T) / 2, 0, 0), BOX_COLOR)
add_cube(stage, "/StorageBox/WallXNeg", (BOX_T, BOX_Y, BOX_Z), (-(BOX_X - BOX_T) / 2, 0, 0), BOX_COLOR)
add_cube(stage, "/StorageBox/WallYPos", (BOX_X - 2 * BOX_T, BOX_T, BOX_Z), (0, +(BOX_Y - BOX_T) / 2, 0), BOX_COLOR)
add_cube(stage, "/StorageBox/WallYNeg", (BOX_X - 2 * BOX_T, BOX_T, BOX_Z), (0, -(BOX_Y - BOX_T) / 2, 0), BOX_COLOR)
# 顶部翻边：沿口四周各加一条外伸薄板（四面都能被夹爪夹住）
_rim_z = BOX_Z / 2 - BOX_RIM_T / 2
add_cube(stage, "/StorageBox/RimXPos", (BOX_RIM_W, BOX_Y + 2 * BOX_RIM_W, BOX_RIM_T),
         (+(BOX_X + BOX_RIM_W) / 2, 0, _rim_z), BOX_COLOR)
add_cube(stage, "/StorageBox/RimXNeg", (BOX_RIM_W, BOX_Y + 2 * BOX_RIM_W, BOX_RIM_T),
         (-(BOX_X + BOX_RIM_W) / 2, 0, _rim_z), BOX_COLOR)
add_cube(stage, "/StorageBox/RimYPos", (BOX_X - 2 * BOX_RIM_W, BOX_RIM_W, BOX_RIM_T),
         (0, +(BOX_Y + BOX_RIM_W) / 2, _rim_z), BOX_COLOR)
add_cube(stage, "/StorageBox/RimYNeg", (BOX_X - 2 * BOX_RIM_W, BOX_RIM_W, BOX_RIM_T),
         (0, -(BOX_Y + BOX_RIM_W) / 2, _rim_z), BOX_COLOR)

# 两端提手：横杆（沿 Y）+ 两端立柱，中间留出 ~1.5cm 给夹爪伸进去
for _sx, _tag in ((+1, "XPos"), (-1, "XNeg")):
    _wall_x = _sx * (BOX_X - BOX_T) / 2
    add_cylinder(stage, f"/StorageBox/Handle{_tag}Bar", BOX_HANDLE_R, BOX_HANDLE_LEN,
                 (_wall_x + _sx * (BOX_HANDLE_OUT + BOX_HANDLE_R), 0, BOX_HANDLE_Z),
                 BOX_COLOR, rot_deg=(90, 0, 0))
    for _sy, _ptag in ((-1, "S"), (+1, "N")):
        add_cube(stage, f"/StorageBox/Handle{_tag}Post{_ptag}", (BOX_HANDLE_OUT, 0.012, 0.012),
                 (_wall_x + _sx * BOX_HANDLE_OUT / 2, _sy * BOX_HANDLE_LEN / 2, BOX_HANDLE_Z), BOX_COLOR)

# 质量：塑料盒 ~0.35 kg（在 USD 里写死，任务里还能用 MassPropertiesCfg 覆盖）
UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr().Set(0.35)
stage.GetRootLayer().Save()
print(f"[props] 已写出 {box_path} ({os.path.getsize(box_path)} 字节)  外尺寸 {BOX_X}x{BOX_Y}x{BOX_Z}")

# ── 香蕉（**兜底用**的程序化版本）─────────────────────────────────────────────
# 说明：优先用官方 YCB 011_banana（本地二进制 assets/scenes/smart_factory/ycb_011_banana.usd，
# 不进 git）；那个文件不在时（例如队友刚 clone 仓库）就用这个程序化香蕉顶上，
# 保证任务永远能跑起来。
BANANA_R = 0.12  # 弧半径
BANANA_HALF_DEG = 50.0  # 弧张角的一半
BANANA_MAX_R = 0.024  # 中段半径
BANANA_SEGS = 6  # 胶囊段数（球 = 段数+1）
BANANA_COLOR = (0.93, 0.82, 0.22)
BANANA_STEM_COLOR = (0.42, 0.36, 0.18)

if args_cli.list:
    print(f"香蕉(兜底): 弧半径 {BANANA_R} 张角 {BANANA_HALF_DEG * 2}° 中段半径 {BANANA_MAX_R} "
          f"→ 约 {2 * BANANA_R * math.radians(BANANA_HALF_DEG):.3f} m 长")


def _banana_points():
    """在 XY 平面里生成一条香蕉弧（弦沿 X，弧往 -Y 鼓）。"""
    pts, radii = [], []
    for i in range(BANANA_SEGS + 1):
        th = math.radians(-BANANA_HALF_DEG + 2 * BANANA_HALF_DEG * i / BANANA_SEGS)
        pts.append((BANANA_R * math.sin(th), BANANA_R * math.cos(th) - BANANA_R, 0.0))
        radii.append(BANANA_MAX_R * (1.0 - 0.35 * (abs(th) / math.radians(BANANA_HALF_DEG)) ** 2))
    return pts, radii


# ── 茄子：胶囊果身 + 圆柱果梗（长轴 Z，尖端朝 -Z）────────────────────────────
egg_path = os.path.join(OUT_DIR, "eggplant.usda")
stage, root = new_stage(egg_path, "Eggplant")
add_capsule(stage, "/Eggplant/Body", EGG_R, EGG_LEN, (0, 0, 0), EGG_COLOR)
add_cylinder(stage, "/Eggplant/Stem", EGG_STEM_R, EGG_STEM_H, (0, 0, EGG_LEN / 2 + EGG_R + EGG_STEM_H / 2 - 0.004),
             EGG_STEM_COLOR)
UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr().Set(0.25)
stage.GetRootLayer().Save()
print(f"[props] 已写出 {egg_path} ({os.path.getsize(egg_path)} 字节)  "
      f"总长 {EGG_LEN + 2 * EGG_R:.3f} 直径 {2 * EGG_R:.3f}")
# ── 程序化香蕉（兜底；YCB 二进制资产不在时任务会用它）────────────────────────
ban_path = os.path.join(OUT_DIR, "banana.usda")
stage, root = new_stage(ban_path, "Banana")
pts, radii = _banana_points()
for i, (pt, r) in enumerate(zip(pts, radii)):
    sp = UsdGeom.Sphere.Define(stage, f"/Banana/Seg{i}")
    sp.CreateRadiusAttr().Set(r)
    UsdGeom.Xformable(sp.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*pt))
    UsdPhysics.CollisionAPI.Apply(sp.GetPrim())
    sp.GetPrim().GetAttribute("physics:collisionEnabled").Set(True)
    UsdShade.MaterialBindingAPI.Apply(sp.GetPrim()).Bind(material(stage, f"/Banana/Seg{i}/Looks/M", BANANA_COLOR))
for i in range(BANANA_SEGS):
    a, b = pts[i], pts[i + 1]
    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2)
    seg_len = math.dist(a, b)
    # 胶囊默认长轴沿 Z：先绕 X 转 90°（放平，长轴 → -Y），再绕 Z 转到该段方向。
    # ⚠️ 只绕 Z 转是没用的（那只是让胶囊绕自身轴自转，仍然是竖着的）—— 实测踩过：
    #    第一版香蕉就是"竖着站"的，静置中心高度 0.821 而不是 0.804。
    #    USD 的 rotateXYZ = Rz·Ry·Rx，所以 rot=(90, 0, ψ+90°) 正好是"先放平再转向"。
    psi = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    add_capsule(stage, f"/Banana/Link{i}", (radii[i] + radii[i + 1]) / 2, seg_len, mid, BANANA_COLOR,
                rot_deg=(90.0, 0.0, psi + 90.0))
end = pts[-1]
add_cylinder(stage, "/Banana/Stem", 0.006, 0.03,
             (end[0] + 0.012, end[1] + 0.008, 0.0), BANANA_STEM_COLOR, rot_deg=(90, 0, 0))
UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr().Set(0.12)
stage.GetRootLayer().Save()
print(f"[props] 已写出 {ban_path} ({os.path.getsize(ban_path)} 字节)  兜底香蕉，弧长≈"
      f"{2 * BANANA_R * math.radians(BANANA_HALF_DEG):.3f} m")
print("[props] 提示：官方 YCB 香蕉在 assets/scenes/smart_factory/ycb_011_banana.usd（本地二进制，不进 git），"
      "任务会优先用它")
