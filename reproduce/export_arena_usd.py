#!/usr/bin/env python
"""把"智慧工厂"场地导出成一个 **USD 文件** —— 导完就能在 Isaac Sim GUI 里直接改，
改完保存（Ctrl+S），任务加载的就是你改过的版本。场地从此**以 USD 文件为准**，
Python（`smart_factory_layout.py`）只负责"初次生成 / 按参数批量重建"。

为什么要导出：之前场地是每次运行时用 Python 代码生成 prim 的，GUI 里拖完的东西存不下来。
导出成 USD 后它就是一个普通资产文件，随便编辑/替换，也能进 git（文本格式）。

★ 实现上刻意**不依赖 IsaacLab / SimulationApp**：这里只需要 USD 本身
（`pxr` 在 Isaac Sim 自带的 python 里就有）。好处：
    - 几秒跑完（不用起 Kit，不用等 60 秒）
    - 不会踩 IsaacLab spawner 依赖全局 stage 的坑
      （实测：headless 下没有 SimulationContext 时，spawn_cuboid 内部
       `get_stage()` 拿到 None → 所有部件都报
       `'NoneType' object has no attribute 'GetPrimAtPath'`）

用法:
    reproduce/export_arena_usd.py                 # 导出到默认路径
    reproduce/export_arena_usd.py --list          # 只看会生成哪些部件（连 pxr 都不用）
    reproduce/export_arena_usd.py --out /tmp/a.usd
    reproduce/export_arena_usd.py --proxy-only    # 只按**当前 USD**重建激光雷达合并网格
                                                  # （在 GUI 里改完场地后跑这个，不用重新生成整个 USD）

默认输出: assets/scenes/smart_factory/smart_factory.usda（**文本 USD**，可以 diff / 直接看）

命令（仓库根目录，用 Isaac Sim 的 python.sh）:
    env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u CONDA_EXE -u VIRTUAL_ENV \\
      OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \\
      LD_LIBRARY_PATH="$PWD/reproduce/compat_libs_isaac6" \\
      <IsaacSim>/python.sh -u reproduce/export_arena_usd.py

（或者直接用 `bash reproduce/open_smart_factory_gui.sh`，它会在需要时自动导出）
"""

import argparse
import importlib.util
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAYOUT_PY = os.path.join(REPO, "source", "leisaac", "leisaac", "assets", "scenes", "smart_factory_layout.py")
DEFAULT_OUT = os.path.join(REPO, "assets", "scenes", "smart_factory", "smart_factory.usda")
ARENA_ROOT = "/Arena"


def load_layout():
    """按**文件路径**直接加载布局模块。

    ⚠️ 不能 `import leisaac.assets...`：那会执行 `leisaac/__init__.py` → `from .tasks import *`
    → 连锁 import isaaclab_tasks，在 Isaac Sim 起来之前导入会把 omni.physx / pxr 绑定搞坏。
    """
    spec = importlib.util.spec_from_file_location("smart_factory_layout", LAYOUT_PY)
    mod = importlib.util.module_from_spec(spec)
    # ⚠️ 必须先登记到 sys.modules：布局模块用了 `from __future__ import annotations` +
    #    @dataclass，dataclasses 会去 sys.modules[cls.__module__] 取命名空间，
    #    不登记就报 `'NoneType' object has no attribute '__dict__'`。
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    return mod


parser = argparse.ArgumentParser(description="导出智慧工厂场地 USD")
parser.add_argument("--out", default=DEFAULT_OUT, help="输出 USD 路径")
parser.add_argument("--root", default=ARENA_ROOT, help="场地根 prim（默认 /Arena，作为 defaultPrim）")
parser.add_argument("--list", action="store_true", help="只打印部件清单，不写 USD")
parser.add_argument("--proxy-only", action="store_true",
                    help="只按**现有 USD**重建激光雷达合并网格（在 GUI 改完场地后跑这个）")
args_cli = parser.parse_args()

layout = load_layout()
if args_cli.list:
    boxes = layout.arena_boxes()
    print(f"将生成 {len(boxes)} 个部件：")
    for b in boxes:
        print(f"  {args_cli.root}/{b.name:18s} size={tuple(round(v, 3) for v in b.size)} "
              f"pos={tuple(round(v, 3) for v in b.pos)} collision={b.collision}  {b.note}")
    sys.exit(0)

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
except ImportError:
    print("[export] ❌ 没找到 pxr（USD）。请用 Isaac Sim 自带的 python：\n"
          "         <IsaacSim>/python.sh -u reproduce/export_arena_usd.py",
          file=sys.stderr)
    sys.exit(2)

# ── 给激光雷达用的"合并网格" ────────────────────────────────────────────────
# 为什么需要它：IsaacLab 的 RayCaster 只支持**一个 Mesh prim**
#   （base_ray_caster.py: `get_first_matching_child_prim(path, typeName == "Mesh")`，
#    并且 len(mesh_prim_paths) != 1 会直接 NotImplementedError）。
# 所以厨房任务的雷达其实只打到了场景里的**第一个 mesh**（这也是它扫描稀疏的原因）。
# 场地要能被 360° 扫到，就得把围栏/货架合成一个网格。
# 做法：读**当前 USD 里**所有 Cube（这样你在 GUI 里改完直接重建即可，不用动 Python 布局）。
PROXY_PARENT = f"{ARENA_ROOT}/RaycastProxy"
PROXY_MESH = f"{PROXY_PARENT}/Mesh"

_CUBE_CORNERS = [
    (-0.5, -0.5, -0.5), (+0.5, -0.5, -0.5), (+0.5, +0.5, -0.5), (-0.5, +0.5, -0.5),
    (-0.5, -0.5, +0.5), (+0.5, -0.5, +0.5), (+0.5, +0.5, +0.5), (-0.5, +0.5, +0.5),
]
_CUBE_FACES = [
    (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
    (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
    (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0),
]


def build_raycast_proxy() -> int:
    """把 /Arena 下所有 Cube 合并成一个不可见 Mesh（给激光雷达用），返回三角形数。"""
    from pxr import Gf, Usd, UsdGeom

    # 先删掉旧的代理（重建时避免重复）
    old = stage.GetPrimAtPath(PROXY_PARENT)
    if old and old.IsValid():
        stage.RemovePrim(PROXY_PARENT)

    UsdGeom.Xform.Define(stage, PROXY_PARENT)
    points, indices = [], []
    xf_cache = UsdGeom.XformCache()
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if prim.GetTypeName() != "Cube" or not path.startswith(ARENA_ROOT + "/"):
            continue
        mat = xf_cache.GetLocalToWorldTransform(prim)
        base = len(points)
        for cx, cy, cz in _CUBE_CORNERS:
            p = mat.Transform(Gf.Vec3d(cx, cy, cz))
            points.append((p[0], p[1], p[2]))
        for a, b, c in _CUBE_FACES:
            indices += [base + a, base + b, base + c]

    mesh = UsdGeom.Mesh.Define(stage, PROXY_MESH)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3] * (len(indices) // 3))
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr().Set("none")
    # 不可见（否则和 Cube 重叠会 z-fighting）；不加 CollisionAPI —— 碰撞还是由 Cube 负责
    UsdGeom.Imageable(mesh.GetPrim()).MakeInvisible()
    return len(indices) // 3

out = os.path.abspath(args_cli.out)
os.makedirs(os.path.dirname(out), exist_ok=True)

if args_cli.proxy_only:
    # 只重建合并网格：直接打开现有 USD，改完存回同一个文件
    if not os.path.isfile(out):
        print(f"[export] ❌ 没有 {out}，先运行不带 --proxy-only 的导出", file=sys.stderr)
        sys.exit(2)
    stage = Usd.Stage.Open(out)
    _tris = build_raycast_proxy()
    stage.GetRootLayer().Save()
    print(f"[export] 已按当前 USD 重建 {PROXY_MESH}（{_tris} 个三角形）→ {out}")
    sys.exit(0)

# 直接从文件建舞台（file-backed root layer）：不需要 Kit / SimulationApp
stage = Usd.Stage.CreateNew(out)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)  # 1 unit = 1 m（与任务里的世界坐标一致）
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

root = UsdGeom.Xform.Define(stage, args_cli.root)
stage.SetDefaultPrim(root.GetPrim())  # 任务用 UsdFileCfg 引用本文件时，引的就是 defaultPrim


def _make_material(prim_path: str, color):
    """一个最简单的 UsdPreviewSurface 材质（纯色，不依赖贴图）。"""
    mat = UsdShade.Material.Define(stage, prim_path)
    shader = UsdShade.Shader.Define(stage, prim_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


boxes = layout.arena_boxes()
fail = []
for box in boxes:
    prim_path = f"{args_cli.root}/{box.name}"
    try:
        cube = UsdGeom.Cube.Define(stage, prim_path)
        cube.GetSizeAttr().Set(1.0)  # 单位立方体 → 用 scale 撑成目标尺寸
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*box.pos))
        xform.AddScaleOp().Set(Gf.Vec3f(*box.size))
        # 碰撞：静态碰撞体 = 有 CollisionAPI、没有 RigidBodyAPI
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        cube.GetPrim().GetAttribute("physics:collisionEnabled").Set(bool(box.collision))
        # 材质
        mat = _make_material(f"{prim_path}/Looks/Arena", box.color)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)
    except Exception as exc:  # noqa: BLE001
        fail.append((box.name, repr(exc)))

_tris = build_raycast_proxy()
stage.GetRootLayer().Save()
print(f"[export] 激光雷达用的合并网格: {PROXY_MESH}（{_tris} 个三角形，不可见、不参与碰撞）")
print(f"[export] 部件 {len(boxes) - len(fail)}/{len(boxes)} 生成成功")
for name, err in fail:
    print(f"[export] ❌ {name}: {err}")
print(f"[export] 已写出: {out} ({os.path.getsize(out) if os.path.isfile(out) else 0} 字节)")
print(f"[export] defaultPrim = {args_cli.root}")
print("[export] 下一步: bash reproduce/open_smart_factory_gui.sh   # 在 Isaac Sim GUI 里编辑，Ctrl+S 保存即生效")
sys.exit(1 if fail else 0)
