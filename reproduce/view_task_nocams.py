#!/usr/bin/env python
"""LeIsaac 场景 GUI 查看器（无相机版）。

与 scripts/view_task.py 相同，但在构建环境前剔除相机传感器及其图像观测词条：
Isaac Sim 6.0.1 下 omni.syntheticdata 在激活相机渲染图节点时会报
``Invalid object in Py_Graph``，而"肉眼看场景"并不需要腕部/前视相机。

用法（仓库根目录为 cwd）:
    reproduce/launch_gui.sh            # 已指向本脚本

调试/自检用参数（都不改变默认行为）:
    --capture PATH        场景就绪后把活动 viewport 截图存成 PNG（用来自证"窗口里到底显示了什么"）
    --capture_after N     就绪后等 N 秒再截图（默认 3，等首帧 shader 编译完）
    --seconds N           运行 N 秒后自动退出（0 = 一直运行到手动关窗）
    --exit_after_capture  截图完成后立即退出（配合 --capture 做无人值守验证）
    --diag                打印可视化器相机 / 活动 viewport / 场景灯光诊断信息
"""
import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)

import argparse
import dataclasses
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="查看 LeIsaac 任务场景（无相机）。")
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0", help="任务名")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--seed", type=int, default=None)
parser.add_argument("--keep_cameras", action="store_true", help="保留相机（默认剔除）")
parser.add_argument("--capture", type=str, default=None, help="把活动 viewport 截图存成 PNG")
parser.add_argument("--capture_after", type=float, default=3.0, help="场景就绪后等 N 秒再截图")
parser.add_argument("--seconds", type=float, default=0.0, help="运行 N 秒后自动退出（0 = 一直运行）")
parser.add_argument("--exit_after_capture", action="store_true", help="截图完成后立即退出")
parser.add_argument("--diag", action="store_true", help="打印可视化器/相机/灯光诊断信息")
parser.add_argument(
    "--separate_viewport",
    action="store_true",
    help="让可视化器新建一个独立 viewport 窗口（相机开启时主视口不渲染的绕法）",
)
parser.add_argument("--no_window", action="store_true", help="不开窗口（内部设置 AppLauncher 的 headless 配置键；该键不能作为 CLI 参数名，会和 AppLauncher 冲突）")
parser.add_argument(
    "--settle_steps",
    type=int,
    default=0,
    help="就绪后先做 N 步纯物理静置（与 teleop 的 --settle_steps 一致），用来看'开始遥控那一刻'的画面",
)
parser.add_argument("--dump_cameras", type=str, default=None, metavar="DIR", help="把各相机 RGB 输出存成 PNG 到该目录")
parser.add_argument(
    "--show_cameras",
    action="store_true",
    help="保留相机并给每台相机开一个浮动视口窗口（等于 --keep_cameras + 相机窗口），用来检查视角",
)
parser.add_argument(
    "--eye",
    type=float,
    nargs=3,
    default=None,
    metavar=("X", "Y", "Z"),
    help="覆盖 viewer 相机位置（默认用任务配置里的 viewer.eye）",
)
parser.add_argument(
    "--lookat",
    type=float,
    nargs=3,
    default=None,
    metavar=("X", "Y", "Z"),
    help="覆盖 viewer 注视点（默认用任务配置里的 viewer.lookat）",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

KEEP_CAMERAS = bool(args_cli.keep_cameras or args_cli.show_cameras)
args_cli.enable_cameras = KEEP_CAMERAS
args_cli.headless = bool(args_cli.no_window)

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import time  # noqa: E402

import gymnasium as gym  # noqa: E402
import leisaac  # noqa: E402, F401  注册任务
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.env_utils import get_task_type  # noqa: E402


def _is_image_term(term) -> bool:
    """判断一个 ObsTerm 是否为图像观测。"""
    func = getattr(term, "func", None)
    if func is not None and (getattr(func, "__name__", None) == "image" or "image" in str(func)):
        return True
    params = getattr(term, "params", None)
    return isinstance(params, dict) and "data_type" in params


def _strip_image_terms(cfg_obj) -> int:
    """递归剔除观测/事件配置中的图像词条，返回剔除数量。"""
    removed = 0
    if cfg_obj is None or not dataclasses.is_dataclass(cfg_obj):
        return 0
    for f in dataclasses.fields(cfg_obj):
        try:
            val = getattr(cfg_obj, f.name)
        except Exception:  # noqa: BLE001
            continue
        if val is None:
            continue
        if _is_image_term(val):
            setattr(cfg_obj, f.name, None)
            removed += 1
            continue
        if dataclasses.is_dataclass(val):
            removed += _strip_image_terms(val)
    return removed


env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
env_cfg.recorders = None

if not KEEP_CAMERAS:
    from isaaclab.sensors import TiledCameraCfg

    removed = 0
    for attr in ("observations", "events"):
        if hasattr(env_cfg, attr):
            removed += _strip_image_terms(getattr(env_cfg, attr))
    for name in list(vars(env_cfg.scene).keys()):
        if isinstance(getattr(env_cfg.scene, name), TiledCameraCfg):
            setattr(env_cfg.scene, name, None)
            removed += 1
    print(f"[view] 已剔除 {removed} 个相机观测词条/传感器（--keep_cameras 可保留）")

task_type = get_task_type(args_cli.task)
env_cfg.use_teleop_device(task_type)
env_cfg.recorders = None

# 相机位姿覆盖。注意：IsaacLab 3.0 已废弃 env_cfg.viewer，但仍会自动转发到
# KitVisualizerCfg（见 isaaclab/envs/common.py::_apply_deprecated_viewer_cfg），
# 所以写 viewer 依然有效。任务配置里的 viewer.lookat 目前指向厨房外的空处
# （机器人整体 +Δy=1.046 时 viewer 被一起搬走、但厨房没动），用 --eye/--lookat 修正。
if args_cli.eye is not None:
    env_cfg.viewer.eye = tuple(args_cli.eye)
    print(f"[view] 覆盖 eye = {env_cfg.viewer.eye}")
if args_cli.lookat is not None:
    env_cfg.viewer.lookat = tuple(args_cli.lookat)
    print(f"[view] 覆盖 lookat = {env_cfg.viewer.lookat}")

if args_cli.separate_viewport:
    # 相机开启时（--keep_cameras / --enable_cameras），Kit 主视口的渲染产物会被相机的
    # Replicator 渲染产物顶掉 —— 表现为主视口空/黑，只有 widget 底色。
    # 让可视化器自建一个独立 viewport 窗口可以绕开：通过 sim.visualizer_cfgs 显式传入
    # create_viewport=True 的 KitVisualizerCfg（CLI 显式选 kit 时它会保留这份 cfg，
    # 并把 env_cfg.viewer 转发来的 eye/lookat 抄进来）。
    from isaaclab_visualizers.kit import KitVisualizerCfg

    env_cfg.sim.visualizer_cfgs = [KitVisualizerCfg(create_viewport=True)]
    print("[view] 已开启独立可视化视口（visualizer_cfgs=[KitVisualizerCfg(create_viewport=True)]）")
if args_cli.seed is not None:
    env_cfg.seed = args_cli.seed

print(f"[view] 正在加载: {args_cli.task}")
env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()

if args_cli.settle_steps > 0:
    # 与 teleop_se3_agent.py 的启动静置一致：纯物理步进（不走 env.step，不污染录制）
    print(f"[view] 物理静置 {args_cli.settle_steps} 步（模拟'按 B 之前'的稳定过程）...")
    for _ in range(args_cli.settle_steps):
        env.sim.step(render=False)
        env.scene.update(env.physics_dt)
    print("[view] 静置完成")
    for _name in ("Plate", "Orange002", "Orange003"):
        if hasattr(env.scene, _name):
            _o = env.scene[_name]
            _p = _o.data.root_pos_w.torch[0] if hasattr(_o.data.root_pos_w, "torch") else _o.data.root_pos_w[0]
            _q = _o.data.root_quat_w.torch[0] if hasattr(_o.data.root_quat_w, "torch") else _o.data.root_quat_w[0]
            print(f"[view] {_name}: pos=({_p[0]:.4f},{_p[1]:.4f},{_p[2]:.4f}) quat=({_q[0]:.4f},{_q[1]:.4f},{_q[2]:.4f},{_q[3]:.4f})")

if args_cli.show_cameras:
    from leisaac.utils.camera_view import create_camera_view_windows

    _cam_windows = create_camera_view_windows(env)

print("[view] 场景已就绪。关闭窗口退出。")

_capture_done = args_cli.capture is None  # 没要求截图就视为已完成（好让 --dump_cameras 单独用也能退出）


def _print_diag() -> None:
    """打印影响画面显示的关键状态：可视化器相机、活动 viewport、场景灯光。"""
    viz_cfg = None
    for viz in getattr(env.sim, "visualizers", []):
        cfg = getattr(viz, "cfg", None)
        viz_cfg = cfg
        print(
            f"[diag] visualizer={type(viz).__name__} eye={getattr(cfg, 'eye', None)} "
            f"lookat={getattr(cfg, 'lookat', None)} create_viewport={getattr(cfg, 'create_viewport', None)} "
            f"origin_type={getattr(cfg, 'origin_type', None)} bg={getattr(cfg, 'background_color', None)}"
        )
    try:
        import omni.kit.viewport.utility as vp_utils

        vp = vp_utils.get_active_viewport()
        if vp is None:
            print("[diag] 没有活动 viewport")
        else:
            print(f"[diag] viewport={vp.id} camera={vp.get_active_camera()} render_product={vp.render_product_path}")
    except Exception as e:  # noqa: BLE001
        print(f"[diag] viewport 查询失败: {e}")
    try:
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        lights = [p.GetPath().pathString for p in stage.Traverse() if p.GetTypeName().endswith("Light")]
        print(f"[diag] 场景灯光 {len(lights)} 个: {lights[:6]}")
        env_prim = stage.GetPrimAtPath("/World/envs/env_0")
        if env_prim:
            print(f"[diag] /World/envs/env_0 visibility={UsdGeom.Imageable(env_prim).ComputeVisibility()}")
        default_prim = stage.GetDefaultPrim()
        print(f"[diag] stage 默认 prim={default_prim.GetPath().pathString if default_prim else None}")
        # RTX scene partition：IsaacLab 3.0 会给每个 env 打 primvars:omni:scenePartition，
        # 视口相机在 /World/envs 之外，拿不到 token 时 RTX 会把几何体全部裁掉 → 视口全黑。
        import carb

        show_all = carb.settings.get_settings().get("/rtx/scenePartitioning/showAllPartitionsByDefault")
        print(f"[diag] /rtx/scenePartitioning/showAllPartitionsByDefault = {show_all}")
        env_attr = env_prim.GetAttribute("primvars:omni:scenePartition") if env_prim else None
        print(f"[diag] /World/envs/env_0 primvars:omni:scenePartition = {env_attr.Get() if env_attr and env_attr.IsValid() else None}")
        cam_path = _active_camera_path()
        if cam_path:
            cam_prim = stage.GetPrimAtPath(cam_path)
            is_camera = bool(cam_prim) and cam_prim.IsA(UsdGeom.Camera)
            token = cam_prim.GetAttribute("omni:scenePartition") if cam_prim else None
            print(
                f"[diag] 视口相机 {cam_path} is_camera={is_camera} "
                f"omni:scenePartition={token.Get() if token and token.IsValid() else None}"
            )
            # 回读相机实际世界位姿（配置了 eye/lookat 不代表真的生效了）。
            # pxr 用行向量约定：cam->world 的第 0/1/2 行分别是相机 X/Y/Z 轴，前向 = -Z 轴。
            from pxr import Gf

            xf = UsdGeom.XformCache().GetLocalToWorldTransform(cam_prim)
            t = xf.ExtractTranslation()
            fwd = -Gf.Vec3d(xf.GetRow(2)[0], xf.GetRow(2)[1], xf.GetRow(2)[2])
            print(
                f"[diag] 相机实际世界位置=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) "
                f"前向=({fwd[0]:.3f}, {fwd[1]:.3f}, {fwd[2]:.3f})  期望 lookat={getattr(viz_cfg, 'lookat', None)}"
            )
    except Exception as e:  # noqa: BLE001
        print(f"[diag] 场景查询失败: {e}")


def _active_camera_path():
    """返回活动 viewport 的相机 prim 路径。"""
    try:
        import omni.kit.viewport.utility as vp_utils

        vp = vp_utils.get_active_viewport()
        return vp.get_active_camera() if vp is not None else None
    except Exception:  # noqa: BLE001
        return None


_capture_requested = False
_dump_done = False


def _dump_cameras(out_dir: str) -> int:
    """把场景里各相机传感器的 RGB 输出存成 PNG，并打印数值摘要；返回成功的相机数。

    这是"相机到底通没通"的判据：能读出 ndarray 且像素非全黑 = 通了。
    """
    import numpy as np
    from PIL import Image

    os.makedirs(out_dir, exist_ok=True)
    saved = 0
    for name, sensor in getattr(env.scene, "sensors", {}).items():
        data = getattr(sensor, "data", None)
        out = getattr(data, "output", None)
        rgb = out.get("rgb") if isinstance(out, dict) else None
        if rgb is None:
            print(f"[cam] {name}: 没有 rgb 输出（sensor={type(sensor).__name__}）")
            continue
        arr = rgb[0].detach().cpu().numpy()
        print(
            f"[cam] {name}: shape={arr.shape} dtype={arr.dtype} "
            f"min={float(arr.min()):.3f} max={float(arr.max()):.3f} mean={float(arr.mean()):.3f}"
        )
        if arr.dtype != np.uint8:
            arr = (arr[..., :3].clip(0.0, 1.0) * 255.0).astype(np.uint8)
        path = os.path.join(out_dir, f"{name}.png")
        Image.fromarray(arr[..., :3]).save(path)
        print(f"[cam] {name}: 已保存 {path}")
        saved += 1
    print(f"[cam] 共导出 {saved} 个相机")
    return saved


def _schedule_capture(path: str) -> None:
    """调度一次 viewport 截图；文件落盘由主循环轮询确认（该 API 返回的不是 awaitable）。

    会同时抓「活动视口」和「可视化器自建视口」（create_viewport=True 时两者不同）：
    分别存到 path 与 path 加后缀 `.viz`，便于对比哪一个在正常渲染。
    """
    import omni.kit.viewport.utility as vp_utils

    # 有可视化器自建视口时，主路径存「可视化器视口」（那才是有正确机位、给用户看的窗口），
    # 主视口另存 .main.png 备查。
    viz_vp = None
    for viz in getattr(env.sim, "visualizers", []):
        viz_vp = getattr(viz, "_viewport_api", None)
        if viz_vp is not None:
            break
    vp = vp_utils.get_active_viewport()
    if viz_vp is not None and (vp is None or viz_vp.id != vp.id):
        print(f"[view] 抓取可视化器视口 {viz_vp.id} -> {path}")
        vp_utils.capture_viewport_to_file(viz_vp, path)
        if vp is not None:
            vp_utils.capture_viewport_to_file(vp, path.replace(".png", ".main.png"))
    elif vp is not None:
        vp_utils.capture_viewport_to_file(vp, path)
    else:
        print("[view] 没有可用 viewport")


if args_cli.diag:
    _print_diag()

t0 = time.time()
while simulation_app.is_running():
    env.sim.render()
    time.sleep(0.01)
    elapsed = time.time() - t0
    if args_cli.dump_cameras and not _dump_done and elapsed >= args_cli.capture_after:
        print(f"[cam] {elapsed:.1f}s：导出相机图像 ...")
        _dump_cameras(args_cli.dump_cameras)
        _dump_done = True
    if args_cli.capture and not _capture_requested and elapsed >= args_cli.capture_after:
        print(f"[view] {elapsed:.1f}s：抓取 viewport 截图 ...")
        _schedule_capture(args_cli.capture)
        _capture_requested = True
    if _capture_requested and not _capture_done:
        if os.path.isfile(args_cli.capture) and os.path.getsize(args_cli.capture) > 0:
            _capture_done = True
            print(f"[view] 截图已保存: {args_cli.capture}")
        elif elapsed > args_cli.capture_after + 20.0:
            _capture_done = True
            print("[view] 截图超时（20s 内没有文件落盘）")
    if args_cli.exit_after_capture and _capture_done and (not args_cli.dump_cameras or _dump_done):
        print("[view] 截图/相机导出完成，退出")
        break
    if args_cli.seconds and elapsed > args_cli.seconds:
        print(f"[view] 到达 --seconds {args_cli.seconds}，自动退出")
        break

env.close()
simulation_app.close()
