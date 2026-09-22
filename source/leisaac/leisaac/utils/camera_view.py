"""在 Isaac Sim GUI 里为相机传感器创建浮动视口窗口。

用途：遥操/采数据时能**同时看到**两个腕部相机 + 一个头部（前视）相机的实时画面。

背景与选型（2026-09-12）：
- 上游 LeIsaac（IsaacLab 2.3）GUI 下**没有**显示相机画面的代码，只有 ``--enable_cameras``
  把相机图像采出来写进 HDF5。IsaacLab 3.0 的 Kit 可视化器也不会自动显示相机。
- IsaacLab 3.0 确实内置了一个"相机流"面板（``KitVisualizerCfg.streaming_view=True``，
  见 ``isaaclab/envs/utils/camera_view.py``），但它的
  ``streaming_sensor_prim_path`` 只能指向**一台** Camera sensor
  （``find_camera_by_prim_path`` 返回单个 Camera，面板是把**同一台相机在多个 env / 多种
  GT 类型**上拼成网格）。我们要同时看三台不同相机，所以改用 Kit 的 viewport window API：
  每台相机各开一个浮动窗口（可拖动/缩放/关闭），窗口里就是那台相机的实时输出 ——
  和录进数据集的是**同一台相机**。

用法::

    from leisaac.utils.camera_view import create_camera_view_windows

    windows = create_camera_view_windows(env)          # 默认 left_wrist/right_wrist/front
    # 保持 windows 的引用，避免被 GC；不需要时由调用方决定要不要调用
"""

from __future__ import annotations

import re

DEFAULT_CAMERA_NAMES = ("left_wrist", "right_wrist", "front")


class CameraImagePanel:
    """把相机传感器的图像直接贴到一个 UI 面板上（三路并排）。

    和 :func:`create_camera_view_windows` 的区别（**性能**）：
    - 视口窗口：**每台相机多开一个 RTX 渲染通道**，实测 +9ms/台（3 台 → 24Hz 掉到 15Hz）。
    - 本面板：**复用相机传感器已经渲染好的图像**（也就是录进数据集的那张），只做一次
      GPU→CPU 拷贝 + 纹理上传，几乎没有额外渲染开销。

    Args:
        env: 已构建好的环境（需 `--enable_cameras`）。
        camera_names: 显示哪些相机。
        width/height: 单张图显示尺寸。
        scale: 显示缩放（1.0 = 按相机原始分辨率）。

    Usage::

        panel = CameraImagePanel(env)
        while ...:
            panel.update()   # 每帧调用
    """

    def __init__(
        self,
        env,
        camera_names: tuple[str, ...] = DEFAULT_CAMERA_NAMES,
        width: int = 320,
        height: int = 240,
        scale: float = 1.0,
        update_every: int = 3,
        #: ⚠️ **必须 ASCII**：omni.ui 的默认字体没有中文字形，中文会渲染成一串 `?`
        #: （2026-09-16 实测：原来的中文标题在窗口标题栏里变成 `??????`）。
        #: 终端 print() 里的中文不受影响，照旧。
        title: str = "Camera views (same 3 cams as teleop/recording)",
    ):
        import omni.ui as ui

        self._env = env
        self._names = tuple(camera_names)
        self._providers: dict[str, object] = {}
        self._sensors: dict[str, object] = {}
        self._update_every = max(int(update_every), 1)
        self._frame_i = 0

        sensors = getattr(env.scene, "sensors", None) or {}
        shown = []
        self._window = ui.Window(title, width=int(width * scale) * min(len(self._names), 3) + 40, height=int(height * scale) + 60)
        with self._window.frame:
            with ui.HStack(spacing=6):
                for name in self._names:
                    sensor = sensors.get(name)
                    if sensor is None:
                        print(f"[camera-view] 场景里没有相机 '{name}'，略过")
                        continue
                    self._sensors[name] = sensor
                    provider = ui.ByteImageProvider()
                    self._providers[name] = provider
                    with ui.VStack(spacing=2):
                        ui.Label(name)
                        ui.ImageWithProvider(provider, width=int(width * scale), height=int(height * scale))
                    shown.append(name)
        print(f"[camera-view] 已创建相机图像面板：{shown}（复用传感器图像，几乎无额外渲染开销）")

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(self._providers.keys())

    def update(self) -> None:
        """把三台相机当前帧贴到面板上（每帧调用一次即可，内部按 ``update_every`` 限频）。

        相机传感器本身是 30Hz 更新（``update_period=1/30``），面板刷太快只是白花
        GPU→CPU 拷贝和纹理上传的时间，所以默认每 3 帧刷一次。
        """
        import numpy as np

        self._frame_i += 1
        if self._frame_i % self._update_every != 0:
            return

        for name, provider in self._providers.items():
            sensor = self._sensors[name]
            out = getattr(getattr(sensor, "data", None), "output", None)
            rgb = out.get("rgb") if isinstance(out, dict) else None
            if rgb is None:
                continue
            arr = rgb[0]
            arr = arr.torch if hasattr(arr, "torch") else arr  # IsaacLab 3.0: ProxyArray
            arr = arr.detach().cpu().numpy()
            if arr.dtype != np.uint8:
                arr = (arr[..., :3].clip(0.0, 1.0) * 255.0).astype(np.uint8)
            h, w = arr.shape[0], arr.shape[1]
            # ByteImageProvider 要 RGBA；相机输出是 RGB，补一个全 255 的 alpha。
            # 注意：set_bytes_data 收的是 **Python 序列**（不是 bytes —— 传 bytes 会报
            # "Unable to cast Python instance of type <class 'bytes'>"）。实测
            # ravel().tolist() 约 1.3ms/张（240×320），3 张约 4ms/帧，可接受。
            rgba = np.empty((h, w, 4), dtype=np.uint8)
            rgba[..., :3] = arr[..., :3]
            rgba[..., 3] = 255
            provider.set_bytes_data(rgba.ravel().tolist(), [w, h])


def resolve_env_prim_path(prim_path: str, env, env_index: int = 0) -> str:
    """把 sensor 的 ``cfg.prim_path`` 变成真正的 USD 路径（env 那级换成 ``env_0``）。

    要点：``cfg.prim_path`` 里写的虽然是 ``{ENV_REGEX_NS}``，但**解析（parse_env_cfg）之后**
    已经是展开后的正则形式 ``/World/envs/env_[^/]+/...``（IsaacLab 约定，见
    ``scene/interactive_scene.py`` 的 ``env_ns``）。两种形式都要处理，否则窗口会绑到不存在的
    相机路径上 —— Kit 会**静默退回**默认透视相机（``/OmniverseKit_Persp``），窗口里就不是
    相机画面了（踩过一次）。
    """
    env_root = None
    env_prim_paths = getattr(getattr(env, "scene", None), "env_prim_paths", None)
    if env_prim_paths:
        try:
            env_root = str(env_prim_paths[env_index])
        except (IndexError, TypeError):
            env_root = None
    if not env_root:
        env_root = f"/World/envs/env_{env_index}"

    if "{ENV_REGEX_NS}" in prim_path:
        prim_path = prim_path.replace("{ENV_REGEX_NS}", env_root)
    # 已经展开的正则形式：/World/envs/env_[^/]+/
    prim_path = re.sub(r"^/World/envs/env_\[\^/\]\+/", env_root + "/", prim_path)
    # 具体数字形式：/World/envs/env_0/
    prim_path = re.sub(r"^/World/envs/env_\d+/", env_root + "/", prim_path)
    return prim_path


def create_camera_view_windows(
    env,
    camera_names: tuple[str, ...] = DEFAULT_CAMERA_NAMES,
    width: int = 320,
    height: int = 240,
    gap: int = 8,
    start_x: int = 8,
    start_y: int = 8,
    env_index: int = 0,
) -> list:
    """为 ``env.scene.sensors`` 里的每个相机各创建一个浮动视口窗口。

    Args:
        env: 已构建好的 IsaacLab 环境（需要 `--enable_cameras` 才有相机传感器）。
        camera_names: 要显示哪些相机（按 scene 里的名字）；缺的会跳过并打印提示。
        width/height: 每个窗口的像素尺寸。
        gap/start_x/start_y: 窗口在屏幕上的排布（从左上角往右排）。

    Returns:
        创建成功的窗口对象列表（**调用方要保留引用**，否则可能被 GC 掉）。
    """
    try:
        import omni.kit.viewport.utility as vp_utils
    except ImportError as exc:  # headless / 未加载 viewport 扩展
        print(f"[camera-view] 跳过相机窗口：无法导入 omni.kit.viewport.utility（{exc}）")
        return []

    windows = []
    sensors = getattr(env.scene, "sensors", None) or {}
    for idx, name in enumerate(camera_names):
        sensor = sensors.get(name)
        if sensor is None:
            print(f"[camera-view] 场景里没有相机 '{name}'，跳过")
            continue
        cfg = getattr(sensor, "cfg", None)
        raw_path = str(getattr(cfg, "prim_path", "") or getattr(sensor, "prim_path", ""))
        prim_path = resolve_env_prim_path(raw_path, env, env_index)
        try:
            win = vp_utils.create_viewport_window(
                name=f"camera: {name}",
                width=width,
                height=height,
                position_x=start_x + idx * (width + gap),
                position_y=start_y,
                camera_path=prim_path,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"[camera-view] 创建 '{name}' 相机窗口失败：{exc}")
            continue
        if win is None:
            print(f"[camera-view] 创建 '{name}' 相机窗口返回 None（UI 不可用？）")
            continue

        # 自检：窗口实际绑到哪台相机（创建时给的路径无效会静默退回默认透视相机）
        actual = None
        try:
            actual = win.viewport_api.camera_path.pathString
        except Exception:  # noqa: BLE001
            try:
                actual = str(win.viewport_api.camera_path)
            except Exception:  # noqa: BLE001
                actual = None
        if actual and actual != prim_path:
            print(f"[camera-view] ⚠ {name}: 窗口相机 = {actual}（期望 {prim_path}）—— 路径可能不对")
        windows.append(win)
        print(f"[camera-view] {name}: 窗口已创建，相机 = {actual or prim_path}")
    if windows:
        print(
            f"[camera-view] 共 {len(windows)} 个相机窗口（可拖动摆放）；"
            "录进数据集的是同三台相机的观测 obs/left_wrist|right_wrist|front"
        )
    return windows
