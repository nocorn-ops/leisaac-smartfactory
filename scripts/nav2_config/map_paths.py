"""地图路径解析 —— 照 duojin01 的做法：能自动挑的绝不让用户敲。

约定（与 duojin01 的 `duojin01_bringup/map_paths.py` 一致）:
- 地图默认放在 ``scripts/nav2_config/maps/``。
- ``map`` 参数**留空**时：优先取**文件名是纯数字**的那些里最大的（``1.yaml`` > ``0.yaml``），
  没有数字名就取**最近修改**的那张 → ``ros2 launch ... navigation.launch.py`` 不用带路径。
  ★ 所以**建图请一律用 ``map_name:=auto``**（数字递增），命名地图只作为兼容/临时用途：
    只要目录里存在数字图，命名图就不会被自动选中。
- ``map`` 可以只给名字（``map:=arena`` → ``maps/arena.yaml``），不用写全路径和后缀。
- ``_`` / ``.`` 开头的 yaml 一律跳过（scratch / 隐藏文件，例如自测留下的 ``_selftest.yaml``）。
- 存图时 ``map_name:=auto``：取"最大数字名 + 1"（``0`` → ``1`` → ``2`` …），不覆盖历史地图。

这个模块只被 `scripts/nav2_config/` 下的 launch 文件 import（它们自己把本目录加进 sys.path）。
"""

from __future__ import annotations

import os

#: 地图子目录名（相对 nav2_config 目录）
MAPS_SUBDIR = "maps"

_YAML_EXTS = (".yaml", ".yml")


def get_maps_dir(config_dir: str) -> str:
    """nav2_config 目录 → maps 目录（不保证存在）。"""
    return os.path.join(os.path.abspath(config_dir), MAPS_SUBDIR)


def _list_map_yamls(maps_dir: str) -> list[str]:
    """列出 maps 目录下的 .yaml/.yml 文件（不含子目录）。

    跳过 ``_`` / ``.`` 开头的文件：它们是 scratch / 隐藏文件（例如自测留下的
    ``_selftest.yaml``），不是真正的场地地图，不该被"自动挑最新"选中。
    """
    if not os.path.isdir(maps_dir):
        return []
    out = []
    for name in sorted(os.listdir(maps_dir)):
        if name.startswith(("_", ".")):
            continue
        path = os.path.join(maps_dir, name)
        if name.endswith(_YAML_EXTS) and os.path.isfile(path):
            out.append(path)
    return out


def _numeric_stem(path: str) -> int | None:
    """文件名主干是纯数字就返回它，否则 None。"""
    stem = os.path.splitext(os.path.basename(path))[0]
    return int(stem) if stem.isdigit() else None


def pick_latest_map_yaml(maps_dir: str) -> str:
    """自动挑一张地图：**数字名最大优先**，没有数字名才取最近修改（duojin01 的规则）。

    挑不到就报错并给出建议。
    """
    yamls = _list_map_yamls(maps_dir)
    if not yamls:
        raise FileNotFoundError(
            f"{maps_dir} 下没有任何地图（*.yaml）。先建图："
            f"ros2 launch scripts/nav2_config/ops/mapping.launch.py auto:=true"
        )

    numeric = []
    newest, newest_mtime = None, -1.0
    for path in yamls:
        num = _numeric_stem(path)
        if num is not None:
            numeric.append((num, path))
        mtime = os.path.getmtime(path)
        if mtime > newest_mtime:
            newest, newest_mtime = path, mtime

    if numeric:
        numeric.sort(key=lambda item: item[0])
        return numeric[-1][1]
    return newest  # type: ignore[return-value]


def resolve_map_yaml(map_value: str, config_dir: str) -> str:
    """把 ``map`` 参数解析成真实存在的 yaml 绝对路径；留空 = 自动挑最新。

    ``map`` 可以只给名字（``arena`` → ``maps/arena.yaml``），不用写全路径和后缀。
    """
    raw = (map_value or "").strip()
    maps_dir = get_maps_dir(config_dir)
    if not raw:
        return pick_latest_map_yaml(maps_dir)

    path = os.path.expanduser(raw)
    # 没写扩展名就补 .yaml / .yml 再试（方便 `map:=arena` 这种写法）
    names = [path]
    if not path.lower().endswith(_YAML_EXTS):
        names += [path + ext for ext in _YAML_EXTS]

    candidates = []
    for name in names:
        if os.path.isabs(name):
            candidates.append(name)
        else:
            candidates += [
                os.path.join(config_dir, name),              # 相对 nav2_config/
                os.path.join(maps_dir, name),                # 相对 maps/
                os.path.join(config_dir, "..", "..", name),  # 相对仓库根
            ]

    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.isfile(candidate):
            return candidate

    tried = "\n  ".join(os.path.abspath(c) for c in candidates)
    raise FileNotFoundError(f"找不到地图 {raw!r}，试过：\n  {tried}")


def next_map_name(maps_dir: str) -> str:
    """存图时的自动命名：最大数字名 + 1（没有数字名就从 "0" 开始）。"""
    top = -1
    for path in _list_map_yamls(maps_dir):
        num = _numeric_stem(path)
        if num is not None:
            top = max(top, num)
    return str(top + 1)


def resolve_map_name(name: str, maps_dir: str) -> str:
    """``map_name`` 参数留空或 ``auto`` → 自动递增；否则原样用。"""
    raw = (name or "").strip()
    if not raw or raw.lower() == "auto":
        return next_map_name(maps_dir)
    return raw
