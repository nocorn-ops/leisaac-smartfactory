"""智慧工厂任务挑战赛 —— 场地布局（**纯数据层**，不 import Isaac，方便单独改尺寸）。

坐标约定
--------
场地内区 ``X ∈ [0, 4.0] m``、``Y ∈ [0, 3.0] m``、``Z`` 向上（地面 ``z = 0``）。
原点在"起点区那一侧的左下角"，与赛事文档平面图方向一致（图里 4000mm 横向 = X、3000mm 纵向 = Y）：

    Y=3 ┌──────────────┬────────────────┐
        │ 起点区(黄)    │    停放区(黄)  │
        │              │                │
        │              │  ▉ 收纳区      │   ▉ = 架子：**长边沿 Y（南北向）**、
        │  ▉ 转运存储区 │  （双层架）     │        开口朝 **+X（东）**，机器人在东侧作业
    Y=0 └──────────────┴────────────────┘
        X=0                             X=4

尺寸来源：`智慧工厂任务挑战赛（线下）.docx` 里的场地平面样图（4000 × 3000mm）。
⚠️ 文档写明"本图仅为示例，实际场景后续群内公布" —— 所以**所有尺寸都是参数**，
   拿到官方图纸后只改这个文件即可，任务/场景代码不用动。

★ 两个架子的朝向（2026-09-16 按用户反馈修正）
    平面图里两个柜子都是**竖着的窄长条**（X 方向约 380mm、Y 方向约 1370mm）：
    长边沿 **Y**、开口朝 **±X**，机器人从**东侧（+X）**靠近作业。
    原来把长边放在 X 上（等于转错 90°），已改。

各区域的**高度**是自定的（图上只标了平面）：
    **2026-09-16 整体降低 0.20 m**：收纳区下层台面 z=0.25、上层台面 z=0.58、
    转运存储区放盒平台 z=0.25。上层 0.58 贴近 SO101 手臂基座（z≈0.57），最好操作。
    这三个高度用 ``TIER_LOW_Z / TIER_HIGH_Z / TRANSFER_PLATFORM_Z`` 三个常量控制。

★ 货物摆放（2026-09-16 改）：三件**全部在上层台面（0.58）**，沿架子长轴 Y
    从 −Y 到 +Y 依次 **茄子 → 收纳盒 → 香蕉**。

⚠️ 激光平面 z≈0.26（见 ``LASER_PLANE_Z``）：降 0.20 之后下层台面/放盒平台
    都掉到激光平面以下了，所以两个架子的**实心底座被钳到 ``BASE_MIN_TOP_Z``（0.30）**，
    否则地图里架子会退回"空心轮廓"。代价是下层台面/平台被包进底座（不再是独立一层）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# ── 场地本体 ────────────────────────────────────────────────────────────────
ARENA_X = 4.0  # 场地内区宽（X 方向，对应图里的 4000mm）
ARENA_Y = 3.0  # 场地内区深（Y 方向，对应图里的 3000mm）
FLOOR_T = 0.02  # 地板厚度
WALL_H = 0.35  # 围栏高（要高于激光雷达平面 z≈0.25 才扫得到）
WALL_T = 0.06  # 围栏厚
ZONE_SIZE = 0.72  # 地面分区标记边长
ZONE_T = 0.008  # 分区标记厚度（薄片，不参与碰撞以免绊机器人）

# ── 四个区域中心（地面坐标）─────────────────────────────────────────────────
START_XY = (0.45, ARENA_Y - 0.45)  # 起点区（图左上）
PARK_XY = (ARENA_X - 0.45, ARENA_Y - 0.45)  # 停放区（图右上）
PICKUP_XY = (2.27, 0.85)  # 收纳区：双层架（图中右），长边沿 Y
TRANSFER_XY = (0.75, 0.85)  # 转运存储区：存放架（图左），长边沿 Y

# ── 各区的"作业位"：底盘该停在哪、朝哪（**场地世界坐标**，不是 /odom 的 map 坐标）──────
#: yaw 是**绝对朝向**（弧度）：0 = 车头朝 +X（东）；π = 车头朝 −X（西，正对架子开口）。
#: 用途：`ros2_chassis_teleop.sh --start_at shelf` 可以直接把机器人摆在作业位开工
#:（上肢数采不用先开车过去）；也方便 `ros2_navigation.sh` 的目标点。
#: `shelf` 是**导航实测过**的（SUCCEEDED，真值误差 1.6 cm，见任务 README 的导航表）。
WORK_ZONES = {
    "home": (START_XY[0], START_XY[1], math.pi / 2),  # 起点区（≈ 场景初始朝向 85.8°）
    "shelf": (2.85, 0.85, math.pi),  # ★ 收纳架前，面向 −X（正对开口）—— 上肢数采用这个
    "transfer": (1.35, 0.85, math.pi),  # 转运存储架前（架子在 (0.75, 0.85)，深 0.42）
    "park": (PARK_XY[0], PARK_XY[1], math.pi / 2),  # 停放区
}

#: 开口朝向：+X（东）—— 两个架子都"背板在西、开口朝东"，机器人在东侧作业。
OPENING_DIR = +1
#: 底部**实心封闭**（2026-09-16 用户提醒后加）：
#: 2D 激光雷达只扫 z≈0.25 一个平面，如果柜子底下是空的，射线会穿进柜子内部，
#: 只扫到背板/侧板那几片薄板的边缘 → 建出来的地图里柜子是"几根线 + 中间未知"，
#: 规划器甚至可能把柜子底下当成可通行区域。封成实心底座后，
#: 激光扫到的是一个**完整矩形**，地图里柜子就是一个实心障碍块（也符合真实柜子有底座）。
BASE_H_FRACTION = 1.0  # 底座从地面一直封到下层台面下方（1.0 = 全封）

#: ★ 激光扫描平面高度（2026-09-16 补）：雷达挂在 `body_collider` 上，
#:    碰撞体中心 z ≈ 0.28（body z=0.01 + offset 0.27），雷达 offset −0.02
#:    → **扫描平面 z ≈ 0.26 m**。
#:    ⚠️ **低于这个高度的东西 2D 激光一律扫不到** —— 所以任何想在地图里
#:    表现为"实心障碍块"的结构，都必须在这个高度之上有实体。
LASER_PLANE_Z = 0.26
#: 实心底座顶面的最低要求：比激光平面高 4 cm，保证射线一定打在实心上
BASE_MIN_TOP_Z = LASER_PLANE_Z + 0.04  # = 0.30

# ── 收纳区双层架（长边沿 Y）─────────────────────────────────────────────────
RACK_LEN = 1.20  # 长（Y）
RACK_DEPTH = 0.40  # 深（X）
RACK_BACK_T = 0.03  # 背板厚
RACK_SIDE_T = 0.03  # 侧板厚
RACK_H = 0.80  # 总高（原 1.00，整体降 0.20）
RACK_BOARD_T = 0.03  # 层板厚
#: ★ 2026-09-16 整体降低 0.20 m（原 0.45 / 0.78）。
#:    上层 0.58 正好贴近 SO101 手臂基座高度（z≈0.57），是最好操作的层高。
TIER_LOW_Z = 0.25  # 下层台面**上表面**高度
TIER_HIGH_Z = 0.58  # 上层台面**上表面**高度

#: 货物/盒子的落位（2026-09-16 改）：**三件全部放在上层台面**，
#: 沿架子长轴（Y）从 **−Y 到 +Y** 依次是 **茄子 → 收纳盒 → 香蕉**。
#: （机器人从 +X 东侧靠近、双臂分居 ±Y，"顺序"就是它横向扫过的顺序。）
_TIER_CX = PICKUP_XY[0] + RACK_BACK_T / 2
_TIER_CY = PICKUP_XY[1]
_TIER_SPACING = 0.38  # 三件沿 Y 的间距
#: 上层台面可用范围：X ∈ [2.10, 2.47]、Y ∈ [0.28, 1.42]（层板比架子略小一圈）
#:   茄子 0.20(Y) / 盒子 0.342(Y，含提手) / 香蕉 0.05(Y) —— 合计 0.59，间距 0.38 足够不打架
BANANA_XY = (_TIER_CX, _TIER_CY + _TIER_SPACING)  # 上层台面 +Y 端
CRATE_XY = (_TIER_CX, _TIER_CY)  # 上层台面中间
EGGPLANT_XY = (_TIER_CX, _TIER_CY - _TIER_SPACING)  # 上层台面 −Y 端

# ── 货物尺寸（M2）───────────────────────────────────────────────────────────
CRATE_SIZE = (0.30, 0.20, 0.16)  # 收纳盒外形（X, Y, Z），板厚在 export_props_usd.py 里
BANANA_LEN = 0.20  # 香蕉（YCB 011_banana 大致尺寸）
EGGPLANT_LEN = 0.20  # 茄子总长（含梗）

#: 货物**摆放位姿**：位置取自上面各台面中心；z 给一个"略微悬空"的值，让它们自己落到台面上
#: （比硬算贴合更省事，也顺带验证"落位稳定"）。
#: ★ 2026-09-16：三件**全部在上层台面**，所以 z 都从 TIER_HIGH_Z 起算。
BANANA_Z = TIER_HIGH_Z + 0.05  # 香蕉（半高约 0.024 → 落到 0.604）
EGGPLANT_Z = TIER_HIGH_Z + 0.06  # 茄子（横放半径 0.035 → 落到 0.615）
CRATE_Z = TIER_HIGH_Z + 0.10  # 收纳盒（半高 0.08 → 落到 0.660）
#: 茄子生成时是**立着**的（长轴 Z），摆到架子上转成**横放**（绕 X 转 90° → 长轴沿 Y）
EGGPLANT_ROT_XYZW = (0.7071068, 0.0, 0.0, 0.7071068)
#: 香蕉长轴沿 X 平放（YCB 资产默认就是躺着的，这里只是显式写出来便于调）
BANANA_ROT_XYZW = (0.0, 0.0, 0.0, 1.0)
#: 收纳盒绕 Z 转 90°：长边(0.30)从沿 X 变成沿 Y，**两个端面提手改为朝向 ±Y**。
#: 这样机器人从东侧（+X）靠近时，左右两条手臂正好各对一个提手：
#:   手臂横向间距 ≈ 0.341 m（±0.170），而提手在盒子中心 ±(0.30/2 + 0.021) = ±0.171 m —— 几乎完全对齐。
CRATE_ROT_XYZW = (0.0, 0.0, 0.7071068, 0.7071068)
# ── 转运存储区存放架（同样长边沿 Y、开口朝 +X）──────────────────────────────
TRANSFER_LEN = 1.10
TRANSFER_DEPTH = 0.42
TRANSFER_BACK_T = 0.03
TRANSFER_SIDE_T = 0.03
TRANSFER_H = 0.60  # 原 0.80，整体降 0.20（装下平台上的收纳盒 0.16 高还有余量）
TRANSFER_PLATFORM_Z = 0.25  # 放收纳盒的平台**上表面**高度（原 0.45，整体降 0.20）
#: ⚠️ 平台的**实际可放面**：底座为了被激光扫到会被钳到 `BASE_MIN_TOP_Z`（=0.30），
#:    而平台在 0.25 —— 平台会被包进底座里，所以盒子实际是落在**底座顶面**上。
TRANSFER_SURFACE_Z = max(TRANSFER_PLATFORM_Z, BASE_MIN_TOP_Z)
#: 盒子在转运架上的落位（任务里搬过去用）
TRANSFER_CRATE_XY = (TRANSFER_XY[0] + TRANSFER_BACK_T / 2, TRANSFER_XY[1])
TRANSFER_CRATE_Z = TRANSFER_SURFACE_Z + 0.10

# ── 机器人初始位姿（M3）─────────────────────────────────────────────────────
#: 底盘停在**起点区中心**，车头朝向**收纳区**（第一个作业点）。
#: 朝向决定 ROS2 /odom 坐标系的 +x 方向（见 HANDOFF §8.7），所以这里选"面向收纳架"最好用。
ROBOT_START_XY = START_XY
#: 车头朝 **+X（世界 x 轴）**。为什么不是"直接冲着收纳区"（-43°）：ROS2 的 /odom（以及
#: SLAM 建出的地图）以**机器人初始朝向**为 +x，初始朝向 = 0° 时**地图坐标 = 世界坐标**，
#: 发导航目标点可以直接用场地坐标（收纳架就在 (2.27, 0.85)），不用做坐标换算。
ROBOT_START_YAW_DEG = 0.0

#: ⚠️ 机械臂相对**底盘**的朝向不能乱猜。厨房任务里底盘 yaw=85.8°、两条臂 root 的 yaw=180°
#:    （见 lerobot_kitchen_env_cfg.py），两者差 94.2° —— 这是这台机器人真实的装配关系
#:    （chassis.py 里有专门注释警告"别假设机械臂朝向 = 底盘朝向 + 180°"）。
#:    所以这里保持**相对角度不变**，只把它随底盘朝向一起转过去。
ARM_REL_YAW_DEG = 180.0 - 85.8  # = 94.2
#: 机械臂在**底盘自身坐标系**下的偏移（从厨房任务的世界偏移反解：底盘 yaw=85.8°）
#:   左臂世界偏移 (-0.1465, +0.0829) → 自身系 (+0.0720, +0.1522)
#:   右臂世界偏移 (+0.1935, +0.0529) → 自身系 (+0.0670, -0.1891)
LEFT_ARM_OFFSET_BODY = (0.0720, 0.1522, 0.53)
RIGHT_ARM_OFFSET_BODY = (0.0670, -0.1891, 0.53)
#: 底盘碰撞体相对底盘原点的偏移（自身坐标系）
BODY_COLLIDER_OFFSET_BODY = (0.001, -0.011, 0.27)
BODY_COLLIDER_SIZE = (0.43, 0.41, 0.54)


def yaw_to_quat_xyzw(yaw_deg: float) -> tuple:
    """yaw（度）→ XYZW 四元数（IsaacLab 3.0 约定）。"""
    import math

    half = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def body_frame_to_world(offset_xy, yaw_deg: float):
    """底盘自身坐标系下的 (dx, dy) → 世界系 (dx, dy)。"""
    import math

    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    dx, dy = offset_xy
    return (c * dx - s * dy, s * dx + c * dy)


# ── 颜色（RGB 0~1，用 PreviewSurface 直接上色，不依赖贴图）──────────────────
COLOR_FLOOR = (0.30, 0.48, 0.72)  # 场地地面：图里的蓝色
COLOR_WALL = (0.28, 0.82, 0.86)  # 围栏：图里的青色
COLOR_ZONE_START = (0.88, 0.85, 0.20)  # 起点区：黄
COLOR_ZONE_PARK = (0.88, 0.85, 0.20)  # 停放区：黄
COLOR_RACK = (0.90, 0.91, 0.93)  # 白色双层架
COLOR_SHELF = (0.86, 0.87, 0.90)  # 转运架：浅灰白


@dataclass(frozen=True)
class Box:
    """一个长方体部件（场景生成脚本 / IsaacLab 配置都吃这个结构）。"""

    name: str
    size: tuple[float, float, float]  # (x, y, z) 全尺寸，单位 m
    pos: tuple[float, float, float]  # 中心点世界坐标
    color: tuple[float, float, float]
    collision: bool = True
    note: str = ""


def _rack_boxes() -> list[Box]:
    """收纳区：双层架（背板 + 两侧板 + 两块层板）。

    长边沿 **Y**、开口朝 **+X**（``OPENING_DIR``）：背板在 -X 侧，层板从背板往 +X 伸出。
    """
    cx, cy = PICKUP_XY
    x_back = cx - OPENING_DIR * (RACK_DEPTH / 2 - RACK_BACK_T / 2)
    x_tier = cx + OPENING_DIR * (RACK_BACK_T / 2)  # 层板中心：贴着背板往前
    base_h = max(TIER_LOW_Z - RACK_BOARD_T, BASE_MIN_TOP_Z)
    #: ★ 钳到 BASE_MIN_TOP_Z（0.30）而不是原来的 0.42 = TIER_LOW_Z−板厚：
    #:    整体降 0.20 后下层台面变成 0.25，封到 0.22 就**低于激光平面 0.26**，
    #:    地图里架子会退回"背板+侧板的轮廓线 + 内部未知"（规划器可能把柜底当可通行）。
    #:    代价：下层台面（0.22~0.25）被包进底座里，视觉上不再是独立一层。
    out = [
        Box("PickupBack", (RACK_BACK_T, RACK_LEN, RACK_H), (x_back, cy, RACK_H / 2), COLOR_RACK,
            note="收纳区背板（-X 侧）"),
        #: 实心底座（把底部封上，激光才能扫出完整轮廓）
        Box("PickupBase", (RACK_DEPTH, RACK_LEN, base_h), (cx, cy, base_h / 2), COLOR_RACK,
            note=f"收纳区实心底座（0~{base_h:.2f}m，保证激光扫到完整矩形）"),
    ]
    for i, sy in enumerate((-1, 1)):
        out.append(
            Box(
                f"PickupSide{'SN'[i]}",
                (RACK_DEPTH, RACK_SIDE_T, RACK_H),
                (cx, cy + sy * (RACK_LEN / 2 - RACK_SIDE_T / 2), RACK_H / 2),
                COLOR_RACK,
                note="收纳区侧板（南北两端）",
            )
        )
    for tag, z in (("Low", TIER_LOW_Z), ("High", TIER_HIGH_Z)):
        out.append(
            Box(
                f"PickupTier{tag}",
                (RACK_DEPTH - RACK_BACK_T, RACK_LEN - 2 * RACK_SIDE_T, RACK_BOARD_T),
                (x_tier, cy, z - RACK_BOARD_T / 2),
                COLOR_RACK,
                note=f"收纳区{'下' if tag == 'Low' else '上'}层板（上表面 z={z}）",
            )
        )
    return out


def _transfer_boxes() -> list[Box]:
    """转运存储区：开口朝 +X 的存放架，中间一块平台给收纳盒。"""
    cx, cy = TRANSFER_XY
    x_back = cx - OPENING_DIR * (TRANSFER_DEPTH / 2 - TRANSFER_BACK_T / 2)
    x_mid = cx + OPENING_DIR * (TRANSFER_BACK_T / 2)
    base_h = max(TRANSFER_PLATFORM_Z - 0.03, BASE_MIN_TOP_Z)
    #: 同上：钳到激光平面以上，否则转运架在地图里也是空心轮廓
    #: （代价：放盒平台 0.22~0.25 被包进底座，盒子实际落在底座顶面 0.30 —— 见 TRANSFER_SURFACE_Z）
    out = [
        Box("TransferBack", (TRANSFER_BACK_T, TRANSFER_LEN, TRANSFER_H), (x_back, cy, TRANSFER_H / 2),
            COLOR_SHELF, note="转运架背板（-X 侧）"),
        Box("TransferBase", (TRANSFER_DEPTH, TRANSFER_LEN, base_h), (cx, cy, base_h / 2), COLOR_SHELF,
            note=f"转运架实心底座（0~{base_h:.2f}m）"),
        Box("TransferPlatform", (TRANSFER_DEPTH - TRANSFER_BACK_T, TRANSFER_LEN - 2 * TRANSFER_SIDE_T, 0.03),
            (x_mid, cy, TRANSFER_PLATFORM_Z - 0.015),
            COLOR_SHELF, note=f"放盒平台（上表面 z={TRANSFER_PLATFORM_Z}）"),
        Box("TransferTop", (TRANSFER_DEPTH, TRANSFER_LEN, 0.03), (cx, cy, TRANSFER_H - 0.015),
            COLOR_SHELF, note="转运架顶板"),
    ]
    for i, sy in enumerate((-1, 1)):
        out.append(
            Box(
                f"TransferSide{'SN'[i]}",
                (TRANSFER_DEPTH, TRANSFER_SIDE_T, TRANSFER_H),
                (cx, cy + sy * (TRANSFER_LEN / 2 - TRANSFER_SIDE_T / 2), TRANSFER_H / 2),
                COLOR_SHELF,
                note="转运架侧板（南北两端）",
            )
        )
    return out


def arena_boxes() -> list[Box]:
    """整个场地的静态部件清单（地板 + 围栏 + 分区标记 + 两个架子）。"""
    cx, cy = ARENA_X / 2, ARENA_Y / 2
    boxes = [
        Box("Floor", (ARENA_X, ARENA_Y, FLOOR_T), (cx, cy, -FLOOR_T / 2), COLOR_FLOOR,
            note="场地地面"),
        # 围栏：南北两条比内区略长，和东西两条形成闭合角
        Box("WallSouth", (ARENA_X + 2 * WALL_T, WALL_T, WALL_H), (cx, -WALL_T / 2, WALL_H / 2),
            COLOR_WALL, note="南围栏"),
        Box("WallNorth", (ARENA_X + 2 * WALL_T, WALL_T, WALL_H), (cx, ARENA_Y + WALL_T / 2, WALL_H / 2),
            COLOR_WALL, note="北围栏"),
        Box("WallWest", (WALL_T, ARENA_Y, WALL_H), (-WALL_T / 2, cy, WALL_H / 2), COLOR_WALL,
            note="西围栏"),
        Box("WallEast", (WALL_T, ARENA_Y, WALL_H), (ARENA_X + WALL_T / 2, cy, WALL_H / 2),
            COLOR_WALL, note="东围栏"),
        # 分区标记：只做视觉，不参与碰撞
        Box("ZoneStart", (ZONE_SIZE, ZONE_SIZE, ZONE_T), (*START_XY, ZONE_T / 2), COLOR_ZONE_START,
            collision=False, note="起点区标记"),
        Box("ZonePark", (ZONE_SIZE, ZONE_SIZE, ZONE_T), (*PARK_XY, ZONE_T / 2), COLOR_ZONE_PARK,
            collision=False, note="停放区标记"),
    ]
    boxes += _rack_boxes()
    boxes += _transfer_boxes()
    return boxes


def arena_root_prim(env_regex_ns: str = "{ENV_REGEX_NS}") -> str:
    """场地所有部件挂在同一个根 prim 下（激光雷达 / 碰撞过滤都按这个子树来）。"""
    return f"{env_regex_ns}/Arena"
