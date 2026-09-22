# `LeIsaac-SmartFactory-v0` —— 智慧工厂任务挑战赛场地（M1：场地骨架已完成）

## 这是什么

按赛事文档《智慧工厂任务挑战赛（线下）》里的场地平面样图搭的仿真场地：
**4000mm × 3000mm**，含 **起点区 / 停放区 / 转运存储区 / 收纳区（双层架）**。

任务流程（文档原文）：机器人从起点区导航到收纳区 → 在**双层桌面上**按对应层高把
**香蕉**和**茄子**放进**收纳盒** → 把盒子连货搬到**转运存储区** → 回到**停放区**（不进扣分）。
全程自动、限时 15 分钟。

## 现在的进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| **M1** | 场地骨架：地面 + 围栏 + 四个分区 + 双层收纳架 + 转运架 | ✅ 已完成并通过自检（17 个部件、碰撞属性正确） |
| **M2** | 货物：香蕉 / 茄子 / 收纳盒（带碰撞的动态刚体，摆到对应层高） | ✅ 已完成，静置测试 3/3 通过 |
| **M3** | 移动双臂机器人进场 + 相机 / LiDAR / ROS2 桥接 + 建图导航 | ✅ 已完成（含两次导航实测，误差 3cm / 1.6cm） |
| M4 | 任务脚本、裁判演示、技术报告素材 | ⬜ |

## 文件

```
assets/scenes/smart_factory/smart_factory.usda         场地资产（**可在 GUI 里编辑**，见下面"场地编辑"）
assets/scenes/smart_factory/storage_box.usda           收纳盒（5 块板拼的真空腔，文本 USD）
assets/scenes/smart_factory/eggplant.usda              茄子（胶囊 + 果梗，文本 USD）
assets/scenes/smart_factory/banana.usda                香蕉（弧 + 梗，程序化，文本 USD）
assets/scenes/smart_factory/ycb_011_banana.usd         官方 YCB 香蕉（**本地二进制，不进 git**，当前未使用）
leisaac/assets/scenes/smart_factory_layout.py          场地布局**纯数据**（尺寸/颜色/部件清单，改这里）
leisaac/tasks/smart_factory/__init__.py                注册 LeIsaac-SmartFactory-v0
leisaac/tasks/smart_factory/smart_factory_env_cfg.py   场景（把布局数据变成 IsaacLab 实体）+ 灯光 + 预览相机
reproduce/shot_scene.py                                多视角截图 + 场地自检（headless，无需 GUI）
reproduce/export_arena_usd.py                          把场地导出成可编辑 USD（几秒，不用起 Kit）
reproduce/export_props_usd.py                          生成三件货物 USD（盒子/茄子/香蕉）
reproduce/verify_smart_factory_props.py                M2 验收：货物落位静置测试 + 出图
reproduce/open_smart_factory_gui.sh                    在 Isaac Sim GUI 里打开可编辑场地
reproduce/out/sf_m1/*.png                              M1 的预览图（top / bird / pickup / transfer / startzone）
```

## 怎么看 / 怎么改

```bash
# 重新出图（仓库根目录；约 1 分钟，headless，不弹窗）
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u CONDA_EXE -u VIRTUAL_ENV \
  OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y LD_LIBRARY_PATH="$PWD/reproduce/compat_libs_isaac6" \
  /home/vedal/WorkStation/isaac-sim-6.0.1/python.sh -u reproduce/shot_scene.py \
  --task LeIsaac-SmartFactory-v0 --out reproduce/out/sf_m1

# 临时换个机位看（eye 3 个数 + lookat 3 个数）
... reproduce/shot_scene.py --task LeIsaac-SmartFactory-v0 --out /tmp/sf \
    --view rack=2.6,3.2,1.4,2.6,0.62,0.5

# 开 GUI 自己转着看（复用现成的场景查看器）
bash reproduce/launch_gui.sh --task LeIsaac-SmartFactory-v0
```

改尺寸只动 `smart_factory_layout.py`：`ARENA_X / ARENA_Y / WALL_H / PICKUP_XY / TRANSFER_XY /
TIER_LOW_Z / TIER_HIGH_Z / TRANSFER_PLATFORM_Z / COLOR_*`。

## 布局（世界坐标，米）

```
Y=3 ┌───────────────┬────────────────┐   场地内区 X∈[0,4] Y∈[0,3]
    │ 起点区(黄)    │   停放区(黄)   │   围栏 0.35 高（高于激光平面 z≈0.26，LiDAR 扫得到）
    │ (0.45, 2.55)  │  (3.55, 2.55)  │
    │               │                │   两个架子都是**长边沿 Y 的窄长条**（和平面图一致）：
    │               │  ▉ 收纳区      │     收纳架 (2.27, 0.85)，长 1.20 / 深 0.40 / 高 0.80
    │  ▉ 转运存储区 │  (2.27, 0.85)  │       下层台面 z=0.25、上层台面 z=0.58
    │  (0.75, 0.85) │                │     转运架 (0.75, 0.85)，长 1.10 / 深 0.42 / 高 0.60
Y=0 └───────────────┴────────────────┘       放盒平台 z=0.25
    X=0                             X=4   **开口都朝 +X（东）** → 机器人在架子东侧作业
```

> ★ **2026-09-16 整体降低 0.20 m**（原 0.45 / 0.78 / 0.45，架子总高原 1.00 / 0.80）。
> 上层 0.58 正好贴近 SO101 手臂基座（z≈0.57），是最好操作的层高。

货物落位（2026-09-16 改，常量在 `smart_factory_layout.py`）：
**三件全部放在上层台面（0.58）**，沿架子长轴（Y）从 −Y 到 +Y 依次
**茄子 → 收纳盒 → 香蕉**（间距 0.38 m）。

⚠️ **两个架子的实心底座被钳到 0.30（`BASE_MIN_TOP_Z`）**：整体降低后下层台面（0.25）
和放盒平台（0.25）都掉到**激光平面 z≈0.26 以下**了，而"底座必须高过激光平面才有用"
（否则地图里架子退回"背板+侧板的轮廓线 + 内部未知"，规划器可能把柜底当可通行）。
代价：下层台面/平台被包进底座里，视觉上不再是独立一层。

## M2：三件货物（已完成，静置测试通过）

| 货物 | 资产 | 尺寸 | 位置 | 静置后中心 z |
|---|---|---|---|---|
| 香蕉 | 程序化（弧半径 0.12、张角 100°、中段半径 0.024） | 约 0.21 × 0.05 × 0.05 m | **上层台面 +Y 端**（y=1.23） | **0.604**（= 0.58 + 0.024）✅ |
| 茄子 | 程序化（胶囊 r=0.035 长 0.13 + 果梗） | 0.20 × 0.07 × 0.07 m | **上层台面 −Y 端**（y=0.47，长轴沿 Y 横放） | **0.615**（= 0.58 + 0.035）✅ |
| 收纳盒 | 程序化（5 块板，**内部真空腔** 0.284×0.184×0.152 + **顶部翻边 + 两端提手**） | 0.30 × 0.20 × 0.16 m | **上层台面中间**（y=0.85，绕 Z 转 90°） | **0.660**（= 0.58 + 0.08）✅ |

> 上表位置/z 是**旧的 0.45/0.78 层高**下的记录（0.804 / 0.485 / 0.530）。
> 2026-09-16 整体降到 0.25/0.58、三件都挪到上层之后**已重跑复测**：
> `reproduce/verify_smart_factory_props.py --steps 300` → **三件全部 ✅停住 ✅层高对 ✅没掉下去 ✅没穿模**，
> 落点（水平位移 ≤ 5 mm，不滑）：
>
> | 货物 | 落点 (x, y, z) | 期望 z |
> |---|---|---|
> | 香蕉 | (2.285, 1.225, **0.604**) | 0.600 |
> | 茄子 | (2.285, 0.470, **0.615**) | 0.615 |
> | 收纳盒 | (2.285, 0.850, **0.660**) | 0.660 |

### 收纳盒的两种抓法（2026-09-16 按用户提醒加的）

原来的盒子是光板一块，"怎么搬"根本没法抓。现在两个端面各有一个**提手横杆**（真实周转箱的拎法），
沿口还有一圈**外伸翻边**（可以夹边缘）：

**★ 两个架子各有一块实心底座**（`PickupBase` / `TransferBase`）：原来架子底下是空的，
2D 激光雷达只扫 z≈0.26 一个平面 → 射线穿进架子内部，只扫到背板/侧板几片薄板的边缘，
地图里柜子是"几根线 + 中间未知"，规划器甚至可能把柜子底下当可通行。
加底座后激光扫到的是完整矩形，地图里柜子就是**一个实心障碍块**，物理上也不会再有东西滚到柜子底下。

- 2026-09-16 之前：底座顶面 = 下层台面下沿（z=0→0.42）
- 2026-09-16 整体降 0.20 之后：下层台面变成 0.25，封到 0.22 就**低于激光平面 0.26**，
  所以底座被**钳到 `BASE_MIN_TOP_Z = 0.30`**（0→0.30，见 `smart_factory_layout.py`）
  ⚠️ 代价：下层台面（0.22~0.25）与转运平台（0.22~0.25）被包进底座里，视觉上不再是独立一层；
     转运架的**实际可放面**变成底座顶面 0.30（常量 `TRANSFER_SURFACE_Z`）
- ⚠️ **不变的铁律**：底座必须**高过激光平面**才有用 —— 在 z=0.03 铺一块薄底板激光是扫不到的。

**★ 盒子已绕 Z 转 90°（2026-09-16 用户要求）**：长边 0.30 从沿 X 变成**沿 Y**（顺着货架长度方向），
两个端面提手改为**朝 ±Y（南北）**。

| 抓法 | 抓哪儿（盒子局部坐标，中心为原点） | 适合 |
|---|---|---|
| **双臂拎提手**（推荐） | 两个提手横杆在 `(±(BOX_X/2+0.021), 0, +0.020)`，r=0.006、长 0.10；横杆内侧与端面留 **15 mm** 缝给夹爪伸进去。转 90° 后它们在**世界坐标 ±Y** 方向 | 机器人从 **+X 东侧**靠近，左右手臂**各拎一个提手** —— 手臂横向间距 ≈ 0.341 m（±0.170），提手在盒子中心 ±0.171 m（世界系 ±Y），**几乎完全对齐** |
| **夹翻边** | 顶部翻边在 `z = +0.075`，四周外伸 12 mm、厚 10 mm | 从 **任意一侧**夹住边缘（备份方案） |

尺寸核对（转 90° 之后）：盒子占 **0.224(X) × 0.342(Y) × 0.16(Z)**（含提手/翻边），
下层台面 0.37(X) × 1.14(Y) → X 方向余量充足；与茄子（在 y=0.57，盒子 y∈[0.96,1.30]）**相距 0.29 m 不打架** ✅。
质量 0.35 kg（USD 里写死，任务侧可再覆盖）。

验收命令与判据：
```bash
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u CONDA_EXE -u VIRTUAL_ENV   OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y LD_LIBRARY_PATH="$PWD/reproduce/compat_libs_isaac6" \
  /home/vedal/WorkStation/isaac-sim-6.0.1/python.sh -u reproduce/verify_smart_factory_props.py \
  --steps 300 --render reproduce/out/sf_m2
```
每件货物都要满足：**停住**（末速 < 0.02 m/s）、**层高对**（±2 cm）、**没掉下去**、**没穿模**；
再出三张图（东侧作业视角 / 东北 3/4 / 俯视）人工核对形状。

## M3：机器人进场 + 建图 + 导航（已完成）

机器人沿用厨房那套（同一躯干 USD + 两个官方 SO101 + 运动学底盘），**实体名字与厨房完全一致**
（`body`/`left_arm`/`right_arm`/`body_collider`/`lidar`/`body_contact`/`left_wrist`/`right_wrist`/`front`），
所以 `reproduce/ros2_chassis_teleop.sh`、`ChassisController` 不用改就能用在场地里。

实测（`reproduce/verify_smart_factory_robot.py`，全部 ✅）：
- 底盘落在起点区 (0.450, 2.550) 偏差 **0.0 cm**；两条臂按"底盘自身系偏移"随朝向摆到 (0.522,2.702)/(0.517,2.361)
- **LiDAR 360/360 条射线命中**，0.45~4.02 m（厨房里只有 146 条 —— 见下面"一个 mesh"那条坑）
- 三路相机都出图（240×320）

> 下面两段记录的是 M3 当时用的**旧 shell 入口**（`reproduce/ros2_*.sh`，仍可用）。
> 现在推荐用等价的 `ops/` launch（自带桥接，一条命令一个操作），命令见操作指南 §3 / §4（发布包 `README.md` / 开发机 `README1.md`）：
> 建图 `ros2 launch scripts/nav2_config/ops/mapping.launch.py auto:=true`；
> 导航 `ros2 launch scripts/nav2_config/ops/navigation.launch.py goal:="X Y YAW" localization:=odom`。
> ⚠️ 注意 `maps/` 里现在有数字图（`0/1.yaml`），自动挑图会选中数字最大的那张 ——
> 要用本验证过的场地图请**显式 `map:=arena`**。

建图（`ros2_mapping.sh --auto --file arena`）：地图 **134×101 @0.03m = 4.02×3.03 m**（正好等于场地），
四面墙闭合、坐标轴与场地对齐、空闲 **77.9%**（厨房那张只有 2%）。地图坐标 = **场地坐标 − (0.45, 2.55)**
（因为 /odom 原点 = 机器人起点；车头初始朝 +X，所以两套坐标轴平行）。

导航（`ros2_navigation.sh <arena.yaml> --auto --localization odom X Y YAW`，两次都 SUCCEEDED）：

| 目标 | map 坐标 | 场地坐标 | 结果 | 真值误差 |
|---|---|---|---|---|
| 停放区 | (3.10, 0.00) | (3.55, 2.55) | SUCCEEDED 15.6s | **3.0 cm** |
| 收纳架前（面向货架） | (2.40, -1.70) | (2.85, 0.85) | SUCCEEDED 15.8s | **1.6 cm** |

### M3 修掉的 4 个坑（都影响厨房任务）

1. **IsaacLab 的 RayCaster 只支持一个 Mesh prim**（`len(mesh_prim_paths)!=1` 直接 NotImplementedError；
   传整个 `/Arena` 又因为里面是 Cube 而报 `Invalid mesh prim path`）。
   → 给场地生成一个**不可见的合并网格** `/Arena/RaycastProxy/Mesh`（`export_arena_usd.py` 生成，
   GUI 改完场地后跑 `--proxy-only` 重建），雷达就打到整个场地了。
2. **nav2 的 `footprint` 参数在本版本不生效**（costmap 里 `robot_radius` 一直是默认 0.1，
   等于把 0.44m 的车当 10cm 圆点规划）→ 改用 `robot_radius: 0.26`；
   同时 `CostCritic.consider_footprint` 必须设 false，否则报
   "no robot footprint provided in the costmap" 直接让 Nav2 起不来。
3. **底盘按固定 `dt=1/30` 积分，而主循环实际只有 10~25 Hz** → 实际速度只有指令的 1/3，
   对 Nav2 是致命的（控制器认为走不动 → 一直 recovery → 卡死）→ 改成用**实测墙钟 dt**。
   ⚠️ 改的时候必须保留一个固定的 `target_dt` 给节流用，否则 `(step-1)*dt` 会算出天文数字的 sleep，
   主循环直接睡死（表现：日志停住、GPU 0%、进程还活着 —— 实测踩了这个自作自受的坑）。
4. **全局代价地图**：叠加实时扫描后在 4×3m 场地里出现成片"内切障碍"（代价 99）把场地堵死
   → 全局图改成**只用静态层**（实时避障交给局部图）、`track_unknown_space: false`、
   planner `tolerance` 0.03→0.10、`inflation_radius` 0.30→0.20。

## ⚠️ 等我方/官方确认的几个尺寸（现在是按图 + 手臂可达性估的）

1. **双层桌的实际层高**：现在取 `下层 0.45 m / 上层 0.78 m`。
2. **收纳盒放哪一层**：图纸里"香蕉→茄子→盒子"从上到下画，看起来盒子在地面；
   但 SO101 基座 z≈0.57、伸展约 0.35 m，**够不到地面**，所以我把盒子放在**下层台面 +Y 侧**，
   茄子在**下层 -Y 侧**、香蕉在**上层**。若官方规定盒子必须在地面，就得加一块矮台/坡道。
3. **围栏高度**：现在 0.35 m（要高于 LiDAR 平面才扫得到）；官方如果是矮挡边（<0.25 m），
   建图时激光看不到，得另想办法（例如把雷达高度降到 0.15 m）。
4. **香蕉/茄子/收纳盒的实际尺寸与摆放方式**：现在用的是自定尺寸（见上面 M2 那节），
   拿到实物/官方图纸后改 `smart_factory_layout.py` 里的 `CRATE_SIZE / BANANA_* / EGGPLANT_*`
   或直接在 GUI 里改 USD。
5. **收纳区/转运区的具体结构**（图纸是示意图，实物可能是货架/柜子/桌面）。

> 文档里写了"本图仅为示例，实际场景后续群内公布"——**拿到官方图纸后只改 `smart_factory_layout.py` 一个文件**。

## 实现上的两个坑（已踩过，记下来省得再踩）

1. **不能写 `observations=None`**（想做"没有机器人、只有场景"的环境）：IsaacLab 3.0 的
   `ManagerBase._resolve_terms_callback` 会直接 `self.cfg.__dict__`，None 会在仿真 reset 时
   抛 `AttributeError`。要给"没有任何 term 的空 configclass"。
2. **传感器必须在场景配置里声明**：IsaacLab 3.0 的 sensor 是在 `PHYSICS_READY` 事件里初始化的，
   事后单独 `TiledCamera(cfg)` 拿不到初始化（`is_initialized=False`、连 `_device` 都没有）。
   所以预览相机写成 `SmartFactorySceneCfg.preview`，截图时用
   `env.scene.sensors["preview"].set_world_poses_from_view(eye, lookat)` 改机位。
3. 相机**正下方俯视**时 look-at 的滚转是退化的（图会随机转 180°，实测踩过）→ 俯视机位给 eye
   一点点 Y 偏移。
4. **YCB 官方香蕉用不了（UsdFileCfg 引用后什么都不生成）**：试过
   `Isaac/Props/YCB/Axis_Aligned/011_banana.usd`（也下载到本地试过），
   日志是 `Could not perform 'modify_rigid_body_properties' on any prims under '/World/envs/env_0/Banana'`
   → 目标路径下 0 个 prim。已改成**程序化香蕉**（弧 + 梗，形状够用）；
   以后要换更好的资产，建议先 flatten 成自包含 USD 再试。
5. **胶囊默认是"竖着"的，只绕 Z 转没用**：`UsdGeom.Capsule` 长轴沿 Z，
   要放平必须先绕 **X** 转 90°，再绕 Z 转到目标方向（USD 的 `rotateXYZ` = Rz·Ry·Rx）。
   第一版香蕉就是只绕了 Z → 胶囊全竖着，静置中心高度 0.821 而不是 0.804（实测踩过）。
6. **架子朝向别照"平面图里看着像横的"做**：平面图里两个柜子是**竖着的窄长条**
   （X≈380mm、Y≈1370mm）→ 长边必须沿 **Y**、开口朝 **±X**。第一版把长边放在 X 上（等于转错 90°），
   被用户一眼看出来。凡是"平面图里贴了正视图图标"的结构，先量一下图上的长宽比再定朝向。
