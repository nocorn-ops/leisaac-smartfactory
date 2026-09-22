"""智慧工厂任务挑战赛 —— 仿真任务（M1：只搭场地，机器人 M3 进场）。

场地（4000mm × 3000mm，含起点区 / 停放区 / 转运存储区 / 收纳区双层架）来自
`leisaac.assets.scenes.smart_factory_layout`（纯数据层，尺寸都在那里改）。

场景里的每个部件都是**基本体**（长方体 + PreviewSurface 上色），不用外部资产，
所以：① 尺寸/颜色完全可控；② 没有二进制依赖；③ 拿官方图纸后只改 layout 就够了。

M1 阶段这个任务**故意不带机器人也不需要动作/观测**：观测/动作/奖励/终止都给**空配置**
（⚠️ 不能写 None —— IsaacLab 3.0 的 `ManagerBase._resolve_terms_callback` 会直接
`self.cfg.__dict__`，None 会在仿真 reset 时抛 `AttributeError`；给个"没有任何 term 的
空 configclass"才是真正可用的空管理器），这样就能直接用 `reproduce/shot_scene.py`
把场地渲染出来看，不被机器人/相机/力控干扰。M2 加货物、M3 把机器人和管理器接回来。
"""

import os

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, TiledCameraCfg
from isaaclab.sensors.ray_caster.patterns import LidarPatternCfg
from isaaclab.utils import configclass
from leisaac.assets.robots.lerobot import SO101_FOLLOWER_CFG

from leisaac.assets.scenes.smart_factory_layout import (
    ARENA_X,
    ARM_REL_YAW_DEG,
    BODY_COLLIDER_OFFSET_BODY,
    BODY_COLLIDER_SIZE,
    LEFT_ARM_OFFSET_BODY,
    RIGHT_ARM_OFFSET_BODY,
    ROBOT_START_XY,
    ROBOT_START_YAW_DEG,
    body_frame_to_world,
    yaw_to_quat_xyzw,
    ARENA_Y,
    BANANA_ROT_XYZW,
    BANANA_XY,
    BANANA_Z,
    CRATE_ROT_XYZW,
    CRATE_SIZE,
    CRATE_XY,
    CRATE_Z,
    EGGPLANT_ROT_XYZW,
    EGGPLANT_XY,
    EGGPLANT_Z,
    arena_boxes,
    arena_root_prim,
)
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils import configclass as _configclass

from ..template import BiArmTaskEnvCfg, BiArmTaskSceneCfg, BiArmTerminationsCfg

from leisaac.utils.constant import REPO_ROOT
from leisaac.utils.physx_compat import apply_physx_settings

#: 场地 USD（**可编辑资产**）：用 `reproduce/export_arena_usd.py` 生成，
#: 之后在 Isaac Sim GUI 里直接改这个文件（Ctrl+S 保存）即可生效 —— 场地以它为准。
SMART_FACTORY_USD = os.path.join(REPO_ROOT, "assets", "scenes", "smart_factory", "smart_factory.usda")
#: 货物资产（M2）：盒子/茄子是本地生成的文本 USD，香蕉用 NVIDIA 官方 YCB 在线资产
PROPS_DIR = os.path.join(REPO_ROOT, "assets", "scenes", "smart_factory")
STORAGE_BOX_USD = os.path.join(PROPS_DIR, "storage_box.usda")
EGGPLANT_USD = os.path.join(PROPS_DIR, "eggplant.usda")
#: 香蕉用**程序化生成**的（文本 USD，弧+梗，约 0.21 m）。
#: ⚠️ 官方 YCB 011_banana（ycb_011_banana.usd）试过但用不了：用 UsdFileCfg 引用它时
#:    Isaac Sim 只在目标路径下什么都不生成（日志里是
#:    `Could not perform 'modify_rigid_body_properties' on any prims under '/World/envs/env_0/Banana'`），
#:    所以先用自己的；要换更好的资产，建议先 flatten 成一个干净的自包含 USD 再试。
BANANA_USD = os.path.join(PROPS_DIR, "banana.usda")


@_configclass
class _EmptyManagerCfg:
    """没有任何 term 的管理器配置（= 空管理器）。"""


#: 四个空管理器（观测 / 动作 / 奖励 / 终止）+ 空事件
EmptyObsCfg = _EmptyManagerCfg
EmptyActionCfg = _EmptyManagerCfg
EmptyRewardCfg = _EmptyManagerCfg
EmptyTerminationCfg = _EmptyManagerCfg
EmptyEventCfg = _EmptyManagerCfg


def _box_asset(box, root: str) -> AssetBaseCfg:
    """把一个 `Box` 变成 IsaacLab 的场景实体（静态碰撞体 + 纯色材质）。"""
    return AssetBaseCfg(
        prim_path=f"{root}/{box.name}",
        spawn=sim_utils.CuboidCfg(
            size=box.size,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=box.color),
            # 静态：有碰撞、没有刚体（不会被推动，也不会掉下去）
            rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=False, disable_gravity=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=box.collision),
            visible=True,
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=box.pos, rot=(0.0, 0.0, 0.0, 1.0)),
    )


@configclass
class SmartFactorySceneCfg(BiArmTaskSceneCfg):
    """场地 + 灯光 + **移动双臂机器人** + 货物。

    场地优先从 **USD 资产**加载（`assets/scenes/smart_factory/smart_factory.usda`）——
    这样你在 Isaac Sim GUI 里改的东西才能生效；USD 不存在时退回"按 Python 布局数据现场生成"
    （保证任何时候都能跑起来，也方便按参数批量重建）。

    机器人沿用厨房任务里那套（同一个躯干 USD + 两个官方 SO101 + 运动学底盘），
    **实体名字保持一致**（`body` / `left_arm` / `right_arm` / `body_collider` / `lidar` /
    `body_contact` / `left_wrist` / `right_wrist` / `front`），
    这样 `reproduce/ros2_chassis_teleop.sh` 和 `ChassisController` 不用改就能直接用在场地里。
    """

    #: 场地：UsdFileCfg 引用 defaultPrim（= /Arena）。文件不存在时是 None → 走 Python 生成。
    #: ⚠️ 字段名必须是 `scene`：模板 `BiArmTaskSceneCfg.scene` 是 MISSING，
    #:    不填会在 `cfg.validate()` 里报 "Missing values ... scene.scene"（实测踩过）。
    scene: AssetBaseCfg = (
        AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Arena",
            spawn=sim_utils.UsdFileCfg(usd_path=SMART_FACTORY_USD),
        )
        if os.path.isfile(SMART_FACTORY_USD)
        else None
    )

    # ── 机器人 ────────────────────────────────────────────────────────────
    #: 双 SO101（官方资产，自碰撞关掉 —— 见 CLAUDE.md §9）
    left_arm: ArticulationCfg = SO101_FOLLOWER_CFG.replace(prim_path="{ENV_REGEX_NS}/Left_Robot")
    left_arm.spawn.articulation_props.enabled_self_collisions = False
    right_arm: ArticulationCfg = SO101_FOLLOWER_CFG.replace(prim_path="{ENV_REGEX_NS}/Right_Robot")
    right_arm.spawn.articulation_props.enabled_self_collisions = False

    #: 躯干/底盘：纯视觉模型（USD xform 由 ChassisController 直接写，所以不挂刚体/碰撞）
    body: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Body",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(REPO_ROOT / "lerobot_robot_1" / "SubUSDs" / "lerobot_robot_no_arms_base.usd"),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(*ROBOT_START_XY, 0.01), rot=yaw_to_quat_xyzw(ROBOT_START_YAW_DEG)
        ),
    )

    #: 底盘碰撞体（隐形、运动学）：给底盘"体积"，能推开物体、能被接触传感器检测
    body_collider: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/BodyCollider",
        spawn=sim_utils.CuboidCfg(
            size=BODY_COLLIDER_SIZE,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visible=False,
            activate_contact_sensors=True,
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0)),
    )

    #: 底盘接触力（"撞到东西就停"的虚拟保险杠）
    body_contact: ContactSensorCfg = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/BodyCollider", update_period=0.0, history_length=1, debug_vis=False
    )

    #: 2D 激光雷达（360° 单平面，模拟 LSLIDAR LSX10）。
    #: ⚠️ frame 必须挂在**运动学碰撞体**上（挂纯视觉 Body 会冻结在初始位置，见 HANDOFF §8.6）；
    #:    mesh_prim_paths 在 __post_init__ 里设成场地 + 货物。
    lidar: RayCasterCfg = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/BodyCollider",
        mesh_prim_paths=[],
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, -0.02)),  # 碰撞体原点 z≈0.28 → 雷达 z≈0.25
        pattern_cfg=LidarPatternCfg(
            channels=1,
            vertical_fov_range=(0.0, 0.0),
            horizontal_fov_range=(-180.0, 180.0),
            horizontal_res=1.0,
        ),
        max_distance=8.0,
        debug_vis=False,
    )

    #: 三路相机（与厨房任务同款同机位）：左腕 / 右腕 / 前视
    left_wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Left_Robot/gripper/left_wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.001, 0.1, -0.04), rot=(-0.912179, -0.0451242, 0.0486914, -0.404379), convention="ros"
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=36.5, focus_distance=400.0, horizontal_aperture=36.83,
            clipping_range=(0.01, 50.0), lock_camera=True,
        ),
        width=320, height=240, update_period=1 / 30.0,
    )
    right_wrist: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Right_Robot/gripper/right_wrist_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(-0.001, 0.1, -0.04), rot=(-0.912179, -0.0451242, 0.0486914, -0.404379), convention="ros"
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=36.5, focus_distance=400.0, horizontal_aperture=36.83,
            clipping_range=(0.01, 50.0), lock_camera=True,
        ),
        width=320, height=240, update_period=1 / 30.0,
    )
    front: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Body/torso_link/front_camera",
        offset=TiledCameraCfg.OffsetCfg(
            pos=(0.3, 0.2, 0.75), rot=(-0.7029630, 0.7095703, -0.0421672, 0.0239783), convention="ros"
        ),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=28.7, focus_distance=400.0, horizontal_aperture=38.11,
            clipping_range=(0.01, 50.0), lock_camera=True,
        ),
        width=320, height=240, update_period=1 / 30.0,
    )

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.80, 0.82, 0.85), intensity=900.0),
    )
    # ── 货物（M2）：三件都做成**动态刚体**，带碰撞 ──────────────────────────
    #: 香蕉（上层台面）：优先官方 YCB 011_banana（本地文件），缺失时用程序化香蕉兜底
    banana: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Banana",
        spawn=sim_utils.UsdFileCfg(
            usd_path=BANANA_USD,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.12),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(*BANANA_XY, BANANA_Z), rot=BANANA_ROT_XYZW),
    )
    #: 茄子（下层台面 -Y 侧）：本地生成的胶囊+梗，默认立着 → 这里绕 X 转 90° 横放
    eggplant: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Eggplant",
        spawn=sim_utils.UsdFileCfg(
            usd_path=EGGPLANT_USD,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.25),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(*EGGPLANT_XY, EGGPLANT_Z), rot=EGGPLANT_ROT_XYZW),
    )
    #: 收纳盒（下层台面 +Y 侧）：5 块板拼的真空腔盒子
    crate: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Crate",
        spawn=sim_utils.UsdFileCfg(
            usd_path=STORAGE_BOX_USD,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.35),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(*CRATE_XY, CRATE_Z), rot=CRATE_ROT_XYZW),
    )

    #: 截图 / 看场地的相机。**必须挂在场景配置里**：IsaacLab 3.0 的传感器是在
    #: "physics ready" 事件里初始化的，事后单独 new 一个 TiledCamera 不会初始化
    #: （实测 `is_initialized=False`、`_device` 都不存在）。
    #: 位置随便给一个初始值，`reproduce/shot_scene.py` 会用 set_world_poses_from_view 改。
    preview: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/PreviewCam",
        offset=TiledCameraCfg.OffsetCfg(pos=(5.4, -2.4, 2.6), rot=(0.0, 0.0, 0.0, 1.0)),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=36.0, clipping_range=(0.05, 100.0)
        ),
        width=960,
        height=720,
        update_period=0.0,
    )

    #: 场地顶上的平行光，让层板/围栏有阴影层次（纯 DomeLight 会显得很平）
    sun = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Sun",
        spawn=sim_utils.DistantLightCfg(color=(1.0, 0.98, 0.94), intensity=700.0, angle=0.8),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(2.0, 1.5, 3.0), rot=(0.6533, 0.2706, 0.2706, 0.6533)),
    )

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.scene is None:
            # USD 还没导出 → 用布局数据现场生成（导出后就走 USD 分支）
            print(
                f"[smart_factory] ⚠️ 没找到场地 USD（{SMART_FACTORY_USD}），"
                f"本次用 Python 布局数据现场生成"
            )
            print(
                "[smart_factory] ⚠️ 注意：这种「现场生成」模式下没有激光雷达用的合并网格"
                "（/Arena/RaycastProxy/Mesh），LiDAR 会起不来。"
                "请先跑：reproduce/export_arena_usd.py（或 open_smart_factory_gui.sh 会自动导出）"
            )
            root = arena_root_prim()
            for box in arena_boxes():
                setattr(self, box.name, _box_asset(box, root))



def apply_robot_pose(scene) -> None:
    """把机器人摆到起点区（**必须在 `BiArmTaskEnvCfg.__post_init__` 之后调用**）。

    ⚠️ 模板 `BiArmTaskEnvCfg.__post_init__` 会把两条臂设成它自己的默认位置 (3.4/-0.65/0.89)，
    所以摆位不能写在场景 cfg 的 `__post_init__` 里 —— 那样会被模板**覆盖掉**
    （实测：臂还停在模板默认位置）。必须在本任务 env cfg 的 `super().__post_init__()` **之后**再设一遍。
    """
# ── 机器人摆位：底盘在起点区中心，两条臂按**底盘自身坐标系**的偏移随朝向转过去 ──
    body_yaw = ROBOT_START_YAW_DEG
    arm_yaw = body_yaw + ARM_REL_YAW_DEG  # 保持厨房任务里"臂相对底盘 94.2°"的装配关系
    for arm, offset in ((scene.left_arm, LEFT_ARM_OFFSET_BODY), (scene.right_arm, RIGHT_ARM_OFFSET_BODY)):
        dx, dy = body_frame_to_world((offset[0], offset[1]), body_yaw)
        arm.init_state.pos = (ROBOT_START_XY[0] + dx, ROBOT_START_XY[1] + dy, offset[2])
        arm.init_state.rot = yaw_to_quat_xyzw(arm_yaw)

    # 底盘碰撞体：位置 = 底盘位置 + 自身系偏移，朝向与底盘一致
    cdx, cdy = body_frame_to_world((BODY_COLLIDER_OFFSET_BODY[0], BODY_COLLIDER_OFFSET_BODY[1]), body_yaw)
    scene.body_collider.init_state.pos = (
        ROBOT_START_XY[0] + cdx, ROBOT_START_XY[1] + cdy, BODY_COLLIDER_OFFSET_BODY[2]
    )
    scene.body_collider.init_state.rot = yaw_to_quat_xyzw(body_yaw)

    # LiDAR 扫描目标：**场地整棵子树**（围栏 + 两个货架都在这下面）。
    # ⚠️ RayCaster 只支持**一个** mesh prim（实测：给 4 个直接
    #    `NotImplementedError: RayCaster currently only supports one mesh prim`），
    #    所以货物没法单独加进来 —— 好在本雷达是 z≈0.25 的**单平面**扫描，
    #    而货物都在 z≥0.45 的台面上，本来就扫不到（想要"看得见货物"得另加相机/雷达）。
    # 打到 `/Arena/RaycastProxy`（不可见的合并网格，由 export_arena_usd.py 生成）：
    # RayCaster 只支持一个 Mesh prim，直接把整个 /Arena 传进去它找不到 Cube 会报
    # "Invalid mesh prim path"（实测）。在 GUI 里改完场地后跑
    # `reproduce/export_arena_usd.py --proxy-only` 重建这个网格。
    scene.lidar.mesh_prim_paths = ["/World/envs/env_0/Arena/RaycastProxy"]


@configclass
class SmartFactoryEnvCfg(BiArmTaskEnvCfg):
    """智慧工厂任务：场地 + 移动双臂机器人 + 货物。

    管理器沿用模板的**双臂**那一套（`actions` / `events` / `rewards` / `terminations` 来自
    `BiArmTaskEnvCfg`），只是观测先留空 —— M3 只需要"机器人能开、相机/雷达能用、ROS2 能接"，
    观测/奖励等 M4 做任务逻辑时再补。
    """

    scene: SmartFactorySceneCfg = SmartFactorySceneCfg(num_envs=1, env_spacing=6.0)

    #: 观测先留空（不是 None，见模块 docstring 的说明）；双臂动作/事件/终止用模板的
    observations: object = EmptyObsCfg()
    terminations: object = BiArmTerminationsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        # ★ 必须在 super() 之后：模板 __post_init__ 会把机械臂设成它自己的默认位置
        apply_robot_pose(self.scene)

        # 物理步长：与厨房任务一致（30 Hz 控制、60 Hz 物理）
        self.decimation = 2
        self.sim.dt = 1.0 / 60.0
        self.sim.render_interval = self.decimation
        # 场景只有静态几何，PhysX 显存需求很小（用兼容层写，IsaacLab 2.x/3.0 都能跑）
        apply_physx_settings(
            self.sim,
            gpu_found_lost_aggregate_pairs_capacity=2**20,
            gpu_max_rigid_contact_count=2**19,
        )
        # 给一份**默认的双臂动作配置**：模板 `BiArmActionsCfg` 的 4 个 term 是 MISSING，
        # 不填会在 cfg.validate() 报错；遥操脚本自己还会再调一次 use_teleop_device（等价，幂等）。
        self.use_teleop_device("bi-so101leader")

        # ★ 提升 PD 刚度/力矩（2026-09-16 补）。
        #   场地原来用的是 `SO101_FOLLOWER_CFG` 的**默认值**（effort_limit_sim=10、
        #   stiffness=17.8、damping=0.6）—— 太弱，**机械臂会被重力拽着下垂**：
        #   实测（`reproduce/verify_bi_keyboard.py`）一条**没被命令**的臂 1 秒内掉了 **22.5 cm**。
        #   后果：遥操手感极差、采下来的数据是脏的（厨房任务早就踩过同样的坑，见
        #   `lerobot_kitchen_env_cfg.py` 里的同名处理）。
        for side in ("left_arm", "right_arm"):
            for _act_name, act_cfg in getattr(self.scene, side).actuators.items():
                act_cfg.effort_limit_sim = 50.0
                act_cfg.stiffness = 35.0
                act_cfg.damping = 1.2
                act_cfg.velocity_limit_sim = 20.0

        # 打开时默认视角：斜俯视整个场地
        self.viewer.eye = (5.6, -2.6, 2.8)
        self.viewer.lookat = (ARENA_X / 2, ARENA_Y / 2, 0.35)
        self.task_description = "智慧工厂任务挑战赛场地（起点区 → 收纳区 → 转运存储区 → 停放区）"
