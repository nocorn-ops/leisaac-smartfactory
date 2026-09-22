"""光轮厨房 + 官方 SO101 双臂 + 可移动底盘 — 数据录制 & 移动操作任务。"""
import isaaclab.sim as sim_utils
import numpy as np
import torch
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, TiledCameraCfg
from isaaclab.sensors.ray_caster.patterns import LidarPatternCfg
from isaaclab.utils import configclass
from isaaclab.scene import InteractiveSceneCfg
from leisaac.assets.robots.lerobot import SO101_FOLLOWER_CFG
from leisaac.assets.scenes.kitchen import KITCHEN_WITH_ORANGE_CFG, KITCHEN_WITH_ORANGE_USD_PATH
from leisaac.utils.constant import REPO_ROOT
from leisaac.utils.general_assets import parse_usd_and_create_subassets
from leisaac.utils.physx_compat import apply_physx_settings

from ..template import (
    BiArmObservationsCfg,
    BiArmTaskEnvCfg,
    BiArmTerminationsCfg,
)
from ..template import mdp
from .mdp import make_success_termination_cfg, oranges_plate_distance


# ── 底盘碰撞体参数 ──────────────────────────────────────────────────────────
# Body 的网格是 **instanced 纯视觉**（IsaacLab 警告 "Could not perform modify_collision_properties"，
# 子树里 mesh=0、没有碰撞/刚体 API），**没法在原模型上开碰撞**，所以单独放一个隐形立方体碰撞体跟着底盘走。
#   世界 AABB = 0.4354 × 0.4563 × 0.9812（含躯干立柱，高到 0.99）
#   底盘初始朝向 85.8° → 反解出**底盘自身坐标系**长宽 ≈ (0.428, 0.405)
#   高度只取到 z≈0.55：手臂基座 z≈0.57 起，取 0.54 高可以避开，**不会推挤自己的手臂**
BODY_COLLIDER_SIZE = (0.43, 0.41, 0.54)
#: 碰撞体中心相对 Body 原点的偏移（世界系，初始朝向下）：AABB 中心 z=0.50 → 车身中心 z≈0.28
BODY_COLLIDER_OFFSET = (0.001, -0.011, 0.27)


@configclass
class LeRobotKitchenSceneCfg(InteractiveSceneCfg):
    """厨房场景 + 两个官方 SO101 机械臂 + 躯干视觉模型."""

    scene: AssetBaseCfg = KITCHEN_WITH_ORANGE_CFG.replace(prim_path="{ENV_REGEX_NS}/Scene")

    left_arm: ArticulationCfg = SO101_FOLLOWER_CFG.replace(prim_path="{ENV_REGEX_NS}/Left_Robot")
    left_arm.spawn.articulation_props.enabled_self_collisions = False

    right_arm: ArticulationCfg = SO101_FOLLOWER_CFG.replace(prim_path="{ENV_REGEX_NS}/Right_Robot")
    right_arm.spawn.articulation_props.enabled_self_collisions = False

    # 躯干/底盘 — 静态视觉模型（通过 USD API 直接移动 prim，无需 RigidBodyAPI）
    body: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Body",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(REPO_ROOT / "lerobot_robot_1" / "SubUSDs" / "lerobot_robot_no_arms_base.usd"),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.29177, -0.84691, 0.01),
            # IsaacLab 3.0 起四元数顺序由 WXYZ 改为 XYZW（见 IsaacLab
            # docs/source/migration/migrating_to_isaaclab_3-0.rst），这里是 2.x 的
            # (w,x,y,z)=(0.7323,0,0,0.6810) 重排后的结果。
            rot=(0.0, 0.0, 0.6810, 0.7323),
        ),
    )

    # 底盘碰撞体（隐形、运动学）：跟着 Body 走，给底盘"体积"，能推开物体、能被接触传感器检测
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

    # 底盘碰撞检测（用于"撞到东西就停"的虚拟保险杠）
    body_contact: ContactSensorCfg = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/BodyCollider",
        update_period=0.0,
        history_length=1,
        debug_vis=False,
    )

    # 左腕相机
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

    # 右腕相机
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

    # 前视相机（躯干）
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

    # 2D 激光雷达（360° 平面扫描，模拟 LSLIDAR LSX10）
    # mesh_prim_paths 在 __post_init__ 中动态设置（不支持 {ENV_REGEX_NS} 占位符）
    # ⚠️ LiDAR 的 frame **不能挂在纯视觉的 Body prim 上**：Body 是用 USD xform 写的，
    #    不会进物理帧视图 → RayCaster 的位姿会**冻结在初始位置**（底盘开走了雷达还停在原地）。
    #    挂到运动学碰撞体 BodyCollider 上（它是 write_root_pose_to_sim 移动的）才会跟随。
    lidar: RayCasterCfg = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/BodyCollider",
        mesh_prim_paths=[],  # 在 __post_init__ 中填充
        # 碰撞体原点在 z≈0.28 → 这里 z=-0.02 让它落在"底盘上方 25cm"（原语义）
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, -0.02)),
        pattern_cfg=LidarPatternCfg(
            channels=1,
            vertical_fov_range=(0.0, 0.0),       # 单平面 2D 扫描
            horizontal_fov_range=(-180.0, 180.0),  # 360° 全向
            horizontal_res=1.0,                    # 1° 分辨率 = 360 rays
        ),
        max_distance=8.0,  # 匹配 duojin01 nav2 laser_max_range
        debug_vis=False,   # 关闭射线可视化（初始化时有 NoneType bug）
    )

    light = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


@configclass
class LeRobotKitchenObservationsCfg(BiArmObservationsCfg):
    """观测：关节 + 三路相机."""

    @configclass
    class PolicyCfg(ObsGroup):
        left_joint_pos = ObsTerm(func=mdp.joint_pos, params={"asset_cfg": SceneEntityCfg("left_arm")})
        left_joint_vel = ObsTerm(func=mdp.joint_vel, params={"asset_cfg": SceneEntityCfg("left_arm")})
        left_joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("left_arm")})
        left_joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("left_arm")})

        right_joint_pos = ObsTerm(func=mdp.joint_pos, params={"asset_cfg": SceneEntityCfg("right_arm")})
        right_joint_vel = ObsTerm(func=mdp.joint_vel, params={"asset_cfg": SceneEntityCfg("right_arm")})
        right_joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("right_arm")})
        right_joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel, params={"asset_cfg": SceneEntityCfg("right_arm")})

        actions = ObsTerm(func=mdp.last_action)
        left_wrist = ObsTerm(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("left_wrist"), "data_type": "rgb", "normalize": False}
        )
        right_wrist = ObsTerm(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("right_wrist"), "data_type": "rgb", "normalize": False}
        )
        front = ObsTerm(
            func=mdp.image, params={"sensor_cfg": SceneEntityCfg("front"), "data_type": "rgb", "normalize": False}
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class LeRobotKitchenTerminationsCfg(BiArmTerminationsCfg):
    """厨房任务终止条件 = 超时 + **成功判据**（橘子都进盘子）。

    模板里的 `BiArmTerminationsCfg` 只有 time_out（它本来是纯采数据任务），
    没有 success 的话 `policy_inference.py` 的成功率永远是 0（其实是"超时"）。
    """

    success = make_success_termination_cfg()


@configclass
class LeRobotKitchenEnvCfg(BiArmTaskEnvCfg):
    """厨房 + 官方 SO101 双臂."""

    scene: LeRobotKitchenSceneCfg = LeRobotKitchenSceneCfg(env_spacing=8.0)
    observations: LeRobotKitchenObservationsCfg = LeRobotKitchenObservationsCfg()
    terminations: LeRobotKitchenTerminationsCfg = LeRobotKitchenTerminationsCfg()
    task_description: str = "Pick oranges with the leRobot bi-arm manipulator."
    #: 评测进度指标（评测脚本会周期性调用它打印"橘子离盘子多远"，不参与训练/终止）
    success_metric_fn: object = None
    success_metric_cfg: object = None
    #: 是否启用 `success` 终止项。**默认 False**：
    #:   IsaacLab 的 env.step() 一旦有终止项触发就会**自动 reset** 该 env，
    #:   而遥操录制脚本不检查 reset_terminated → 采数据时会突然重置场景、录出断裂的数据。
    #:   所以采数据/遥操保持关闭；只有**评测**（如 policy_inference.py）才打开它。
    enable_success_termination: bool = False

    def enable_success(self) -> None:
        """打开 success 终止项（评测前调用；见 `enable_success_termination` 字段注释）。

        评测脚本按鸭子类型调用（`getattr(env_cfg, "enable_success", None)`），
        不需要知道具体任务名 —— 任何任务都可以提供同名方法。
        """
        self.enable_success_termination = True
        self.terminations.success = make_success_termination_cfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        # 视口相机（GUI）。历史：厨房台面/盘子固定在 y≈-0.44，而机器人曾在 2026-07-02 整体平移
        # Δy=+1.046（见 CLAUDE.md §13）；viewer 跟着机器人被搬走、厨房没动 → 看向台面外 1 米处的
        # 空场景（视口全黑）。修好后又发现**原机位几乎与台面齐平**：盘子是侧着看的，容易被看成
        # "倒扣的圆顶"（用户反馈"盘子倒置"）。2026-09-12 改成**机器人后上方俯视台面**的机位
        # （eye 在机器人后方 0.75m、高 1.6m，看向台面上盘子/橙子所在的工作区中心），
        # 盘子/橙子、双手爪、台面全部清清楚楚。对比截图见 HANDOFF.md §6.6-4。
        self.viewer.eye = (2.20, -1.60, 1.60)
        self.viewer.lookat = (2.25, -0.45, 0.60)
        self.dynamic_reset_gripper_effort_limit = False
        self.scene.left_arm.init_state.pos = (2.14527, -0.76399, 0.54)
        self.scene.right_arm.init_state.pos = (2.48527, -0.79399, 0.54)
        # 双臂 root 朝向：原值 (0,0,0,1) 是按 2.x 的 wxyz 写的 = 绕 Z 轴 180°（2.x 的单位四元数是
        # (1,0,0,0)，见 IsaacLab 2.3 的 asset_base_cfg.py）。3.0 起四元数是 XYZW，(0,0,0,1) 变成
        # "不旋转"，双臂朝向会与录制数据差 180°。这里保留原语义：绕 Z 轴 180° = (0,0,1,0)。
        # 依据：datasets/kitchen_biarm.hdf5 中 demo_0 录制的 left/right_arm root_pose 四元数正是
        # (0,0,0,1)（2.x 的 wxyz 写法）。
        self.scene.left_arm.init_state.rot = (0.0, 0.0, 1.0, 0.0)
        self.scene.right_arm.init_state.rot = (0.0, 0.0, 1.0, 0.0)

        # success 终止项默认关闭（见上面字段注释）；评测脚本调 `enable_success()` 打开
        if not self.enable_success_termination:
            self.terminations.success = None

        # 评测用的进度指标（见 mdp/terminations.py::oranges_plate_distance）
        self.success_metric_fn = oranges_plate_distance
        self.success_metric_cfg = {
            "oranges_cfg": [SceneEntityCfg("Orange002"), SceneEntityCfg("Orange003")],
            "plate_cfg": SceneEntityCfg("Plate"),
        }

        # 底盘碰撞体：位置 = Body 初始位置 + 偏移，朝向与 Body 相同（XYZW）
        _bp = self.scene.body.init_state.pos
        self.scene.body_collider.init_state.pos = (
            _bp[0] + BODY_COLLIDER_OFFSET[0],
            _bp[1] + BODY_COLLIDER_OFFSET[1],
            _bp[2] + BODY_COLLIDER_OFFSET[2],
        )
        self.scene.body_collider.init_state.rot = self.scene.body.init_state.rot

        # LiDAR 扫描目标：厨房场景中的所有物体（env_0 的 Scene prim）
        self.scene.lidar.mesh_prim_paths = ["/World/envs/env_0/Scene"]

        # ── 底盘可移动配置 ──
        # 机械臂相对于底盘 body 的位置偏移（body 移动时 arms 保持相对位置不变）
        body_pos = np.array(self.scene.body.init_state.pos)
        self._left_arm_offset = np.array(self.scene.left_arm.init_state.pos) - body_pos
        self._right_arm_offset = np.array(self.scene.right_arm.init_state.pos) - body_pos
        self._body_init_rot = np.array(self.scene.body.init_state.rot)  # quaternion (x,y,z,w)，IsaacLab 3.0 起为 XYZW

        # 底盘速度限制 (m/s, rad/s)
        self.chassis_max_vx = 0.6
        self.chassis_max_vy = 0.6
        self.chassis_max_wz = 2.0

        # 从场景 USD 中提取桌子、橙子、盘子为独立刚体，使其获得 PhysX 碰撞
        parse_usd_and_create_subassets(
            KITCHEN_WITH_ORANGE_USD_PATH,
            self,
            specific_name_list=["Table038", "Orange001", "Orange002", "Orange003", "Plate"],
        )
        # 桌子设为静态碰撞体（固定不动，不参与动力学计算）
        if hasattr(self.scene, "Table038"):
            from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg

            self.scene.Table038.spawn = RigidObjectSpawnerCfg(
                func=self.scene.Table038.spawn.func,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    rigid_body_enabled=False,  # 静态碰撞体，不受重力影响
                    kinematic_enabled=False,
                ),
                collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            )

        # 关于台面物体的出生高度：**不需要任何抬高**，保持场景 USD 里的原始位姿即可。
        # 历史（2026-09-12）：曾经看到"物体出生即嵌进台面 4cm、去穿透乱弹、盘子像倒扣的碗"，
        # 一度以为是 6.0 的台面变高了，于是加过"抬高 5cm"的补丁 —— 那是**错的**。
        # 真正原因是 `utils/general_assets.py::get_prim_pos_rot` 把 USD 的 WXYZ 四元数直接塞进
        # IsaacLab 3.0 要求 XYZW 的 `init_state.rot`：没有旋转的物体变成"绕 X 轴 180°"，
        # 翻转后几何比原点多往下伸 ~4cm 才插进台面。修好四元数后实测：
        #   出生(原始 USD) → 静止        （台面顶面 = 0.5631，5.1 录制数据静止 z ≈ 0.5629）
        #   Plate      0.5702 → 0.5631
        #   Orange002  0.5657 → 0.5619
        #   Orange003  0.5643 → 0.5627
        # 即只掉 2~7mm、姿态不变，与原项目/录制数据完全一致。
        # 量化工具：reproduce/measure_geometry.py（含世界包围盒，平放时 dz 很小）。

        # 降低 GPU 物理内存（默认值对复杂厨房场景过高，8GB 显存不够）
        # gpu_found_lost_aggregate_pairs_capacity 默认 2^25=640MB → 2^20=20MB
        apply_physx_settings(
            self.sim, gpu_found_lost_aggregate_pairs_capacity=2**20, gpu_max_rigid_contact_count=2**19
        )

        # 增加 PD 控制器力和刚度，防止夹爪碰到橙子时手臂被弹开
        for act_name, act_cfg in self.scene.left_arm.actuators.items():
            act_cfg.effort_limit_sim = 50.0
            act_cfg.stiffness = 35.0
            act_cfg.damping = 1.2
            act_cfg.velocity_limit_sim = 20.0
        for act_name, act_cfg in self.scene.right_arm.actuators.items():
            act_cfg.effort_limit_sim = 50.0
            act_cfg.stiffness = 35.0
            act_cfg.damping = 1.2
            act_cfg.velocity_limit_sim = 20.0

