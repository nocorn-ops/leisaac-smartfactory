"""底盘（运载机器人）运动学控制 —— 给"跑任务"的脚本复用。

本项目的底盘是**运动学模型**（不是带轮关节的物理底盘）：直接写 Body 的 USD 变换，
机械臂用 `write_root_pose_to_sim` 跟着走（见 CLAUDE.md §19）。所以：
- 优点：不会因为轮子打滑/摩擦参数导致跑偏，位置可控、可复现；
- 注意：**没有碰撞响应**（会"穿墙"），避障要靠上层（LiDAR / Nav2）自己保证。

用法::

    chassis = ChassisController(env)
    ...
    chassis.step(vx=0.2, vy=0.0, wz=0.5, dt=env.physics_dt)   # 局部速度 (m/s, rad/s)
    x, y, yaw = chassis.pose
"""

from __future__ import annotations

import numpy as np

try:  # 只在有 USD 的进程里可用（Isaac Sim 内）
    import omni.usd
    from pxr import Gf, UsdGeom
except ImportError:  # pragma: no cover
    omni = None


def quaternion_to_yaw(q_wxyz) -> float:
    """从 (w, x, y, z) 四元数提取 yaw。"""
    w, x, y, z = q_wxyz
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def yaw_to_quaternion(yaw: float) -> np.ndarray:
    """yaw → (w, x, y, z) 四元数（绕 Z 轴）。"""
    return np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])


def quat_mul_wxyz(a, b) -> np.ndarray:
    """(w,x,y,z) 四元数乘法 a⊗b（先施加 b 再施加 a）。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def yaw_to_quat_xyzw(yaw: float) -> np.ndarray:
    """yaw → **XYZW** 四元数（IsaacLab 3.0 的约定）。"""
    return np.array([0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0))])


def quat_mul_xyzw(a, b) -> np.ndarray:
    """XYZW 四元数乘法 a⊗b（世界系里"先 b 再 a"）。"""
    ax, ay, az, aw = (float(v) for v in a)
    bx, by, bz, bw = (float(v) for v in b)
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def yaw_of_xyzw(q_xyzw) -> float:
    """从 IsaacLab 3.0 的 XYZW 四元数里取绕 Z 的 yaw。"""
    qx, qy, qz, qw = (float(v) for v in q_xyzw)
    return float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))


class ChassisController:
    """跟踪底盘位姿 (x, y, yaw)，移动 Body prim 并同步两条机械臂。

    Args:
        env: 环境（需要 `cfg.scene.body/left_arm/right_arm` 与 `cfg._left_arm_offset`）。
        env_index: 用哪个 env 的 prim（默认 0）。
    """

    #: 默认跟随底盘一起走的"附件"（手臂 + 底盘碰撞体）
    DEFAULT_ATTACHMENTS = ("left_arm", "right_arm", "body_collider")

    def __init__(
        self,
        env,
        env_index: int = 0,
        attachments: tuple[str, ...] = DEFAULT_ATTACHMENTS,
        collision_force_threshold: float = 20.0,
        lidar_stop_distance: float = 0.0,
        lidar_forward_half_angle_deg: float = 40.0,
    ):
        self._env = env
        cfg = env.cfg
        self.pos = np.array(cfg.scene.body.init_state.pos, dtype=np.float64)
        self.yaw = 0.0  # 相对初始朝向的增量
        # 机械臂的初始位姿/朝向**从 articulation 数据里读**（权威，且是 IsaacLab 3.0 的 XYZW）。
        # ⚠️ 不要假设"机械臂朝向 = 底盘朝向 + 180°"：本项目里底盘初始 yaw≈85.8°、
        #    机械臂初始 yaw=180°（配置 rot=(0,0,1,0)），两者没有这种关系（踩过）。
        stage = omni.usd.get_context().get_stage()
        self._body_prim = stage.GetPrimAtPath(f"/World/envs/env_{env_index}/Body")

        # ★ 初始朝向**从 prim 的世界变换里读**，不要用 cfg 里的四元数元组：
        #   IsaacLab 3.0 的 init_state.rot 是 XYZW（2.x 是 WXYZ），按错约定算出的 yaw 会差几十度，
        #   于是"前进"会把机器人开向错误方向（踩过）。pxr 的 Gf.Quatd 是 (real=w, imaginary=xyz)。
        cache = UsdGeom.XformCache()
        world = cache.GetLocalToWorldTransform(self._body_prim)
        t = world.ExtractTranslation()
        self._body_init_pos = np.array([float(t[0]), float(t[1]), float(t[2])])
        q = world.ExtractRotationQuat()
        qx, qy, qz = (float(q.GetImaginary()[i]) for i in range(3))
        qw = float(q.GetReal())
        self._init_yaw = float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
        self.pos = self._body_init_pos.copy()

        # 跟随底盘的附件（手臂、底盘碰撞体…）：记录它们相对底盘的初始偏移与初始朝向
        self._attachments: list[tuple[str, object, np.ndarray, np.ndarray]] = []
        for name in attachments:
            try:
                obj = env.scene[name]
            except Exception:  # noqa: BLE001
                continue  # 该场景没有这个附件（例如没加碰撞体）
            pos = obj.data.root_pos_w
            pos = pos.torch if hasattr(pos, "torch") else pos
            quat = obj.data.root_quat_w
            quat = quat.torch if hasattr(quat, "torch") else quat
            pos0 = pos[0].detach().cpu().numpy().astype(np.float64)
            quat0 = quat[0].detach().cpu().numpy().astype(np.float64)  # XYZW
            self._attachments.append((name, obj, pos0 - self._body_init_pos, quat0))

        # 传感器：接触力（推挤动态物体时）与 LiDAR（静态障碍"急停"用）
        sensors = getattr(env.scene, "sensors", None) or {}
        self._contact_sensor = sensors.get("body_contact")
        self._lidar = sensors.get("lidar")
        self.collision_force_threshold = float(collision_force_threshold)
        # 0 = 关闭。⚠️ 运动学底盘**不会**被静态障碍（墙/台面）物理挡住（PhysX 不生成
        # kinematic-vs-static 接触），所以"静态障碍不许撞"要靠这个 LiDAR 前向急停来兜。
        self.lidar_stop_distance = float(lidar_stop_distance)
        self.lidar_forward_half_angle = float(np.radians(lidar_forward_half_angle_deg))
        self.blocked = False
        self.last_contact_force = 0.0
        self.last_forward_clearance = float("inf")

        # 初始朝向必须是"纯 yaw"（绕 Z）；否则本类只保留 yaw 部分会丢掉 roll/pitch
        tilt = float(np.sqrt(max(0.0, 1.0 - (qw * qw + qz * qz)) * 4.0))
        if tilt > 1e-3:
            print(f"[chassis] ⚠ 底盘的初始朝向不是纯 yaw（|xy|≈{tilt:.3f}），只保留 yaw={np.degrees(self._init_yaw):.1f}°")

    @property
    def has_collider(self) -> bool:
        """底盘碰撞体是否已挂上（场景配置里有 `body_collider` 时为 True）。"""
        return any(name == "body_collider" for name, *_ in self._attachments)

    @property
    def pose(self) -> tuple[float, float, float]:
        """(x, y, yaw_total) —— yaw 是绝对朝向（含初始朝向）。"""
        return float(self.pos[0]), float(self.pos[1]), float(self._init_yaw + self.yaw)

    def reset(self) -> None:
        """回到初始位姿。"""
        self.pos = self._body_init_pos.copy()
        self.yaw = 0.0
        self._write(0.0, 0.0)

    def set_pose(self, x: float | None = None, y: float | None = None, yaw: float | None = None) -> None:
        """把底盘**直接摆到**世界坐标 ``(x, y, yaw)`` 并立即生效（机械臂跟着一起走）。

        Args:
            x, y: 世界坐标（米）；不传则保持当前值。
            yaw: **绝对朝向**（弧度，与 `pose` 属性同一约定；0 = 朝 +X）；不传则保持当前值。

        用途：数采时不想先开车去作业位 —— 统一入口的 `--start_at shelf` 就是启动时调它。
        """
        if x is not None:
            self.pos[0] = float(x)
        if y is not None:
            self.pos[1] = float(y)
        if yaw is not None:
            self.yaw = float(yaw) - self._init_yaw
        self._write(0.0, 0.0)

    def apply_pose(self) -> None:
        """把**当前**位姿重新写一遍（不改数值）—— 主要是把机械臂重新贴回底盘上。

        ★ 必须在每次 `env.reset()` 之后调用。实测（`reproduce/verify_chassis_pose.py`）：

        | 对象 | `env.reset()` 之后 |
        |---|---|
        | `Body`（底盘） | **不动** —— 我们是直接写 USD xform 的，`reset()` 写回 init 的那次不生效 |
        | `left_arm` / `right_arm` | **飞回场景初始位姿** —— 臂-底盘间距从 0.20 m 变成 **2.78 m** |

        于是按 `N`/`R` 换 demo 时，底盘留在作业位、两条手臂却"飘"回起点那片区域。
        调一次本方法即把附件（手臂/碰撞体）按当前底盘位姿重新写回。
        """
        self._write(0.0, 0.0)

    def update_sensors(self) -> None:
        """更新接触力与前方净空（用于虚拟保险杠 / LiDAR 急停）。"""
        self.last_contact_force = 0.0
        self._contact_dir = None
        if self._contact_sensor is not None:
            forces = getattr(self._contact_sensor.data, "net_forces_w", None)
            if forces is not None:
                forces = forces.torch if hasattr(forces, "torch") else forces
                f = forces[0].reshape(-1, 3).sum(dim=0).detach().cpu().numpy()
                self.last_contact_force = float(np.linalg.norm(f))
                if self.last_contact_force > 1e-6:
                    self._contact_dir = f / self.last_contact_force

        self.last_forward_clearance = float("inf")
        if self._lidar is not None and (self.lidar_stop_distance > 0 or True):
            try:
                origin = self._lidar.data.pos_w[0]
                origin = origin.torch if hasattr(origin, "torch") else origin
                hits = self._lidar.data.ray_hits_w[0]
                hits = hits.torch if hasattr(hits, "torch") else hits
                origin = origin.detach().cpu().numpy()
                hits = hits.detach().cpu().numpy()
                d = hits - origin
                dist = np.linalg.norm(d, axis=1)
                ok = np.isfinite(dist)
                if ok.any():
                    # 只统计**朝车身前方 ±half_angle** 的射线
                    fwd_yaw = self._init_yaw + self.yaw
                    fwd = np.array([np.cos(fwd_yaw), np.sin(fwd_yaw)])
                    ang = np.arctan2(d[:, 1], d[:, 0]) - fwd_yaw
                    ang = (ang + np.pi) % (2 * np.pi) - np.pi
                    in_front = np.abs(ang) <= self.lidar_forward_half_angle
                    sel = ok & in_front
                    if sel.any():
                        self.last_forward_clearance = float(dist[sel].min())
            except Exception:  # noqa: BLE001
                pass

    def step(self, vx: float, vy: float, wz: float, dt: float) -> tuple[float, float, float]:
        """按**本体坐标系**速度 (vx 前, vy 左, wz 逆时针) 前进 dt 秒（带虚拟保险杠）。"""
        total_yaw = self._init_yaw + self.yaw
        cos_y, sin_y = np.cos(total_yaw), np.sin(total_yaw)
        # 本体速度 → 世界速度（绕 Z 转 yaw）
        vx_w = vx * cos_y - vy * sin_y
        vy_w = vx * sin_y + vy * cos_y

        self.update_sensors()
        self.blocked = False
        # ① 接触保险杠：正在往受力反方向顶（即推向障碍物）→ 停住平移，允许转向/后退
        if self.last_contact_force > self.collision_force_threshold and self._contact_dir is not None:
            if vx_w * self._contact_dir[0] + vy_w * self._contact_dir[1] < 0:
                self.blocked = True
                vx_w = vy_w = 0.0
        # ② LiDAR 前向急停：静态障碍（墙/台面）不会给运动学底盘接触力（PhysX 不生成
        #    kinematic-vs-static 接触），所以靠激光测距兜住"继续往前"的分量
        if (
            not self.blocked
            and self.last_forward_clearance < self.lidar_stop_distance
            and (vx_w * cos_y + vy_w * sin_y) > 0
        ):
            self.blocked = True
            vx_w = vy_w = 0.0

        self.pos[0] += vx_w * dt
        self.pos[1] += vy_w * dt
        self.yaw += wz * dt
        self._write(vx_w, vy_w)
        return self.pose

    # ── 内部：写 USD + 机械臂跟随 ─────────────────────────────────────────
    def _write(self, vx_w: float, vy_w: float) -> None:
        import torch

        total_yaw = self._init_yaw + self.yaw
        xform = UsdGeom.Xformable(self._body_prim)
        xform.ClearXformOpOrder()  # 重建 op，避免顺序混乱
        # 精度必须与场景里已有的 op 一致（原 prim 的 xformOp:translate 是 double3，
        # 用 PrecisionFloat 会抛 "typeName 'double3' does not match the requested precision"）。
        xform.AddTranslateOp().Set(Gf.Vec3d(float(self.pos[0]), float(self.pos[1]), float(self.pos[2])))
        xform.AddRotateXYZOp().Set(Gf.Vec3d(0.0, 0.0, float(np.degrees(total_yaw))))

        # 附件（手臂、碰撞体）：位置偏移随 yaw 旋转到世界系，朝向叠加同一个 yaw。
        # ⚠️ 全程用 **XYZW**（IsaacLab 3.0 约定）：`root_quat_w` 与 `write_root_pose_to_sim` 都是 XYZW。
        #    之前把读到的 XYZW 当 WXYZ 重排了一次 → 机械臂朝向完全错乱（底盘一转就"位姿乱飞"）。
        cos_d, sin_d = np.cos(self.yaw), np.sin(self.yaw)
        dev = self._env.device
        dq = yaw_to_quat_xyzw(self.yaw)  # 绕世界 Z 转 self.yaw

        for _name, obj, offset, init_quat_xyzw in self._attachments:
            world = np.array(
                [
                    cos_d * offset[0] - sin_d * offset[1],
                    sin_d * offset[0] + cos_d * offset[1],
                    offset[2],
                ]
            )
            q = quat_mul_xyzw(dq, init_quat_xyzw)
            pose = torch.tensor([[*self.pos + world, *q]], device=dev, dtype=torch.float32)
            obj.write_root_pose_to_sim(pose)


def send_lidar_scan(env, bridge) -> bool:
    """把 RayCaster 的 2D 扫描通过 SimBridge 发给 ROS2（返回是否发送成功）。"""
    if bridge is None:
        return False
    try:
        lidar = env.scene["lidar"]
        sensor_pos = lidar.data.pos_w[0]
        sensor_pos = sensor_pos.torch if hasattr(sensor_pos, "torch") else sensor_pos
        ray_hits = lidar.data.ray_hits_w[0]
        ray_hits = ray_hits.torch if hasattr(ray_hits, "torch") else ray_hits
        sensor_pos = sensor_pos.detach().cpu().numpy()
        hits = ray_hits.detach().cpu().numpy()
        # 命中点 → 距离；无穷远（没打中）按 max_distance 处理
        d = np.linalg.norm(hits - sensor_pos, axis=1)
        max_d = float(lidar.cfg.max_distance)
        d = np.where(np.isfinite(d), np.clip(d, 0.0, max_d), max_d)
        import math

        n = len(d)
        bridge.send_scan(
            {
                "ranges": d.tolist(),
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": 2 * math.pi / n,
                "range_min": 0.05,
                "range_max": max_d,
                "frame_id": "laser_frame",
            }
        )
        return True
    except Exception:  # noqa: BLE001
        return False
