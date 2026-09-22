"""LeRobot Kitchen 自定义环境——注入运行时碰撞体。"""
from isaaclab.envs import ManagerBasedRLEnv


class LeRobotKitchenEnv(ManagerBasedRLEnv):
    """在环境创建后给夹爪添加碰撞球体。"""

    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode)
        # 注入运行时碰撞体（绕过 USD 实例代理限制）
        self._add_gripper_colliders()

    def _add_gripper_colliders(self):
        """在 live stage 上给夹爪链接添加碰撞球体。"""
        try:
            import omni.usd as usd
            from pxr import UsdGeom, UsdPhysics

            stage = usd.get_context().get_stage()
            robot_path = "/World/envs/env_0/Robot"

            colliders = [
                ("left_gripper_link", 0.05, 0.015, -0.03, 0.025),
                ("right_gripper_link", 0.05, -0.015, -0.03, 0.025),
                ("left_moving_jaw_link", 0.055, 0.02, -0.038, 0.02),
                ("right_moving_jaw_link", 0.055, -0.02, -0.038, 0.02),
            ]

            for link, x, y, z, r in colliders:
                coll_path = f"{robot_path}/{link}/collisions/runtime_sphere"
                if stage.GetPrimAtPath(coll_path):
                    continue
                sphere = UsdGeom.Sphere.Define(stage, coll_path)
                sphere.CreateRadiusAttr(r)
                UsdGeom.XformCommonAPI(sphere).SetTranslate((x, y, z))
                sp = stage.GetPrimAtPath(coll_path)
                UsdPhysics.CollisionAPI.Apply(sp)
                mesh_api = UsdPhysics.MeshCollisionAPI.Apply(sp)
                mesh_api.CreateApproximationAttr().Set("convexHull")
        except Exception as e:
            print(f"[LeRobotKitchenEnv] WARNING: Failed to add gripper colliders: {e}")
