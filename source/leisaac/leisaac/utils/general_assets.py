from pxr import Usd, UsdGeom, UsdPhysics


def get_all_prims(stage, prim=None, prims_list=None):
    if prims_list is None:
        prims_list = []
    if prim is None:
        prim = stage.GetPseudoRoot()
    for child in prim.GetChildren():
        prims_list.append(child)
        get_all_prims(stage, child, prims_list)
    return prims_list


def classify_prim(prim):
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return "Articulation"
    elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return "RigidBody"
    else:
        return "Normal"


def is_articulation_root(prim):
    return prim.HasAPI(UsdPhysics.ArticulationRootAPI)


def is_rigidbody(prim):
    return prim.HasAPI(UsdPhysics.RigidBodyAPI)


def get_all_joints(stage):
    joints = []

    def recurse(prim):
        if UsdPhysics.Joint(prim):
            joints.append(prim)
        for child in prim.GetChildren():
            recurse(child)

    recurse(stage.GetPseudoRoot())
    return joints


def get_stage(usd_path):
    stage = Usd.Stage.Open(usd_path)
    return stage


def get_prim_pos_rot(prim):
    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return None, None
    matrix = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    if matrix.Orthonormalize(issueWarning=True):
        rot = matrix.ExtractRotationQuat()
        # ★ IsaacLab 3.0 起 init_state.rot / data.root_quat_w 都是 **XYZW**（2.x 是 WXYZ，
        #   见 dependencies/IsaacLab/docs/source/migration/migrating_to_isaaclab_3-0.rst
        #   §"The quaternion format changed from WXYZ to XYZW"）。pxr 的 Gf.Quatd 是
        #   (real=w, imaginary=xyz)，必须重排成 XYZW。
        #   ⚠️ 不重排的话：**USD 里没有旋转的物体**会返回 [1,0,0,0]，在 XYZW 下 =
        #   (x=1,y=0,z=0,w=0) = 绕 X 轴 180° → 盘子/橙子等所有物体**直接倒置**
        #   （这就是"一打开仿真盘子和橘子是倒置的"的根因）。
        rot_list = [rot.GetImaginary()[0], rot.GetImaginary()[1], rot.GetImaginary()[2], rot.GetReal()]
    else:
        rot_list = [0.0, 0.0, 0.0, 1.0]  # XYZW 单位四元数
    pos = matrix.ExtractTranslation()
    pos_list = list(pos)

    return pos_list, rot_list


def get_articulation_joints(articulation_prim):
    joints = []

    def recurse(prim):
        if UsdPhysics.Joint(prim):
            joints.append(prim)
        for child in prim.GetChildren():
            recurse(child)

    recurse(articulation_prim)
    return joints


def get_joint_type(joint_prim):
    joint = UsdPhysics.Joint(joint_prim)
    return joint.GetTypeName()


def is_fixed_joint(prim):
    return prim.GetTypeName() == "PhysicsFixedJoint"


def is_revolute_joint(prim):
    return prim.GetTypeName() == "PhysicsRevoluteJoint"


def is_prismatic_joint(prim):
    return prim.GetTypeName() == "PhysicsPrismaticJoint"


def get_joint_name_and_qpos(joint_prim):
    joint = UsdPhysics.Joint(joint_prim)
    return joint.GetName(), joint.GetPositionAttr().Get()


def get_all_joints_without_fixed(articulation_prim):
    joints = get_articulation_joints(articulation_prim)
    return [joint for joint in joints if not is_fixed_joint(joint)]


try:  # Isaac Sim 6.0+：原 isaacsim.core.utils 迁移到 experimental 命名空间
    import isaacsim.core.experimental.utils.prim as prim_utils
except ImportError:  # Isaac Sim 5.x
    try:
        import isaacsim.core.utils.prims as prim_utils
    except ImportError:
        # 兜底：直接用 USD API 取 prim，不依赖 isaacsim.core 扩展是否已使能
        import omni.usd

        class _PrimUtilsShim:
            @staticmethod
            def get_prim_at_path(prim_path):
                return omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)

        prim_utils = _PrimUtilsShim()
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.assets.rigid_object import RigidObjectCfg
from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
from isaaclab.sim.utils import clone


def match_specific_name(prim_path, specific_name_list, exlude_name_list):
    match_specific = (
        True
        if specific_name_list is None
        else any([specific_name in prim_path for specific_name in specific_name_list])
    )
    match_exclude = False if exlude_name_list is None else any([exclude in prim_path for exclude in exlude_name_list])

    return match_specific and not match_exclude


@clone
def spawn_from_prim_path(prim_path, spawn, translation, orientation):
    return prim_utils.get_prim_at_path(prim_path)


def parse_usd_and_create_subassets(usd_path, env_cfg, specific_name_list=None, exclude_name_list=None):
    stage = get_stage(usd_path)
    prims = get_all_prims(stage)
    articulation_sub_prims = list()
    create_attr_record = dict()
    for prim in prims:
        if is_articulation_root(prim) and match_specific_name(
            prim.GetPath().pathString, specific_name_list, exclude_name_list
        ):
            pos, rot = get_prim_pos_rot(prim)
            joints = get_all_joints_without_fixed(prim)
            if not joints:
                continue
            orin_prim_path = prim.GetPath().pathString
            name = orin_prim_path.split("/")[-1]
            if name not in create_attr_record:
                create_attr_record[name] = 0
            else:
                create_attr_record[name] += 1
                name = f"{name}_{create_attr_record[name]}"
            sub_prim_path = orin_prim_path[orin_prim_path.find("/", 1) + 1 :]
            prim_path = f"{{ENV_REGEX_NS}}/Scene/{sub_prim_path}"
            artcfg = ArticulationCfg(
                prim_path=prim_path,
                spawn=None,
                init_state=ArticulationCfg.InitialStateCfg(
                    pos=pos,
                    rot=rot,
                ),
                actuators={},
            )
            setattr(env_cfg.scene, name, artcfg)
            articulation_sub_prims.extend(get_all_prims(stage, prim))
    for prim in prims:
        if is_rigidbody(prim) and match_specific_name(prim.GetPath().pathString, specific_name_list, exclude_name_list):
            if prim in articulation_sub_prims:
                continue
            pos, rot = get_prim_pos_rot(prim)
            orin_prim_path = prim.GetPath().pathString
            name = orin_prim_path.split("/")[-1]
            if name not in create_attr_record:
                create_attr_record[name] = 0
            else:
                create_attr_record[name] += 1
                name = f"{name}_{create_attr_record[name]}"
            sub_prim_path = orin_prim_path[orin_prim_path.find("/", 1) + 1 :]
            prim_path = f"{{ENV_REGEX_NS}}/Scene/{sub_prim_path}"
            rigidcfg = RigidObjectCfg(
                prim_path=prim_path,
                spawn=RigidObjectSpawnerCfg(func=spawn_from_prim_path),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=pos,
                    rot=rot,
                ),
            )
            setattr(env_cfg.scene, name, rigidcfg)
