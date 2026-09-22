import torch


def dynamic_reset_gripper_effort_limit_sim(env, teleop_device):
    need_to_set = []
    if "bi-so101leader" in teleop_device:
        need_to_set = [env.scene.articulations["left_arm"], env.scene.articulations["right_arm"]]
    elif "so101leader" in teleop_device or teleop_device in ["keyboard", "gamepad"]:
        need_to_set = [env.scene["robot"]]
    for arm in need_to_set:
        write_gripper_effort_limit_sim(env, arm)
    return


def write_gripper_effort_limit_sim(env, env_arm):
    gripper_pos = env_arm.data.body_link_pos_w[:, -1]  # [num_envs, 3]
    num_envs = gripper_pos.shape[0]

    object_positions = []
    object_masses = []
    object_names = []

    for name, obj in env.scene._rigid_objects.items():
        pos = obj.data.body_link_pos_w[:, 0]  # [num_envs, 3]
        object_positions.append(pos)
        object_masses.append(obj.data.default_mass)
        object_names.append(name)

    if not object_positions:
        return

    object_positions = torch.stack(object_positions)  # [num_objects, num_envs, 3]
    object_masses = torch.stack(object_masses)  # [num_objects, num_envs, 1]

    distances = torch.sqrt(torch.sum((object_positions - gripper_pos.unsqueeze(0)) ** 2, dim=2))

    min_distances, min_indices = torch.min(distances, dim=0)  # [num_envs]

    target_masses = object_masses[min_indices.cpu(), 0, 0]  # [num_envs]

    # IsaacLab 3.0 起 joint_effort_limits 是 ProxyArray，其 .device 是 warp Device
    # （不是 torch.device），直接 .to(...) 会报 "to() received an invalid combination of
    # arguments - got (Device)"。先用 .torch 取回 tensor，再拿它的 torch.device。
    effort_limits = env_arm._data.joint_effort_limits
    if hasattr(effort_limits, "torch"):
        effort_limits = effort_limits.torch
    target_effort_limits = (target_masses / 0.15).to(effort_limits.device)

    current_effort_limit_sim = effort_limits[:, -1]  # [num_envs]
    need_update = torch.abs(target_effort_limits - current_effort_limit_sim) > 0.1

    if torch.any(need_update):
        new_limits = current_effort_limit_sim.clone()
        new_limits[need_update] = target_effort_limits[need_update]

        # IsaacLab 3.0 对写入形状校验更严：limits 必须是 (len(env_ids), len(joint_ids))，
        # joint_ids 是所有 env 共用的关节下标列表（2.x 时按 env 各给一个）。
        env_arm.write_joint_effort_limit_to_sim(limits=new_limits.unsqueeze(-1), joint_ids=[5])


def get_task_type(task: str, task_type: str | None = None) -> str:
    """
    Make sure the task type is in the supported teleop devices.
    """
    if task_type is not None:
        return task_type
    if "BiArm" in task or "LeRobot" in task or "SmartFactory" in task:
        # SmartFactory = 移动双臂机器人（和厨房任务同一台），所以也是双臂主手
        return "bi-so101leader"
    elif "LeKiwi" in task:
        return "lekiwi-leader"
    else:
        return "so101leader"


def delete_attribute(obj, attr_name):
    if hasattr(obj, attr_name):
        delattr(obj, attr_name)
