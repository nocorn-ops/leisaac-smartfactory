#!/usr/bin/env python3
"""从 URDF 中完全删除双臂——link、joint、transmission、注释全部清理。"""
import re, shutil, os

urdf = "assets/lerobot_robot(1)/lerobot_robot/urdf/lerobot_robot.urdf"
out_urdf = urdf.replace(".urdf", "_body_only.urdf")

# 从原始 URDF 重新开始（不用之前修改过的）
orig = "assets/lerobot_robot(1)/lerobot_robot/urdf/lerobot_robot.urdf.backup"
if not os.path.exists(orig):
    orig = urdf  # 如果没有备份就用当前版本

with open(orig) as f:
    content = f.read()

# 双臂 link 名
arm_links = [
    "left_shoulder_mount", "left_shoulder_link", "left_upper_arm_link",
    "left_lower_arm_link", "left_wrist_link", "left_gripper_link",
    "left_moving_jaw_link", "left_gripper_frame_link",
    "right_shoulder_mount", "right_shoulder_link", "right_upper_arm_link",
    "right_lower_arm_link", "right_wrist_link", "right_gripper_link",
    "right_moving_jaw_link", "right_gripper_frame_link",
]

# 双臂 joint 名
arm_joints = [
    "left_shoulder_mount_joint", "left_shoulder_pan", "left_shoulder_lift",
    "left_elbow_flex", "left_wrist_flex", "left_wrist_roll", "left_gripper",
    "left_gripper_frame_joint",
    "right_shoulder_mount_joint", "right_shoulder_pan", "right_shoulder_lift",
    "right_elbow_flex", "right_wrist_flex", "right_wrist_roll", "right_gripper",
    "right_gripper_frame_joint",
]

# 删除所有臂 link 定义
for link in arm_links:
    content = re.sub(rf'<link name="{link}">.*?</link>', '', content, flags=re.DOTALL)

# 删除所有臂 joint 定义
for joint in arm_joints:
    content = re.sub(rf'<joint name="{joint}".*?</joint>', '', content, flags=re.DOTALL)

# 删除所有引用臂 joint 的 transmission
for joint in arm_joints:
    content = re.sub(rf'<transmission.*?<joint name="{joint}".*?</transmission>', '', content, flags=re.DOTALL)

# 删除纯注释块（左右臂注释分隔区）
content = re.sub(r'<!-- =+ -->\s*<!-- LEFT.*?-->', '', content, flags=re.DOTALL)
content = re.sub(r'<!-- LEFT ARM.*?-->', '', content)
content = re.sub(r'<!-- =+ -->\s*<!-- RIGHT.*?-->', '', content, flags=re.DOTALL)
content = re.sub(r'<!-- RIGHT ARM.*?-->', '', content)
content = re.sub(r'<!-- LEFT SHOULDER MOUNT.*?-->', '', content)
content = re.sub(r'<!-- RIGHT SHOULDER MOUNT.*?-->', '', content)
content = re.sub(r'<!-- Link shoulder -->', '', content)
content = re.sub(r'<!-- Link upper_arm -->', '', content)
content = re.sub(r'<!-- Link lower_arm -->', '', content)
content = re.sub(r'<!-- Link wrist -->', '', content)
content = re.sub(r'<!-- Link gripper -->', '', content)
content = re.sub(r'<!-- gripper_frame \(dummy\) -->', '', content)
content = re.sub(r'<!-- moving_jaw -->', '', content)

# 清理多余空行
content = re.sub(r'\n{3,}', '\n\n', content)

with open(out_urdf, 'w') as f:
    f.write(content)

# 验证
for check in arm_links + arm_joints:
    if check in content:
        print(f"  WARNING: '{check}' still in output!")
    else:
        print(f"  OK: '{check}' removed")

print(f"\nSaved: {out_urdf}")
print("GUI 导入 → 导出 USDZ")
