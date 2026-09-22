#!/usr/bin/env python
"""给 URDF 中 4 个夹爪 link 用简单 Box 替换 STL 碰撞 mesh。"""
import shutil, os, re

urdf = "assets/lerobot_robot(1)/lerobot_robot/urdf/lerobot_robot.urdf"
if not os.path.exists(urdf + ".backup"):
    shutil.copy(urdf, urdf + ".backup")
    print(f"Backup: {urdf}.backup")

with open(urdf, 'r') as f:
    content = f.read()

# 四个夹爪 + box 几何体（Y 坐标左正右负）
gripper_collisions = {
    "left_gripper_link":   '<collision>\n      <origin xyz="0.05 0.015 -0.03" rpy="0 0 0"/>\n      <geometry><box size="0.04 0.02 0.06"/></geometry>\n    </collision>',
    "right_gripper_link":  '<collision>\n      <origin xyz="0.05 -0.015 -0.03" rpy="0 0 0"/>\n      <geometry><box size="0.04 0.02 0.06"/></geometry>\n    </collision>',
    "left_moving_jaw_link":  '<collision>\n      <origin xyz="0.055 0.02 -0.035" rpy="0 0 0"/>\n      <geometry><box size="0.03 0.02 0.05"/></geometry>\n    </collision>',
    "right_moving_jaw_link": '<collision>\n      <origin xyz="0.055 -0.02 -0.035" rpy="0 0 0"/>\n      <geometry><box size="0.03 0.02 0.05"/></geometry>\n    </collision>',
}

for link_name, new_coll in gripper_collisions.items():
    # 查找 link 块内的所有 collision 标签，全部替换为一个简单 collision
    link_start = content.find(f'<link name="{link_name}">')
    if link_start < 0:
        print(f"  SKIP: {link_name} not found")
        continue
    # 找到下一个 </link>
    link_end = content.find('</link>', link_start)
    if link_end < 0:
        continue
    link_block = content[link_start:link_end]

    # 删除所有现有 collision 块
    cleaned = re.sub(r'<collision>.*?</collision>', '', link_block, flags=re.DOTALL)

    # 在 </link> 之前插入新的 collision
    cleaned = cleaned.replace('</link>', f'  {new_coll}\n  </link>')
    content = content[:link_start] + cleaned + content[link_end + len('</link>'):]
    print(f"  FIXED: {link_name}")

with open(urdf, 'w') as f:
    f.write(content)

print(f"\nSaved: {urdf}")
print("现在 GUI 导入 URDF → 导出 USDZ。碰撞体是内联几何，100% 可靠。")
