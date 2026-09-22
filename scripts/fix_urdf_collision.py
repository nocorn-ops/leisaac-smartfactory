#!/usr/bin/env python3
"""在 URDF 的 4 个夹爪 link 中，在第一个 <collision> 前插入简单 box 碰撞体。
不改动原有 STL collision，不破坏 XML 结构。"""
import shutil, os

urdf = "assets/lerobot_robot(1)/lerobot_robot/urdf/lerobot_robot.urdf"
if not os.path.exists(urdf + ".bak2"):
    shutil.copy(urdf, urdf + ".bak2")
    print(f"备份: {urdf}.bak2")

with open(urdf) as f:
    content = f.read()

# 为每个 link 准备新的碰撞块，插入在第一个 <collision> 之前
new_collisions = {
    # link_name -> (collision_block, unique_anchor_string_for_this_link)
    "left_gripper_link": (
        '<collision>\n      <origin xyz="0.05 0.015 -0.03" rpy="0 0 0"/>\n      <geometry>\n        <box size="0.04 0.02 0.06"/>\n      </geometry>\n    </collision>\n    ',
        # 这个 link 独有的 collision origin 用来定位
        'xyz="0.0077 0.0001 -0.0234" rpy="-1.5708 0 0"',
    ),
    "right_gripper_link": (
        '<collision>\n      <origin xyz="0.05 -0.015 -0.03" rpy="0 0 0"/>\n      <geometry>\n        <box size="0.04 0.02 0.06"/>\n      </geometry>\n    </collision>\n    ',
        'xyz="0.0077 0.0001 -0.0234" rpy="-1.5708 0 0"',
    ),
    "left_moving_jaw_link": (
        '<collision>\n      <origin xyz="0.055 0.02 -0.038" rpy="0 0 0"/>\n      <geometry>\n        <box size="0.03 0.02 0.05"/>\n      </geometry>\n    </collision>\n    ',
        'xyz="0 0 0.0189" rpy="0 0 0"',
    ),
    "right_moving_jaw_link": (
        '<collision>\n      <origin xyz="0.055 -0.02 -0.038" rpy="0 0 0"/>\n      <geometry>\n        <box size="0.03 0.02 0.05"/>\n      </geometry>\n    </collision>\n    ',
        'xyz="0 0 0.0189" rpy="0 0 0"',
    ),
}

for link_name, (new_block, anchor) in new_collisions.items():
    # 找到该 link 的起始位置
    link_start = content.find(f'<link name="{link_name}">')
    if link_start < 0:
        print(f"  SKIP: {link_name} not found")
        continue

    # 在该 link 范围内找到 anchor
    anchor_pos = content.find(anchor, link_start)
    if anchor_pos < 0:
        print(f"  SKIP: anchor for {link_name} not found")
        continue

    # 找到 anchor 所在的 <collision> 起始标签
    coll_start = content.rfind('<collision>', anchor_pos - 200, anchor_pos)
    if coll_start < 0:
        print(f"  SKIP: collision start for {link_name} not found")
        continue

    # 在 <collision> 之前插入新碰撞块
    content = content[:coll_start] + new_block + content[coll_start:]
    print(f"  INSERTED: {link_name}")

with open(urdf, 'w') as f:
    f.write(content)

print(f"\n保存: {urdf}")
print("完成。现在可在 GUI 导入 URDF → 导出 USD。")
