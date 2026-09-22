# 项目出处、许可证与贡献者

本仓库是一个**基于开源项目的二次开发**成果：底层仿真引擎与上层框架来自上游开源社区，
本仓库在其之上完成了移动双臂机器人的仿真集成、智慧工厂赛项场地与任务、ROS2 全链路、
数据采集闭环与发布工程化。为避免混淆，本文把「上游已有」与「本仓库的工作」分开写清楚。

> 相关时间线：上游 LeIsaac 开源（2025-07）、本仓库的机器人集成阶段（2026-06 ~ 2026-07）、
> 仿真栈迁移与赛项落地阶段（2026-09 起）。

---

## 一、依赖与上游项目

本仓库**不包含** Isaac Sim 本体（约 38 GB，由 `reproduce/install_all.sh` 按需下载安装），
`dependencies/IsaacLab` 的许可证声明保留在其目录内。

| 项目 | 版本 | 在本项目中的角色 | 许可证 / 版权 |
|---|---|---|---|
| **NVIDIA Isaac Sim** | 6.0.1 standalone | 仿真引擎本体（物理 / 渲染 / 传感器）。**不随仓库分发**，由安装脚本下载 | NVIDIA 专有许可（见 nvidia.com 上的 Isaac Sim 许可条款） |
| **NVIDIA IsaacLab** | 3.0.0（`release/3.0.0`） | 仿真框架：环境/管理器/传感器抽象。位于 `dependencies/IsaacLab` | **BSD-3-Clause**，Copyright (c) 2022-2026, The Isaac Lab Project Developers — https://github.com/isaac-sim/IsaacLab |
| **LightwheelAI / leisaac** | 0.4.0 | **本仓库的基础框架**：任务框架、遥操作设备抽象、录制增强、策略客户端等 | **Apache-2.0**，Copyright Lightwheel — https://github.com/LightwheelAI/leisaac |
| **HuggingFace LeRobot** | 0.4.2 | 依赖库（非 fork）：数据集格式（`LeRobotDataset`）、训练（`lerobot-train`）、策略推理生态 | **Apache-2.0** — https://github.com/huggingface/lerobot |

> `LICENSE` 文件为 Apache-2.0（继承自上游 LeIsaac），`CITATION.cff` 保留上游 LeIsaac 的引用信息，
> 均按原样保留以符合各上游许可证的要求。

---

## 二、本仓库完成的工作

### 2.1 由 [@nocorn-ops](https://github.com/nocorn-ops) 完成（2026-09 起）

- **仿真栈跨版本迁移**：Isaac Sim 5.1.0 + IsaacLab 2.3.0 → **6.0.1 + 3.0.0**
  - 定位并解决 Blackwell 架构显卡（RTX 5070 Ti）与 5.1 RTX 渲染器的代际不兼容
    （驱动段错误、`vkCreateRayTracingPipelinesKHR` 失败），确立 6.0.1 技术路线
  - 排查 IsaacLab 3.0 四元数顺序变更（WXYZ → XYZW）导致的相机与机械臂基准朝向错误
    （原按 2.x 的 wxyz 写法导致相机朝向全错、双臂基准朝向差 180°），并用历史录制数据做 A/B 验证
  - 修复 USD 四元数被当作 XYZW 解析导致的无旋转物体倒置问题（插进台面 / 弹跳 / 掉地）
- **智慧工厂赛项场地与任务**：`LeIsaac-SmartFactory-v0`
  - 按赛事图纸构建 4000×3000 场地：起点区 / 停放区 / 转运存储区 / 双层收纳区，17 部件自检通过
  - 货物动态刚体（香蕉 / 茄子 / 收纳盒）落位与静置验证
  - 场地为**纯文本 .usda 资产**，可在 GUI 中直接编辑
- **ROS2 全链路**：Isaac Sim ↔ ROS2 桥接、SLAM 建图（slam_toolbox）、Nav2 导航端到端
  - 两次导航实测误差 **3 cm / 1.6 cm**
  - 单实例锁、桥接冲突检测、激光急停等工程健壮性处理
- **遥操作与数据采集闭环**
  - 双臂真主手 / 双臂键盘（IK）四种遥操方式；绝对位姿映射 + 对齐门槛 + 软启动
  - HDF5 流式录制（定时分集 + 热键）、三路相机同步采集
  - 12 维（主手）/ 16 维（键盘）动作空间转换契约与验收脚本
- **ACT 策略推理链路的问题定位**：TCP 动作服务 + 客户端联通性验证，
  定位推理成功率低的一个关键失配（渲染亮度与训练数据差约 2.5 倍）
- **发布与部署工程化**
  - 代码与底层依赖、二进制资产解耦（资产独立仓库 + tag 固定）
  - 跨机器路径自动探测（`reproduce/leisaac_env.sh`），不依赖任何本机绝对路径
  - 一键安装脚本（含 12 GB Isaac Sim 的断点续传）、发布包打包与自检脚本
  - 面向 Ubuntu 22.04 + 原生 ROS2 Humble 的完整操作指南与安装文档

### 2.2 由 [@Nagasakiianno](https://github.com/Nagasakiianno) 完成（2026-06 ~ 2026-07）

- **自研移动双臂机器人的仿真集成**
  - 自研麦克纳姆轮底盘 + 双臂躯干 URDF / 资产导入（`lerobot_robot_1/`、`assets/lerobot_robot*`）
  - 解决自研躯干与官方 SO101 双臂集成的 `shoulder_pan` 自碰撞抖动问题
  - 多关节 articulation 的 DOF 交错排列与 `preserve_order` 映射问题
- **数据转换工具**：HDF5 → LeRobot v3 数据集（`scripts/convert/hdf5_to_lerobot_v3.py` 等）
- **双臂 ACT 推理管线**：动作服务端与客户端联通（`scripts/evaluation/act_action_server.py`）、
  录制器/动作空间/显存优化等一系列修复
- **技术文档**：技术手册与 ACT 训练/验证流程文档

### 2.3 基于上游 LeIsaac 已有能力（非本仓库原创）

任务框架与模板（`tasks/template`）、遥操作设备抽象（`devices/`）、
仿真录制增强（`enhance/`）、策略客户端（`policy/`）、
单臂任务（`pick_orange`、`lift_cube`、`fold_cloth` 等）、厨房场景
（`lerobot_kitchen`、`assets/scenes/kitchen_with_orange`）等。

---

## 三、许可证合规说明

- 本仓库保留上游 `LICENSE`（Apache-2.0）与 `CITATION.cff`，未删除任何版权声明。
- `dependencies/IsaacLab` 内的 `LICENSE`（BSD-3-Clause）与其版权头按原样保留。
- 分发或再发布本仓库内容时，请同样保留上述许可证与署名。
- 上游项目的商标与名称（Isaac Sim、IsaacLab、LeRobot、LeIsaac）归各自权利人所有。

## 四、如何引用

学术或技术文档中引用本项目时，请同时引用其上游基础：

- LeIsaac — https://github.com/LightwheelAI/leisaac （Apache-2.0）
- Isaac Lab — https://github.com/isaac-sim/IsaacLab （BSD-3-Clause）
- LeRobot — https://github.com/huggingface/lerobot （Apache-2.0）
