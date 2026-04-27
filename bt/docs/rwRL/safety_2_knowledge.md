# Galaxea R1 Pro 安全实施 — 补充知识

> 本文档汇集对 [safety_2.md](safety_2.md) 各章节的深入解读，供部署工程师参考。

---

## 1. 坐标系确认的相关知识

> 对应 [safety_2.md §2.1 坐标系确认](safety_2.md#21-坐标系确认)

### 1.1 什么是 `torso_link4` 坐标系

`torso_link4` 是 R1 Pro URDF 运动链中的一个刚体链接 (link)，对应躯干 4 DoF 关节链的末端——即**双臂肩部的安装基座**。它本身不会动（相对于躯干末端），但会随躯干关节的运动而改变在世界坐标系中的位置和朝向。

在安全系统中，`torso_link4` 充当 **EE（末端执行器）位姿的参考原点**。选择它而非 `base_link`（底盘）的原因是：臂的可达工作空间相对于肩部基座是固定的，但底盘可以移动和旋转。如果以 `base_link` 为参考，躯干关节一旦转动，安全盒就需要同步做坐标变换，增加复杂性和出错风险。以 `torso_link4` 为参考，安全盒的数值边界始终稳定。

在官网的[手臂姿态控制](https://docs.galaxea-dynamics.com/zh/Guide/R1Pro/software_introduction/ros1/R1Pro_Software_Introduction_ROS1/#_15)中也提到:
```
当前末端姿态控制的相对位姿是URDF中gripper_link相对于torso_link4的姿态转换。以下图中的左臂为例，左臂的left_gripper_link坐标系相对于躯干的torso_link4坐标系的相对关系中，包含了x、y、z的偏移量以及orientation对应的旋转偏移。
```
![torso_link4的位置](https://docs.galaxea-dynamics.com/zh/Guide/R1Pro/software_introduction/ros1/assets/R1Pro_arm_pose_control_cn.png)

### 1.2 什么是 frame（坐标帧）

`frame` 指 ROS2 TF 系统中的坐标帧 (coordinate frame)——一个固定在刚体上的三维坐标系，由原点 + 三个正交轴 (x, y, z) 定义。

当 mobiman 发布 EE 姿态 `[x, y, z, qx, qy, qz, qw]` 时，这 7 个数值表示 **EE 在 `torso_link4` 坐标帧下的位置和朝向**。在 ROS2 的 `PoseStamped` 消息中，`header.frame_id` 字段就是用来声明"这个 pose 是在哪个坐标帧中表达的"：

```python
# r1_pro_controller.py:571-573
msg = PoseStamped()
msg.header.stamp = self._node.get_clock().now().to_msg()
msg.header.frame_id = "torso_link4"   # 声明：下面的 xyz/quat 相对于 torso_link4
```

如果 `frame_id` 换成了 `base_link`，同一个 EE 物理位置对应的 `x, y, z` 数值会完全不同，因为参考原点变了。

### 1.3 为什么两个 EE 共用同一个参考坐标系

R1 Pro 有左右两个手臂、两个末端执行器（`pose_ee_arm_right` 和 `pose_ee_arm_left`），但它们的位姿都**相对于同一个 `torso_link4`** 来表达。这里要区分两个概念：

| 概念 | 含义 | 数量 |
|------|------|------|
| **被描述的坐标系** (target frame) | EE 自身的位姿 | 左右各一个 |
| **参考坐标系** (reference frame) | 度量 EE 位姿的"尺子原点" | 两臂共用一个 |

`torso_link4` 是参考坐标系，不是 EE 自身。两个 EE 都从同一个肩部基座生长出来：

```
torso_link4（参考原点，肩部基座）
├── right_ee_pose = [x, y, z, qx, qy, qz, qw]  ← 右手 EE 相对于 torso_link4 的位姿
└── left_ee_pose  = [x, y, z, qx, qy, qz, qw]  ← 左手 EE 相对于 torso_link4 的位姿
```

这也是 L3a 安全盒能工作的前提：`right_ee_min/max` 和 `left_ee_min/max` 都在同一个 `torso_link4` 坐标系下定义边界，所以可以直接拿 `state.right_ee_pose` 的 xyz 与边界值比较，不需要额外的坐标变换。

### 1.4 验证命令解读

safety_2.md 要求部署前执行：

```bash
ros2 topic echo /motion_control/pose_ee_arm_right --once
```

这条命令的含义：

| 部分 | 含义 |
|------|------|
| `ros2 topic echo` | 订阅一个 ROS2 话题并把收到的消息打印到终端 |
| `/motion_control/pose_ee_arm_right` | 话题名——mobiman 发布的右臂 EE 实时位姿 |
| `--once` | 只打印一条消息就退出 |

执行后终端会输出类似：

```yaml
header:
  stamp:
    sec: 1714200000
    nanosec: 123456789
  frame_id: "torso_link4"      # ← 重点检查这里
pose:
  position:
    x: 0.35
    y: -0.12
    z: 0.28
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

**验证目的**：确认 `frame_id` 确实是 `"torso_link4"`。如果实际输出的是 `"base_link"`、`"world"` 或其他值，说明代码中的坐标系假设与硬件不符，L3a 安全盒的边界值全部失效，必须在部署前修正。

### 1.5 相关代码位置

| 文件 | 行号 | 内容 |
|------|------|------|
| [r1_pro_controller.py](../../../rlinf/envs/realworld/galaxear/r1_pro_controller.py) | 573 | `msg.header.frame_id = "torso_link4"` — 发送 EE 目标时指定坐标帧 |
| [r1_pro_robot_state.py](../../../rlinf/envs/realworld/galaxear/r1_pro_robot_state.py) | 65, 94-96 | EE pose 字段定义，注释标注 `frame torso_link4` |
| [r1_pro_safety.py](../../../rlinf/envs/realworld/galaxear/r1_pro_safety.py) | 69-76 | L3a 安全盒参数，注释标注 `xyz + rpy in torso_link4` |
