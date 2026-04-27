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


## 2. 关节限位验证的相关知识

arm_q_min 是 R1 Pro A2 机械臂 7 个关节的最小角度限位，单位是弧度 (rad)。配合 arm_q_max 一起使用，定义了每个关节允许旋转的安全范围。

逐关节解读
A2 臂有 7 个旋转关节 (J1–J7)，数组中的 7 个元素一一对应：

|索引	|关节|	q_min (rad)|	q_max (rad)|	范围 (°)|	含义|
|------|------|------|------|------|------|
|0	|J1 (肩偏转)	|-2.7	|+2.7	|±154.7°	    |  对称，大范围旋转|
|1	|J2 (肩俯仰)	|-1.8	|+1.8	|±103.1°	    |  对称，范围较窄|
|2	|J3 (臂旋转)	|-2.7	|+2.7	|±154.7°	    |  对称|
|3	|J4 (肘关节)	|-3.0	|0.0	|单向 171.9°	|  不对称——肘只能向一侧弯|
|4	|J5 (腕偏转)	|-2.7	|+2.7	|±154.7°	    | 对称|
|5	|J6 (腕俯仰)	|-0.1	|3.7	|不对称 217.7°|	极不对称——几乎只能向一侧转|
|6	|J7 (腕旋转)	|-2.7	|+2.7	|±154.7°	    |  对称|

关键要点:
1. 这是"|软限位"而|非"硬限位"——定义在 r1_pro_safety.py:62-63 的 SafetyConfig 中，由 SafetySupervisor 的 L2 层检查。但 safety_2.md 指出 L2 目前仅为注释，不直接强制执行，实际保护依赖 L3a + L4。

2. J4 和 J6 的不对称性值得注意——J4 是肘关节，物理上只能向一个方向弯曲（想象人的手肘）；J6 是腕俯仰，结构决定了它的活动范围偏向一侧。这些值必须与 A2 臂硬件规格书一致，否则安全限位会"放过"超出物理极限的指令。

3. 左右臂共用同一组限位——代码注释写的是 right arm; left arm uses same。这在大多数情况下合理（A2 臂左右结构对称），但安装方式或线缆走向可能导致实际可达范围略有差异。

4. q 代表的是关节角度 (generalized coordinate)——在机器人学中 q 是广义坐标的标准符号，对旋转关节就是角度值。qvel 则是角速度，qtau 是力矩。

目前R1 Pro用的是A2臂:
- 但官网尚找不到说明文档. 但可参考:
  + [R1 Pro 硬件文档](https://docs.galaxea-dynamics.com/Guide/R1Pro/hardware_introduction/R1Pro_Hardware_Introduction/)
  + [R1 Pro 软件文档](https://docs.galaxea-dynamics.com/Guide/R1Pro/software_introduction/R1Pro_Software_Guide_ROS2/)
  + [Galaxea URDF GitHub 仓库](https://github.com/userguide-galaxea/URDF)
  + [A1臂官方文档](https://docs.galaxea-ai.com/Guide/A1/A1_Hardware_Guide/), A2 是 A1 的 7DoF 升级版，部分关节行程可能相似
- 参考**R1 Pro 机器上的 SDK**——Galaxea SDK 安装后，mobiman 包中应有 URDF 文件，里面的 `<joint><limit lower="..." upper="..."/></joint>` 就是关节限位的权威来源. 也就是R1 Pro 的 Orin 控制器上，Galaxea SDK 安装后通常在 `<sdk_root>/install/config/` 下有 YAML 文件，包含每个关节的 joint_limits（位置、速度、力矩上下限）.
- **URDF/Xacro 文件**:	R1 Pro 的 URDF 模型中，每个 `<joint>` 标签下通常有 `<limit lower="..." upper="..." velocity="..." effort="..."/>` 字段，这也是关节限位的权威来源之一

为什么 safety_2.md 强调"对照":
- RLinf 的 SafetyConfig 中写的 [-2.7, -1.8, ...] 只是默认值（注释标明 "tuned for M1 single-arm bring-up"）。如果这些默认值与实际 A2 臂硬件的物理极限不一致，安全限位就会出现两种风险：
  + 设得太宽 → 允许关节运动到物理极限之外 → 损坏电机或减速器
  + 设得太窄 → 不必要地限制工作空间 → 影响任务完成能力
- 所以部署前必须拿硬件规格书来交叉验证。




<!-- 
怎样才能按照r1pro5op47.md中`F.3.1 系统部署图(全 ROS2)`一章中提到的部署架构进行部署? 具体要怎么实施? 部署完后如何验证部署的正确性? 如何在不移动机器人的前提下验证GPU Server与R1 Pro Orin和R1 Pro physical body之间是联通的? 深入分析一下,然后把方案作为一个子章节写在 @bt/docs/rwRL/r1pro5op47.md 里 `附录 F:全 ROS2 相机部署变体(腕部 + 头部均走 ROS2)`中.

安全实施文档safety_2.md中`2.2 关节限位验证`提到的"`arm_qvel_max = [3.0, 3.0, 3.0, 3.0, 5.0, 5.0, 5.0]` rad/s"是指什么? 应该怎么解释和理解?
-->