# 插插座任务的 Franka 停止与回 home

## Home 的定义

`home_pose.json` 使用 `b/d/frk1/plug/abs_stats.json` 中
`observation.state.arm.mean` 的七个关节值：

```text
[-0.2405779213, 0.1457375735, 0.1872396618, -2.0599503517,
 -0.0552775823, 2.2011394501, 0.6998188496] rad
```

这是当前代码已经使用的 4DWVLA 插座评测 home（训练集全局均值），不是
数据集标注的“每个 episode 插入前姿势”。`abs_stats.json` 只有聚合统计，
`keypoints_meta.json` 只描述 keypoint 归一化，URDF 只描述运动学，三者都没有
home 标签。实际 episode 起始关节还存在明显差异，因此不能声称数据集存在唯一
语义 home。这个全局均值会混合抬升、接近、插入和撤回阶段；若用于相机画面对齐，
可能比训练视频中的 episode 起始画面更低。

按 `fr3v2_1_franka_hand.urdf` 做 FK，TCP 参考值为：

```text
position      [0.5718130770, -0.0362816192, 0.2629817814] m
quaternion_xyzw
               [0.9994657126, 0.0314695103, -0.0014139629, 0.0087155006]
rpy_xyz        [3.1242589922, 0.0033749663, 0.0629226173] rad
```

## 使用前提

1. 先停止 RLinf/Ray/VLA 进程，并确认没有其它进程占用 FCI。FCI 不允许第二个
   `franky.Robot` 客户端接管已有控制连接。训练仍在运行时，应使用现有
   `KeyboardVLAEvalWrapper` 的 `r`（中止/停止）或 `h`（回位）路径；本脚本
   不是跨进程抢占工具。
2. 确认机器人周围清空、急停可触达、Desk 已启用 FCI。`robot.stop()` 是软件
   controlled stop，不等价于硬件急停。
3. 默认不操作夹爪，避免持有插头时意外掉落。只有明确传入
   `--open-gripper` 才会以 0.08 m 打开。

## 命令

在 Franky 容器中运行：

```bash
# 只读检查 home 文件，不连接机器人
python b/x/franky_ext/dsplug/stop_and_home.py --print-home

# 原控制进程已退出后，停止本客户端当前运动，不回 home
python b/x/franky_ext/dsplug/stop_and_home.py \
  --robot-ip 172.16.0.2 --stop-only

# 停止、人工确认后回到数据集参考 home
python b/x/franky_ext/dsplug/stop_and_home.py \
  --robot-ip 172.16.0.2 --execute
```

非交互执行必须由现场操作者明确承担风险：

```bash
python b/x/franky_ext/dsplug/stop_and_home.py \
  --robot-ip 172.16.0.2 --execute --yes
```

`--recover-errors` 只在已经确认硬件状态和运动空间安全时使用；UserStopped、
Reflex 或 Desk 急停通常仍应先由人工在 Desk 中处理。脚本会检查 robot mode、
关节速度、FR3v2.1 关节限位和最终关节误差，并输出
`RESULT DSPLUG_STOP PASS` 或 `RESULT DSPLUG_HOME PASS`。

## 手动重新标定 HOME

如果目标是让 HOME 与当前真实工装、腕部相机画面和插座上方安全姿态一致，
不要继续使用全局统计均值。建议流程是：

1. 停止 RLinf/Ray/VLA 控制进程，释放 FCI。
2. 使用 Franka Desk/使能装置的手动引导，把机械臂放到期望 HOME；不要让脚本
   在手动引导期间发送运动命令。
3. 松开引导按钮，等待机器人回到 `RobotMode.Idle` 且关节速度为零。
4. 只读预览当前关节：

   ```bash
   python b/x/franky_ext/dsplug/stop_and_home.py \
     --robot-ip 172.16.0.2 --capture-home
   ```

5. 确认输出的关节角、TCP 位置和腕部相机画面正确后，再显式写入：

   ```bash
   python b/x/franky_ext/dsplug/stop_and_home.py \
     --robot-ip 172.16.0.2 --capture-home --write-home
   ```

   输入 `WRITE_HOME` 后，原文件会先备份为 `home_pose.json.bak`。

之后使用普通 `--execute` 时，脚本会读取新的 `home_pose.json`。评测环境
`b/x/4dwvla_ext/franky_joint_env.py` 已改为通过 `home_pose.load_home_joints()`
读取同一个文件，所以不需要再在别处同步关节角。

## 夹爪 homing 标定

`gripper_homing.py` 检查、并在确认后重跑 Franka Hand 的 homing。它解决的是
**报告宽度与物理宽度不一致**的问题：libfranka 报告的 `max_width` 来自上一次
homing，本工位上它报告 66.4 mm，而卡尺实测张开为 80 mm。这 13.6 mm 偏差会整体
平移夹爪通道，因为训练用的映射是 $a = 1 - w / 0.08$：真实张开 79 mm 时"保持张开"
对应 $a = 0.008$，而按 66.4 mm 报告则变成 $a = 0.170$。详见
`b/d/frk1/grperr_1.2.md` Q1。

默认只读：

```bash
# 只读，打印报告宽度、当前宽度，以及与示教数据的差异
python b/x/franky_ext/dsplug/gripper_homing.py --robot-ip 172.16.0.2

# 确认手指之间无异物、未夹持后重跑 homing（输入 HOME_GRIPPER 确认）
python b/x/franky_ext/dsplug/gripper_homing.py --robot-ip 172.16.0.2 --execute
```

homing 会让手指走完整行程，因此手指之间不能有插头或任何物体；脚本在检测到
`holding` 时会直接拒绝执行。输出 `RESULT DSPLUG_GRIPPER_INSPECT PASS` 或
`RESULT DSPLUG_GRIPPER_HOMING PASS`。

**homing 之后必须同步配置。** `FRANKA_GRIPPER_MAX_WIDTH_M` 的语义是
"libfranka 报告的宽度"，不是卡尺实测的手指间距。把实测值填进去会让钳位失效，
这正是 2026-09-18 那次机械臂抖动的直接原因。脚本结束时会打印应当写入
`b/x/4dwvla_ext/configs/franka_plug_eval.env` 的数值。运行时环境还会再取一次
`min(配置值, 硬件报告值)`，所以配置填大了不会重现该故障，但**填小了会限制夹爪
的最大张开**——homing 成功把报告值恢复到 ~0.079 后若忘记更新配置，夹爪将永远
打不开到位。
