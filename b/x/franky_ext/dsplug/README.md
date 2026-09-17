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

之后使用普通 `--execute` 时，脚本会读取新的
`home_pose.json`。如果训练/评测代码也要使用新 HOME，还必须同步更新
`b/x/4dwvla_ext/franky_joint_env.py` 中的 `HOME_JOINTS`；否则脚本和评测
环境会各自使用不同的 home。
