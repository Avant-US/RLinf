# 4DWVLA 关节阻抗评测客户端

这是 `b/x/4dwvla_ext/franka_vla_client.py` 的手臂执行替换版。推理协议、相机、键盘、夹爪和 L1–L8 安全检查仍走原客户端；手臂不再调用阻塞的 `JointWaypointMotion`。

每个关节动作是阻抗平衡点：

```text
action[0:7]
  → FrankyJointEnv.check_action_safety
  → JointImpedanceController.move_joints
  → franky.JointImpedanceTracker.set_target(q, dq)
```

`set_target` 立即返回。跟踪由 libfranka 的力矩环持续完成，所以实测控制频率不再被“等每个 waypoint 停稳”卡住。夹爪的 `move` / `grasp` 仍可能阻塞。

回 HOME、急停和 motion-guard 制动会先停掉 tracker，再使用原来的阻塞关节运动。两种运动生成器不能同时占用 FCI。

## 启动

```bash
source b/x/4dwvla_ext/configs/franka_plug_eval.env
source b/x/4dwvla_ext_ipd/configs/franka_plug_impedance.env
python b/x/4dwvla_ext_ipd/franka_vla_client_ipd.py \
    --robot-ip 172.16.0.2 \
    --task "plug into socket" \
    --use-realsense
```

默认刚度、阻尼与 `rlinf/envs/realworld/franka/franky_controller.py` 的 `JointImpedanceTracker` 相同。换机器人或要改接触刚度时，另写一份 env 文件覆盖 `VLA_IPD_STIFFNESS` 和 `VLA_IPD_DAMPING`，不要改控制器源码。

先保持 `--control-hz 10`。确认手臂跟踪且不振荡后，再单独提高频率。

## 离线检查

```bash
python b/x/4dwvla_ext_ipd/tests/test_impedance_offline.py
```

## 示教阻抗回放（独立脚本）

不经过 VLA 客户端，直接把 `plug_into_socket_lrb_4D_8sml` episode 0 的
`action.arm` / `action.gripper` 以 30 Hz 关节阻抗发到真机（无安全裁剪）：

```bash
python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --dry-run
python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --robot-ip 172.16.0.2
python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --robot-ip 172.16.0.2 --frames 1
python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --list-episodes
python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --robot-ip 172.16.0.2 --episode 3
```

默认会先慢速回到 ``franky_ext/dsplug/home_pose.json``（``--home-dynamics 0.05``，
分段 waypoint 每步约 0.02 rad），再启动阻抗回放。跳过 HOME：``--skip-home``。
更慢：``--home-dynamics 0.03``。
