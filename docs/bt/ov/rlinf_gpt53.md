# 具身 RL 上手路线图（配置 -> 运行 -> 排障）

按"先跑通、再稳定、最后提速"来做最省时间。下面这版默认先走 `ManiSkill + OpenVLA-OFT`（最常见起步组合）。

---

## 1) 先把依赖和资源准备好（一次性）

```bash
cd /path/to/RLinf

# 1) 安装具身依赖
bash requirements/install.sh embodied --model openvla-oft --env maniskill_libero
source .venv/bin/activate

# 2) 下载 ManiSkill 资源
cd rlinf/envs/maniskill
hf download --repo-type dataset RLinf/maniskill_assets --local-dir ./assets
cd /path/to/RLinf

# 3) 下载模型和 LoRA
hf download RLinf/Openvla-oft-SFT-libero10-trajall \
  --local-dir /path/to/model/Openvla-oft-SFT-libero10-trajall
hf download RLinf/RLinf-OpenVLAOFT-ManiSkill-Base-Lora \
  --local-dir /path/to/models/oft-sft/lora_004000
```

---

## 2) 配置只改这几项（最小改动）

编辑 `examples/embodiment/config/maniskill_ppo_openvlaoft_quickstart.yaml`：

- `rollout.model.model_path` -> 你的 OpenVLA-OFT 路径
- `actor.model.model_path` -> 同上
- `actor.model.lora_path` -> 你的 LoRA 路径
- `cluster.component_placement` -> 按你 GPU 数调整（1 卡就保留 `0-0`）
- 如果 OOM，先减 `env.train.total_num_envs` 和 `actor.micro_batch_size`

---

## 3) 启动训练（先用同步 PPO 跑通）

```bash
bash examples/embodiment/run_embodiment.sh maniskill_ppo_openvlaoft_quickstart LIBERO
```

这个脚本会自动设置 `ROBOT_PLATFORM`，拼 Hydra 命令，并把日志写到 `logs/` 下。

---

## 4) 跑起来后先看这 4 个信号（10 分钟内判断是否健康）

- `logs/.../run_embodiment.log` 里有没有连续 rollout/training 输出
- `ray status` 能看到预期 GPU 资源
- TensorBoard：`tensorboard --logdir ./logs --port 6006`
- 关键指标优先看：`success_once`、`return`、`approx_kl`、`clip_fraction`

---

## 5) 评估流程（ID/OOD）

### ID 评估

```bash
bash examples/embodiment/eval_embodiment.sh maniskill_ppo_openvlaoft_quickstart LIBERO
```

### ManiSkill OOD 评估

先填 `examples/embodiment/eval_mani_ood.sh` 里的 `EVAL_NAME`、`CKPT_PATH`、`CONFIG_NAME`，再执行：

```bash
bash examples/embodiment/eval_mani_ood.sh
```

---

## 6) 最常见报错的定位顺序（实战版）

### A. 配置断言报错（最常见）

优先检查：

- `env.train.total_num_envs` 是否能被 `env_world_size`、`rollout.pipeline_stage_num` 整除
- `env.*.max_steps_per_rollout_epoch` 是否能被 `actor.model.num_action_chunks` 整除
- `actor.global_batch_size % (actor.micro_batch_size * actor_world_size) == 0`

### B. EGL/MUJOCO 渲染报错

- 先检查驱动图形能力是否完整
- 不行先切 `osmesa` 保证可跑（但速度会慢很多）

### C. NCCL `cuda invalid argument`

- 先 `ray stop` 再重启任务
- 再检查 `component_placement` 是否与实际 GPU 分配一致

### D. Ray 连接 GCS 失败（多机）

- 先确认 head IP 对 worker 可达
- 再检查 Ray 启动脚本选取的网卡/IP 是否正确

### E. 动作维度/归一化异常

- `ROBOT_PLATFORM` 必须和数据/策略设定一致（`LIBERO` / `ALOHA` / `BRIDGE`）

---

## 7) 跑稳后再升级到 Async（提吞吐）

```bash
bash examples/embodiment/run_async.sh maniskill_async_ppo_openvla LIBERO
```

建议先从保守参数起步：

- `staleness_threshold: 1`
- `behave_weight_threshold: 2`

先稳定，再逐步提高吞吐。

---

## 8) 一句话的最小闭环

先用 quickstart 配置只改模型路径，跑通同步 PPO，确认 `success_once/return` 有趋势，再切 Async 做吞吐优化，最后再扩展到 OOD 和多机。
