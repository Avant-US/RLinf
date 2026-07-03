# pi0.5 (openpi_au) 在 pushdoor 数据上的 SFT 示例

本目录是一个**自包含**的示例：用 RLinf 的 `openpi_au` 版 pi0.5 模型，在 R1-Pro 采集的
"push door" LeRobot 数据集上做监督微调（SFT），单机 8×H200，冒烟量级（50 步）跑通即止。

- 模型实现：[`rlinf/models/embodiment/openpi_au/`](../../../rlinf/models/embodiment/openpi_au/)
- 入口脚本：直接复用 [`examples/sft/train_vla_sft_au.py`](../../sft/train_vla_sft_au.py)（内含 EMA + faithful augmentation 的 `FSDPVlaSftWorkerAu`）
- 默认数据：`/mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data_tst1`（12 条 episode / 13136 帧，任务 `open0622`）

> 本示例**不改动** RLinf 任何框架/模型代码。`pushdoor` 的数据接线（`pi05_pushdoor`
> 配置、`LeRobotPushdoorDataConfig`、`PushdoorInputs/Outputs`）已经在 `openpi_au` 里
> 注册好，这里只是新增了配置 YAML + 三个 shell 脚本 + 本 README。

## 设计依据（详见 `b/d/pi/`）

微调配方（EMA、fp32-master + bf16 计算、warmup+cosine 学习率、逐样本图像增强、量化归一化）
沿用以下深度分析文档，与 LIBERO / robotwin 的 `*_au` 参考例保持一致：

- [`b/d/pi/p05_1.md`](../../../b/d/pi/p05_1.md) —— π₀.₅ 在 RLinf 框架中的实现：深度技术解析
- [`b/d/pi/rlinf_pi05_2.md`](../../../b/d/pi/rlinf_pi05_2.md) —— RLinf 复现 openpi π₀.₅ SFT（v2 · `openpi_au` 零侵入隔离版）
- [`b/d/pi/rlinfpi_ema_aug_ckp_1.md`](../../../b/d/pi/rlinfpi_ema_aug_ckp_1.md) —— EMA / 图像增强 / Batch-权重-Norm 三项细化落地方案
- [`b/d/pi/rlinfpi_lr_mxp_grdckp_1.md`](../../../b/d/pi/rlinfpi_lr_mxp_grdckp_1.md) —— 学习率调度 / 混合精度 / 梯度检查点 三项细化落地方案

## 数据 → 模型的关键映射（pushdoor）

由 [`pushdoor_dataconfig.py`](../../../rlinf/models/embodiment/openpi_au/dataconfig/pushdoor_dataconfig.py)
与 [`pushdoor_policy.py`](../../../rlinf/models/embodiment/openpi_au/policies/pushdoor_policy.py) 定义：

- **state（20 维）** = `left_arm`(7) + `right_arm`(7) + `left_gripper`(1) + `right_gripper`(1) + `torso`(4)
- **action（23 维）** = 上述 20 维布局 + `chassis.velocities`(3)
- **delta 动作掩码** = 双臂 + 躯干转 delta，双夹爪保持绝对；`chassis.velocities` 是底盘速度指令，天然保持绝对
- **相机（3 路）** = `base_0_rgb ← head_rgb`、`left_wrist_0_rgb ← wrist_left_rgb`、`right_wrist_0_rgb ← wrist_right_rgb`（数据里的 `head_rgb_right` 不读取）
- **语言指令**：固定 prompt `"push open the door"`（数据自带 task 标签 `open0622` 不是自然语言，故用 `prompt_from_task=False` + `default_prompt`）

## 前置条件

1. openpi 虚拟环境：`/mnt/r/VENV/openpi_venv`（可用 `PY=` 覆盖）。
2. openpi 源码：`/home/physical/SRC/Robot/openpi05/src`（可用 `OPENPI_SRC=` 覆盖）。
3. pi0.5 基座权重目录（含 `model.safetensors` + `config.json`）：默认
   `/mnt/r/CKPT/VLA/pi05_base_pt_fp32`（可用 `BASE_CKPT=` 覆盖，例如指向转换后的
   `pi05_r1pro_chassis_alig_newnorm` 权重）。
4. LeRobot 数据集在本地：`/mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data_tst1`（可用
   `RLINF_PUSHDOOR_DATA=` 覆盖）。

## 三步跑通

所有脚本从仓库根目录（`RLinf/`）执行；也可从任意目录调用，脚本内部会自解析路径。

### 1. 计算归一化统计（norm_stats）

pi0.5 用量化归一化（q01/q99 + mean/std），需要在**本数据集**上先算一份 `norm_stats.json`。
脚本单卡、离线、无缓冲运行，把结果写到示例模型目录 `_ckpt/pi05_pushdoor_tst1/rlinf/pushdoor_open0622/norm_stats.json`：

```bash
bash examples/au/pi/run_norm_stats.sh
```

- 冒烟默认 `MAX_FRAMES=1024`；要全量统计设 `MAX_FRAMES=13136`。
- `asset_id` 固定为 `rlinf/pushdoor_open0622`，与数据在磁盘上的路径无关，训练时 `get_model` 也从这个子目录读取。

### 2. 启动 SFT 训练

`run_train.sh` 会先把基座权重（`model.safetensors` + `config.json`）**软链接**进示例模型目录
（省 14GB 磁盘），做前置检查（norm_stats、数据集存在），再用 8 卡 FSDP 启动：

```bash
bash examples/au/pi/run_train.sh
```

- 复用 [`examples/sft/train_vla_sft_au.py`](../../sft/train_vla_sft_au.py)，通过 `--config-path examples/au/pi --config-name pushdoor_sft_pi05_au` 载入本例配置；
- `EMBODIED_PATH` 指到 `examples/sft`，让 group 默认（`model/pi0_5_au`、`training_backend/fsdp`）能被 Hydra 搜索到；
- `RAY_ADDRESS=local` 保证起一个独立的临时 Ray 集群，不干扰机器上其它 Ray 作业。

### 3. 查看产物

```bash
ls examples/au/pi/_out/pi05_pushdoor_tst1_smoke50/checkpoints/
```

冒烟配置在第 25、50 步各存一次 checkpoint（含 EMA 权重）。控制台按 `log_interval` 打印 loss/grad-norm/lr。

## 关键配置一览（[`pushdoor_sft_pi05_au.yaml`](pushdoor_sft_pi05_au.yaml)）

- `cluster.component_placement: actor,env,rollout: 0-7`（单机 8 卡）
- `actor.micro_batch_size: 16`，`global_batch_size: 128`（8×16，梯度累积 = 1）
- `actor.model.action_dim: 23`，`num_action_chunks: 10`，`openpi.config_name: pi05_pushdoor`
- 精度：`fp32_master_weights: false` + `fsdp_config.mixed_precision` 全 `null`（由 openpi 自管 selective bf16；若让 FSDP 统一 bf16 会破坏 openpi 的 fp32 动作头）
- 学习率：`openpi_cosine`，warmup 100 / decay 1000 步，peak `5e-5` → `5e-6`；`ema_decay: 0.999`
- `runner.max_steps: 50`，`save_interval: 25`（冒烟；真微调请调大 `max_steps` 与 `optim.total_training_steps`）

## 常见问题 / 可调项

- **改 GPU 数**：同步改 `cluster.component_placement`（如 `0-0` 单卡）、`CUDA_VISIBLE_DEVICES`，并保证 `global_batch_size` 能被 `卡数 × micro_batch_size` 整除。单卡可设 `global_batch_size: 16`、`sharding_strategy: no_shard`。
- **换基座权重**：设 `BASE_CKPT=/path/to/your_ckpt_dir`（需含 `model.safetensors` + `config.json`）。换权重后建议删掉 `_ckpt/pi05_pushdoor_tst1/` 里的旧软链接重跑。
- **换数据集**：设 `RLINF_PUSHDOOR_DATA=/path/to/other_pushdoor_lerobot_root`（须与 pushdoor schema 一致），并重算 norm_stats。
- **norm_stats 未找到**：`run_train.sh` 会报错并提示先跑第 1 步。
- **离线**：脚本默认 `HF_HUB_OFFLINE=1`，只用本地 LeRobot 缓存，避免多 rank 同时拉数据触发 HF 429。
- **卡在无输出**：多为 Python 块缓冲（已在脚本里设 `PYTHONUNBUFFERED=1`）或视频解码 CPU 受限（调大 `NUM_WORKERS`）。

## 目录内容

- `pushdoor_sft_pi05_au.yaml` —— 训练配置（8 卡冒烟）
- `compute_norm_stats_au.py` —— 归一化统计计算（自包含，默认 tst1）
- `run_norm_stats.sh` —— 计算 norm_stats 的封装脚本
- `run_train.sh` —— staging + 预检 + 启动训练
- `_ckpt/`、`_out/` —— 运行时生成（权重软链接 + norm_stats / checkpoints + 日志），不入库
