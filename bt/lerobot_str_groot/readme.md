# str_groot_1 — StarVLA Qwen-GR00T 在 LIBERO 上的训练

## 概述

将 StarVLA 的 Qwen3-VL + GR00T FlowMatching action head（`str_groot`）适配到 LeRobot 框架，
在 `HuggingFaceVLA/libero` 数据集上进行微调训练。

模型结构：Qwen3-VL-4B (VLM backbone) → DiT-B FlowMatching (action head)，总参数 ~5B，可训练参数 ~161M（冻结 VLM 时）。

## 文件说明

| 文件 | 说明 |
|---|---|
| `train_str_groot_libero.py` | 训练入口脚本 |
| `test_str_groot_libero37.py` | 分阶段验证脚本（config → dataset → model → forward → predict → e2e） |
| `test_episodes_37.json` | 上次测试使用的 37 条 episode 索引 |

## 快速端到端验证

从 LIBERO（共 1693 条 episode）中随机抽 37 条，跑 2 步训练，验证完整流水线：

```bash
python bt/str_groot_1/train_str_groot_libero.py \
  --episodes 0 1 3 4 11 12 13 15 16 18 24 25 27 28 29 30 32 35 36 37 \
             39 40 41 42 46 48 49 50 53 54 56 57 58 60 61 62 63 \
  --steps 2 \
  --batch-size 1 \
  --freeze-vlm \
  --no-save-checkpoint \
  --starvla-checkpoint "" \
  --log-freq 1 \
  --num-workers 0
```

> `--starvla-checkpoint ""` 表示不加载 StarVLA 预训练权重（action head 随机初始化），仅用于快速验证流水线。
> 正式训练时去掉此参数，默认使用 `StarVLA/Qwen3VL-GR00T-Bridge-RT-1`。

### 验证通过的输出（2026-03-20）

```
dataset.num_frames  = 5338
dataset.num_episodes = 37
num_learnable_params = 161473799 (161M)
num_total_params     = 4599289607 (5B)

step:1 smpl:1 ep:0 epch:0.00 loss:1.108 grdn:4.057  lr:5.5e-05 updt_s:1.388
step:2 smpl:2 ep:0 epch:0.00 loss:2.147 grdn:12.221 lr:1.0e-05 updt_s:0.072
End of training
```

## 正式训练

```bash
# 全量训练（全部 1693 episodes, 50k steps）
python bt/str_groot_1/train_str_groot_libero.py \
  --steps 50000 \
  --batch-size 8 \
  --freeze-vlm \
  --wandb

# 指定 episode 子集训练
python bt/str_groot_1/train_str_groot_libero.py \
  --episodes 0 1 3 4 11 12 13 15 16 18 24 25 27 28 29 30 32 35 36 37 \
  --steps 100 \
  --batch-size 8
```

## 主要参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--dataset-repo` | `HuggingFaceVLA/libero` | HuggingFace 数据集 repo |
| `--episodes` | 全部 | 指定 episode 索引列表 |
| `--starvla-checkpoint` | `StarVLA/Qwen3VL-GR00T-Bridge-RT-1` | 预训练权重（HF repo 或本地路径，空字符串跳过） |
| `--base-vlm` | `Qwen/Qwen3-VL-4B-Instruct` | VLM backbone |
| `--action-dim` | 7 | 动作维度 |
| `--state-dim` | 7 | 状态维度（会自动从数据集推断） |
| `--freeze-vlm` | False | 是否冻结 VLM 只训练 action head |
| `--steps` | 50000 | 训练步数 |
| `--batch-size` | 8 | 批大小 |
| `--lr` | 1e-4 | 学习率 |
| `--save-freq` | 5000 | checkpoint 保存间隔 |
| `--wandb` | False | 启用 WandB 日志 |

## 已修复的问题

### state_dim 维度不匹配 (2026-03-20)

**现象**：训练时 `state_encoder` 报 `RuntimeError: mat1 and mat2 shapes cannot be multiplied (8x8 and 7x1024)`。

**原因**：`StrGrootConfig.state_dim` 默认为 7，但 LIBERO 数据集的 `observation.state` 实际维度为 8。
`factory.py` 在创建 policy 前会从数据集自动填充 `input_features`，但 `validate_features()` 没有将
`state_dim` 同步为数据集的真实维度，导致 `state_encoder` 的 MLP 用错误的输入维度构建。

**修复**（`configuration_str_groot.py`）：在 `validate_features()` 中，当 `observation.state` 已存在于
`input_features` 时，自动将 `state_dim` 更新为数据集提供的实际维度。



## 使用 `lerobot-train` 微调 `str_groot`（`StarVLA/Qwen3VL-GR00T-Bridge-RT-1`）

下面给出直接使用 LeRobot 标准入口 `lerobot-train` 的方式，无需再写单独 Python 训练入口。

### 1) 快速冒烟测试（随机 37 条 episode，训练 2 步）

脚本会自动：
- 从 `HuggingFaceVLA/libero` 随机抽取 37 条 episode
- 保存到 `bt/str_groot_1/test_episodes_37.random.json`
- 用 `lerobot-train` 跑 `policy.type=str_groot` 的 2-step 烟测

```bash
bash bt/str_groot_1/test_random37_lerobot_train.sh
```

可选覆盖参数示例：

```bash
STEPS=10 BATCH_SIZE=2 POLICY_DEVICE=cuda:0 \
STARVLA_CHECKPOINT=StarVLA/Qwen3VL-GR00T-Bridge-RT-1 \
bash bt/str_groot_1/test_random37_lerobot_train.sh
```

### 2) 单卡正式微调

```bash
lerobot-train \
  --policy.type=str_groot \
  --policy.push_to_hub=false \
  --policy.device=cuda:0 \
  --policy.starvla_checkpoint=StarVLA/Qwen3VL-GR00T-Bridge-RT-1 \
  --policy.freeze_vlm=true \
  --policy.tune_vlm=false \
  --policy.tune_action_head=true \
  --policy.state_indices=[0,1,2,3,4,5,7] \
  --dataset.repo_id=HuggingFaceVLA/libero \
  --steps=50000 \
  --batch_size=8 \
  --num_workers=4 \
  --eval_freq=0 \
  --log_freq=7 \
  --save_checkpoint=true \
  --save_freq=10 \
  --wandb.enable=true \
  --wandb.project=lrb_strgrt_1 \
  --wandb.disable_artifact=true \
  --output_dir=outputs/bt/str_groot_1/ft_cli \
  --job_name=str_groot_libero_ft
```

### 3) 多卡微调（`accelerate`）

```bash
export CUDA_LAUNCH_BLOCKING=1
accelerate launch \
  --multi_gpu \
  --num_processes=8 \
  $(which lerobot-train) \
  --policy.type=str_groot \
  --policy.push_to_hub=false \
  --policy.device=cuda \
  --policy.starvla_checkpoint=StarVLA/Qwen3VL-GR00T-Bridge-RT-1 \
  --policy.freeze_vlm=true \
  --policy.tune_vlm=false \
  --policy.tune_action_head=true \
  --policy.state_indices=[0,1,2,3,4,5,7] \
  --dataset.repo_id=HuggingFaceVLA/libero \
  --batch_size=32 \
  --steps=30000 \
  --save_checkpoint=true \
  --save_freq=10 \
  --log_freq=7 \
  --eval_freq=0 \
  --wandb.enable=true \
  --wandb.project=lrb_strgrt_1 \
  --wandb.disable_artifact=true \
  --output_dir=/mnt/g/CKPT/VLA/Libero/r0_a/ \
  --job_name=r0_a
```

后台运行示例（带日志重定向与 PID 文件）：

```bash
export BTPRJNAME="lrb_strgrt_1"
export BTJOBNAME="r1"
export OUT_DIR="/mnt/g/CKPT/VLA/Libero/${BTPRJNAME}_${BTJOBNAME}"

nohup accelerate launch \
  --multi_gpu \
  --num_processes=8 \
  $(which lerobot-train) \
  --policy.type=str_groot \
  --policy.push_to_hub=false \
  --policy.device=cuda \
  --policy.starvla_checkpoint=StarVLA/Qwen3VL-GR00T-Bridge-RT-1 \
  --policy.freeze_vlm=true \
  --policy.tune_vlm=false \
  --policy.tune_action_head=true \
  --policy.state_indices=[0,1,2,3,4,5,7] \
  --dataset.repo_id=HuggingFaceVLA/libero \
  --batch_size=64 \
  --steps=30000 \
  --save_checkpoint=true \
  --save_freq=1000 \
  --log_freq=100 \
  --eval_freq=0 \
  --wandb.enable=true \
  --wandb.project=${BTPRJNAME} \
  --wandb.disable_artifact=true \
  --output_dir="${OUT_DIR}" \
  --job_name="${BTJOBNAME}" \
  > /mnt/g/CKPT/VLA/Libero/${BTPRJNAME}_${BTJOBNAME}.log 2>&1 &

echo $! > /mnt/g/CKPT/VLA/Libero/${BTPRJNAME}_${BTJOBNAME}.pid
echo "训练已在后台启动，PID=$(cat /mnt/g/CKPT/VLA/Libero/${BTPRJNAME}_${BTJOBNAME}.pid)，日志：/mnt/g/CKPT/VLA/Libero/${BTPRJNAME}_${BTJOBNAME}.log"

```

### 4) `resume=true` 从特定 step 的 checkpoint 继续微调

先查看已有 checkpoint（目录名通常是 6 位步数，如 `000500`）：

```bash
ls -1 outputs/bt/str_groot_1/ft_cli/checkpoints
```

然后指定该 step 下的 `train_config.json` 继续训练，例如从 `step=500` 继续：

```bash
lerobot-train \
  --resume=true \
  --config_path=outputs/bt/str_groot_1/ft_cli/checkpoints/000500/pretrained_model/train_config.json \
  --steps=50000 \
  --save_freq=500 \
  --log_freq=50 \
  --eval_freq=0
```

说明：
- `--config_path` 必须指向某个 checkpoint 的 `pretrained_model/train_config.json`。
- `--steps` 要 **大于** 已保存的训练步数（例如已到 500，则设成 500 以上）；否则不会继续训练。
- `resume=true` 时会自动恢复该 checkpoint 的模型权重、optimizer/scheduler 状态和随机数状态。
- 常用快捷方式：如果想从最近一次保存继续，可用 `checkpoints/last/pretrained_model/train_config.json`。

#### 示例：从 `000500` 续训到 `000800`（后台运行 + 日志重定向）

```bash
export RUN_DIR="outputs/bt/str_groot_1/ft_cli"
export RESUME_STEP="000500"
export TARGET_STEP="800"
export LOG_DIR="${RUN_DIR}/resume_logs"
mkdir -p "${LOG_DIR}"

nohup $(which lerobot-train) \
  --resume=true \
  --config_path="${RUN_DIR}/checkpoints/${RESUME_STEP}/pretrained_model/train_config.json" \
  --output_dir="${RUN_DIR}" \
  --steps="${TARGET_STEP}" \
  --save_checkpoint=true \
  --save_freq=100 \
  --log_freq=20 \
  --eval_freq=0 \
  > "${LOG_DIR}/resume_${RESUME_STEP}_to_${TARGET_STEP}.log" 2>&1 &

echo $! > "${LOG_DIR}/resume_${RESUME_STEP}_to_${TARGET_STEP}.pid"
echo "续训已启动，PID=$(cat "${LOG_DIR}/resume_${RESUME_STEP}_to_${TARGET_STEP}.pid")"
echo "日志文件: ${LOG_DIR}/resume_${RESUME_STEP}_to_${TARGET_STEP}.log"
```

> 上面 `--output_dir="${RUN_DIR}"` 表示继续写入原训练目录。  
> 如果想把续训产物写到新目录，可把 `--output_dir` 改成新的路径（例如 `outputs/bt/str_groot_1/ft_cli_resume`）。

### 5) 关键参数说明（`lerobot-train` 版本）

- `--policy.type=str_groot`：启用 StrGroot policy。
- `--policy.starvla_checkpoint=StarVLA/Qwen3VL-GR00T-Bridge-RT-1`：指定 StarVLA 预训练权重。
- `--policy.freeze_vlm=true` + `--policy.tune_vlm=false`：冻结 VLM，仅训练 action head。
- `--policy.tune_action_head=true`：开启 action head 微调。
- `--policy.state_indices=[0,1,2,3,4,5,7]`：LIBERO 常用配置，跳过 pad 维度索引 6。
- `--policy.push_to_hub=false`：不上传 HuggingFace Hub 时必须设为 false，避免 `policy.repo_id` 校验报错。

### 6) 常见问题

- **首次运行会下载较大模型和数据**：耗时较长属正常。
- **看到某些 framework 子模块导入 warning**（如 `snntorch` 缺失）：只要 `str_groot` 正常创建并开始训练，可忽略。
- **仅做流水线验证**：可将 `--policy.starvla_checkpoint=` 设为空字符串，跳过 StarVLA 预训练权重加载。

## EVAL

```bash
export CUDA_VISIBLE_DEVICES=7
export BTPRJNAME="lrb_strgrt_1"
export BTJOBNAME="r1"
export MUJOCO_GL=egl
lerobot-eval \
  --env.type=libero \
  --env.task=libero_spatial \
  --env.obs_type=pixels_agent_pos \
  --env.observation_height=256 \
  --env.observation_width=256 \
  --env.control_mode=relative \
  --env.max_parallel_tasks=1 \
  --eval.batch_size=1 \
  --eval.n_episodes=10 \
  --eval.use_async_envs=false \
  --policy.path=/mnt/g/CKPT/VLA/Libero/${BTPRJNAME}_${BTJOBNAME}/checkpoints/030000/pretrained_model \
  --policy.device=cuda \
  --policy.use_amp=false \
  --seed=1000 \
  --output_dir=/mnt/g/CKPT/VLA/Libero/${BTPRJNAME}_${BTJOBNAME}/eval/spatial030000/

```