# RLinf π₀.₅ LIBERO 8-GPU Training Example

RLinf 版的、从 **pi05_base** 开始在 **LIBERO** 数据集上微调 **π₀.₅** 的训练示例。使用
`rlinf/models/embodiment/openpi_au/` 里的 PyTorch π₀.₅，所有配置/优化与 openpi 的 JAX 参考实现
（[`openpi05/b/tst/libero/`](/home/physical/SRC/Robot/openpi05/b/tst/libero/)）保持一致：
**8×H200 GPU、全局 batch 128、1000 步、每 200 步保存 checkpoint、EMA 0.999、bf16 计算 + fp32 敏感层、
warmup+cosine 学习率、quantile 归一化**。

> 设计约束：**不修改 openpi 源码**；RLinf 仅以**扩展**方式改动（新增 `tests_au/example/libero/` 下的文件，
> 以及在扩展文件 `rlinf/workers/sft/fsdp_vla_sft_worker_au.py` 中新增方法/函数；基类 worker 不动）。
> 所有操作在虚拟环境 `/mnt/r/VENV/openpi_venv/` 中进行。

---

## 环境要求

| 项目 | 值 |
|------|-----|
| Python | 3.11（venv `/mnt/r/VENV/openpi_venv`） |
| PyTorch | 2.7.1 + CUDA 12.6（8×H200 可见） |
| openpi | 以 editable 方式装在 venv（`/home/physical/SRC/Robot/openpi05/src`） |
| RLinf | 以 editable 方式装在 venv（`uv pip install -e . --no-deps` + ray/hydra/torchdata/accelerate） |
| transformers | 4.53.2 + 已打 `transformers_replace` 补丁（openpi 要求） |
| 数据集 | `physical-intelligence/libero`（LeRobot，本地缓存于 `~/.cache/huggingface/lerobot/...`） |
| 预训练权重 | pi05_base，已转换为 PyTorch（`model.safetensors`，见下） |

---

## 目录文件

```
tests_au/example/libero/
├── rlinfpi_readme.md            # 本文件
├── compute_norm_stats_au.py     # RLinf 侧重算 LIBERO 归一化统计（用 openpi_au dataconfig）
├── libero_sft_pi05_au_8gpu.yaml # 训练配置（8 GPU / batch128 / 1000 步 / save 200 / JAX 对齐）
├── run_norm_stats.sh            # 跑归一化统计的脚本
├── run_train.sh                 # 8-GPU 训练脚本
└── _ckpt/pi05_base_pt/          # 自包含模型目录
    ├── model.safetensors -> /mnt/r/CKPT/VLA/pi05_base_pt_fp32/model.safetensors  (symlink)
    ├── config.json       -> .../config.json                                       (symlink)
    └── physical-intelligence/libero/norm_stats.json   (本脚本重算生成)
```

训练产物默认写到 `tests_au/example/libero/_out/pi05_libero_8gpu_1k/`。

---

## Step-by-Step 使用说明

### Step 0：准备 PyTorch 权重（一次性）

示例的 `_ckpt/pi05_base_pt/` 通过 symlink 复用已转换好的 pi05_base PyTorch 权重
（`/mnt/r/CKPT/VLA/pi05_base_pt_fp32/model.safetensors`）。若该权重不存在，先用 RLinf 自带转换器从
JAX 的 `pi05_base` 转换（与 openpi 推荐流程一致）：

```bash
/mnt/r/VENV/openpi_venv/bin/python rlinf/utils/ckpt_convertor/convert_openpi_jax_to_python.py \
    --checkpoint_dir ~/.cache/openpi/openpi-assets/checkpoints/pi05_base \
    --config_name pi05_libero \
    --output_path /mnt/r/CKPT/VLA/pi05_base_pt_fp32 \
    --precision float32
```

然后建立示例模型目录（symlink 大权重 + config）：

```bash
cd tests_au/example/libero
mkdir -p _ckpt/pi05_base_pt
ln -sf /mnt/r/CKPT/VLA/pi05_base_pt_fp32/model.safetensors _ckpt/pi05_base_pt/model.safetensors
ln -sf /mnt/r/CKPT/VLA/pi05_base_pt_fp32/config.json       _ckpt/pi05_base_pt/config.json
```

> 转换需要 openpi 的 `transformers_replace` 补丁已安装：
> `cp -r /home/physical/SRC/Robot/openpi05/src/openpi/models_pytorch/transformers_replace/* /mnt/r/VENV/openpi_venv/lib/python3.11/site-packages/transformers/`

### Step 1：确认环境与数据

```bash
/mnt/r/VENV/openpi_venv/bin/python -c "import torch; print(torch.__version__, torch.cuda.device_count())"  # 2.7.1+cu126 8
ls ~/.cache/huggingface/lerobot/physical-intelligence/libero/data/   # 应有 chunk-000/ ...
```

### Step 2：计算归一化统计

```bash
bash tests_au/example/libero/run_norm_stats.sh
```

生成 `_ckpt/pi05_base_pt/physical-intelligence/libero/norm_stats.json`（state/actions 的 mean/std + q01/q99）。
本次重算结果与 openpi JAX 参考的 `assets/pi05_libero/...` 逐元素误差 ~1e-6（实质一致）。

### Step 3：8-GPU 训练

```bash
bash tests_au/example/libero/run_train.sh
```

等价于（节选）：

```bash
export EMBODIED_PATH=/home/physical/SRC/RL/RLinf/examples/sft   # 解析 model/、training_backend/ 组默认
export PYTHONPATH=/home/physical/SRC/RL/RLinf:/home/physical/SRC/Robot/openpi05/src:$PYTHONPATH
export CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7
export HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1                   # 只用本地 LeRobot 缓存

/mnt/r/VENV/openpi_venv/bin/python examples/sft/train_vla_sft_au.py \
    --config-path tests_au/example/libero \
    --config-name libero_sft_pi05_au_8gpu
```

### Step 4：训练产出

```
tests_au/example/libero/_out/pi05_libero_8gpu_1k/checkpoints/
├── global_step_200/actor/{model_state_dict/, dcp_checkpoint/, ema.pt}
├── global_step_400/...
├── global_step_600/...
├── global_step_800/...
└── global_step_1000/...
```

- `model_state_dict/`（~8GB）：FSDP 全量权重（训练权重）。
- `dcp_checkpoint/`（~24GB）：分布式 checkpoint（含 optimizer state，可断点续训）。
- `ema.pt`（~1GB）：EMA 权重（推理/评测建议使用）。

### Step 5（可选）：TensorBoard 日志

示例默认 `logger_backends: []`（指标走 tqdm/控制台，无额外依赖）。若需 TensorBoard：

```bash
/mnt/r/VENV/openpi_venv/bin/pip install tensorboard   # 系统级操作
# 然后把 yaml 里的 logger_backends 改成 ["tensorboard"]
```

---

## 与 openpi JAX 参考的配置对照

| 维度 | openpi JAX (`b/tst/libero`) | RLinf 本示例 | 一致性 |
|------|------------------------------|--------------|--------|
| 模型 | `Pi0Config(pi05=True, action_horizon=10)` | `openpi_au` + `config_name=pi05_libero`, `num_action_chunks=10` | ✓ |
| 预训练权重 | `pi05_base/params`（JAX） | `pi05_base` 转 PyTorch（`model.safetensors`） | ✓（同源） |
| GPU / FSDP | `--fsdp-devices=8` | `component_placement: 0-7`, `sharding_strategy=full_shard` | ✓ 8 卡分片 |
| 全局 batch | 128（每卡 16） | `global_batch_size=128`, `micro_batch_size=16`（grad_accum=1） | ✓ |
| 训练步数 | `--num-train-steps=1000` | `runner.max_steps=1000` | ✓ |
| Checkpoint | `--save-interval=200` | `runner.save_interval=200` | ✓ |
| EMA | `--ema-decay=0.999` | `optim.ema_decay=0.999`（`ModelEMA`，FSDP 分片感知） | ✓ |
| 学习率 | warmup=100, peak=5e-5, decay_steps=1000, decay_lr=5e-6（cosine） | `lr_scheduler=openpi_cosine` + 同参数 | ✓ optax 等价 |
| 优化器 | AdamW b1=0.9 b2=0.95 wd=1e-10 | 同 | ✓ |
| 梯度裁剪 | `clip_gradient_norm=1.0` | `optim.clip_grad=1.0` | ✓ |
| 混合精度 | bf16 激活 + fp32 权重/梯度 | openpi 原生 PyTorch 选择性 bf16（`to_bfloat16_for_selected_params`）；FSDP 不二次 cast | ✓（见问题5） |
| 激活重计算 | 内置 `nn.remat + nn.scan` | `gradient_checkpointing=true` + `sft_gradient_checkpointing=true` | ✓ |
| 图像增强 | augmax RandomCrop/Rotate/ColorJitter | `faithful_augment`（逐样本 crop/rotate + 亮度感知色彩抖动） | ✓ |
| 归一化 | `use_quantile_norm=True`（Q01/Q99） | 同（`pi05` 默认）| ✓ |

---

## 训练结果

8×H200 上 1000 步耗时约 **24.8 分钟**（~1.2 s/it；JAX 参考约 16.5 分钟，PyTorch FSDP 略慢，属正常）。

| Step | LR | Loss | Grad Norm | Param Norm |
|------|------|--------|-----------|------------|
| 1 | 9.9e-7 | 0.0789 | 0.524 | 1.47e3 |
| 120 | 4.99e-5 (peak) | 0.0371 | 0.148 | 1.47e3 |
| 200 | 4.86e-5 | 0.0318 | 0.237 | 1.47e3 |
| 400 | 3.88e-5 | 0.0231 | 0.092 | 1.47e3 |
| 600 | 2.36e-5 | 0.0236 | 0.125 | 1.47e3 |
| 800 | 1.03e-5 | 0.0188 | 0.078 | 1.47e3 |
| 1000 | 5e-6 (decay_lr) | 0.0239 | 0.100 | 1.47e3 |

Loss 从 ~0.079 稳定下降到 ~0.02，LR 在第 100 步左右达到峰值 5e-5、随后余弦衰减到 5e-6，与 JAX 参考
（loss 0.089→0.020）的轨迹一致；梯度范数收敛。**5 个 checkpoint（200/400/600/800/1000）均成功保存（含 EMA）。**

---

## 遇到的问题与解决方案

> 全部修复均为「扩展」改动：只动 `rlinf/workers/sft/fsdp_vla_sft_worker_au.py`（扩展文件）、
> `rlinf/models/embodiment/openpi_au/augmentation.py`（openpi_au 副本）与本示例目录下的文件；
> openpi 源码与 RLinf 基类 worker 均未修改。

### 问题 1：`openpi_au` 无法走数据加载（KeyError）

**现象**：`FSDPVlaSftWorker.build_dataloader` 只匹配 `SupportedModel.OPENPI`，`openpi_au` 落到
`else` 分支抛 `KeyError: not support such model type openpi_au`。

**解决**：在扩展子类 `FSDPVlaSftWorkerAu` 中**覆写** `build_dataloader` 与 `get_max_steps_per_epoch`，
当 `model_type == openpi_au` 时改用 `rlinf.models.embodiment.openpi_au.dataconfig.get_openpi_config` +
openpi 的 `create_data_loader`；其他类型 `super()`。基类零改动。

### 问题 2：HuggingFace 429 限流（8 卡并发拉数据集）

**现象**：8 个 rank 同时 `LeRobotDataset(repo_id)`，并发请求 HF（含 xet 下载 token）超过
`1000 req / 5 min`，报 `429 Too Many Requests`。

**解决**：在 `build_dataloader` 中 `os.environ.setdefault("HF_HUB_OFFLINE","1")`（及 `HF_DATASETS_OFFLINE`），
并在 `run_train.sh` 中也 export，强制只用本地 LeRobot 缓存。与 openpi JAX 参考「只用本地 episode」的思路一致。

### 问题 3：`OfflineModeIsEnabled`（offline 下 LeRobot 仍触发 `/refs`）

**现象**：开 offline 后，`LeRobotDataset` 在「并非所有所需 episode 文件都在本地」时会走
`get_safe_version()`→ HF `/refs`，offline 下直接报错。

**解决**：用运行时 monkey-patch（不改 openpi 源码）把 openpi 的 `create_torch_dataset` 限制到
**本地已缓存的 episode**（mirror openpi JAX `train_pi05_libero.py` 的做法）。当传入的 episode 文件全部本地存在时，
LeRobot 的 “all files present” 断言通过，直接从磁盘加载，不触发任何 HF 调用。

### 问题 4：图像增强 dtype 不匹配（`expected scalar type BFloat16 but found Float`）

**现象**：`faithful_augment` 的 `grid_sample` 在 bf16 图像 + fp32 采样网格上报 dtype 不一致。

**解决**：在 `augmentation.py::faithful_augment` 中记录输入 dtype，**统一在 float32 下做增强**，
结束后再 `.to(orig_dtype)`。既避免 dtype 冲突，也提升数值保真。

### 问题 5：FSDP `param_dtype=bf16` 与 openpi 前向不兼容（`mat1 and mat2 ... Float vs BFloat16`）

**现象**：把 FSDP `mixed_precision.param_dtype` 设成 `bf16` 会把**所有**权重在前向 cast 成 bf16，
但 openpi 的 `pi0_pytorch.forward` 内部刻意混用 dtype（如 `state_proj` 作用于 fp32 state、
`action_out_proj` 前把 `suffix_out.to(float32)`），导致 Linear 的输入/权重 dtype 不一致而崩溃。
这与本仓库此前在验收文档中记录的「pure-bf16 与 openpi fp32 动作头不兼容」是同一根因。

**解决**：改用 **openpi 原生 PyTorch 混合精度**——`fp32_master_weights=false` 让 `get_model` 调用
`to_bfloat16_for_selected_params`（只把 transformer 选定参数转 bf16，敏感层保持 fp32），并把 FSDP
`mixed_precision` 全设 `null`（FSDP 不再二次 cast）。这正是 RLinf 基线 openpi SFT（`libero_sft_openpi.yaml`）
的可用配置，也是 openpi 自身 PyTorch 训练的混合精度方式（bf16 激活 + 关键层 fp32），与 JAX 语义一致。

### 问题 6：`IndexError: index 1684 is out of bounds`（episode 非连续）

**现象**：本地缓存的 episode 不连续（数量 1684，但最大索引 1684，存在一个空洞）。LeRobot 的
`episode_data_index` 按「连续 0..N-1」假设用原始 episode index 取下标，遇到空洞会越界。

**解决**：把限制逻辑从「全部已存在的索引」改为「**从 0 起的最大连续前缀** `range(M)`」
（本例 M=1672）。既保证文件全在本地（断言通过、offline 安全），又保证索引连续（不越界）。
与 openpi JAX 参考的 `range(available_episodes)` 一致。

### 其它（非阻断）说明

- 日志里反复出现的 `127.0.0.1:8265 connection refused` 来自 Ray 退出时的 `signal_handler` 调用
  dashboard API（dashboard 未启用），不影响训练；崩溃时它会刷屏，但真正的 root error 在更上方。
- LeRobot 提示数据集为 v2.0 格式（全局统计），仅为兼容性警告，不影响训练。
- `tensorboard` 未安装：示例默认 `logger_backends: []`（控制台/tqdm 指标），需要时按 Step 5 安装。

---

## 涉及的扩展改动清单

| 文件 | 改动 |
|------|------|
| `rlinf/workers/sft/fsdp_vla_sft_worker_au.py` | 新增 `build_dataloader`/`get_max_steps_per_epoch` 覆写（支持 `openpi_au`）；新增 `_local_episode_indices` / `_local_episodes_patch`（本地连续 episode + offline）；`HF_HUB_OFFLINE` 设置 |
| `rlinf/models/embodiment/openpi_au/augmentation.py` | `faithful_augment` 改为 fp32 内部计算并 cast 回原 dtype |
| `tests_au/example/libero/*` | 新增：配置、归一化脚本、两个 run 脚本、模型目录、本 README |

> 基类 `rlinf/workers/sft/fsdp_vla_sft_worker.py`、openpi 源码均**未改动**。
