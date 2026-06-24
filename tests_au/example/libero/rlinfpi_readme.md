# RLinf Pi0.5 LIBERO 训练示例

基于 OpenPI 框架的 JAX 训练基础设施，从 RLinf 目录结构中运行 π₀.₅ (pi0.5) 模型在 LIBERO 数据集上的微调训练。
所有配置和优化方法完全匹配 OpenPI 参考实现 (`/home/physical/SRC/Robot/openpi05/b/tst/libero/`)。

## 环境要求

| 项目 | 要求 |
|------|------|
| Python | >= 3.11（本例使用 3.11.14） |
| JAX | 0.5.3（含 CUDA 12 支持） |
| Flax | 0.10.2 |
| GPU | 8x NVIDIA H200（143GB HBM3e） |
| 虚拟环境 | `/mnt/r/VENV/openpi_venv/` |
| 数据集 | `physical-intelligence/libero`（LeRobot 格式，约 994 episodes） |
| OpenPI 代码库 | `/home/physical/SRC/Robot/openpi05/`（不做任何修改） |

## 文件说明

```
tests_au/example/libero/
├── rlinfpi_readme.md              # 本文件
├── train_pi05_libero_rlinf.py     # RLinf 版训练包装器
└── run_train.sh                   # 8-GPU 训练启动脚本
```

### 核心文件说明

**`train_pi05_libero_rlinf.py`** — 训练包装器脚本。通过 `sys.path.insert` 导入 OpenPI 的训练模块，使用 `unittest.mock.patch` 对 `create_torch_dataset` 进行 monkey-patch，使其仅加载本地已缓存的 episode 数据，避免对 HuggingFace Hub 的网络依赖。最终调用 OpenPI 原始的 `train.main()` 执行完整的 JAX 训练循环。

**`run_train.sh`** — Shell 启动脚本。设置 XLA/CUDA 环境变量，切换工作目录到 OpenPI 根目录，以完整的 CLI 参数调用训练脚本。

## Step-by-Step 使用说明

### Step 1: 确认虚拟环境就绪

```bash
/mnt/r/VENV/openpi_venv/bin/python --version
# 应输出: Python 3.11.x

/mnt/r/VENV/openpi_venv/bin/python -c "import jax; print(jax.__version__, jax.devices())"
# 应输出: 0.5.3 [CudaDevice(id=0), ..., CudaDevice(id=7)]
```

确认 JAX 版本和 8 张 CUDA 设备均可见。

### Step 2: 确认数据集已缓存

LIBERO 数据集需要预先下载到本地缓存：

```bash
# 检查本地缓存
ls ~/.cache/huggingface/lerobot/physical-intelligence/libero/data/
# 应看到 chunk-000/, chunk-001/ 等目录

# 计算可用 episode 数量
find ~/.cache/huggingface/lerobot/physical-intelligence/libero/data/ -name "*.parquet" | wc -l
# 完整数据集约 994 个 episode
```

如果数据未缓存，首次下载：
```bash
/mnt/r/VENV/openpi_venv/bin/python -c \
  "from lerobot.common.datasets.lerobot_dataset import LeRobotDataset; LeRobotDataset('physical-intelligence/libero')"
```

### Step 3: 确认归一化统计文件存在

```bash
ls /home/physical/SRC/Robot/openpi05/assets/pi05_libero/physical-intelligence/libero/norm_stats.json
```

如果不存在，需先计算：
```bash
cd /home/physical/SRC/Robot/openpi05
UV_PROJECT_ENVIRONMENT=/mnt/r/VENV/openpi_venv uv run scripts/compute_norm_stats.py --config-name pi05_libero
```

### Step 4: 确认 GPU 空闲

训练需要全部 8 张 GPU 的完整显存。启动前务必确认没有其他进程占用 GPU：

```bash
nvidia-smi --query-compute-apps=pid,name,used_memory --format=csv,noheader
# 应为空，表示无 GPU 占用
```

如有旧进程：
```bash
kill <pid>  # 终止占用 GPU 的进程
```

### Step 5: 启动训练

**方式一：使用 Shell 脚本（推荐）**

```bash
bash /home/physical/SRC/RL/RLinf/tests_au/example/libero/run_train.sh
```

**方式二：直接运行 Python**

```bash
export XLA_PYTHON_CLIENT_MEM_FRACTION=0.9
export CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7
export XLA_FLAGS="--xla_gpu_enable_latency_hiding_scheduler=true"
export JAX_COMPILATION_CACHE_DIR="${HOME}/.cache/jax_compilation_cache"

cd /home/physical/SRC/Robot/openpi05

/mnt/r/VENV/openpi_venv/bin/python \
    /home/physical/SRC/RL/RLinf/tests_au/example/libero/train_pi05_libero_rlinf.py \
    pi05_libero \
    --exp-name=rlinf_pi05_libero_8gpu_1k \
    --batch-size=128 \
    --num-train-steps=1000 \
    --save-interval=200 \
    --log-interval=50 \
    --fsdp-devices=8 \
    --ema-decay=0.999 \
    --lr-schedule.warmup-steps=100 \
    --lr-schedule.peak-lr=5e-5 \
    --lr-schedule.decay-steps=1000 \
    --lr-schedule.decay-lr=5e-6 \
    --overwrite
```

**注意**: 工作目录必须为 OpenPI 根目录（`/home/physical/SRC/Robot/openpi05`），否则资源文件定位会失败。

### Step 6: 训练产出

训练完成后，checkpoint 保存在：

```
/home/physical/SRC/Robot/openpi05/checkpoints/pi05_libero/rlinf_pi05_libero_8gpu_1k/999/
├── _CHECKPOINT_METADATA   # Orbax 元数据
├── assets/                # norm_stats 等元数据
├── params/                # EMA 模型权重（用于推理）
└── train_state/           # 完整训练状态（optimizer state 等）
```

Checkpoint 大小约 42GB。W&B 日志同步到 `wandb.ai` 的 `openpi` 项目。

### Step 7: 使用训练好的模型推理（可选）

```bash
cd /home/physical/SRC/Robot/openpi05

UV_PROJECT_ENVIRONMENT=/mnt/r/VENV/openpi_venv uv run scripts/serve_policy.py \
    policy:checkpoint \
    --policy.config=pi05_libero \
    --policy.dir=checkpoints/pi05_libero/rlinf_pi05_libero_8gpu_1k/999
```

## 训练参数说明

| 参数 | 值 | 说明 |
|------|-----|------|
| 配置名 | `pi05_libero` | OpenPI 内置配置，π₀.₅ + LIBERO |
| 模型 | `Pi0Config(pi05=True, action_horizon=10)` | π₀.₅ 架构（PaliGemma 2B + Action Expert 300M） |
| 预训练权重 | `gs://openpi-assets/checkpoints/pi05_base/params` | Physical Intelligence 官方预训练基座 |
| 全局 batch size | 128 | 8 GPU × 16 per GPU |
| 训练步数 | 1000 | 短期微调演示 |
| Checkpoint 间隔 | 200 步 | 步 200/400/600/800/999 各保存一次 |
| 日志间隔 | 50 步 | 每 50 步记录 loss / grad_norm / param_norm |
| 学习率 | peak 5e-5, decay to 5e-6 | 100 步 warmup + cosine decay |
| EMA decay | 0.999 | 推理时使用 EMA 权重，效果更稳定 |
| FSDP 设备 | 8 | 全部 GPU 参与模型参数 8 路分片 |
| Checkpoint 保留 | `max_to_keep=1` | 仅保留最新 checkpoint |

## 启用的优化与技巧

所有优化与 OpenPI 参考实现完全一致：

| 优化 | 配置 | 说明 |
|------|------|------|
| **FSDP** | `--fsdp-devices=8` | Fully Sharded Data Parallelism，模型参数分片到 8 张 GPU |
| **EMA** | `--ema-decay=0.999` | 指数移动平均，推理时使用 EMA 权重 |
| **bfloat16** | 默认启用 | 激活值 bfloat16，权重/梯度 float32 |
| **梯度裁剪** | `clip_gradient_norm=1.0` | 全局梯度范数裁剪，防止梯度爆炸 |
| **Cosine LR + Warmup** | warmup=100, decay=1000 | 预热后余弦衰减学习率 |
| **AdamW** | b1=0.9, b2=0.95, wd=1e-10 | 带权重衰减的 Adam 优化器 |
| **XLA 内存优化** | `XLA_PYTHON_CLIENT_MEM_FRACTION=0.9` | 分配 90% GPU 显存给 JAX |
| **XLA Latency Hiding** | `xla_gpu_enable_latency_hiding_scheduler` | 重叠计算与通信，隐藏延迟 |
| **JAX 编译缓存** | `JAX_COMPILATION_CACHE_DIR` | 缓存 XLA 编译结果，后续启动更快 |
| **Activation Remat** | 内置 `nn.remat + nn.scan` | 激活值重计算，常数显存占用 |
| **图像增强** | 内置 RandomCrop/Rotate/ColorJitter | 训练时数据增强提升泛化性 |
| **Buffer Donation** | 内置 `donate_argnums` | JIT 中捐赠输入 buffer，减少显存拷贝 |
| **分位数归一化** | `use_quantile_norm=True` | 使用 Q01/Q99 分位数归一化，对异常值鲁棒 |
| **Flow Matching** | Beta(1.5,1) 时间采样 | 条件流匹配训练目标，Euler ODE 推理采样 |
| **AdaRMSNorm** | Zero-init adaLN | 自适应归一化层，零初始化确保稳定训练启动 |

## 训练结果

成功完成 1000 步训练，在 8x H200 上耗时约 **16 分钟**（~1.2 it/s，即 ~155 samples/s）。

训练期间 GPU 利用率 99-100%，每卡显存使用 133GB / 143GB。

| Step | Loss | Grad Norm | Param Norm |
|------|------|-----------|------------|
| 0 | 0.0893 | 0.9612 | 1802.39 |
| 50 | 0.0477 | 0.2087 | 1802.39 |
| 100 | 0.0320 | 0.1492 | 1802.40 |
| 150 | 0.0299 | 0.1154 | 1802.44 |
| 200 | 0.0284 | 0.0911 | 1802.48 |
| 250 | 0.0273 | 0.0916 | 1802.53 |
| 300 | 0.0264 | 0.0761 | 1802.58 |
| 350 | 0.0261 | 0.0796 | 1802.62 |
| 400 | 0.0250 | 0.0739 | 1802.65 |
| 450 | 0.0242 | 0.0779 | 1802.68 |
| 500 | 0.0243 | 0.0711 | 1802.71 |
| 550 | 0.0237 | 0.0776 | 1802.74 |
| 600 | 0.0226 | 0.0657 | 1802.75 |
| 650 | 0.0221 | 0.0638 | 1802.77 |
| 700 | 0.0221 | 0.0719 | 1802.78 |
| 750 | 0.0216 | 0.0666 | 1802.78 |
| 800 | 0.0211 | 0.0691 | 1802.79 |
| 850 | 0.0204 | 0.0620 | 1802.79 |
| 900 | 0.0200 | 0.0631 | 1802.80 |
| 950 | 0.0199 | 0.0545 | 1802.80 |

Loss 从 0.0893 稳定下降至 0.0199（下降 78%），梯度范数从 0.96 收敛至 0.05，训练全程无 NaN 或发散。

Checkpoint 保存记录：
- Step 200: 保存成功（42GB, 45s）
- Step 400: 保存成功（42GB, 42s），删除 step 200
- Step 600: 保存成功（42GB, 42s），删除 step 400
- Step 800: 保存成功（42GB, 42s），删除 step 600
- Step 999: 保存成功（42GB, 42s），删除 step 800

最终保留 step 999 的 checkpoint，包含 EMA 权重（`params/`）和完整训练状态（`train_state/`）。

W&B 记录: https://wandb.ai/luo_1cn/openpi/runs/pkuz9c1i

## 与 OpenPI 参考实现的对比

| 维度 | OpenPI 参考 (`b/tst/libero/`) | RLinf 版本 |
|------|------|------|
| 训练基础设施 | OpenPI JAX (Flow Matching + FSDP) | 相同（直接调用 OpenPI 训练循环） |
| 模型配置 | `pi05_libero` | 相同 |
| Batch size | 128 (8 GPU × 16) | 相同 |
| 训练步数 | 1000 | 相同 |
| LR schedule | 5e-5 → 5e-6 cosine, 100 step warmup | 相同 |
| Optimizer | AdamW (b1=0.9, b2=0.95, clip=1.0) | 相同 |
| 所有优化 trick | 全部启用 | 全部启用 |
| 数据加载 | Monkey-patch 仅用本地缓存 | 相同方式 |
| 运行方式 | 从 `b/tst/libero/` 运行 | 从 RLinf `tests_au/example/libero/` 运行 |

## 遇到的问题与解决方案

### 问题 0: 训练运行顺利，未遇到新错误

本次 RLinf 版训练从启动到完成未遇到任何错误。这得益于预先借鉴了 OpenPI 参考实现中记录的已知问题（见下文），在脚本中提前做了规避。

### 已知问题 1: XLA_FLAGS 不兼容（已预防）

**问题描述**: JAX 0.5.3 的 XLA 版本不支持 `--xla_gpu_enable_triton_softmax_fusion=true` 和 `--xla_gpu_triton_gemm_any=true` 标志，使用会导致启动时致命错误：
```
F external/xla/xla/parse_flags_from_env.cc:233] Unknown flag in XLA_FLAGS: --xla_gpu_enable_triton_softmax_fusion=true
```

**解决方案**: 在 `run_train.sh` 中仅使用当前 JAX 版本支持的优化标志：
```bash
export XLA_FLAGS="--xla_gpu_enable_latency_hiding_scheduler=true"
```

### 已知问题 2: GPU 显存被旧进程占用导致 OOM（已预防）

**问题描述**: 如果之前的训练进程仍在后台运行，新训练进程无法分配到足够的 GPU 显存，会报 NCCL/CUDA OOM 错误。

**解决方案**: 训练前使用 `nvidia-smi` 检查 GPU 占用，终止占用 GPU 的旧进程：
```bash
nvidia-smi --query-compute-apps=pid,name,used_memory --format=csv,noheader
kill <old_pid>
```

### 已知问题 3: LeRobot 数据集版本警告（无害）

**警告信息**:
```
The dataset you requested (physical-intelligence/libero) is in 2.0 format.
```

**说明**: 本地缓存数据集使用 LeRobot v2.0 格式（全局统计），而当前 LeRobot 库使用 v2.1 格式（按 episode 统计）。LeRobot 保持向后兼容，此警告不影响训练，可安全忽略。

## 架构说明

### 设计原则

RLinf 版本不修改 OpenPI 原代码，而是通过以下方式复用 OpenPI 的 JAX 训练基础设施：

1. **路径注入**: 通过 `sys.path.insert` 将 OpenPI 的 `src/` 和 `scripts/` 目录加入 Python 搜索路径
2. **Monkey-patch**: 使用 `unittest.mock.patch` 替换 `create_torch_dataset` 函数，限制只使用本地已缓存的 episode
3. **委托执行**: 调用 OpenPI 原始的 `train.main(config)` 执行完整训练

这种方式保证了训练行为与 OpenPI 参考实现 100% 一致，同时允许从 RLinf 的目录结构中独立运行。

### FSDP 分片策略

使用 `fsdp_devices=8` 创建 mesh shape `(1, 8)`：
- Batch 轴大小 = 1（无数据并行复制）
- FSDP 轴大小 = 8（模型参数 8 路分片）
- 大于 4MB 的参数张量沿最大可整除维度切分到 FSDP 轴
- 小参数复制到所有设备
- 数据沿 `DATA_AXIS = (BATCH, FSDP)` 分配，128 samples 均匀分布在 8 GPU 上

### LR Schedule 适配

官方 `pi05_libero` 配置默认 warmup_steps=10000、decay_steps=1000000（适用于 30k+ 步训练）。本例训练仅 1000 步，因此通过 CLI override 调整为：
- `warmup_steps=100`（前 10% 预热）
- `decay_steps=1000`（匹配总步数）
- `peak_lr=5e-5`（与官方一致）
- `decay_lr=5e-6`（衰减到 peak 的 1/10）

### Checkpoint 管理

OpenPI 使用 Orbax AsyncCheckpointer。配置 `max_to_keep=1` 意味着：
- 每次保存新 checkpoint 后，旧 checkpoint 被异步删除
- 单个 checkpoint 约 42GB（含 train_state、EMA params、assets）
- 异步保存不阻塞训练（约 42s 后台完成）
- 最终仅保留 step 999 的 checkpoint
