# FastWAM r1_pro SFT 数值对齐测试方案

> **文档性质**：[`fw_sft_design_op46_4_r1pr_cp25_2.md`](fw_sft_design_op46_4_r1pr_cp25_2.md) 的配套验证方案。  
> **目标**：确保 RLinf 包装的 FastWAM SFT 训练与 FastWAM 原生训练在**权重和 loss 上差别极小**。  
> **代码基线**：FastWAM `/home/Luogang/SRC/Robot/FastWAM` · RLinf `/home/Luogang/SRC/RL/RLinf`  
> **日期**：2026-06-02

---

## 目录

1. [对齐的困难与策略](#1-对齐的困难与策略)
2. [两条训练管道的差异分析](#2-两条训练管道的差异分析)
3. [测试层级总览](#3-测试层级总览)
4. [T0 — 可训参数集合一致性](#4-t0--可训参数集合一致性)
5. [T1 — 单步 loss 精确对齐（核心测试）](#5-t1--单步-loss-精确对齐核心测试)
6. [T2 — 多步梯度与权重对齐](#6-t2--多步梯度与权重对齐)
7. [T3 — Checkpoint 权重互换](#7-t3--checkpoint-权重互换)
8. [T4 — 端到端多步 loss 曲线对齐](#8-t4--端到端多步-loss-曲线对齐)
9. [T5 — 分布式一致性](#9-t5--分布式一致性)
10. [容差标准与判定规则](#10-容差标准与判定规则)
11. [影响对齐的因素与消除方法](#11-影响对齐的因素与消除方法)
12. [测试脚本实现指南](#12-测试脚本实现指南)
13. [检查清单](#13-检查清单)

---

## 1. 对齐的困难与策略

### 1.1 为什么对齐不是 trivial 的

两条管道共享**同一个** `FastWAM.training_loss()` 函数，但包装层存在显著差异：

```
FastWAM 原生：  Accelerate + DeepSpeed ZeRO-1 → AdamW(dit.parameters()) → loss.backward()
RLinf：         Ray + FSDP1(no_shard) → AdamW(requires_grad 过滤) → grad_scaler.scale(loss).backward()
```

即使 `training_loss()` 代码完全相同，以下因素也会引入数值差异：

| 因素 | 说明 |
|------|------|
| RNG 状态 | `training_loss` 内部调用 `torch.randn_like`（噪声）和 `torch.rand`（时间步采样），依赖全局 CUDA RNG |
| FSDP `MixedPrecision` | `param_dtype: bf16` 会让 FSDP 在 all-gather 前后做 dtype 转换，即使 `no_shard` 也可能引入 cast |
| AMP autocast | 原生用 `accelerator.autocast()`（bf16），RLinf 的 `amp_autocast.enabled: false` 但 FSDP `mixed_precision` 隐式 cast |
| 梯度累积 | 原生 `gradient_accumulation_steps=1`；RLinf `loss /= gradient_accumulation` |
| 梯度裁剪 | 原生 `accelerator.clip_grad_norm_(model.parameters(), 1.0)`；RLinf `strategy.clip_grad_norm_` |
| Grad scaler | 原生 DeepSpeed 管理；RLinf `ShardedGradScaler(enabled=False)` |
| Optimizer 参数集合 | 原生明确 `dit.parameters() + proprio_encoder.parameters()`；RLinf 按 `requires_grad` 过滤所有参数 |
| `warmup_optimizer_state` | RLinf 在 optimizer 创建后执行空步初始化 optimizer state |

### 1.2 总体策略：分层消除差异

```
T0  可训参数集合相同吗？
     ↓
T1  给定相同输入 + 相同 RNG，单步 loss 一致吗？
     ↓
T2  给定相同输入，N 步后梯度和权重一致吗？
     ↓
T3  一方训出的 checkpoint 能无损加载到另一方吗？
     ↓
T4  端到端跑 N 步，loss 曲线差异在容差内吗？
     ↓
T5  多卡时一致性保持吗？
```

---

## 2. 两条训练管道的差异分析

### 2.1 训练管道逐步对比

```mermaid
flowchart TB
    subgraph native ["FastWAM 原生 (trainer.py)"]
        N1["set_global_seed(seed)"]
        N2["_apply_dit_only_train_mode(model)"]
        N3["AdamW(dit.parameters() + proprio)"]
        N4["accelerator.prepare(model, optimizer, loader, scheduler)"]
        N5["accelerator.accumulate(model)"]
        N6["accelerator.autocast()"]
        N7["training_loss(sample) → loss, loss_dict"]
        N8["accelerator.backward(loss)"]
        N9["clip_grad_norm_(model.parameters(), 1.0)"]
        N10["optimizer.step()"]
        N11["scheduler.step()"]
        N1 --> N2 --> N3 --> N4 --> N5 --> N6 --> N7 --> N8 --> N9 --> N10 --> N11
    end
    subgraph rlinf ["RLinf (fsdp_sft_worker.py)"]
        R1["seed_everything(seed)"]
        R2["model = get_model(cfg) → FastWAMPolicy"]
        R3["gradient_checkpointing_enable()"]
        R4["FSDP(model, no_shard, use_orig_params, mixed_precision=bf16)"]
        R5["build_optimizer: AdamW(requires_grad 过滤)"]
        R6["warmup_optimizer_state(optimizer)"]
        R7["model.train() → FastWAMPolicy.train()"]
        R8["amp_context (disabled)"]
        R9["model(forward_type=SFT, data=batch) → training_loss"]
        R10["loss /= gradient_accumulation"]
        R11["grad_scaler.scale(loss).backward()"]
        R12["strategy.clip_grad_norm_(model, 1.0)"]
        R13["grad_scaler.step(optimizer)"]
        R14["lr_scheduler.step()"]
        R1 --> R2 --> R3 --> R4 --> R5 --> R6 --> R7 --> R8 --> R9 --> R10 --> R11 --> R12 --> R13 --> R14
    end
```

### 2.2 关键差异表

| 维度 | FastWAM 原生 | RLinf | 影响 |
|------|-------------|-------|------|
| **分布式后端** | Accelerate + DeepSpeed ZeRO-1 | Ray + FSDP1 `no_shard` | all-reduce 实现不同；单卡时无影响 |
| **精度控制** | `accelerator.autocast()` bf16 | FSDP `MixedPrecision(param_dtype=bf16)` + `amp_autocast=false` | 前者用 autocast context，后者让 FSDP 全局 cast |
| **Optimizer 构造** | `AdamW(dit.parameters() + proprio)` 明确列表 | `AdamW(requires_grad 过滤)` + `warmup_optimizer_state` | 参数集合应一致（需验证 T0） |
| **梯度累积** | `accelerator.accumulate(model)` | `loss /= grad_accum` + `model.no_sync()` | 单步 `grad_accum=1` 时等价 |
| **梯度裁剪** | `accelerator.clip_grad_norm_(model.parameters(), max_grad_norm)` | `nn.utils.clip_grad_norm_(model.parameters(), max_norm)` (no_shard 分支) | 两者调用同一 PyTorch 函数，应一致 |
| **Grad scaler** | DeepSpeed 内部处理 | `ShardedGradScaler(enabled=False)` = no-op | RLinf scaler 禁用时无影响 |
| **损失除以 grad_accum** | Accelerate 内部处理 | 显式 `loss /= self.gradient_accumulation` | 当 `grad_accum=1` 时等价 |
| **Gradient checkpointing** | 由 `use_gradient_checkpointing` 标志控制 | `gradient_checkpointing_enable()` 设相同标志 | 数值等价（但影响内存 layout） |
| **RNG 播种** | `set_global_seed(seed)` 含 rank offset | `seed_everything(seed)` | 播种位置和 rank offset 方式不同 |
| **DataLoader** | `ResumableEpochSampler(seed)` | `DistributedSampler(shuffle=True)` | 样本顺序不同 |
| **Collation** | PyTorch 默认 stack | `fastwam_collate_fn` | 逻辑相同（都是 stack） |

### 2.3 单卡 `gradient_accumulation=1` 时可消除的差异

将两者都设为**单卡、单步、无累积、无分布式**后，差异收敛为：

1. FSDP `MixedPrecision` 的 cast vs `accelerator.autocast()`
2. `warmup_optimizer_state()` 的影响
3. `torch.compiler.cudagraph_mark_step_begin()` 的影响
4. RNG 播种方式

其中 (1) 可通过禁用 FSDP mixed_precision 消除，(2) 对第一步无影响（因为 warmup 只是初始化 state 为零），(3) 不影响数值，(4) 可手动同步 RNG 状态。

---

## 3. 测试层级总览

| 层级 | 名称 | 目的 | 硬件 | 容差 |
|------|------|------|------|------|
| **T0** | 可训参数集合 | 验证两套系统训练的参数集合完全一致 | CPU | 精确匹配 |
| **T1** | 单步 loss 对齐 | 同一 batch + 同一 RNG → loss 精确匹配 | 1 GPU | `rtol=1e-4, atol=1e-3` |
| **T2** | 多步梯度对齐 | N 步后权重 diff | 1 GPU | 逐步验证 |
| **T3** | Checkpoint 互换 | RLinf 权重加载到原生 → 推理一致 | 1 GPU | `allclose(rtol=1e-5)` |
| **T4** | 端到端 loss 曲线 | 独立跑 N 步，曲线漂移在可控范围 | 1 GPU | 相对差 < 5% |
| **T5** | 分布式一致性 | 多卡 RLinf vs 单卡 RLinf loss 等价 | 4 GPU | `allclose(rtol=1e-3)` |

---

## 4. T0 — 可训参数集合一致性

### 4.1 目的

验证 `FastWAMPolicy.train()` 后 `requires_grad=True` 的参数名集合，与原生 `_apply_dit_only_train_mode` + `dit.parameters() + proprio_encoder.parameters()` 的参数名集合**完全一致**。

### 4.2 方法

```python
def test_t0_trainable_params_match():
    """T0: 可训参数集合一致性"""
    # --- 原生路径 ---
    from fastwam.runtime import create_fastwam
    from fastwam.trainer import Wan22Trainer
    native_model = create_fastwam(...)  # 用测试 config
    Wan22Trainer._apply_dit_only_train_mode(native_model)
    native_trainable = set()
    for n, p in native_model.named_parameters():
        if p.requires_grad:
            native_trainable.add(n)

    # --- RLinf 路径 ---
    from rlinf.models.embodiment.fastwam import get_model
    policy = get_model(cfg, torch.bfloat16)
    policy.train()  # 调用 FastWAMPolicy.train()
    rlinf_trainable = set()
    for n, p in policy.named_parameters():
        if p.requires_grad:
            rlinf_trainable.add(n)

    # --- 对齐 ---
    # RLinf 的参数名有 "fastwam." 前缀（因为 policy.fastwam = model）
    rlinf_stripped = {n.replace("fastwam.", "", 1) for n in rlinf_trainable}

    assert native_trainable == rlinf_stripped, (
        f"原生独有: {native_trainable - rlinf_stripped}\n"
        f"RLinf 独有: {rlinf_stripped - native_trainable}"
    )
```

### 4.3 已知风险

- **`mot.py`（ModuleDict）vs `mot2.py`（_ExpertMixtures）**：当前生产用 `mot.py`，`dit = mot`，`dit.requires_grad_(True)` 会传播到 ModuleDict 内的 expert。RLinf Policy 的 `train()` 也用 `dit.requires_grad_(True)`，逻辑等价。若切到 `mot2.py`（`_ExpertMixtures`），则需改用注释中的显式 expert 解冻方式。
- **`_promote_scalar_params_to_1d`**：RLinf 会把 0-dim 参数升到 1D。需确认原生模型无 0-dim 参数（已验证 DiTBlock 无 scalar param）。

---

## 5. T1 — 单步 loss 精确对齐（核心测试）

### 5.1 目的

给定**完全相同的输入 batch** 和 **完全相同的 RNG 状态**，验证两条管道产出的 `loss_total`、`loss_video`、`loss_action` 精确匹配。

### 5.2 为什么要控制 RNG

`training_loss()` 内部有 4 个 RNG 依赖调用：

```python
noise_video  = torch.randn_like(input_latents)    # CUDA RNG
timestep_video = scheduler.sample_training_t(...)  # torch.rand → CUDA RNG
noise_action = torch.randn_like(action)            # CUDA RNG
timestep_action = scheduler.sample_training_t(...) # torch.rand → CUDA RNG
```

如果两条管道在调用 `training_loss` 前的 CUDA RNG 状态不同，噪声和时间步就会不同，loss 就不可比。

### 5.3 方法

```python
def test_t1_single_step_loss_alignment():
    """T1: 单步 loss 精确对齐"""
    device = torch.device("cuda:0")
    dtype = torch.bfloat16

    # ===== 1. 构造共享的 batch =====
    # 直接从数据集取一个真实 batch，或构造合成 batch
    batch = _make_deterministic_batch(device, dtype)

    # ===== 2. 创建原生模型 =====
    native_model = _create_native_model(device, dtype)
    Wan22Trainer._apply_dit_only_train_mode(native_model)

    # ===== 3. 创建 RLinf 模型（不 FSDP 包装，纯 Policy） =====
    policy = _create_rlinf_policy(device, dtype)
    policy.train()

    # ===== 4. 权重同步 =====
    # 用原生模型的 state_dict 覆盖 RLinf 模型，确保权重完全一致
    _sync_weights_native_to_rlinf(native_model, policy)

    # ===== 5. 控制 RNG =====
    rng_state = torch.cuda.get_rng_state(device)

    # ===== 6. 原生前向 =====
    torch.cuda.set_rng_state(rng_state, device)
    with torch.amp.autocast("cuda", dtype=dtype):
        loss_native, loss_dict_native = native_model.training_loss(batch)

    # ===== 7. RLinf 前向 =====
    torch.cuda.set_rng_state(rng_state, device)
    # 注意：sft_forward 调用 training_loss 前有一个
    # torch.compiler.cudagraph_mark_step_begin() — 不消耗 RNG
    output_rlinf = policy.sft_forward(data=batch)
    loss_rlinf = output_rlinf["loss"]

    # ===== 8. 比较 =====
    assert torch.allclose(loss_native, loss_rlinf, rtol=1e-4, atol=1e-3), (
        f"loss 不匹配: native={loss_native.item():.6f}, rlinf={loss_rlinf.item():.6f}"
    )
    assert torch.allclose(
        torch.tensor(loss_dict_native["loss_video"]),
        output_rlinf["dynamics_loss"],
        rtol=1e-4, atol=1e-3,
    )
    assert torch.allclose(
        torch.tensor(loss_dict_native["loss_action"]),
        output_rlinf["action_loss"],
        rtol=1e-4, atol=1e-3,
    )
```

### 5.4 辅助函数

```python
def _sync_weights_native_to_rlinf(native_model, policy):
    """将原生模型权重同步到 RLinf Policy"""
    native_sd = native_model.state_dict()
    policy_sd = policy.state_dict()
    mapped = {}
    for native_key, value in native_sd.items():
        rlinf_key = f"fastwam.{native_key}"
        if rlinf_key in policy_sd:
            mapped[rlinf_key] = value
    policy.load_state_dict(mapped, strict=False)

def _make_deterministic_batch(device, dtype):
    """生成确定性的合成 batch 或从数据集取第一个 batch"""
    B = 1
    return {
        "video": torch.randn(B, 3, 9, 384, 320, device=device, dtype=dtype),
        "action": torch.randn(B, 32, 23, device=device, dtype=dtype),
        "proprio": torch.randn(B, 32, 23, device=device, dtype=dtype),
        "context": torch.randn(B, 128, 4096, device=device, dtype=dtype),
        "context_mask": torch.ones(B, 128, device=device, dtype=torch.bool),
        "image_is_pad": torch.zeros(B, 9, device=device, dtype=torch.bool),
        "action_is_pad": torch.zeros(B, 32, device=device, dtype=torch.bool),
    }
```

### 5.5 FSDP 包装后的 T1 变体

上面的 T1 绕过了 FSDP 包装。增加一个变体，验证 FSDP(`no_shard`) 包装**不改变前向输出**：

```python
def test_t1b_fsdp_wrapped_loss_alignment():
    """T1b: FSDP 包装后 loss 仍然与未包装一致"""
    # 1. 创建未包装 policy，计算 loss
    # 2. 用 FSDP(no_shard, use_orig_params=True) 包装同一 policy
    # 3. 同一 batch + 同一 RNG → 比较 loss
    # FSDP no_shard 不分片，mixed_precision bf16 对已经是 bf16 的模型是 no-op
    # 预期差异 = 0
```

---

## 6. T2 — 多步梯度与权重对齐

### 6.1 目的

验证 N 步（建议 N=5）训练后，两条管道的**权重变化量**匹配。

### 6.2 方法

```python
def test_t2_multi_step_weight_alignment():
    """T2: 5 步训练后权重对齐"""
    N_STEPS = 5
    device = torch.device("cuda:0")
    dtype = torch.bfloat16
    lr = 1e-4

    # ===== 准备 =====
    batches = [_make_deterministic_batch(device, dtype, seed=42+i) for i in range(N_STEPS)]

    # ===== 原生路径 =====
    native_model = _create_native_model(device, dtype)
    Wan22Trainer._apply_dit_only_train_mode(native_model)
    native_optimizer = torch.optim.AdamW(
        list(native_model.dit.parameters()) + list(native_model.proprio_encoder.parameters()),
        lr=lr, betas=(0.9, 0.95), weight_decay=1e-2,
    )
    native_losses = []
    for step in range(N_STEPS):
        native_optimizer.zero_grad(set_to_none=True)
        rng_state = _fixed_rng_for_step(step)
        torch.cuda.set_rng_state(rng_state, device)
        with torch.amp.autocast("cuda", dtype=dtype):
            loss, _ = native_model.training_loss(batches[step])
        loss.backward()
        torch.nn.utils.clip_grad_norm_(native_model.parameters(), 1.0)
        native_optimizer.step()
        native_losses.append(loss.detach().item())

    # ===== RLinf 路径 =====
    policy = _create_rlinf_policy(device, dtype)
    policy.train()
    _sync_weights_from_native_checkpoint(native_init_sd, policy)
    rlinf_optimizer = torch.optim.AdamW(
        [p for p in policy.parameters() if p.requires_grad],
        lr=lr, betas=(0.9, 0.95), weight_decay=1e-2,
    )
    rlinf_losses = []
    for step in range(N_STEPS):
        rlinf_optimizer.zero_grad(set_to_none=True)
        torch.cuda.set_rng_state(_fixed_rng_for_step(step), device)
        output = policy.sft_forward(data=batches[step])
        loss = output["loss"]
        loss.backward()
        torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
        rlinf_optimizer.step()
        rlinf_losses.append(loss.detach().item())

    # ===== 比较 =====
    # 1. 每步 loss
    for step in range(N_STEPS):
        assert abs(native_losses[step] - rlinf_losses[step]) < 1e-3, (
            f"Step {step}: native={native_losses[step]:.6f}, rlinf={rlinf_losses[step]:.6f}"
        )

    # 2. 最终权重 diff
    native_final = native_model.state_dict()
    rlinf_final = policy.state_dict()
    max_diff = 0.0
    for native_key in native_final:
        rlinf_key = f"fastwam.{native_key}"
        if rlinf_key in rlinf_final and native_final[native_key].requires_grad:
            diff = (native_final[native_key].float() - rlinf_final[rlinf_key].float()).abs().max()
            max_diff = max(max_diff, diff.item())
    assert max_diff < 1e-3, f"最大权重差: {max_diff}"
```

### 6.3 关键约束

- 两路必须使用**完全相同的 optimizer 配置**（lr、betas、eps、weight_decay）
- 不使用 LR scheduler（或使用相同 scheduler state）
- 不使用梯度累积（`gradient_accumulation=1`）
- 不使用 FSDP 包装（纯 PyTorch，消除 mixed_precision cast 差异）

---

## 7. T3 — Checkpoint 权重互换

### 7.1 目的

验证 RLinf 训练产出的 checkpoint 可以无损加载到 FastWAM 原生模型中推理。

### 7.2 RLinf → 原生 checkpoint 转换

RLinf 提供 `fastwam_save_helper()`（[`rlinf/utils/ckpt_convertor/fsdp_convertor/utils.py:64-80`](../../rlinf/utils/ckpt_convertor/fsdp_convertor/utils.py)），将 FSDP state_dict 转为原生格式：

```
FSDP key:   fastwam.mot.mixtures.video.blocks.0.modulation
原生 key:   mixtures.video.blocks.0.modulation
```

转换逻辑：
- `fastwam.mot.{key}` → `{key}` 存入 `payload["mot"]`
- `fastwam.proprio_encoder.{key}` → `{key}` 存入 `payload["proprio_encoder"]`

### 7.3 方法

```python
def test_t3_checkpoint_interop():
    """T3: RLinf checkpoint → 原生模型加载 → 推理一致"""
    device = torch.device("cuda:0")
    dtype = torch.bfloat16
    batch = _make_deterministic_batch(device, dtype)

    # ===== 1. RLinf 训练 5 步并保存 =====
    policy = _create_rlinf_policy(device, dtype)
    policy.train()
    _train_n_steps(policy, batches=[batch]*5, lr=1e-4)
    rlinf_sd = policy.state_dict()

    # ===== 2. 转换为原生格式 =====
    mot_sd = {}
    pe_sd = {}
    for k, v in rlinf_sd.items():
        if k.startswith("fastwam.mot."):
            mot_sd[k.replace("fastwam.mot.", "")] = v
        elif k.startswith("fastwam.proprio_encoder."):
            pe_sd[k.replace("fastwam.proprio_encoder.", "")] = v

    # ===== 3. 原生模型加载 =====
    native_model = _create_native_model(device, dtype)
    native_model.mot.load_state_dict(mot_sd, strict=False)
    if pe_sd:
        native_model.proprio_encoder.load_state_dict(pe_sd, strict=False)

    # ===== 4. 对比 forward =====
    rng_state = torch.cuda.get_rng_state(device)

    torch.cuda.set_rng_state(rng_state, device)
    Wan22Trainer._apply_dit_only_train_mode(native_model)
    with torch.amp.autocast("cuda", dtype=dtype):
        loss_native, _ = native_model.training_loss(batch)

    torch.cuda.set_rng_state(rng_state, device)
    policy.eval()
    output = policy.sft_forward(data=batch)
    loss_rlinf = output["loss"]

    assert torch.allclose(loss_native, loss_rlinf, rtol=1e-5, atol=1e-5), (
        f"Checkpoint 互换后 loss 不一致: native={loss_native.item()}, rlinf={loss_rlinf.item()}"
    )
```

### 7.4 反向测试：原生 → RLinf

```python
def test_t3b_native_checkpoint_to_rlinf():
    """T3b: 原生 checkpoint → RLinf Policy 加载"""
    # 原生模型训练后保存 native_checkpoint.pt
    # RLinf policy.fastwam.load_checkpoint(path) 加载
    # 对同一 batch 比较 loss
```

---

## 8. T4 — 端到端多步 loss 曲线对齐

### 8.1 目的

在各自的完整管道下独立运行 N 步（建议 N=50），比较 loss 曲线。

### 8.2 方法

两条管道**独立运行**，使用相同的：
- seed
- 数据路径
- 超参数（lr、betas、weight_decay、clip_grad、scheduler）
- micro_batch_size=1, global_batch_size=1（单卡无累积）

但 **不** 同步 RNG（因为 DataLoader/sampler 实现不同，batch 顺序会不同）。

```bash
# === 原生 ===
cd ${FASTWAM_ROOT}
bash scripts/train_zero1.sh 1 \
  task=r1_pro_chassis_uncond_3cam_384_1e-4 \
  batch_size=1 \
  max_steps=50 \
  gradient_accumulation_steps=1 \
  seed=42

# === RLinf ===
cd ${REPO_ROOT}
bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  runner.max_steps=50 \
  actor.micro_batch_size=1 \
  actor.global_batch_size=1 \
  actor.seed=42 \
  cluster.component_placement.actor=0
```

### 8.3 判定标准

| 指标 | 通过标准 |
|------|----------|
| 初始 loss（step 0） | 两者 loss 量级相同（差异 < 10%） |
| loss 趋势 | 两者均下降且无 NaN/Inf |
| 收敛 loss（step 50） | 相对差 < 5%（`|L_native - L_rlinf| / max(|L_native|, |L_rlinf|) < 0.05`） |
| grad_norm 量级 | 同一数量级 |

> **注意**：由于 DataLoader 顺序不同，T4 不要求 loss 曲线点对点匹配，仅验证统计趋势一致性。T1/T2 才做精确匹配。

---

## 9. T5 — 分布式一致性

### 9.1 目的

验证 RLinf 4 卡训练与 1 卡训练的 loss 行为一致（排除 FSDP 在 `no_shard` 模式下的 all-reduce 引入的精度差异）。

### 9.2 方法

```bash
# === 单卡 ===
bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  runner.max_steps=20 \
  actor.micro_batch_size=1 \
  actor.global_batch_size=1 \
  cluster.component_placement.actor=0

# === 4 卡 ===
bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  runner.max_steps=20 \
  actor.micro_batch_size=1 \
  actor.global_batch_size=4 \
  cluster.component_placement.actor=4-7
```

### 9.3 判定标准

| 指标 | 通过标准 |
|------|----------|
| 初始 loss | 两者相同（共享预训练权重） |
| Step 20 loss | 量级相同，差异 < 5% |
| grad_norm | 同一数量级（4 卡有 all-reduce 平均） |
| 无 CUDA Error / Hang | 20 步完成无异常 |

---

## 10. 容差标准与判定规则

### 10.1 容差表

| 测试 | 比较对象 | rtol | atol | 说明 |
|------|----------|------|------|------|
| T0 | 参数名集合 | — | — | 精确集合匹配 |
| T1 | 单步 loss | 1e-4 | 1e-3 | bf16 精度下，loss 量级 ~0.1–10 |
| T1b | FSDP 包装前后 loss | 1e-5 | 1e-5 | `no_shard` 应该是 bit-exact |
| T2 | 多步 loss/权重 | 1e-3 | 1e-3 | 允许累积误差 |
| T3 | Checkpoint 互换后 loss | 1e-5 | 1e-5 | 权重完全一致时应 bit-exact |
| T4 | 端到端 loss 曲线 | 5% 相对差 | — | 统计趋势 |
| T5 | 单卡 vs 多卡 | 5% 相对差 | — | all-reduce 精度差 |

### 10.2 bf16 精度特性

bfloat16 尾数仅 7 位（vs float32 23 位），所以：
- 单步 loss 精度约 `~1e-3`
- 累积 N 步后精度下降至 `~N × 1e-3`
- 乘法/除法会引入 rounding 差异

### 10.3 判定流程

```
T0 FAIL → 停止，修复参数集合问题
T1 FAIL → 检查 RNG 同步、autocast 差异、模型权重同步
T2 FAIL → 检查 optimizer 配置、梯度裁剪实现差异
T3 FAIL → 检查 state_dict key 映射、dtype 转换
T4 大幅偏离 → 检查 DataLoader/sampler、LR scheduler
T5 FAIL → 检查 FSDP all-reduce、no_sync 逻辑
```

---

## 11. 影响对齐的因素与消除方法

### 11.1 RNG 同步

| 阶段 | RNG 调用 | 消除方法 |
|------|----------|----------|
| `training_loss` 内 `torch.randn_like` | CUDA RNG | `torch.cuda.set_rng_state(saved_state)` |
| `scheduler.sample_training_t` 内 `torch.rand` | CUDA RNG | 同上 |
| DataLoader worker | CPU RNG | 使用相同 seed + worker_init_fn |
| `torch.amp.autocast` | 无 RNG | — |

### 11.2 FSDP MixedPrecision

`FSDP(mixed_precision=MixedPrecision(param_dtype=bf16))` 的效果：
- 参数在 forward 前 cast 到 bf16（如果已经是 bf16 则无实际 cast）
- 梯度在 reduce 前 cast 到 bf16

**消除方法**（仅用于 T1/T2 精确测试）：
```yaml
mixed_precision:
  param_dtype: null
  reduce_dtype: null
  buffer_dtype: null
```

### 11.3 warmup_optimizer_state

RLinf 在 optimizer 创建后调用 `warmup_optimizer_state(optimizer)`，执行一个空的 `optimizer.step()` 以初始化 state dict（用于后续 checkpoint 兼容）。

**影响**：
- 消耗 `optimizer.state` 的 step counter（step=1）
- 但因为梯度为零，momentum 和 variance 均为零
- **对实际训练步骤的数值无影响**

### 11.4 `torch.compiler.cudagraph_mark_step_begin()`

`FastWAMPolicy.sft_forward` L26 调用此函数。这是一个编译器 hint，**不消耗 RNG 也不影响数值**。

### 11.5 梯度累积的 loss 缩放

- 原生：`accelerator.accumulate(model)` 内部处理 loss 缩放
- RLinf：显式 `loss = loss / self.gradient_accumulation`

**当 `gradient_accumulation=1` 时两者等价**。多步累积时需注意 Accelerate 的 loss 缩放行为。

---

## 12. 测试脚本实现指南

### 12.1 文件组织

```
tests/
├── unit_tests/
│   └── test_fastwam_alignment.py    # T0, T1, T1b
├── integration_tests/
│   └── test_fastwam_multi_step.py   # T2, T3
└── e2e_tests/
    └── test_fastwam_e2e_alignment.sh  # T4, T5 (shell 脚本 + Python 分析)
```

### 12.2 模型创建工厂

```python
# tests/conftest.py 或 test_fastwam_alignment.py 顶部

import os
import sys
import torch
from omegaconf import OmegaConf

# FastWAM 路径
FASTWAM_ROOT = os.environ.get("FASTWAM_ROOT", "/home/Luogang/SRC/Robot/FastWAM")
sys.path.insert(0, os.path.join(FASTWAM_ROOT, "src"))


def create_native_fastwam(device="cuda:0", dtype=torch.bfloat16):
    """创建原生 FastWAM 模型"""
    from fastwam.runtime import create_fastwam
    model = create_fastwam(
        model_id="Wan-AI/Wan2.2-TI2V-5B",
        tokenizer_model_id="Wan-AI/Wan2.1-T2V-1.3B",
        tokenizer_max_len=128,
        load_text_encoder=False,
        proprio_dim=23,
        video_dit_config={
            "has_image_input": False,
            "patch_size": [1, 2, 2],
            "in_dim": 48,
            "hidden_dim": 3072,
            "ffn_dim": 14336,
            "freq_dim": 256,
            "text_dim": 4096,
            "out_dim": 48,
            "num_heads": 24,
            "attn_head_dim": 128,
            "num_layers": 30,
            "eps": 1e-6,
            "seperated_timestep": True,
            "require_clip_embedding": False,
            "require_vae_embedding": False,
            "fuse_vae_embedding_in_latents": True,
            "use_gradient_checkpointing": False,
            "video_attention_mask_mode": "first_frame_causal",
            "action_conditioned": False,
            "action_dim": 23,
            "action_group_causal_mask_mode": "group_diagonal",
        },
        action_dit_config={
            "action_dim": 23,
            "hidden_dim": 1024,
            "ffn_dim": 4096,
            "num_heads": 24,
            "attn_head_dim": 128,
            "num_layers": 30,
            "text_dim": 4096,
            "freq_dim": 256,
            "eps": 1e-6,
            "use_gradient_checkpointing": False,
        },
        action_dit_pretrained_path=os.path.join(
            os.environ["DIFFSYNTH_MODEL_BASE_PATH"],
            "ActionDiT_linear_interp_Wan22_alphascale_1024hdim.pt",
        ),
        mot_checkpoint_mixed_attn=False,
        video_scheduler={"train_shift": 5.0, "infer_shift": 5.0, "num_train_timesteps": 1000},
        action_scheduler={"train_shift": 5.0, "infer_shift": 5.0, "num_train_timesteps": 1000},
        loss={"lambda_action": 1.0},
        device=str(device),
        model_dtype=dtype,
    )
    return model


def create_rlinf_policy(device="cuda:0", dtype=torch.bfloat16):
    """创建 RLinf FastWAMPolicy（不含 FSDP 包装）"""
    from rlinf.models.embodiment.fastwam import get_model
    from rlinf.models.embodiment.fastwam.fastwam_config import FastWAMConfig

    cfg = OmegaConf.create({
        "model_type": "fastwam",
        "is_lora": False,
        "model_id": "Wan-AI/Wan2.2-TI2V-5B",
        "tokenizer_model_id": "Wan-AI/Wan2.1-T2V-1.3B",
        "tokenizer_max_len": 128,
        "load_text_encoder": False,
        "redirect_common_files": True,
        "mot_checkpoint_mixed_attn": False,
        "action_dit_pretrained_path": os.path.join(
            os.environ["DIFFSYNTH_MODEL_BASE_PATH"],
            "ActionDiT_linear_interp_Wan22_alphascale_1024hdim.pt",
        ),
        "proprio_dim": 23,
        "video_dit_config": {
            "has_image_input": False, "patch_size": [1, 2, 2], "in_dim": 48,
            "hidden_dim": 3072, "ffn_dim": 14336, "freq_dim": 256, "text_dim": 4096,
            "out_dim": 48, "num_heads": 24, "attn_head_dim": 128, "num_layers": 30,
            "eps": 1e-6, "seperated_timestep": True, "require_clip_embedding": False,
            "require_vae_embedding": False, "fuse_vae_embedding_in_latents": True,
            "use_gradient_checkpointing": False, "video_attention_mask_mode": "first_frame_causal",
            "action_conditioned": False, "action_dim": 23,
            "action_group_causal_mask_mode": "group_diagonal",
        },
        "action_dit_config": {
            "action_dim": 23, "hidden_dim": 1024, "ffn_dim": 4096,
            "num_heads": 24, "attn_head_dim": 128, "num_layers": 30,
            "text_dim": 4096, "freq_dim": 256, "eps": 1e-6, "use_gradient_checkpointing": False,
        },
        "video_scheduler": {"train_shift": 5.0, "infer_shift": 5.0, "num_train_timesteps": 1000},
        "action_scheduler": {"train_shift": 5.0, "infer_shift": 5.0, "num_train_timesteps": 1000},
        "loss": {"lambda_action": 1.0},
    })
    policy = get_model(cfg, dtype)
    policy = policy.to(device)
    return policy
```

### 12.3 合成 batch 工厂

```python
def make_synthetic_batch(
    batch_size=1,
    num_video_frames=9,
    video_h=384,
    video_w=320,
    action_horizon=32,
    action_dim=23,
    context_len=128,
    context_dim=4096,
    device="cuda:0",
    dtype=torch.bfloat16,
    seed=42,
):
    """创建确定性合成 batch（用于不依赖 VAE/数据集的测试）"""
    gen = torch.Generator(device=device)
    gen.manual_seed(seed)

    return {
        "video": torch.randn(
            batch_size, 3, num_video_frames, video_h, video_w,
            device=device, dtype=dtype, generator=gen,
        ),
        "action": torch.randn(
            batch_size, action_horizon, action_dim,
            device=device, dtype=dtype, generator=gen,
        ),
        "proprio": torch.randn(
            batch_size, action_horizon, action_dim,
            device=device, dtype=dtype, generator=gen,
        ),
        "context": torch.randn(
            batch_size, context_len, context_dim,
            device=device, dtype=dtype, generator=gen,
        ),
        "context_mask": torch.ones(
            batch_size, context_len, device=device, dtype=torch.bool,
        ),
        "image_is_pad": torch.zeros(
            batch_size, num_video_frames, device=device, dtype=torch.bool,
        ),
        "action_is_pad": torch.zeros(
            batch_size, action_horizon, device=device, dtype=torch.bool,
        ),
    }
```

> **注意**：合成 batch 中的 `video` 字段是**原始像素**（3-channel `[B,3,T,H,W]`），`build_inputs` 会经过 VAE 编码。如果想跳过 VAE 以减少差异来源，可以直接构造 `input_latents`（`[B,48,3,24,20]`）并 monkey-patch `build_inputs` 或调用 `training_loss` 的内部子步骤。

### 12.4 pytest markers

```python
import pytest

requires_gpu = pytest.mark.skipif(
    not torch.cuda.is_available(),
    reason="需要 GPU",
)
requires_fastwam_weights = pytest.mark.skipif(
    not os.path.exists(os.environ.get("DIFFSYNTH_MODEL_BASE_PATH", "")),
    reason="需要 FastWAM 预训练权重",
)
requires_r1pro_data = pytest.mark.skipif(
    not os.path.exists(os.environ.get("R1PRO_DATA", "")),
    reason="需要 r1_pro 数据集",
)
```

---

## 13. 检查清单

### 13.1 实施优先级

| 优先级 | 测试 | 阻塞条件 |
|--------|------|----------|
| **P0** | T0 可训参数集合 | 无（CPU 可运行） |
| **P0** | T1 单步 loss | 需 1 GPU + 预训练权重 |
| **P1** | T1b FSDP 包装后 loss | 需 1 GPU + 预训练权重 |
| **P1** | T2 多步权重对齐 | 需 1 GPU |
| **P1** | T3 Checkpoint 互换 | 需 1 GPU |
| **P2** | T4 端到端 loss 曲线 | 需 1 GPU + r1_pro 数据 |
| **P2** | T5 分布式一致性 | 需 4 GPU + r1_pro 数据 |

### 13.2 快速验证命令

```bash
# T0 + T1（单 GPU，约 5 分钟）
pytest tests/unit_tests/test_fastwam_alignment.py -v -k "t0 or t1" --tb=short

# T2 + T3（单 GPU，约 10 分钟）
pytest tests/integration_tests/test_fastwam_multi_step.py -v --tb=short

# T4（单 GPU，约 30 分钟）
bash tests/e2e_tests/test_fastwam_e2e_alignment.sh
```

### 13.3 CI 集成建议

- T0 纳入 CI（无 GPU 需求）
- T1/T1b 纳入 GPU CI（有预训练权重时）
- T2/T3 作为 release gate
- T4/T5 仅手动或 nightly

---

**文档版本**：v1 · 2026-06-02 · 基于 cp25_2 代码基线。
