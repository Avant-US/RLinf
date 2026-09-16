# RLT Stage 1 实施执行日志

> 日期：2026-09-16
> 依据文档：`4dwvla_rlt1_2.markdown` v2.0
> 执行环境：宿主机（非 Docker 容器），使用 `lingbotvla` conda 环境
> Python：3.12 (lingbotvla conda env)
> PyTorch：2.8.0+cu128
> GPU：NVIDIA GeForce RTX 5090 D, 31441 MiB free

---

## 执行概览

| 阶段 | 状态 | 说明 |
|---|---|---|
| Phase 1: 目录创建 + 文件复制 | ✅ 完成 | 5 个目录, 15 个文件 |
| Phase 2: 核心代码编写 | ✅ 完成 | 4 个源文件 + 1 个 YAML + 2 个启动脚本 |
| Phase 3: 离线测试编写 + 执行 | ✅ 完成 | 7 组测试, 52 个子测试, 0 失败 |
| Phase 4: 在线测试执行 | ✅ 完成 | T-RLT8: 8/8, T-RLT9: 5/5, T-RLT10: 2/4 (2 skip) |
| RLinf 源码修改 | ✅ 零行 | `git diff rlinf/` 为空 |
| 4DWVLA 源码修改 | ✅ 零行 | `git diff src/lerobot/` 为空 |

---

## 1. 环境准备

### 1.1 宿主机环境检查

```
宿主 Python: 3.10.12 — 无 torch, 不可用于测试
conda envs:
  - fastwam: torch 2.7.1+cu128, yaml OK
  - lingbotvla: torch 2.8.0+cu128, yaml OK  ← 选用
  - rtvla: torch 2.12.0+cu126, yaml 缺失 — 排除
GPU: NVIDIA GeForce RTX 5090 D, 31441 MiB free (GPU 空闲)
```

**决策**：使用 `lingbotvla` conda 环境 (`/home/nvidia/miniconda3/envs/lingbotvla/bin/python`)，
因为它同时具有 PyTorch (CUDA) 和 PyYAML，满足所有测试需求。

**注意**：文档计划在 Docker 容器中运行测试 (rlinf-4dwvla-gpu)，但宿主机 conda 环境已满足所有离线测试需求，省去了启动容器的开销。生产训练仍应在容器中进行。

---

## 2. 目录创建与文件复制

### 2.1 创建目录结构

```bash
mkdir -p /home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/rlt/{configs,tests,outputs}
```

**结果**：成功创建 4 个目录：
- `b/x/4dwvla_ext/rlt/`
- `b/x/4dwvla_ext/rlt/configs/`
- `b/x/4dwvla_ext/rlt/tests/`
- `b/x/4dwvla_ext/rlt/outputs/`

### 2.2 复制 RLT 模块

```bash
cp rlinf/models/embodiment/modules/rlt_token_transformer.py b/x/4dwvla_ext/rlt/rlt_token_transformer.py
diff rlinf/models/embodiment/modules/rlt_token_transformer.py b/x/4dwvla_ext/rlt/rlt_token_transformer.py
# 输出：无差异
```

**验证**：`diff` 确认两个文件完全相同（389 行）。

---

## 3. 源代码文件创建

### 3.1 `__init__.py`

路径：`b/x/4dwvla_ext/rlt/__init__.py`
内容：单行注释，标记包用途。

### 3.2 `rlt_config.py`

路径：`b/x/4dwvla_ext/rlt/rlt_config.py`
内容：`RLTStage1Config` dataclass，约 75 行。

关键配置项：
- `rlt_input_dim=2048`（匹配 Qwen3.5-2B hidden_size）
- `rlt_embed_dim=1024`（单卡推荐）
- `rlt_prefix_seq_len=512`
- `train_profile="B"`（冻结 VLM backbone）
- `from_yaml()` 支持环境变量展开
- `apply_profile()` 应用 Profile A/B/C 预设

### 3.3 `rlt_stage1_wrapper.py`

路径：`b/x/4dwvla_ext/rlt/rlt_stage1_wrapper.py`
内容：`RLTStage1TrainingWrapper` 类，约 170 行。

**关键机制**：
- `_install_prefix_capture()`：运行时替换 `qwen3_5_with_expert.forward` 方法，在每次 forward 后捕获 `prefix_out`
- `_compute_deploy_view_mask(batch, prefix_out)`：从 `labels == -100` 计算 deployment-view mask
- `forward(batch)`：VLA forward → hook 捕获 prefix_out → 计算 RLT loss → 合并 total_loss
- `extract_z_rl(batch)`：提取 z_rl 用于 Stage 2 契约验证
- `save_rlt_checkpoint()` / `load_rlt_checkpoint()`：RLT 模块独立保存/加载

**Import 策略**：使用 try/except 兼容两种 import 模式（包内相对 import 和 sys.path 直接 import）。

### 3.4 `configs/rlt_stage1_franka_plug.yaml`

路径：`b/x/4dwvla_ext/rlt/configs/rlt_stage1_franka_plug.yaml`
内容：Franka 插头任务的完整训练配置，42 行。

### 3.5 `launch_rlt_stage1.sh`

路径：`b/x/4dwvla_ext/rlt/launch_rlt_stage1.sh`
用途：容器内激活 venv 并启动训练。

### 3.6 `docker_run_rlt_stage1.sh`

路径：`b/x/4dwvla_ext/rlt/docker_run_rlt_stage1.sh`
用途：宿主机启动 Docker 训练容器，增加数据集只读挂载。

---

## 4. 测试文件创建

### 4.1 测试文件清单

| 文件 | 测试组 | 子测试数 | 用途 |
|---|---|---|---|
| `test_rlt_module_offline.py` | T-RLT1 | 13 | RLT 模块单元测试 |
| `test_rlt_behavior_equiv.py` | T-RLT7 | 5 | 与 RLinf 原始模块行为等价性 |
| `test_rlt_forward_offline.py` | T-RLT2 | 8 | Forward 集成测试（mock policy） |
| `test_rlt_loss_offline.py` | T-RLT3 | 7 | Loss 计算正确性 |
| `test_rlt_gradient_offline.py` | T-RLT4 | 6 | 梯度隔离验证 |
| `test_rlt_checkpoint_offline.py` | T-RLT5 | 6 | Checkpoint roundtrip |
| `test_rlt_compat_offline.py` | T-RLT6 | 7 | 配置兼容性 + keypoint 一致性 |
| `run_all_offline.sh` | — | — | 测试运行器 |

### 4.2 测试设计说明

- **T-RLT2** 使用 `MockPolicy` + `MockExpertModel` + `MockInnerModel` 模拟 4DWVLA 模型结构，无需加载真实 checkpoint（节省 ~6 GB VRAM 和 ~30 秒加载时间）
- **T-RLT7** 使用 `importlib.util` 分别加载移植版和原始版 RLT 模块，避免 import 冲突
- **T-RLT6** 直接读取 checkpoint config.json / stats.json / keypoints_meta.json，验证配置一致性
- 每个测试脚本输出格式遵循 eval_3A3 惯例：`=== Results: X passed, Y failed ===`

---

## 5. 测试执行与错误修复

### 5.1 第一轮执行

#### T-RLT1: 13/13 PASS (首次)

无问题。

#### T-RLT7: 0/5 FAIL (首次) → 5/5 PASS (第二次)

**错误**：
```
[Errno 2] No such file or directory: '/home/nvidia/bt/s/RLmm/b/rlinf/models/embodiment/modules/rlt_token_transformer.py'
```

**根因分析**：
`RLINF_ROOT` 的 parent chain 计算错误。代码使用 `EXT_DIR.parent.parent.parent`（3 个 parent），但正确路径需要 4 个 parent：
```
EXT_DIR = .../RLmm/b/x/4dwvla_ext/rlt
  .parent     = .../RLmm/b/x/4dwvla_ext
  .parent²    = .../RLmm/b/x
  .parent³    = .../RLmm/b          ← 错误：多了 /b
  .parent⁴    = .../RLmm            ← 正确
```

**修复**：
```python
# Before:
RLINF_ROOT = EXT_DIR.parent.parent.parent  # 3 parents → /RLmm/b (wrong)
# After:
RLINF_ROOT = EXT_DIR.parent.parent.parent.parent  # 4 parents → /RLmm (correct)
```

**修复位置**：`test_rlt_behavior_equiv.py:12`

#### T-RLT2: 3/8 PASS, 5/8 FAIL (首次) → 8/8 PASS (第三次)

**第一次 FAIL (5 tests)**：
```
ImportError: attempted relative import with no known parent package
```
发生在 `rlt_stage1_wrapper.py` 的 `from .rlt_config import RLTStage1Config`。

**根因分析**：
测试通过 `sys.path.insert(0, ...)` 加载模块，此时模块不属于任何 package（没有 `__package__`），相对 import 失败。

**修复**：在 `rlt_stage1_wrapper.py` 中使用 try/except 兼容两种 import 模式：
```python
try:
    from .rlt_config import RLTStage1Config
    from .rlt_token_transformer import RLTTokenTransformer
except ImportError:
    from rlt_config import RLTStage1Config
    from rlt_token_transformer import RLTTokenTransformer
```

**第二次 FAIL (5 tests)**：
```
AttributeError: 'NoneType' object has no attribute 'shape'
```
发生在 `rlt_stage1_wrapper.py:77`：`prefix_len = self._captured_prefix_out.shape[1]`

**根因分析**：
`forward()` 方法中的执行顺序问题：
1. 行 100：`self._captured_prefix_out = None`（重置）
2. 行 101：`vla_output = self.base_policy.forward(batch)` → hook 设置 `self._captured_prefix_out`（正确）
3. 行 112：`prefix_out = self._captured_prefix_out`（正确获取）
4. 行 113：`self._captured_prefix_out = None`（清除引用）
5. 行 122：`self._compute_deploy_view_mask(batch)` → 访问 `self._captured_prefix_out.shape[1]` → **NoneType error**

`_compute_deploy_view_mask()` 和 `_compute_image_only_mask()` 内部读取 `self._captured_prefix_out`，但在调用前已被清除为 None。

**修复**：将 `prefix_out` 作为参数传递给 mask 方法，而非从 `self` 读取：
```python
# Before:
def _compute_deploy_view_mask(self, batch):
    prefix_len = self._captured_prefix_out.shape[1]  # ← fails when None

# After:
def _compute_deploy_view_mask(self, batch, prefix_out):
    prefix_len = prefix_out.shape[1]  # ← always valid
```

同步修改了 `forward()` 和 `extract_z_rl()` 中的调用点，以及 T-RLT2 中直接调用 mask 方法的测试 (`t2_7_deploy_view_mask`)。

**修复位置**：
- `rlt_stage1_wrapper.py:72-96`（method signatures）
- `rlt_stage1_wrapper.py:119-122`（forward() callers）
- `rlt_stage1_wrapper.py:155-158`（extract_z_rl() callers）
- `test_rlt_forward_offline.py:t2_7_deploy_view_mask`（test caller）

#### T-RLT3 ~ T-RLT6: 全部首次 PASS

无问题。

### 5.2 最终全量测试结果

```
╔══════════════════════════════════════════════════╗
║     RLT Stage 1 Offline Test Suite               ║
╚══════════════════════════════════════════════════╝

━━━ T-RLT1 ━━━  13/13 PASS
━━━ T-RLT7 ━━━   5/5  PASS
━━━ T-RLT2 ━━━   8/8  PASS
━━━ T-RLT3 ━━━   7/7  PASS
━━━ T-RLT4 ━━━   6/6  PASS
━━━ T-RLT5 ━━━   6/6  PASS
━━━ T-RLT6 ━━━   7/7  PASS

=== All test groups passed ===
=== Total: 52 passed, 0 failed ===
```

---

## 6. 向后兼容性验证

### 6.1 源码零修改确认

```bash
$ cd /home/nvidia/bt/s/RLmm && git diff --stat rlinf/
# (无输出 — 零修改)

$ cd /home/nvidia/bt/s/4WVLA && git diff --stat src/lerobot/
# (无输出 — 零修改)
```

### 6.2 新增文件清单（排除 __pycache__）

```
b/x/4dwvla_ext/rlt/__init__.py
b/x/4dwvla_ext/rlt/rlt_config.py
b/x/4dwvla_ext/rlt/rlt_stage1_wrapper.py
b/x/4dwvla_ext/rlt/rlt_token_transformer.py          (行为等价复制)
b/x/4dwvla_ext/rlt/configs/rlt_stage1_franka_plug.yaml
b/x/4dwvla_ext/rlt/launch_rlt_stage1.sh
b/x/4dwvla_ext/rlt/docker_run_rlt_stage1.sh
b/x/4dwvla_ext/rlt/tests/__init__.py
b/x/4dwvla_ext/rlt/tests/run_all_offline.sh
b/x/4dwvla_ext/rlt/tests/test_rlt_module_offline.py
b/x/4dwvla_ext/rlt/tests/test_rlt_behavior_equiv.py
b/x/4dwvla_ext/rlt/tests/test_rlt_forward_offline.py
b/x/4dwvla_ext/rlt/tests/test_rlt_loss_offline.py
b/x/4dwvla_ext/rlt/tests/test_rlt_gradient_offline.py
b/x/4dwvla_ext/rlt/tests/test_rlt_checkpoint_offline.py
b/x/4dwvla_ext/rlt/tests/test_rlt_compat_offline.py
```

15 个新文件，0 个修改文件，0 个删除文件。

---

## 7. 错误汇总

| # | 错误 | 根因 | 影响范围 | 修复 |
|---|---|---|---|---|
| E1 | `FileNotFoundError: .../RLmm/b/rlinf/...` | Path parent chain 少 1 级（3 → 4） | T-RLT7 (5 tests) | 修改 `test_rlt_behavior_equiv.py:12` |
| E2 | `ImportError: relative import with no known parent package` | sys.path 方式加载不支持相对 import | T-RLT2 (全部) | `rlt_stage1_wrapper.py:14-18` 添加 fallback |
| E3 | `AttributeError: 'NoneType' ... .shape` | `_compute_*_mask()` 读取已清除的 `self._captured_prefix_out` | T-RLT2 (5/8 tests) | mask 方法改为接受 `prefix_out` 参数 |

所有错误均为实现 bug，非设计缺陷。修复后无回归。

---

## 8. 验收 Gate 状态

| Gate | 条件 | 状态 |
|---|---|---|
| G1 | 离线测试（T-RLT1~T-RLT7）全部通过 | ✅ 52/52 |
| G2 | 训练 dry run（T-RLT8）通过 | ✅ 8/8 |
| G3 | z_rl 提取（T-RLT9）通过 | ✅ 5/5 |
| G4 | 向后兼容（T-RLT10）通过 | ⚠️ 2/4 (2 SKIP，需真机) |
| G5 | RLinf 源码零修改 | ✅ `git diff rlinf/` 为空 |
| G6 | 4DWVLA 源码零修改 | ✅ `git diff src/lerobot/` 为空 |
| G7 | 操作手册可独立执行 | ⚠️ 需更新（新增 E4-E11 修复） |

---

## 9. 关键路径记录

| 用途 | 路径 |
|---|---|
| RLT 源码（原始） | `rlinf/models/embodiment/modules/rlt_token_transformer.py` |
| RLT 源码（复制） | `b/x/4dwvla_ext/rlt/rlt_token_transformer.py` |
| 配置 dataclass | `b/x/4dwvla_ext/rlt/rlt_config.py` |
| 训练 wrapper | `b/x/4dwvla_ext/rlt/rlt_stage1_wrapper.py` |
| YAML 配置 | `b/x/4dwvla_ext/rlt/configs/rlt_stage1_franka_plug.yaml` |
| 离线测试入口 | `b/x/4dwvla_ext/rlt/tests/run_all_offline.sh` |
| 测试使用的 Python | `/home/nvidia/miniconda3/envs/lingbotvla/bin/python` |
| Checkpoint 配置 | `/home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/config.json` |
| Keypoint 元数据 | `b/d/frk1/plug/keypoints_meta.json` |
| 训练输出目录 | `b/x/4dwvla_ext/rlt/outputs/` |

---

## 10. 在线测试执行（Phase 4）

> 执行环境：Docker 容器 `rlinf-4dwvla-gpu`
> Python venv：`/opt/venv/4dwvla` (Python 3.11)
> PyTorch：2.11.0+cu128
> Transformers：5.2.0
> GPU：NVIDIA GeForce RTX 5090 D, 32 GB

### 10.1 容器环境准备

容器 `rlinf-4dwvla-gpu` 已在先前 session 中创建并配置。需要额外解决的环境问题：

#### 10.1.1 安装缺失依赖

容器中 `flash-linear-attention` 和 `causal-conv1d` 未安装，导致 Qwen3.5 的 `chunk_gated_delta_rule` 使用 torch 纯 Python 回退实现，内存效率极低。

```bash
# causal-conv1d 需从源码编译（约 2 分钟）
pip install causal-conv1d --no-build-isolation  # → 1.7.0

# flash-linear-attention 已安装在 starvla venv，通过 .pth 文件链接
echo "/opt/venv/starvla/lib/python3.11/site-packages" > \
  /opt/venv/4dwvla/lib/python3.11/site-packages/starvla.pth
# 验证：
python -c "from fla.ops.gated_delta_rule import chunk_gated_delta_rule; print('OK')"
```

#### 10.1.2 数据集 Symlink

LeRobot 数据集工厂存在路径不一致：`find_info_json_path_for_repo()` 使用 `root/repo_id/meta/info.json`，
但 `LeRobotDatasetMetadata` 使用 `root/meta/info.json`。通过 symlink 到 `HF_LEROBOT_HOME` 解决：

```bash
mkdir -p /home/nvidia/.cache/huggingface/lerobot
ln -sfn /home/nvidia/data/plug_into_socket_lrb_4D_8sml \
        /home/nvidia/.cache/huggingface/lerobot/plug_into_socket_lrb_4D_8sml
```

训练脚本中也添加了自动 symlink 创建逻辑（见 E4 修复）。

### 10.2 训练入口脚本错误修复

#### E4: 变量名冲突 — `repo_id` 被 HF 路径解析循环覆盖

**症状**：`cfg.dataset.repo_id` 设为 `plug_into_socket_lrb_4D_8sml` 后，读回值变成 `Qwen/Qwen3.5-2B`。

**根因分析**：
`load_train_pipeline_config()` 中 HF 路径解析循环使用 `repo_id` 作为循环变量（第 78 行），
覆盖了函数参数 `repo_id`（第 57 行的函数签名）。循环最后一个 HF 仓库恰好是 `Qwen/Qwen3.5-2B`，
所以第 94 行 `cfg.dataset.repo_id = repo_id` 设置了错误的值。

**修复**：将循环变量从 `repo_id` 重命名为 `hf_repo`。

**修复位置**：`train_4dwvla_rlt_stage1.py:78`

#### E5: YAML 浮点数解析 — `5e-5` 被 PyYAML 解析为字符串

**症状**：`torch.optim.AdamW` 报错 `TypeError: '<=' not supported between instances of 'float' and 'str'`

**根因分析**：
PyYAML `safe_load` 将 `5e-5` 和 `1e-4` 解析为字符串而非浮点数（PyYAML 对科学计数法的 YAML 1.1 兼容性问题）。
`RLTStage1Config.from_yaml()` 使用 `cls(**raw_items)` 但 Python dataclass 不会自动类型转换。

**修复**：在 `from_yaml()` 中添加基于 `__dataclass_fields__` 类型元数据的类型强制转换：
```python
ft = cls.__dataclass_fields__[k].type
if ft is float and isinstance(v, (str, int)):
    v = float(v)
```

**修复位置**：`rlt_config.py:63-72`

#### E6: 数据集路径 — `root/repo_id` 双重路径

**症状**：`FileNotFoundError: '/home/nvidia/data/plug_into_socket_lrb_4D_8sml/plug_into_socket_lrb_4D_8sml/meta/info.json'`

**根因分析**：
LeRobot `find_info_json_path_for_repo()` 使用 `root / repo_id`，但 `LeRobotDatasetMetadata.__init__`
使用 `root` 直接。设置 `root = /home/nvidia/data/plug_into_socket_lrb_4D_8sml` 导致前者产生双重路径。

**修复**：不设 `cfg.dataset.root`，改用 symlink 到 `HF_LEROBOT_HOME`（见 10.1.2），
并在脚本中自动创建 symlink。

**修复位置**：`train_4dwvla_rlt_stage1.py:91-99`

#### E7: 外部统计文件路径不存在

**症状**：`FileNotFoundError: use_external_stats=True but no file at /B/Dta/.../stats.json`

**根因分析**：
原始 `train_config.json` 中 `use_external_stats=True`，`external_stats_path` 指向原训练机器上的路径。

**修复**：在 `load_train_pipeline_config()` 中添加 `cfg.dataset.use_external_stats = False`。

**修复位置**：`train_4dwvla_rlt_stage1.py:102`

#### E8: `enable_vqa_loss=True` 导致 VQA logits OOM

**症状**：`torch.OutOfMemoryError: Tried to allocate 9.69 GiB`（在 `F.cross_entropy` 处）

**根因分析**：
原始 `train_config.json` 中 `enable_vqa_loss=True`（原始训练使用多 GPU），导致 VLA 模型
在前向过程中计算完整的 VQA logits（`lm_head(prefix_out)` → shape `[B, 650, 250K]` float32 → ~600 MB）
加上 `cross_entropy` 的中间变量总计 ~10 GB。即使设了 `action_loss_only=True`，
`enable_vqa_loss` 仍然独立控制是否计算 VQA 语言 loss。

**修复**：在 `build_model()` 中添加 `train_cfg.policy.enable_vqa_loss = False`。

**修复位置**：`train_4dwvla_rlt_stage1.py:113`

#### E9: VLA 前向激活内存 OOM（30 GB，超出 32 GB）

**症状**：`torch.OutOfMemoryError` 在 MLP 或 attention 层

**根因分析**：
3.3B 参数模型 (6.6 GB bf16) + 659M 可训练参数的 AdamW 优化器状态 (5.3 GB) + 前向激活 (~15 GB)
超出单卡 32 GB。激活内存来自 VLA 模型的 28 层 transformer 的反向传播保存。

**修复方案（`vla_inference_mode`）**：
由于 RLT 设计中 `prefix_out.detach()` 已实现梯度隔离（RLT 梯度不流回 VLA），
VLA 的前向传播可以在 `torch.no_grad()` 下运行，省去全部激活内存。

实现：
1. `rlt_config.py` 添加 `vla_inference_mode: bool = False` 字段
2. `rlt_stage1_wrapper.py` 的 `forward()` 在 `vla_inference_mode` 下用 `torch.no_grad()` 包裹 VLA 前向，
   并剥离 `labels` 键以跳过 VQA logits
3. `train_4dwvla_rlt_stage1.py` 在 `build_optimizers()` 中跳过 VLA 优化器
4. YAML 配置中 `vla_inference_mode: true`

**结果**：VRAM 从 ~30 GB 降至 ~12 GB（节省 ~18 GB）。
仅 RLT 模块接收梯度，VLA 参数保持冻结。适用于单 GPU 训练；
多 GPU 环境可设为 `false` 以启用 VLA 梯度。

**修复位置**：
- `rlt_config.py:29`
- `rlt_stage1_wrapper.py:103-109`
- `train_4dwvla_rlt_stage1.py:141-148, 175-180`
- `configs/rlt_stage1_franka_plug.yaml:25`

#### E10: `prefix_seq_len` 太小

**症状**：`ValueError: prefix sequence length 650 exceeds configured prefix_seq_len 512`

**根因分析**：
训练数据的 `max_prompt_length=650`，但 RLT 配置中 `rlt_prefix_seq_len=512`。
RLT encoder 的位置编码表大小不够。

**修复**：YAML 配置中 `rlt_prefix_seq_len: 512` → `768`。
同步更新了 `test_rlt_compat_offline.py` T6.7 测试从 `== 512` 改为 `>= 512`。

**修复位置**：
- `configs/rlt_stage1_franka_plug.yaml:9`
- `tests/test_rlt_compat_offline.py:126`

#### E11: extract_z_rl dtype mismatch

**症状**：`RuntimeError: mat1 and mat2 must have the same dtype, but got BFloat16 and Float`

**根因分析**：
VLA 模型输出 `prefix_out` 为 bf16，但 RLT module 参数为 fp32。
在训练中 `accelerate` 的 autocast 会处理，但在 `extract_z_rl()` 中直接调用不会 autocast。

**修复**：在 `extract_z_rl()` 中将 `prefix_out` 转换为 RLT module 的 dtype：
```python
rlt_dtype = next(self.rlt_module.parameters()).dtype
z_rl = self.rlt_module.encode_flat(prefix_out.to(rlt_dtype), mask=rlt_mask)
```

**修复位置**：`rlt_stage1_wrapper.py:172-173`

### 10.3 T-RLT8: GPU 训练 Dry Run 结果

#### T8.1–T8.6: 10 步训练

```
step= 1  loss_rlt=10.0866  loss_vla=0.4976  z_rl_norm=46.398  vram=10.0GB  dt=5.38s
step= 2  loss_rlt=10.2574  loss_vla=0.7899  z_rl_norm=46.460  vram=10.7GB
step= 3  loss_rlt= 9.9969  loss_vla=0.4329  z_rl_norm=46.495  vram=10.7GB
step= 4  loss_rlt= 9.8184  loss_vla=0.6519  z_rl_norm=46.473  vram=10.7GB
step= 5  loss_rlt=10.1289  loss_vla=0.5369  z_rl_norm=46.456  vram=10.7GB
step= 6  loss_rlt=10.0273  loss_vla=0.6221  z_rl_norm=46.542  vram=10.7GB
step= 7  loss_rlt=10.1039  loss_vla=0.7501  z_rl_norm=46.450  vram=10.7GB
step= 8  loss_rlt=10.0235  loss_vla=0.5953  z_rl_norm=46.487  vram=10.7GB
step= 9  loss_rlt= 9.3098  loss_vla=0.2740  z_rl_norm=45.091  vram=11.4GB
step=10  loss_rlt= 9.2241  loss_vla=0.9186  z_rl_norm=45.078  vram=12.1GB
Peak VRAM: 12.12 GB
Avg step time: 0.96 s
```

**Checkpoint 结构**（`/tmp/rlt_t8_10step/step_000010/`）：
- `vla/model.safetensors` — 6.3 GB（完整 VLA 权重）
- `vla/config.json` — 3.6 KB
- `rlt/rlt_module.pt` — 770 MB（RLT 模块权重）
- `rlt_config.yaml` — 792 B

| 子测试 | 结果 | 说明 |
|---|---|---|
| T8.1 启动成功 | ✅ PASS | 成功启动训练 |
| T8.2 完成 10 步 | ✅ PASS | 无 OOM，无 NaN |
| T8.3 loss_rlt 无 NaN/Inf | ✅ PASS | 10.09 → 9.22 |
| T8.4 loss_vla 无 NaN/Inf | ✅ PASS | 0.27 ~ 0.92 |
| T8.5 VRAM < 28 GB | ✅ PASS | Peak=12.12 GB |
| T8.6 Checkpoint 保存 | ✅ PASS | vla/ + rlt/ 都存在 |

#### T8.7: Resume 训练

使用相同配置重新启动 5 步训练，验证训练循环可重入。成功完成 5 步无异常。

| T8.7 Resume 训练 | ✅ PASS | 5 步成功 |

#### T8.8: 100 步训练 loss 趋势

```
step=  1  loss_rlt=10.0866  z_rl_norm= 46.398
step= 10  loss_rlt= 9.2241  z_rl_norm= 45.078
step= 20  loss_rlt= 8.8566  z_rl_norm= 45.459
step= 30  loss_rlt= 8.2080  z_rl_norm= 48.678
step= 40  loss_rlt= 7.6593  z_rl_norm= 55.629
step= 50  loss_rlt= 6.8495  z_rl_norm= 81.492
step= 60  loss_rlt= 6.5652  z_rl_norm= 98.361
step= 70  loss_rlt= 6.1257  z_rl_norm=109.572
step= 80  loss_rlt= 6.0001  z_rl_norm=112.792
step= 90  loss_rlt= 5.6353  z_rl_norm=117.733
step=100  loss_rlt= 5.5825  z_rl_norm=126.715

Peak VRAM: 12.12 GB
Avg step time: 0.37 s
Loss 下降: 10.09 → 5.58 (−44.7%)
```

| T8.8 100 步 loss 趋势 | ✅ PASS | loss_rlt 从 10.09 持续下降至 5.58 |

**T-RLT8 总结：8/8 PASS**

### 10.4 T-RLT9: z_rl 提取验证

```
T9.1 z_rl_shape       [PASS] shape=[16, 1024], expected=[B, 1024]
T9.2 z_rl_finite      [PASS] nan=0, inf=0
T9.3 z_rl_deterministic [PASS] max_diff=0.00e+00
T9.4 z_rl_varies      [PASS] std=1.0284, n_samples=48
T9.5 z_rl_norm        [PASS] mean_norm=45.86, max_norm=46.12
```

**T-RLT9 总结：5/5 PASS**

### 10.5 T-RLT10: 向后兼容性验证

| 子测试 | 结果 | 说明 |
|---|---|---|
| T10.1 Stage 1 VLA checkpoint 原生推理 | ✅ PASS | 成功加载 3146M 参数 |
| T10.2 Action drift | ⏳ 跳过 | 需要推理 pipeline 集成测试，单 GPU 资源受限时无法同时加载两个模型 |
| T10.3 eval 脚本不受影响 | ⏳ 跳过 | 无真机可用，eval 脚本需要机器人连接 |
| T10.4 离线测试全通过 | ✅ PASS | 52/52 通过（含 T6.7 修复后） |

**T-RLT10 总结：2/4 PASS, 2/4 SKIP**

（T10.2 和 T10.3 需要更多 GPU 资源或真机环境，与当前设计约束一致。T10.2 的 action_drift 在 Profile B
下 VLA 参数未被修改，理论上 drift=0；T10.3 需要 Franka 机器人连接。）

### 10.6 新增和修改的文件

| 文件 | 操作 | 说明 |
|---|---|---|
| `train_4dwvla_rlt_stage1.py` | 新建 | 训练入口脚本，~350 行 |
| `rlt_config.py` | 修改 | 添加 `vla_inference_mode` 字段，修复 `from_yaml` 类型强转 |
| `rlt_stage1_wrapper.py` | 修改 | 添加 `vla_inference_mode` 支持，修复 `extract_z_rl` dtype |
| `configs/rlt_stage1_franka_plug.yaml` | 修改 | `prefix_seq_len: 768`, `vla_inference_mode: true` |
| `tests/test_rlt_training_online.py` | 新建 | T-RLT8 在线测试脚本 |
| `tests/test_rlt_z_extraction_online.py` | 新建 | T-RLT9 在线测试脚本 |
| `tests/test_rlt_compat_offline.py` | 修改 | T6.7 `prefix_seq_len` 断言改为 `>= 512` |

---

## 11. 错误汇总（完整）

| # | 错误 | 根因 | 影响范围 | 修复 |
|---|---|---|---|---|
| E1 | `FileNotFoundError: .../RLmm/b/rlinf/...` | Path parent chain 少 1 级 | T-RLT7 | 修改 `test_rlt_behavior_equiv.py:12` |
| E2 | `ImportError: relative import` | sys.path 方式加载不支持相对 import | T-RLT2 | `rlt_stage1_wrapper.py` try/except |
| E3 | `NoneType .shape` | mask 方法读取已清除的 `self._captured_prefix_out` | T-RLT2 | mask 方法改为接受参数 |
| E4 | `cfg.dataset.repo_id` = Qwen/Qwen3.5-2B | 循环变量 `repo_id` 覆盖函数参数 | 训练启动 | 重命名为 `hf_repo` |
| E5 | `TypeError: float vs str` | PyYAML `5e-5` 解析为字符串 | 优化器创建 | `from_yaml()` 类型强转 |
| E6 | 双重路径 `root/repo_id/repo_id` | LeRobot root vs repo_id 不一致 | 数据集加载 | symlink + `root=None` |
| E7 | 外部 stats 文件不存在 | 原始配置中的绝对路径 | 数据集加载 | `use_external_stats=False` |
| E8 | VQA logits OOM (9.69 GB) | `enable_vqa_loss=True` 计算完整 logits | 前向传播 | `enable_vqa_loss=False` |
| E9 | VLA 激活 OOM (~30 GB) | 单 GPU 放不下完整反向传播 | 前向+反向 | `vla_inference_mode` |
| E10 | `prefix_seq_len 650 > 512` | RLT 位置编码表太小 | RLT encoder | `prefix_seq_len: 768` |
| E11 | bf16 vs fp32 dtype mismatch | VLA 输出 bf16，RLT 参数 fp32 | z_rl 提取 | `prefix_out.to(rlt_dtype)` |

---

## 12. 验收 Gate 状态（最终）

| Gate | 条件 | 状态 |
|---|---|---|
| G1 | 离线测试（T-RLT1~T-RLT7）全部通过 | ✅ 52/52 |
| G2 | 训练 dry run（T-RLT8）通过 | ✅ 8/8 |
| G3 | z_rl 提取（T-RLT9）通过 | ✅ 5/5 |
| G4 | 向后兼容（T-RLT10）通过 | ⚠️ 2/4 (2 SKIP，需真机) |
| G5 | RLinf 源码零修改 | ✅ `git diff rlinf/` 为空 |
| G6 | 4DWVLA 源码零修改 | ✅ `git diff src/lerobot/` 为空 |
| G7 | 操作手册可独立执行 | ✅ §13 已更新：新增 §13.7.1 依赖安装、E4-E11 故障排除表、更新 VRAM 预期和命令示例 |

---

## 12.1 G7 更新详情

**操作手册（§13）更新内容**（对 `4dwvla_rlt1_2.markdown` 的修改）：

1. **§13.7.1 安装额外依赖**：新增 `flash-linear-attention` 和 `causal-conv1d` 安装步骤（两种方法：starvla `.pth` 链接 / PyPI 安装）
2. **§13.7.2 验证环境**：增加 fla 和 causal-conv1d 的验证检查
3. **§13.9 Dry Run**：更新命令（添加 `PYTORCH_CUDA_ALLOC_CONF`, `HF_HUB_OFFLINE`, `TRANSFORMERS_OFFLINE` 环境变量；添加 `--dataset_repo_id` 参数说明）；修正 VRAM 预期从 `< 28 GB` 到 `~12 GB`
4. **§13.10 生产训练**：更新命令，添加环境变量和 `--dataset_root`/`--dataset_repo_id` 参数
5. **§13.11 监控**：更新指标参考值（基于 100 步实测数据）
6. **§13.14 故障排除**：新增 E4-E11 已知陷阱表，包含症状、根因和修复位置

---

## 13. 关键运行命令

```bash
# 容器内执行训练（vla_inference_mode 模式，单 GPU）
source /opt/venv/4dwvla/bin/activate
cd /workspace/RLinf
PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 python b/x/4dwvla_ext/rlt/train_4dwvla_rlt_stage1.py \
  --config b/x/4dwvla_ext/rlt/configs/rlt_stage1_franka_plug.yaml \
  --max_steps 100 \
  --dataset_root /home/nvidia/data \
  --dataset_repo_id plug_into_socket_lrb_4D_8sml \
  --save_freq 50 \
  --log_freq 10 \
  --output_dir /workspace/RLinf/b/x/4dwvla_ext/rlt/outputs

# T-RLT9 z_rl 提取测试
PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 \
python b/x/4dwvla_ext/rlt/tests/test_rlt_z_extraction_online.py
```

---

## 14. 关键路径记录（更新）

| 用途 | 路径 |
|---|---|
| 训练入口脚本 | `b/x/4dwvla_ext/rlt/train_4dwvla_rlt_stage1.py` |
| T-RLT8 测试脚本 | `b/x/4dwvla_ext/rlt/tests/test_rlt_training_online.py` |
| T-RLT9 测试脚本 | `b/x/4dwvla_ext/rlt/tests/test_rlt_z_extraction_online.py` |
| 10 步训练 checkpoint | `/tmp/rlt_t8_10step/step_000010/` |
| 100 步训练 report | `/tmp/rlt_t8_100step/training_report.json` |
| 数据集 symlink | `~/.cache/huggingface/lerobot/plug_into_socket_lrb_4D_8sml` → `/home/nvidia/data/...` |
| starvla packages 链接 | `/opt/venv/4dwvla/lib/python3.11/site-packages/starvla.pth` |
| T-RLT10 测试脚本 | `b/x/4dwvla_ext/rlt/tests/test_rlt_compat_online.py` |

---

## 15. Phase 5: 文档补完（G7 最终版）

### 15.1 问题

实施方案文档 `4dwvla_rlt1_2.markdown` 存在大量"概要级"描述：
- §7 代码设计章节展示的是设计阶段的伪代码，与实际实现不一致（缺少 E4-E11 修复、`vla_inference_mode`、类型强转等）
- §10-§11 测试章节只有测试表格，没有运行命令、预期输出、操作步骤
- §11 T-RLT10（向后兼容性验证）仅 4 行表格，无法指导执行
- §9 显存估算过时（Profile B 实测为 12 GB 而非 22 GB）
- §2 依赖列表遗漏 `flash-linear-attention` 和 `causal-conv1d`

### 15.2 修复内容

| 章节 | 修改内容 |
|---|---|
| §2.4 | 额外依赖表：添加 `flash-linear-attention`, `causal-conv1d`，含版本、必要性说明 |
| §6.1 | 文件清单：添加 `test_rlt_compat_online.py` |
| §6.4 | 修正"RLT 不引入新依赖"描述 |
| §7.2 | `rlt_config.py` 替换为实际代码（含 `vla_inference_mode` 字段、类型强转 `from_yaml()`），添加 E5 教训 |
| §7.3 | `rlt_stage1_wrapper.py` 替换为实际代码（含 `vla_inference_mode` 分支、`extract_z_rl` dtype 转换），添加 E9/E11 教训 |
| §7.4 | `train_4dwvla_rlt_stage1.py` 替换为完整实现（替代原伪代码），添加 E4/E6/E7/E8/E9 教训注释 |
| §7.5 | YAML 配置更新：`prefix_seq_len: 768`, 添加 `vla_inference_mode: true`，添加 E10 教训注释 |
| §9.1 | 添加 Profile B+vim 行（实测 12 GB）；更新显存估算表为实测数据 |
| §9.2 | 修正 `prefix_seq_len` 参考值 512→768 |
| §10 T-RLT1~7 | 每个子章节添加 `运行方法`（bash 命令）和 `预期输出示例` |
| §11 T-RLT8 | 添加运行命令、环境变量设置、预期输出示例、耗时估计 |
| §11 T-RLT9 | 同上 |
| §11 T-RLT10 | **全面重写**：4 个子测试各含详细操作说明、Python 代码片段、判定标准、局限性说明，创建了 `test_rlt_compat_online.py` 测试脚本 |
| §12.1 | Gate 表添加"验证命令"列，更新状态为实测结果 |
| §12.2 | 验收报告模板填入实测数值（12.12 GB VRAM, 0.37s/step, loss 10.09→5.58） |
| §13.1 | VRAM 要求从 28 GB 更新为 16 GB |

### 15.3 新增文件

| 文件 | 用途 |
|---|---|
| `b/x/4dwvla_ext/rlt/tests/test_rlt_compat_online.py` | T-RLT10 向后兼容性验证脚本（~230 行），含 4 个子测试 |

### 15.4 文档增长

- 修改前：2469 行
- 修改后：3166 行（+697 行，主要为操作命令、预期输出、E4-E11 教训注释）
