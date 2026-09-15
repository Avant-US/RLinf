# 4DWVLA 纯 VLA 评估 — GPU 容器离线测试执行日志

> **日期**: 2026-09-14
> **基于文档**: `4wvla_rlinf_eval_3A3.md` v3A3.6
> **范围**: §14.2 GPU 容器离线测试 — T1, T4
> **执行环境**: GPU 容器 (`rlinf/rlinf:agentic-rlinf0.4-maniskill_libero`)

---

## 0. 执行前环境检查

### 0.1 宿主机环境

| 项目 | 值 |
|:---|:---|
| OS | Linux 5.15.0-1032-realtime |
| Docker | 已安装 |
| GPU 镜像 | `rlinf/rlinf:agentic-rlinf0.4-maniskill_libero` |
| 现有 GPU 容器 | `rlinf-rlt-gpu` (Exited) |

### 0.2 检查点路径映射

| 位置 | 路径 |
|:---|:---|
| 宿主机 | `/home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/` |
| Docker 挂载 | `CKPT_DIR=/home/nvidia/bt/ckp` → `/home/nvidia/ckpts` |
| 容器内实际路径 | `/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/` |
| 测试脚本中的路径 | `/home/nvidia/ckpts/4wvlaFrkPlugCkp010420` (**错误**, 缺少中间目录) |

### 0.3 测试文件

| 文件 | 宿主机路径 | 容器内路径 |
|:---|:---|:---|
| `test_transforms_offline.py` | `RLmm/b/x/4dwvla_ext/tests/test_transforms_offline.py` | `/workspace/RLinf/b/x/4dwvla_ext/tests/test_transforms_offline.py` |
| `setup_4dwvla_venv.sh` | `RLmm/b/x/4dwvla_ext/configs/setup_4dwvla_venv.sh` | `/workspace/RLinf/b/x/4dwvla_ext/configs/setup_4dwvla_venv.sh` |

---

## 1. 启动 GPU 容器

### 1.1 第一次启动 (缺少 HF 缓存挂载)

**命令**:
```bash
docker run -d --gpus all --privileged --network host --shm-size=20g \
    --name rlinf-4dwvla-gpu \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /home/nvidia/bt/s/RLmm:/workspace/RLinf \
    -v /home/nvidia/bt/s/4WVLA:/workspace/4WVLA \
    -v /home/nvidia/bt/ckp:/home/nvidia/ckpts:ro \
    -w /workspace/RLinf \
    rlinf/rlinf:agentic-rlinf0.4-maniskill_libero sleep infinity
```

**问题**: 容器内无法访问宿主机 HuggingFace 缓存. 后续 T4 测试中, `Qwen3_5ForConditionalGeneration.from_pretrained("Qwen/Qwen3.5-2B")` 试图从 HF Hub 下载模型 (~5 GB), 因无认证 token 且网速慢, 卡死超过 10 分钟.

### 1.2 第二次启动 (修复后)

**操作**: 停止并删除原容器, 重新启动并添加 HF 缓存挂载:
```bash
docker stop rlinf-4dwvla-gpu && docker rm rlinf-4dwvla-gpu
docker run -d --gpus all --privileged --network host --shm-size=20g \
    --name rlinf-4dwvla-gpu \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e HF_HOME=/home/nvidia/.cache/huggingface \
    -v /home/nvidia/bt/s/RLmm:/workspace/RLinf \
    -v /home/nvidia/bt/s/4WVLA:/workspace/4WVLA \
    -v /home/nvidia/bt/ckp:/home/nvidia/ckpts:ro \
    -v /home/nvidia/.cache/huggingface:/home/nvidia/.cache/huggingface \
    -w /workspace/RLinf \
    rlinf/rlinf:agentic-rlinf0.4-maniskill_libero sleep infinity
```

**新增挂载**: `-v /home/nvidia/.cache/huggingface:/home/nvidia/.cache/huggingface` + `-e HF_HOME=...`

**副作用**: 删除容器导致之前在容器 overlay 中创建的 4dwvla venv 丢失, 需要重新创建.

---

## 2. 4dwvla 虚拟环境创建

### 2.1 setup_4dwvla_venv.sh 的问题 (E3)

**原始脚本**: `RLmm/b/x/4dwvla_ext/configs/setup_4dwvla_venv.sh`

**问题**: 脚本使用 `uv venv /opt/venv/4dwvla` 创建虚拟环境后, 用 `/opt/venv/4dwvla/bin/pip install ...` 安装包. 但 uv 0.12.2 的 `uv venv` 默认不安装 pip, 导致 `/opt/venv/4dwvla/bin/pip: No such file or directory`.

**根因**: uv 与传统 `python -m venv` 不同, 其 `uv venv` 只创建最小虚拟环境结构, 不包含 pip.

**待修复**: 脚本中应使用 `uv pip install` 替代直接调用 `pip`, 或在创建 venv 后运行 `uv pip install pip` 安装 pip.

### 2.2 uv pip install torch 卡死 (E4)

**命令**: `uv pip install torch==2.11.0 --index-url https://download.pytorch.org/whl/cu128`

**现象**: 进程 0% CPU 占用, 无下载进度, 持续数分钟无响应.

**根因**: 网络解析 PyTorch 索引页问题, 可能因为 `cu128` 索引页体积庞大且网络不稳定.

**解决**: 杀死进程, 改用快速路径 — 复制已有的 `starvla` venv.

### 2.3 快速路径: 复制 starvla venv

`starvla` venv 已有 torch 2.11.0+cu128, flash_attn 2.8.3, Python 3.11.14 — 满足绝大部分依赖.

**操作步骤**:
```bash
# 1. 复制 starvla venv
docker exec rlinf-4dwvla-gpu cp -a /opt/venv/starvla /opt/venv/4dwvla

# 2. 修正 shebang 和路径引用
docker exec rlinf-4dwvla-gpu bash -c '
  cd /opt/venv/4dwvla/bin
  sed -i "s|/opt/venv/starvla|/opt/venv/4dwvla|g" activate activate.csh activate.fish pip pip3 pip3.11
  for f in python python3 python3.11; do
    rm -f $f && ln -s /usr/bin/$f $f
  done
'

# 3. 升级 transformers 到 5.2.0
docker exec rlinf-4dwvla-gpu bash -c '
  source /opt/venv/4dwvla/bin/activate
  uv pip install transformers==5.2.0
'

# 4. 安装 lerobot (4WVLA 框架)
docker exec rlinf-4dwvla-gpu bash -c '
  source /opt/venv/4dwvla/bin/activate
  cd /workspace/4WVLA && uv pip install -e .
'

# 5. 复制 transformers 补丁 (Qwen3.5 模型代码)
docker exec rlinf-4dwvla-gpu bash -c '
  source /opt/venv/4dwvla/bin/activate
  TDIR=$(python -c "import transformers, pathlib; print(pathlib.Path(transformers.__file__).parent)")
  cp -r /workspace/4WVLA/src/lerobot/policies/internvla_a1_5/transformers_replace/models/* ${TDIR}/models/
  cp -r /workspace/4WVLA/src/lerobot/policies/pi0/transformers_replace/models/* ${TDIR}/models/
  cp -r /workspace/4WVLA/src/lerobot/policies/pi05/transformers_replace/models/* ${TDIR}/models/
'
```

**验证**:
```
torch=2.11.0+cu128 transformers=5.2.0
```

**耗时**: 约 3 分钟 (主要在 `uv pip install -e .` 安装 lerobot 依赖).

---

## 3. T1: Transform 管道测试

### 3.1 测试文件

| 项目 | 值 |
|:---|:---|
| 测试文件 | `/workspace/RLinf/b/x/4dwvla_ext/tests/test_transforms_offline.py` |
| 测试内容 | T1.1 状态归一化, T1.2 动作反归一化, T1.3 图像变换 |
| 子测试数 | 10 |

### 3.2 第一次运行 — T1.1 mean+std->one FAIL (E5)

**结果**: 9/10 passed, 1/10 failed

**失败项**: `T1.1 mean+std->one`

**失败详情**: 测试预期 `normalize(mean + std)` 结果为 `1.0` (atol=1e-5), 但实际值为 `~0.999969`.

**根因分析**:

`NormalizeTransformFn` (位于 `4WVLA/src/lerobot/transforms/core.py:252`) 使用 `eps=1e-6` 做分母保护:
```python
x = (x - mean) / (std + eps)
```

当输入为 `mean + std` 时, 结果为 `std / (std + eps)`. 对于 gripper 维度 (`std=0.0324`):
```
result = 0.0324 / (0.0324 + 1e-6) = 0.0324 / 0.0324010 ≈ 0.999969
```

偏离 1.0 约 3.1e-5, 超过 `atol=1e-5` 的容差.

### 3.3 修复方案

**文件修改**: `RLmm/b/x/4dwvla_ext/tests/test_transforms_offline.py` 第 58-59 行

**修改前**:
```python
check("mean+std->one", np.allclose(result[OBS_STATE].numpy(), 1.0, atol=1e-5))
```

**修改后**:
```python
# NormalizeTransformFn uses eps=1e-6 in denominator: (x-mean)/(std+eps)
# For small std (e.g. gripper std=0.0324), result = std/(std+1e-6) ≈ 0.999969
expected = state_std / (state_std + 1e-6)
check("mean+std->one", np.allclose(result[OBS_STATE].numpy(), expected, atol=1e-5))
```

**修改原因**: 测试应反映实际的 `NormalizeTransformFn` 行为 (含 eps), 而非理想化的数学等式.

### 3.4 第二次运行 — 全部通过

**命令**:
```bash
docker exec rlinf-4dwvla-gpu bash -c \
  'source /opt/venv/4dwvla/bin/activate && \
   python -u /workspace/RLinf/b/x/4dwvla_ext/tests/test_transforms_offline.py'
```

**输出**:
```
=== T1.1: State Normalization ===
  [PASS] mean->zero
  [PASS] mean+std->one
  [PASS] q4 tokenization bin~128

=== T1.2: Action UnNormalization ===
  [PASS] zero->mean
  [PASS] arm within FR3 limits

=== T1.3: Image Transforms ===
  [PASS] resize observation.images.global
  [PASS] resize observation.images.wrist
  [PASS] image0 exists
  [PASS] image1 exists
  [PASS] image2 exists (padded)

=== Results: 10 passed, 0 failed ===
```

**结果**: **PASS** (10/10)

---

## 4. T4: 模型加载测试

### 4.1 测试目的

验证模型检查点能加载到 GPU, 参数量和显存占用在合理范围.

**验收标准**:
- 无 `RuntimeError` 或 `CUDA out of memory`
- VRAM < 16 GB
- Params > 2000M
- 输出 `[PASS] Model loads OK`

### 4.2 检查点路径问题

**文档 (eval_3A3.md) 中的路径**: `/home/nvidia/ckpts/4wvlaFrkPlugCkp010420`

**实际路径**: `/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/`

**差异**: 文档缺少中间目录 `4wvlaFrk/plug/`. 宿主机挂载为 `/home/nvidia/bt/ckp` → `/home/nvidia/ckpts`, 而检查点在宿主机上的完整路径为 `/home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/`.

**检查点文件**:
| 文件 | 大小 |
|:---|:---|
| config.json | 配置 |
| model.safetensors | 5.89 GB |
| stats.json | 归一化统计 |
| train_config.json | 训练配置 |

### 4.3 第一次尝试 — HF Hub 下载卡死 (E6)

**现象**: `Loading model...` 后无输出, GPU 显存 7979 MiB, 进程存活但无进展, 超过 10 分钟.

**根因**: 容器无 HF 缓存挂载. `InternVLAA15WithExpertModel.__init__` (line 614) 调用:
```python
self.qwen3_5 = Qwen3_5ForConditionalGeneration.from_pretrained(vlm_model_name_or_path)
```
其中 `vlm_model_name_or_path = "Qwen/Qwen3.5-2B"`, 触发从 HF Hub 下载完整的 Qwen3.5-2B 基础模型权重 (~5 GB). 因无 HF_TOKEN 认证, 下载速率极低.

**处置**: 杀死进程, 删除容器, 添加 HF 缓存挂载后重启 (见 §1.2).

### 4.4 第二次尝试 — 仍然卡死 (E7)

**环境**: 已挂载 HF 缓存, 已重建 4dwvla venv.

**现象**: 同样卡在 `Loading model...`, GPU 7979 MiB, 进程 PID 396 处于 `futex_wait_queue_me` 状态.

**诊断**: 检查 `/proc/396/fd/` 发现进程打开了:
```
/home/nvidia/.cache/huggingface/hub/models--Qwen--Qwen3.5-2B/blobs/aa33250c...54aeecb4.incomplete
```

**根因**: HF 缓存中 Qwen3.5-2B 的 **tokenizer 文件完整** (tokenizer.json, vocab.json, merges.txt, config.json 等均以符号链接存在于 snapshot 中), 但 **模型权重 blob 未下载完成** — 仅有多个 `.incomplete` 文件 (最大 1.07 GB, 实际需要 ~5 GB).

`Qwen3_5ForConditionalGeneration.from_pretrained` 即使设了 `HF_HOME` 仍会尝试下载缺失的权重文件, 而非仅使用已缓存的部分.

### 4.5 解决方案: 离线 Monkey-Patch

**核心思路**: `InternVLAA15WithExpertModel.__init__` 先用 `from_pretrained` 加载 Qwen3.5-2B 基础权重, 然后 `PreTrainedPolicy.from_pretrained` 立即用检查点权重覆盖. 因此基础权重的实际值无关紧要 — 随机初始化与下载基础权重在语义上等价.

**Monkey-Patch**:
```python
os.environ["HF_HUB_OFFLINE"] = "1"
os.environ["TRANSFORMERS_OFFLINE"] = "1"

@classmethod
def _offline_from_pretrained(cls, pretrained_model_name_or_path, *args, **kwargs):
    qwen_config = AutoConfig.from_pretrained(
        pretrained_model_name_or_path, local_files_only=True  # 使用已缓存的 config.json
    )
    model = cls(qwen_config)  # 随机初始化, 不下载权重
    return model

Qwen3_5ForConditionalGeneration.from_pretrained = _offline_from_pretrained
```

**关键前提**: `AutoConfig.from_pretrained("Qwen/Qwen3.5-2B", local_files_only=True)` 能正常工作, 因为 `config.json` 已完整缓存.

**额外修复**: 需要在加载 config 前先 import `InternVLAA15Config` 以触发 draccus ChoiceRegistry 注册, 否则报 `KeyError: 'internvla_a1_5'`.

### 4.6 测试脚本

**文件**: `/tmp/t4_model_load_test.py` (临时文件, 仅用于本次测试)

**主要步骤**:
1. 设置 `HF_HUB_OFFLINE=1`, `TRANSFORMERS_OFFLINE=1`
2. 验证检查点目录存在且 `model.safetensors` 完整
3. 导入 `InternVLAA15Config` (注册到 draccus registry)
4. 加载检查点 config, 设置 `action_loss_only=True`, `inference_backend="optimized"`
5. Monkey-patch `Qwen3_5ForConditionalGeneration.from_pretrained`
6. 调用 `policy_cls.from_pretrained(CKPT, config=config)` 加载模型
7. 移至 CUDA, bfloat16 精度
8. 验证参数量和显存

### 4.7 测试输出

```
============================================================
T4: Model Loading Test (offline)
============================================================

[Step 1] Checking checkpoint: /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420
  Files: ['train_config.json', 'config.json', 'model.safetensors', 'stats.json']
  model.safetensors: 5.89 GB

[Step 2] Loading config...
  Policy type: internvla_a1_5
  VLM model: Qwen/Qwen3.5-2B
  action_loss_only=True
  inference_backend=optimized

[Step 3] Patching VLM init for offline loading...
  Patch applied.

[Step 4] Loading model from checkpoint...
  Policy class: InternVLAA15Policy
  [patch] Loading Qwen3.5-2B config from cache (skip weight download)...
  [patch] Model architecture initialized (random weights)
  Model loaded in 64.3s

[Step 5] Verification...
  Params: 3146.0M
  VRAM:   6.73 GB
  [PASS] No CUDA OOM
  [PASS] VRAM < 16 GB
  [PASS] Params > 2000M
  [PASS] Model loads OK

=== T4 Results: 4 passed, 0 failed ===
```

**关键数据**:
| 项目 | 值 | 验收标准 |
|:---|:---|:---|
| 参数量 | 3146.0M | > 2000M ✓ |
| VRAM | 6.73 GB | < 16 GB ✓ |
| 加载时间 | 64.3s | < 5 min ✓ |
| 推理后端 | optimized (跳过 WAN) | — |
| 精度 | bfloat16 | — |

**警告 (非致命)**:
- `The fast path is not available because one of the required library is not installed` — 缺少 `flash-linear-attention`, 回退到 torch 实现. 不影响功能正确性.
- `Unexpected key(s): model._wan_grid_sizes, model.learnable_to_wan_proj.*` — WAN 视频分支的权重在 `action_loss_only=True` 模式下不需要, 正常忽略.

**结果**: **PASS** (4/4)

---

## 5. 错误汇总

| 编号 | 错误 | 根因 | 修复 | 状态 |
|:---|:---|:---|:---|:---|
| E3 | `setup_4dwvla_venv.sh`: `/opt/venv/4dwvla/bin/pip: No such file or directory` | uv 0.12.2 的 `uv venv` 不安装 pip | 脚本修复: `PIP="${UV} pip"` 替代 `PIP="${VENV_DIR}/bin/pip"` | 已修复 |
| E4 | `uv pip install torch` 0% CPU 卡死 | 网络解析 PyTorch cu128 索引页问题 | 绕过: 复制 starvla venv | 绕过 |
| E5 | T1.1 `mean+std->one` FAIL | `NormalizeTransformFn` 使用 `eps=1e-6` 分母, `std/(std+eps) ≠ 1.0` | 修改测试预期值为 `state_std / (state_std + 1e-6)` | 已修复 |
| E6 | T4 首次加载超时 (>10 min) | 容器无 HF 缓存挂载, 下载 Qwen3.5-2B ~5GB 无认证 | 重建容器, 添加 `-v .cache/huggingface` 挂载 | 已修复 |
| E7 | T4 二次加载仍卡死 | HF 缓存中 tokenizer 完整但模型权重 blob 未下载完成 (.incomplete) | Monkey-patch: 用 `AutoConfig` + 随机初始化替代 `from_pretrained` 下载 | 已修复 |
| E8 | `KeyError: 'internvla_a1_5'` (draccus registry) | 加载 config 前未 import `InternVLAA15Config`, 导致 policy type 未注册 | 添加 `from lerobot.policies.internvla_a1_5.configuration_internvla_a1_5 import InternVLAA15Config` | 已修复 |

---

## 6. 文件变更清单

| 文件 | 操作 | 原因 |
|:---|:---|:---|
| `RLmm/b/x/4dwvla_ext/tests/test_transforms_offline.py:58-59` | 修改 | 修复 T1.1 `mean+std->one` eps 容差问题 (E5) |
| `/tmp/t4_model_load_test.py` (容器内) | 新建 (临时) | T4 离线模型加载测试脚本, 含 monkey-patch |

| `RLmm/b/x/4dwvla_ext/configs/docker_run_4dwvla_gpu.sh` | 修改 | 添加 HF 缓存挂载 (`-v .cache/huggingface`) 和 `HF_HOME` 环境变量 |
| `RLmm/b/x/4dwvla_ext/configs/setup_4dwvla_venv.sh` | 修改 | `PIP="${VENV_DIR}/bin/pip"` 改为 `PIP="${UV} pip"`, 修复 E3 |
| `RLmm/b/d/frk1/4wvla_rlinf_eval_3A3.md` | 修改 | 修复检查点路径 (所有引用添加 `4wvlaFrk/plug/`), T4 增加离线模式代码和常见问题, T1.1 验证点更新 eps 说明, 版本升至 v3A3.7 |

---

## 7. T_FK: FK Keypoint 计算测试 (v3A3.8 新增)

### 7.1 背景

v3A3.8 发现检查点 `enable_keypoint_predictor=True`, 推理时需要 4D keypoint 输入. 新增 `fk_keypoints.py` 和 `test_fk_keypoints_offline.py`.

### 7.2 测试输出

```
=== T_FK.1: Shape and Metadata ===
  [PASS] num_joints == 8
  [PASS] kpt_dim == 7 (pos_rot)
  [PASS] history_max_len == 200
  [PASS] output shape (8, 7)

=== T_FK.2: Normalization Conventions ===
  [PASS] joint0 quat unit norm ... joint7 quat unit norm
  [PASS] joint0 qw >= 0 (hemisphere) ... joint7 qw >= 0 (hemisphere)
  [PASS] positions in reasonable range (< 2.0 after /bbox_radius)

=== T_FK.3: History Buffer ===
  [PASS] after 1 step: his_len == 1
  [PASS] his shape (200, 8, 7)
  [PASS] first frame non-zero
  [PASS] second frame zero (padding)
  [PASS] after 11 steps: his_len == 11
  [PASS] after reset + 1 step: his_len == 1

=== T_FK.4: Determinism ===
  [PASS] same input → same output

=== T_FK Results: 28 passed, 0 failed ===
```

**结果**: **PASS** (28/28)

---

## 8. 最终结果

| 测试 | 子测试数 | 通过 | 失败 | 状态 |
|:---|:---|:---|:---|:---|
| T1 (Transform 管道) | 10 | 10 | 0 | **PASS** ✓ |
| T\_FK (FK Keypoint) | 28 | 28 | 0 | **PASS** ✓ |
| T4 (模型加载) | 4 | 4 | 0 | **PASS** ✓ |
| **总计** | **42** | **42** | **0** | **全部通过** |

---

## 9. 文档一致性修正 (v3A3.9)

**时间**: 2026-09-14

v3A3.8 修复了 D8 致命缺陷后, 文档中部分嵌入代码和分析结论未同步更新. v3A3.9 修正以下不一致:

| 位置 | 问题 | 修正 |
|:---|:---|:---|
| §5.3 嵌入代码 | 仍为旧版 server 代码 (无 keypoint 支持, 硬编码 `optimized` 后端) | 同步为最新 `vla_inference_server.py` |
| §5.1 嵌入代码 | `PIP="${VENV_DIR}/bin/pip"` (实际文件为 `PIP="${UV} pip"`); 版本约束未加引号 | 修正 PIP 定义 + 加引号 |
| §7.1 嵌入代码 | 缺少 `HF_CACHE` 变量, `-e HF_HOME`, HF 缓存挂载 | 同步实际 docker 脚本 |
| §7.3 配置表 | 缺少 `HF_CACHE` 环境变量行; 挂载映射表缺少 HF 缓存 | 新增两行 |
| §9.1 B1 行 | "**否** (action\_loss\_only=True, 跳过关键点分支)" | "**是** (v3A3.8: FK keypoint 归一化使用 bbox\_radius)" |
| §9.2 | "B1 为什么不涉及" + 旧推理后端描述 | 改为 "B1 为什么涉及" + 新后端描述 |
| §9.3 表 | "Mode A 是否使用: **否**" | "**是** (v3A3.8: FK keypoint pos 归一化)" |
| §16.2 | "B1 bbox: 0.8361 m (**Mode A 不使用**)" | "B1 bbox: 0.8361 m (FK keypoint pos 归一化)" |

**文件变更**:

| 文件 | 变更 |
|:---|:---|
| `4wvla_rlinf_eval_3A3.md` | v3A3.8 → v3A3.9: 7 处嵌入代码/分析结论同步修正 |

---

## 10: v3A3.11 D9 修复 + FAST/State 架构分析 + 测试覆盖扩展

**发现**: 训练数据 `tasks.parquet` 中 task 为 `"plug into socket"`, 而推理代码和文档中为 `"plug the charger into the socket"` (task prompt 不匹配, D9 缺陷).

**代码修复**:

| 文件 | 变更 |
|:---|:---|
| `franka_vla_client.py` line 13 | `--task "plug the charger into the socket"` → `"plug into socket"` |
| `test_ipc_offline.py` line 76 | `"task": "plug the charger"` → `"plug into socket"` |

**文档修改** (`4wvla_rlinf_eval_3A3.md` v3A3.10 → v3A3.11):

| 位置 | 修改 |
|:---|:---|
| 全文 11 处 `--task` | `"plug the charger into the socket"` → `"plug into socket"` |
| §4 标题 | "8 项缺陷" → "9 项缺陷" |
| §4.1 缺陷表 | 新增 D9 行 (严重: task prompt 不匹配) |
| §4.1.2 (新增) | D9 深度分析: task prompt 影响链路 (Mermaid 图 + tokenization 偏移分析) |
| §4.1.3 (新增) | FAST token 与 state 双通路架构分析 (确认推理方案正确) |
| §9.4 一致性检查清单 | 新增 6 行 (task prompt / FAST token / state 通路 1+2 / system message) |
| §12 代码复用表 | 新增 N16 (`test_task_prompt_offline.py`) |
| §14.0 测试流程 | [A] 组新增 T11 |
| §14.0 测试表 | 新增 T11 行 |
| §14.1 标题 | "(T2, T3, T10)" → "(T2, T3, T10, T11)" |
| §14.1 (新增 T11 节) | T11 详细描述 (4 子测试组, 13 子测试, 含验收标准) |
| §14.4 验收总表 | 新增 T11 行 |
| 文件树 | 新增 `test_task_prompt_offline.py` |

**新增测试文件**: `tests/test_task_prompt_offline.py` (T11, 13 子测试):

```
=== T11 Results: 13 passed, 0 failed ===
```

---

## §11: v3A3.12 变更记录

> **日期**: 2026-09-15
> **版本**: v3A3.11 → v3A3.12

### 11.1 发现的缺陷

**D10: Stats 字段键缺失 + 动作维度不匹配 (致命)**

`stats.json` 仅含子字段键 (`observation.state.arm`[7], `observation.state.gripper`[1], `action.arm`[7], `action.gripper`[1]), 不含组合键 (`observation.state`[8], `action`[8]). `load_stats()` 查找组合键时触发 `KeyError`. 另外, 模型 `output_features.action.shape=[32]` (max\_action\_dim padding), 但 stats 仅 8D, 反归一化时广播维度不匹配.

### 11.2 代码修复

| 文件 | 修改 |
|:---|:---|
| `vla_inference_server.py` `load_stats()` | 签名 `(ckpt_path)` → `(ckpt_path, schema)`, 新增 `compose_sub_field_stats()` + `pick_or_compose()` |
| `vla_inference_server.py` `serve()` | `load_stats(ckpt)` → `load_stats(ckpt, schema)`, 新增 `actual_action_dim` |
| `vla_inference_server.py` serve 循环 | `action_pred[:n_exec]` → `action_pred[:n_exec, :actual_action_dim]` |

### 11.3 文档修改

| 位置 | 修改 |
|:---|:---|
| §4 标题 | "9 项缺陷" → "10 项缺陷" |
| §4.1 缺陷表 | 新增 D10 行 (致命) |
| §4.1.4 (新增) | D10 深度分析: stats 键缺失 + 动作维度不匹配 (含 Mermaid 图 + 数学等价性证明) |
| §4.6 (新增) | 训推参数全面对比审计: 46 项参数, 6 大类, 4 个关键差异影响分析 |
| §5.3 嵌入代码 | 同步 `load_stats()` 和 `serve()` 修改 |
| §9.4 一致性检查 | 新增 2 行 (stats 键组合 / 动作维度裁切) |
| §12 代码复用 | 新增 N17 |
| §14.0 测试流程 | [A] 组新增 T12 |
| §14.0 测试表 | 新增 T12 行 |
| §14.1 标题 | 新增 T12 |
| §14.1 (新增 T12 节) | T12 详细描述 (4 子测试组, 23 子测试) |
| §14.4 验收总表 | 新增 T12 行 |

### 11.4 新增测试

`tests/test_stats_composition_offline.py` (T12, 23 子测试):

```
=== T12 Results: 23 passed, 0 failed ===
```

T11 回归测试:
```
=== T11 Results: 13 passed, 0 failed ===
```

T2 回归测试:
```
=== Results: 10 passed, 0 failed ===
```

