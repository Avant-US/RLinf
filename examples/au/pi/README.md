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

1. rlinf 虚拟环境：`/mnt/r/VENV/rlinf`（可用 `PY=` 覆盖）。
2. openpi(aupi) 源码：`/home/physical/SRC/Robot/aupi05/src`（可用 `OPENPI_SRC=` 覆盖）。
3. pi0.5 基座权重目录（含 `model.safetensors` + `config.json`）：默认
   `/mnt/r/CKPT/VLA/DEMO/pi05_pushdoor_tst1`（可用 `BASE_CKPT=` 覆盖）。
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

## 在 `/mnt/r/VENV/rlinf/` 共享 venv 中跑通本例踩过的坑（排障记录）

本例最初按上面「前置条件」写的是独立的 `/mnt/r/VENV/openpi_venv`。但该 venv 后来被删除，
实际改用的是 **`/mnt/r/VENV/rlinf/`**——一个由
[`requirements/au_install.sh`](../../../requirements/au_install.sh) 的
`--model aupi` 分支（`install_aupi_model()`）统一管理、**RLinf 和 openpi(aupi) 共用**的
venv。因为一个 venv 里要同时满足 RLinf 自己的 `pyproject.toml` 依赖锁定和 openpi
(`aupi05/pyproject.toml`) 的依赖锁定，这两套锁定在 `torch` 系工具链上直接冲突，由此引出下面
两个环境级的坑。第三个坑是运行期资源竞争，与代码/环境无关，单独记录供以后排查参考。

### 坑 1：`torchcodec` 与 `torch` 版本错配，CPU 视频解码算子注册不上

**现象**

`run_norm_stats.sh`（本质是加载 LeRobot 数据集触发视频解码）或 `run_train.sh` 报：

```
NotImplementedError: Could not run 'torchcodec_ns::add_video_stream' with arguments
from the 'CPU' backend. ... 'torchcodec_ns::add_video_stream' is only available for
these backends: [CUDA, Meta, ...]
```

即 `add_video_stream` 这个算子只注册到了 `CUDA` dispatch key，`CPU` 版没注册上——数据集加载
（在 CPU 上解码视频帧）直接失败。

**根因**

`install_aupi_model()` 里 RLinf 与 openpi 的 `uv sync` 顺序执行时，`torch` 系三件套的版本被
来回拉动：

1. 先按 RLinf 的 `pyproject.toml`（`torch==2.6.0` / `torchvision==0.21.0` /
   `torchaudio==2.6.0` / `torchcodec==0.2`）装好一套自洽的 2.6 组合；
2. `uv sync --active --inexact` 装 openpi 时，openpi 的 `pyproject.toml` 钉的是
   `torch==2.7.1`，于是把 `torchvision` 拉到 `0.22.1`（配 2.7.1）、把 `torchcodec` 拉到
   `0.4.0`（`torchcodec` 官方兼容表：`0.4`↔`torch 2.7`），但 **`--inexact` 不会主动升级
   已经装好的、带本地版本标签的 `torch==2.6.0+cu126`**；
3. 结果就是 `torch 2.6.0 + torchcodec 0.4.0` 这个错配组合被留在了 venv 里
   （`torchcodec 0.4` 只支持 `torch 2.7`，见
   [官方兼容表](https://github.com/pytorch/torchcodec?tab=readme-ov-file#installing-torchcodec)）。
   `torchcodec` 的自定义算子按 ffmpeg 主版本（4/5/6/7）编译成不同 `.so`，`add_video_stream`
   在 `TORCH_LIBRARY_IMPL(torchcodec_ns, CPU, ...)` 里注册；`0.4.0` 编译时用的 torch ABI 与
   已装的 2.6.0 不一致，导致这个 CPU 实现没能挂上去（只剩 `BackendSelect` 等设备无关的算子能用，
   例如 `create_from_file`，这也是为什么 `import torchcodec` 本身不报错、只在真正解码时才炸）。

**修复**

在 `install_aupi_model()` 原来"把三件套对齐到 openpi 的 torch 2.7.1"这一步（2b 步），
改成**反向对齐**：把 openpi 拉起来的 `torchvision 0.22.1` / `torchcodec 0.4.0` 也拉回
RLinf 的 2.6 系，四件套一次性从同一个 cu126 索引重装，保证 ABI 一致：

```bash
uv pip install --reinstall --no-config \
    torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 torchcodec==0.2.1 \
    --index-url https://download.pytorch.org/whl/cu126
```

选择"向下对齐到 2.6"而不是"向上对齐到 2.7.1"，是因为实测 **torch 2.6 对 RLinf 和 openpi 的
PyTorch 训练路径都是自洽的**（SFT 训练能正常跑），换 2.7.1 反而要解另一组新的 ABI 兼容性问题
（且 2.7.1 那条路径当时也没能稳定跑通）。

### 坑 2：RLinf 的 `override-dependencies` 把 `torchcodec==0.2.1` 悄悄降成 `0.2.0`，而 `0.2.0` 缺 ffmpeg4 变体

**现象**

坑 1 的修复脚本跑完后，最后一步的校验（`from torchcodec.decoders import VideoDecoder`）仍然
报错：

```
RuntimeError: Could not load libtorchcodec. Likely causes: ...
[start of libtorchcodec loading traceback]
libavutil.so.59: cannot open shared object file: No such file or directory
libavutil.so.58: cannot open shared object file: No such file or directory
libavutil.so.57: cannot open shared object file: No such file or directory
[end of libtorchcodec loading traceback].
```

装完之后 `pip show torchcodec` 一看，版本是 `0.2.0+cu126`，不是脚本里写的 `0.2.1`。

**根因**

`install_aupi_model()` 是在 RLinf 仓库根目录下执行的，而
[`RLinf/pyproject.toml`](../../../pyproject.toml) 的 `[tool.uv] override-dependencies`
里有一条 `"torchcodec==0.2"`。`uv pip install` 默认会读取当前目录的 `pyproject.toml` /
`uv.toml` 配置，且 **override-dependencies 的优先级高于命令行显式给出的版本号**——所以命令行
写的 `torchcodec==0.2.1` 被静默重写成了 `0.2.0`（`==0.2` 在 uv 里精确匹配到 `0.2.0`）。

而 `torchcodec==0.2.0+cu126` 这个 wheel 里**只打包了 `libtorchcodec5.so` /
`libtorchcodec6.so` / `libtorchcodec7.so`**（分别对应 ffmpeg 5/6/7，需要
`libavutil.so.57/58/59`），**没有 ffmpeg4 变体**；而本机系统只装了 ffmpeg 4
（`libavutil.so.56`），于是 `torchcodec` 按 7→6→5→4 顺序尝试加载全部失败。
`torchcodec==0.2.1+cu126` 才补齐了 `libtorchcodec4.so`，能对上系统的 ffmpeg 4。

**修复**

给这条 `uv pip install` 加 `--no-config`，让它忽略 `pyproject.toml` / `uv.toml`（因此绕开
`override-dependencies` 里的 `torchcodec==0.2`），命令行显式给的 `torchcodec==0.2.1` 才会被
真正采纳；cu126 索引仍然靠显式的 `--index-url` 指定，不受影响：

```bash
uv pip install --reinstall --no-config \
    torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 torchcodec==0.2.1 \
    --index-url https://download.pytorch.org/whl/cu126
```

### 坑 3：`torchcodec` 的 cu126 CUDA 构建缺 NPP 运行时库

**现象**

坑 2 修完、`torchcodec` 装成 `0.2.1+cu126` 后，`import torchcodec.decoders` 又换了个错误：

```
[start of libtorchcodec loading traceback]
libnppicc.so.12: cannot open shared object file: No such file or directory
...
[end of libtorchcodec loading traceback].
```

**根因**

`torchcodec==0.2.1+cu126` 是 **CUDA 构建**（哪怕只在 CPU 上解码，它仍然是这个变体，因为要跟
已装的 `torch==2.6.0+cu126` 保持同一套 CUDA ABI），其解码 `.so` 在 `dlopen` 时依赖
`libnppicc.so.12`（NVIDIA Performance Primitives 里的图像色彩空间转换库）。这个依赖 `uv`/
`pip` 不会自动拉入——它既不是 `torch`、也不是 `torchvision`/`torchaudio` 的直接依赖，本机系统
也没有单独装 NPP 的 deb 包，所以全新装完之后这个 `.so` 就是缺失的。

**修复**

1. 显式装 `nvidia-npp-cu12`（pip 包，提供 `libnppicc.so.12` 等一组 NPP 运行时库，装在
   `site-packages/nvidia/npp/lib/`）：

   ```bash
   uv pip install nvidia-npp-cu12==12.4.1.87
   ```

2. 把这个目录追加进 venv 的 `bin/activate`，这样任何 `source .../activate` 的进程（包括继承
   环境变量启动的 Ray worker）都能在不用手动 `export` 的情况下找到该库；用
   `grep -qsF` 判断是否已存在，避免复用 venv 时重复追加同一行：

   ```bash
   npp_lib="$VENV_DIR/lib/python${_aupi_py_mm}/site-packages/nvidia/npp/lib"
   if ! grep -qsF "$npp_lib" "$VENV_DIR/bin/activate"; then
       echo "export LD_LIBRARY_PATH=\"${npp_lib}:\$LD_LIBRARY_PATH\"" >> "$VENV_DIR/bin/activate"
   fi
   ```

3. 装完立刻用一次真实的 `from torchcodec.decoders import VideoDecoder` 导入做校验，装的时候
   就能发现问题，而不是等到跑训练/跑数据时才炸。

### 坑 1～3 的最终代码改动：`requirements/au_install.sh`

以上三个坑全部改在 [`requirements/au_install.sh`](../../../requirements/au_install.sh) 的
`install_aupi_model()` 函数里（原第 2b 步，现拆成 2b + 2c 两步），**没有改动 RLinf 任何框架/
模型代码，也没有改动本示例目录下的任何文件**——因为坏的从来不是这个例子本身，而是它依赖的共享
venv 的安装脚本：

- **改（2b 步）**：把 `uv pip install --reinstall torch==2.7.1 torchvision==0.22.1
  torchaudio==2.7.1 --index-url .../cu126`（原意：对齐到 openpi 的 2.7.1）改成
  `uv pip install --reinstall --no-config torch==2.6.0 torchvision==0.21.0
  torchaudio==2.6.0 torchcodec==0.2.1 --index-url .../cu126`（反向对齐到 RLinf 的 2.6，
  且四件套一起管、加 `--no-config` 绕开 override）。同时补装 `nvidia-npp-cu12`。
  为什么这样改：见坑 1、坑 2 的根因分析——2.7.1 那条路当时验证不稳定，且 openpi 的 SFT 训练
  路径在 2.6 上是自洽的，向下对齐改动面更小、也更贴近 RLinf 自身默认锁定的版本。
- **增（2c 步，新增）**：把 NPP 库目录写进 `$VENV_DIR/bin/activate` 的 `LD_LIBRARY_PATH`，
  并在装完后立即做一次 `VideoDecoder` 导入校验。为什么要新增：坑 3 的 NPP 依赖不会被 `uv`/
  `pip` 自动解析出来，必须手动补，且必须让它在 venv 激活时自动生效（而不是要求每次运行示例前
  都手动 `export`），否则任何人（包括 Ray 起的 worker 子进程）换个终端重新 `source
  activate` 就会再次踩到坑 3。
- 未改动：`examples/au/pi/*`（本目录下的 YAML/脚本/README）、RLinf 框架代码
  （`rlinf/` 下任何文件）、openpi(aupi05) 代码。这也印证了本例 README 开头所说的
  "本示例不改动 RLinf 任何框架/模型代码"这一原则——即使要修 bug，也优先修在
  "环境搭建"这一层，而不是去动示例或框架代码。

修完后在 `/mnt/r/VENV/rlinf/` 里重新走一遍 `install_aupi_model()`，端到端验证通过：
`torch 2.6.0+cu126` / `torchvision 0.21.0+cu126` / `torchaudio 2.6.0+cu126` /
`torchcodec 0.2.1+cu126`（`VideoDecoder` 导入 OK）、`transformers 4.53.2` +
`transformers_replace` 补丁生效、`flash-attn 2.7.4.post1` 已装。

### 坑 4（运行期，非代码/环境问题）：`run_train.sh` 以 `PY=` 直接调用 python 二进制时，坑 3 的 `activate` 修复不生效

**现象**

`run_train.sh` 跑到第一个 training step 报：

```
RuntimeError: Could not load libtorchcodec. ...
[start of libtorchcodec loading traceback]
libnppicc.so.12: cannot open shared object file: No such file or directory
...
[end of libtorchcodec loading traceback].
```

与坑 3 完全相同的 `libnppicc.so.12` 找不到。

**根因**

坑 3 的修复是把 NPP 库路径写进了 `$VENV_DIR/bin/activate` 的 `LD_LIBRARY_PATH`。但
`run_train.sh` **不 source activate**——它直接通过 `PY=/mnt/r/VENV/rlinf/bin/python`
调用 Python 二进制。`activate` 里追加的 `export LD_LIBRARY_PATH=...` 从未被执行过，于是
`nvidia-npp-cu12` 虽然已装（装在
`site-packages/nvidia/npp/lib/libnppicc.so.12`），链接器仍然找不到它。

**修复**

在启动 `run_train.sh` 时**手动**在命令行传入 `LD_LIBRARY_PATH`：

```bash
LD_LIBRARY_PATH="/mnt/r/VENV/rlinf/lib/python3.11/site-packages/nvidia/npp/lib:${LD_LIBRARY_PATH:-}" \
  bash examples/au/pi/run_train.sh
```

如果将来不想每次都手动传，可以在 `run_train.sh` 脚本里加一行：

```bash
export LD_LIBRARY_PATH="$(dirname "$PY")/../lib/python3.11/site-packages/nvidia/npp/lib:${LD_LIBRARY_PATH:-}"
```

### 坑 5（运行期，非代码/环境问题）：多卡显存被同机其它作业占用，导致 checkpoint 存档时崩溃

**现象**

坑 1～3 修完后，`run_norm_stats.sh` 稳定成功；`run_train.sh` 也能正常起 8 卡 FSDP、loss/
grad-norm 指标健康下降，但**两次独立运行都精确崩在第 25 步**（配置里的
`runner.save_interval: 25`，即第一次存 checkpoint 的那一步），报错形式不完全一样：

- 第一次：`SystemExit: 1` 发生在 `FullyShardedDataParallel._post_forward` 的
  `torch.profiler.record_function` 上下文退出处，外层被 Ray 包装成
  `RuntimeError: SystemExit caught in ...`，紧随其后是 `SIGTERM`。
- 第二次：base（非 EMA）分片 checkpoint 已经完整写出到
  `_out/pi05_pushdoor_tst1_smoke50/checkpoints/global_step_25/actor/dcp_checkpoint/`，
  但紧接着的 EMA 权重存档崩溃：先是 `RuntimeError: DataLoader worker (pid ...) is killed
  by signal: Terminated`，然后 `ema.swap_out()` 里
  `RuntimeError: The size of tensor a (526647296) must match the size of tensor b
  (65830912) at non-singleton dimension 0`（`526647296 ÷ 8 = 65830912`，正好是
  world_size=8 的分片大小——说明 FSDP 的 unshard 被中途打断，参数还停留在分片状态，
  EMA 却想拿完整尺寸的备份张量去覆盖）。

**根因**

与代码、环境搭建都无关，是**运行时资源竞争**：这台机器是共享的，另一个用户的作业
（`/mnt/r/share/zwy/Project/pi/openpi/.venv/bin/python3`）在训练全程一直占着 8 张卡各
~109 GB（H200 单卡 143.7 GB），留给本示例的只有 **~34 GB/卡**。第 25 步存档需要 FSDP 把分片
参数临时 `all-gather` 成完整张量（`get_model_state_dict` → `_full_pre_state_dict_hook` →
unshard），这是训练全程显存需求最高的瞬间；在只剩 ~34 GB 余量的情况下，这一刻最容易被系统
或其它进程的显存压力挤爆，某个 worker 被 `SIGTERM`，进而级联导致 DataLoader / EMA 状态不一致
而崩溃。主机内存（2.6+ TB 空闲）和磁盘都非常充足，`dmesg` 也没有内核 OOM-killer 记录，
进一步排除了内存/磁盘问题。

**不属于"修复"的应对建议**（供以后在这台共享机器上重跑时参考，未改动任何文件）：

- 等占用显存的作业结束后原样重跑；
- 或临时调大 `runner.save_interval`（甚至设到 `> runner.max_steps`）跳过存档，只验证训练
  循环本身不崩；
- 或给 `actor.fsdp_config` 打开 `cpu_offload`，降低存档瞬间的峰值显存需求；
- 这与本例、`au_install.sh` 均无关——不建议为了"绕开这台机器当前的显存紧张"去改动框架的
  checkpoint 保存逻辑。

## tensorboardX 集成（2026-07-04）

### 背景：为什么不用 `torch.utils.tensorboard`

RLinf 框架的 [`rlinf/utils/metric_logger.py`](../../../rlinf/utils/metric_logger.py)
原来硬编码使用 `from torch.utils.tensorboard import SummaryWriter`。
`torch.utils.tensorboard` 依赖官方 `tensorboard` 包，而 `tensorboard` 又传递依赖
`tensorflow`，由此引出 **protobuf/tensorboard/tensorflow 三方版本冲突链**：

1. `tensorboard` 的 proto gencode 要求 `protobuf >= 6.31.1`；
2. 但 `protobuf 6.x` 与 `tensorflow` 依赖的 `ml_dtypes` 不兼容
   （`AttributeError: module 'ml_dtypes' has no attribute 'float4_e2m1fn'`）；
3. 降回 `protobuf 5.x` 又会触发
   `VersionError: gencode 6.31.1 runtime 5.29.6`。

这条链在 GCP Vertex AI 的 Docker 镜像里已被证实无解（见
[`b/gcp/demo4/guide_2.md`](../../../b/gcp/demo4/guide_2.md) 附录 Error 2→3→4），
GCP 多机训练配置不得不把 `logger_backends` 设为空来绕开。

**tensorboardX** 是纯 Python 实现的 TensorBoard 写入库，不依赖 TensorFlow，也不依赖官方
`tensorboard` 包，`SummaryWriter` API 与 `torch.utils.tensorboard` 兼容。用它可以在
不引入上述冲突链的前提下正常写 TensorBoard 事件文件。

### 文件改动

#### 1. `rlinf/utils/metric_logger.py`（改）

将 `_TensorboardLogger.__init__` 的 import 改成 **fallback** 模式——优先尝试
tensorboardX，没装则退回 `torch.utils.tensorboard`：

```python
class _TensorboardLogger:
    def __init__(self, log_path):
        try:
            from tensorboardX import SummaryWriter
        except ImportError:
            from torch.utils.tensorboard import SummaryWriter
        self.writer = SummaryWriter(log_path)
```

为什么用 fallback 而不是直接硬切：保留向后兼容——没装 tensorboardX 的环境（例如已有
`tensorflow` 且 protobuf 版本自洽的旧 venv）仍然能用原路径。

#### 2. `examples/au/pi/pushdoor_sft_pi05_au.yaml`（改）

```yaml
logger_backends: ["tensorboard"]    # 原来是 []
```

为什么：本地单机例子用 tensorboardX 不再有 protobuf 链问题，启用 tensorboard backend
以产生训练曲线。

#### 3. `/mnt/r/VENV/rlinf/`（venv 安装）

```bash
/mnt/r/VENV/rlinf/bin/pip install tensorboardX
```

安装了 `tensorboardX==2.6.5`。

### 运行时遇到的错误

启用 `logger_backends: ["tensorboard"]` 后首次运行 `run_train.sh`，遇到了**坑 4**中描述的
`libnppicc.so.12` 找不到的问题（根因是 `run_train.sh` 用 `PY=` 直接调用 python 二进制，
没有 source venv 的 `activate` 脚本，坑 3 写入 `activate` 的 `LD_LIBRARY_PATH` 不生效）。
这不是 tensorboardX 自身的问题，而是 `torchcodec` 的 CUDA 构建在解码 LeRobot 视频数据时
需要 NPP 库——详见坑 4 的记录。

### 运行命令

修复后，本地训练完整运行命令：

```bash
PY=/mnt/r/VENV/rlinf/bin/python \
OPENPI_SRC=/home/physical/SRC/Robot/aupi05/src \
RLINF_PUSHDOOR_MODEL=/mnt/r/CKPT/VLA/DEMO/pi05_pushdoor_tst1 \
BASE_CKPT=/mnt/r/CKPT/VLA/DEMO/pi05_pushdoor_tst1 \
LD_LIBRARY_PATH="/mnt/r/VENV/rlinf/lib/python3.11/site-packages/nvidia/npp/lib:${LD_LIBRARY_PATH:-}" \
bash examples/au/pi/run_train.sh
```

### 验证结果

训练 50/50 步全部完成，exit code 0。关键指标：

| 指标 | 值 |
|------|-----|
| 总步数 | 50/50 |
| 最终 loss | 0.115 |
| 最终 grad_norm | 0.94 |
| 训练时长 | ~5 分 36 秒 |
| checkpoint | `global_step_25/`、`global_step_50/`（含 EMA） |
| tensorboard 事件 | 300 条 scalar（6 指标 × 50 步） |
| 事件文件 | `_out/tensorboard/events.out.tfevents.*`（15528 字节） |

tensorboardX 正常工作的直接证据：事件文件可解析，包含 `train/loss`、`train/grad_norm`、
`train/learning_rate`、`train/param_norm`、`time/step`、`time/training` 共 6 个 tag，
每 tag 50 个数据点。

## tensorflow 版 tensorboard 恢复（2026-07-05）

### 背景

上一节用 tensorboardX 绕开了 protobuf/tensorboard/tensorflow 三方冲突链。随后在 venv
`/mnt/r/VENV/rlinf/` 中**重新安装了 tensorflow 和 tensorboard 的依赖**，使三方版本对齐，
protobuf `runtime_version` 问题消失。重装过程中 tensorboardX 被移除。

此时 `metric_logger.py` 的 fallback 机制自动生效：优先尝试 `from tensorboardX import
SummaryWriter`（未安装，`ImportError`），退回到 `from torch.utils.tensorboard import
SummaryWriter`（成功）。无需改动代码，tensorflow 版 tensorboard 直接可用。

### 脚本改动（与本轮一并完成）

#### 1. `examples/au/pi/run_train.sh`（改）

| 项 | 改前 | 改后 |
|----|------|------|
| 虚拟环境 | `PY="${PY:-/mnt/r/VENV/rlinf/bin/python}"`，直接调 python 二进制 | `VENV="${VENV:-/mnt/r/VENV/rlinf}"`，`source "${VENV}/bin/activate"` |
| 调用 python | `"$PY" examples/sft/train_vla_sft_au.py ...` | `python examples/sft/train_vla_sft_au.py ...` |
| NPP 库路径 | 手动拼 `NPP_LIB=...` + `export LD_LIBRARY_PATH=...` | 不再需要（`activate` 里已设好） |
| openpi 源码 | `OPENPI_SRC=.../openpi05/src` | `OPENPI_SRC=.../aupi05/src` |
| 基座权重 | `BASE_CKPT=.../pi05_base_pt_fp32` | `BASE_CKPT=.../DEMO/pi05_pushdoor_tst1` |
| norm_stats staging | 仅 symlink `model.safetensors` / `config.json` | 新增：若 `BASE_CKPT/rlinf/` 存在也 symlink 进来 |

为什么改成 `source activate`：之前用 `PY=` 直接调用 python 二进制跳过了 `activate`，导致
`activate` 里设置的 `LD_LIBRARY_PATH`（NPP 库路径，见坑 3）不生效，需要在脚本里手动补。
改成 `source activate` 后，venv 的所有环境变量自动就位，脚本更简洁也不容易漏。

#### 2. `examples/au/pi/run_norm_stats.sh`（改）

同上：`PY=` → `VENV=` + `source activate`，`"$PY"` → `python`，去掉手动 `NPP_LIB`。

#### 3. `examples/au/pi/README.md`（改）

前置条件中的路径同步更新（`openpi_venv` → `rlinf`，`openpi05` → `aupi05`，
`pi05_base_pt_fp32` → `DEMO/pi05_pushdoor_tst1`）。

### 运行命令

```bash
bash examples/au/pi/run_train.sh                           # 默认 VENV=/mnt/r/VENV/rlinf
VENV=/path/to/other/venv bash examples/au/pi/run_train.sh  # 换虚拟环境
```

### 遇到的错误

无。tensorflow/tensorboard 依赖修复后，训练全程零错误。

### 验证结果

| 指标 | 值 |
|------|-----|
| 总步数 | 50/50 |
| 最终 loss | 0.132 |
| 最终 grad_norm | 1.06 |
| 训练时长 | ~6 分 6 秒 |
| checkpoint | `global_step_25/`、`global_step_50/`（含 EMA） |
| tensorboard 事件 | 300 条 scalar（6 指标 × 50 步） |
| 事件文件 | `_out/tensorboard/events.out.tfevents.*`（15576 字节） |
| 后端 | `torch.utils.tensorboard`（tensorflow 版），通过 `metric_logger.py` fallback 路径 |

验证方式：用 `tensorboard.compat.proto.event_pb2.Event` 解析事件文件，确认 300 条 scalar、
6 个 tag（`train/loss`、`train/grad_norm`、`train/learning_rate`、`train/param_norm`、
`time/step`、`time/training`）全部可读。
