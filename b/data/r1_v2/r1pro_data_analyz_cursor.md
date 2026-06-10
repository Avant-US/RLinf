# Parquet文件内容分析
_Exported on 6/4/2026 at 20:06:02 GMT+8 from Cursor (3.6.31)_

---

**User**

分析一下 /mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis/data/chunk-000/episode_000063.parquet 里装了什么?

---

**Cursor**

正在用 Python 检查该 parquet 文件的结构与内容。


## 文件概览

这是 **LeRobot v2.1** 格式的单条轨迹（episode）数据文件，对应数据集 `r1_pro_data_convert_chassis` 中的 **第 63 条 episode**。

| 属性 | 值 |
|------|-----|
| 文件大小 | **~777 MB**（图像以 PNG 字节内嵌，体积大） |
| 行数 | **1052 帧**（每行 = 一个控制步） |
| 时长 | **~75.07 s**（`timestamp` 0 → 75.07） |
| 采样率 | **14 Hz**（与 `meta/info.json` 中 `fps: 14` 一致） |
| 机器人类型 | `r1_pro`（含底盘/chassis） |

任务描述（来自 `meta/episodes.jsonl`）：

> *Open the door with a downward-press handle, go through it, and enter the room.*

---

## 表里装了什么（10 列）

每一行是一条时间步，列结构如下：

```mermaid
flowchart LR
  subgraph per_frame [每一帧 parquet 行]
    IMG[三相机 RGB<br/>head / left_wrist / right_wrist]
    ST[state 23维]
    AC[actions 23维]
    META[timestamp / frame_index / episode_index / index / task_index]
  end
  IMG --> ST
  ST --> AC
```

### 1. 三相机图像（占绝大部分体积）

| 列名 | 存储形式 | 分辨率 (W×H) | 说明 |
|------|----------|--------------|------|
| `head_rgb` | `struct{bytes, path}` | **640×360** RGB | 头部相机 |
| `left_wrist_rgb` | 同上 | **640×480** RGB | 左腕相机 |
| `right_wrist_rgb` | 同上 | **640×480** RGB | 右腕相机 |

- `bytes`：该帧 **PNG 编码** 的原始字节（解码后即为 RGB 图）
- `path`：帧文件名占位，如 `frame_000000.png`、`frame_001051.png`（最后一帧）
- 平均每帧 `head_rgb` 约 **214 KB** PNG；三相机合计使整文件约 777 MB

数据集 `meta/info.json` 里写的 shape 是 `[H, W, C]`（如 head `360×640×3`），与 PIL 读出的 `(640, 360)` 一致。

另：`info.json` 里还有 `video_path` 指向 `videos_backup/.../*.mp4`，但 **本 parquet 已内嵌逐帧图像**，不依赖外部 mp4 即可读图。

### 2. 本体状态与动作（各 23 维 float32）

| 列名 | shape | 含义 |
|------|-------|------|
| `state` | `(23,)` | 当前 proprio / 底盘状态 |
| `actions` | `(23,)` | 该步要执行的动作（与 state 同布局） |

23 维语义由 `meta/modality.json` 定义（**双臂 + 夹爪 + 底盘**）：

| 索引 | 语义 | 维度 |
|------|------|------|
| 0–6 | 左臂 `left_arm` | 7 |
| 7–13 | 右臂 `right_arm` | 7 |
| 14 | 左夹爪 `left_gripper` | 1 |
| 15 | 右夹爪 `right_gripper` | 1 |
| 16–19 | 底盘位姿 `chassis_pose` | 4 |
| 20–22 | 底盘速度 `chassis_velocity` | 3 |

本 episode 数值范围大致：`actions` / `state` 全局约 **[-1.5, 102.2]**（个别关节/底盘分量变化较大，符合移动操作轨迹）。

FastWAM 训练配置（`configs/data/r1_pro_chassis.yaml`）正是用这 3 路 `*_rgb` + `actions` + `state`，23 维 action/state。

### 3. 索引与时间元数据

| 列名 | 本文件取值 | 含义 |
|------|------------|------|
| `timestamp` | 0.0 → 75.07 |  episode 内相对时间（秒） |
| `frame_index` | 0 → 1051 | episode 内帧号 |
| `episode_index` | **恒为 63** | 轨迹 ID |
| `index` | 60871 → 61922 | **全数据集全局帧索引**（前面 0–62 条共 60871 帧） |
| `task_index` | **恒为 0** | 对应 `tasks.jsonl` 中唯一任务 |

---

## 在数据集里的位置

- 路径模式：`data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet`
- 本文件：`chunk-000` / `episode_000063` → 全库 **最后一条**（`info.json` 写 `total_episodes: 63` 通常表示 0–63 共 64 条，或命名与计数方式需与 `episodes.jsonl` 行数对照）
- 全库约 **60992 帧**；本条 **1052 帧** 约占 1.7%

---

## 小结

`episode_000063.parquet` **不是**抽象特征表，而是 **一条完整机器人演示轨迹的时间序列**：

1. **1052 步 × 三相机 PNG 图像**（内嵌 bytes，体积最大）  
2. **每步 23 维 state + 23 维 action**（双臂、夹爪、底盘 pose/velocity）  
3. **LeRobot 标准索引列**（时间戳、帧号、episode/全局 index、任务 ID）

若只做 SFT/策略学习、不需要在 parquet 里反复解码 PNG，可关注同目录数据集是否还有 `videos_backup/` 下的 mp4；当前这份 parquet 则是 **“图像+状态+动作”一体化、可单文件自包含读取** 的 LeRobot 落盘格式。

---

**User**

请把  /mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis/data/chunk-000/ 里所有的 parquet 文件合并成一个大 parquet 文件, 然后对里面每个字段的数据分布做详细分析.

---

**Cursor**

先查看 chunk-000 下有多少 parquet 文件及总大小，再合并并做各字段分布分析。


正在编写合并与分析脚本；64 个文件约 43GB，合并需较长时间。


全表载入内存可能 OOM；改为分批流式分析。


合并已完成（43GB）。先前因 `head` 管道中断导致分析未完成，正在单独运行分析。


## 合并结果

已将 `chunk-000/` 下 **64** 个 `episode_*.parquet` 合并为单文件：

| 项 | 值 |
|----|-----|
| **输出文件** | `/mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis/data/chunk-000/merged_all_episodes.parquet` |
| **大小** | **~42.2 GB**（磁盘显示约 45 GB） |
| **总行数** | **61,923** 帧（64 条 episode，index 0–61922 连续无断档） |
| **列** | 10 列（与单 episode 相同 LeRobot schema） |

**分析报告**（完整逐维表格）：
- Markdown：`.../merged_all_episodes.analysis.md`（已复制到 `b/test/r1_pro_chunk000_merged_analysis.md`）
- JSON：`.../merged_all_episodes.analysis.json`
- 复现脚本：`b/test/merge_and_analyze_r1_pro_parquet.py`

```bash
# 仅重新分析（不合并）
/home/Luogang/miniforge3/bin/python b/test/merge_and_analyze_r1_pro_parquet.py --skip-merge
```

---

## 各字段分布摘要

### 索引与时间

| 字段 | 要点 |
|------|------|
| `timestamp` | 0 → 81.5 s；均值 35.3 s（跨 episode 拼接后的分布，非单条时长） |
| `frame_index` | 每条 episode 内 0…(L−1)；全局 min=0, max=1141 |
| `episode_index` | 0–63 共 64 条；**每集帧数**：均值 **968**，中位 **984**，范围 **[10, 1142]** |
| `index` | 全局 0–61922，**严格单调 +1**，无空洞 |
| `task_index` | **全部为 0**（单一语言任务） |

**异常 episode**：`episode_index=35` 仅 **10 帧**（约 0.7 s @14Hz），其余多在 850–1142 帧；训练时建议排查或过滤。

`meta/info.json` 写的是 `total_frames: 60992`，与合并后 **61923** 差 **931** 帧，元数据可能未更新。

---

### `state` / `actions`（23 维）

语义见 `modality.json`：左/右臂(7+7)、夹爪(1+1)、底盘位姿(4)、底盘速度(3)。

| 维度段 | 含义 | 分布特征 |
|--------|------|----------|
| **0–13** | 双臂关节 | 大致在 [-1.6, 1.05]；大量值接近 0（p50≈0） |
| **14–15** | 左/右夹爪 | **主导方差**：state 均值 ~77/87，最大 ~102；actions 大量 **quantized 到 0 或 90**（p50=p95=90） |
| **16–19** | 底盘位姿 | **几乎常数**：state 中 dim16≈0.90, dim17≈−1.50, dim18≈−0.70；actions 中 dim16–19 **完全固定**（0.9, −1.5, −0.7, 0） |
| **20–22** | 底盘速度 | state 有小幅变化；**actions 的 dim22 全为 0**，dim20–21 多数为 0 |

**`actions` vs `state`**（逐维平均绝对差）：
- 手臂关节 (0–13)：差值很小（~0.001–0.014），接近 **行为克隆 / 下一步目标≈当前状态**
- 夹爪 (14–15)：差最大（**~3.1–3.4**），actions 与 state 明显不同（离散控制）
- 底盘速度 (20–22)：actions 常为零而 state 非零（**~0.04**），说明动作通道里速度指令稀疏或未填

---

### 三相机图像（PNG 内嵌）

| 字段 | 分辨率 | PNG 大小（均值） | 全库 PNG 合计 |
|------|--------|------------------|---------------|
| `head_rgb` | 640×360 | **197.5 KB** | **11.66 GB** |
| `left_wrist_rgb` | 640×480 | **257.0 KB** | **15.18 GB** |
| `right_wrist_rgb` | 640×480 | **261.1 KB** | **15.42 GB** |

像素统计（随机解码 **2000** 帧/相机）：
- **head**：RGB 均值 ≈ [0.49, 0.29, 0.22]，偏暖、略暗场景
- **腕部**：均值更高 (~0.39–0.41)，方差更大（手眼近距离、运动模糊）
- 像素值范围 **[0, 1]**（归一化后）

三相机 PNG 合计约 **42.3 GB**，与合并文件体积一致——**体积几乎全在图像列**。

---

## 结构关系图

```mermaid
flowchart TB
  subgraph merged [merged_all_episodes.parquet 61923 rows]
    E[episode_index 0-63]
    T[timestamp / frame_index / index]
    I[head + L/R wrist PNG bytes ~42GB]
    S[state 23D]
    A[actions 23D]
  end
  E --> T
  E --> I
  E --> S
  S -.->|手臂近似| A
  S -->|夹爪/底盘差异大| A
```

---

## 使用建议

1. **训练**：若只需向量，可用 `pyarrow` 按列读取 `state`/`actions`，避免加载图像列。
2. **数据清洗**：重点检查 **episode 35（10 帧）** 与 **夹爪/actions 离散化** 是否符合预期。
3. **存储**：若磁盘紧张，可保留分 episode 文件，合并文件主要用于全局统计；需要时可 `--skip-merge` 只跑分析。

更细的逐维 p5/p50/p95 表见 `merged_all_episodes.analysis.md`。若需要，我可以再出一张按 modality 分组（臂/夹爪/底盘）的可视化或 histogram 脚本。

---

**User**

<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: aborted
task_id: 708769
title: Restart merge and show first lines of output
output_path: /home/Luogang/.cursor/projects/home-Luogang-SRC-RL-RLinf/terminals/708769.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

**任务已完成。**

`chunk-000/` 下 64 个 episode parquet 已合并为：

`/mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis/data/chunk-000/merged_all_episodes.parquet`（约 **42 GB**，**61,923** 行）

各字段分布分析已写入同目录下的 `merged_all_episodes.analysis.md` / `.json`，副本在 `b/test/r1_pro_chunk000_merged_analysis.md`。复现脚本：`b/test/merge_and_analyze_r1_pro_parquet.py`（加 `--skip-merge` 可只重跑分析）。

**要点**：体积主要在三个相机的内嵌 PNG；`state`/`actions` 为 23 维；`task_index` 全为 0；`episode 35` 仅 10 帧，建议训练前排查。

---

**User**

我把刚才合并成的那个大 parquet 文件移动到 `/mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis/data/` 了, 请基于该`merged_all_episodes.parquet`对每个episode的第一个frame进行数据分析, 特别是'state'和'actions'字段.

---

**Cursor**

基于新路径的合并 parquet，提取每个 episode 的首帧并分析 `state` 与 `actions`。


已基于新路径 `.../data/merged_all_episodes.parquet`，对 **64 个 episode 的首帧**（`frame_index == 0`）完成 `state` / `actions` 分析。

## 输出文件

| 文件 | 说明 |
|------|------|
| `b/test/r1_pro_first_frame_per_episode.md` | 逐维统计表 + 64 条首帧向量 |
| `b/test/r1_pro_first_frame_per_episode.json` | 机器可读，含每条 episode 的 state/actions |

---

## 核心结论

### 1. 首帧 `actions` 与 `state` 从不完全相同

- **0/64** 条 episode 在 atol=1e-5 下完全一致。
- L2\(\|a-s\|\)：**均值 5.09**，范围 **[2.99, 119.39]**。
- 差异几乎全来自 **夹爪维 (14–15)**；手臂与底盘可忽略：

| 维度段 | L2 均值 | 说明 |
|--------|---------|------|
| 0–13 手臂 | **~0.0028** | 首帧 action ≈ 当前 state（微调） |
| 14–15 夹爪 | **~5.09** | 主要差异来源 |
| 16–22 底盘 | **~0.0017** | 位姿/速度接近，actions 多为离散常数 |

### 2. 夹爪：固定 action，变化的 state

| dim | state（首帧跨 64 ep） | actions（首帧） |
|-----|----------------------|-----------------|
| 14 左夹爪 | mean **90.72**, std **10.76**, range **[5.4, 94.4]** | **恒为 90.0**（std=0） |
| 15 右夹爪 | mean **91.16**, std **10.76**, range **[5.8, 94.2]** | **恒为 90.0** |

含义：多数 episode 起始 **state 夹爪已闭合 (~91–92)**，首帧 **action 指令为 90**（差约 1–2）；**episode 35** 例外：state 仅 **~5.4 / 5.8**（几乎张开），action 仍为 **90**，单维差 ~84 → **L2=119**。

### 3. 手臂首帧：跨 episode 有姿态差异，但 s≈a

- 关节 0–13：`|a−s|` 均值 **< 0.002**，corr **≈ 1.0**。
- 个别 episode 初始姿态较大（如 ep5：左臂 dim2≈**−0.86**，dim4≈**1.04**），首帧 action 仍与 state 对齐。
- 跨 episode 标准差最大在 **dim2、dim4**（肩/elbow 类），说明不同演示起始构型不同。

### 4. 底盘首帧

| dim | state | actions |
|-----|-------|---------|
| 16–18 位姿 | ~0.90, −1.50, −0.70（ep 间 std < 0.001） | **固定** 0.9, −1.5, −0.7 |
| 19–22 | 接近 0 | **恒 0** |

首帧几乎静止；actions 为离散 setpoint，state 为连续观测。

---

## 异常 episode

| episode | L2(\|a−s\|) | 说明 |
|---------|-------------|------|
| **35** | **119.39** | 仅 10 帧；夹爪 state≈5.4/5.8，action=90 |
| 45, 44, 40… | 3.5–5.8 | 夹爪 state 与 90 差约 2–4（多数 ep 在 91–94） |

---

## 训练含义（简要）

1. **首帧 BC**：手臂可近似 `action ≈ state`；夹爪需单独建模（state 连续、action 量化到 90）。
2. **数据清洗**：**episode 35** 建议重点检查（短轨迹 + 首帧夹爪 state/action 严重不一致）。
3. **归一化**：夹爪 state 跨度大（5–94），与手臂 [-1.6, 1.04] 量级差很多，宜按 `modality.json` 分块归一化。

完整逐维表与 64 条首帧 23 维向量见 `b/test/r1_pro_first_frame_per_episode.md`。

---

**User**

请补充对每个episode的第一个frame的state和action的p10, p25, p50, p75, p90, p99的值. 同时补充,统计state全为0的episode占百分多少, action全为0的episode占百分多少, state和action同时全为0的episode占百分多少. 补充到 @RLinf/b/test/r1_pro_first_frame_per_episode.md 后面.

---

**Cursor**

计算各 episode 首帧 state/action 的分位数，并统计全零 episode 占比，追加到报告末尾。


已追加到 `b/test/r1_pro_first_frame_per_episode.md` 末尾，并同步更新了 `r1_pro_first_frame_per_episode.json` 中的 `supplement` 字段。

## 补充内容说明

### 1. 各 Episode 首帧分位数表（p10 / p25 / p50 / p75 / p90 / p99）

对每条 episode 的**首帧 23 维向量内部**求分位数（不是跨 episode、也不是逐维）：

- **state**：左半列 p10–p99  
- **actions**：右半列 p10–p99  

典型模式：
- 多数 episode 的 state **p90≈0.72、p99≈92**（夹爪维拉高尾部）
- actions **p90≈0.72、p99=90**（夹爪 action 恒为 90）
- **episode 35** 异常：state **p99≈5.68**（夹爪 state≈5.4，未闭合），其余 episode 的 state p99 多在 92 附近

### 2. 全零 Episode 占比（64 条首帧）

| 条件 | 数量 | 占比 |
|------|------|------|
| state 全为 0 | 0 | **0.00%** |
| actions 全为 0 | 0 | **0.00%** |
| state 与 actions 同时全为 0 | 0 | **0.00%** |

（`|x|≤1e-6` 视为 0 时结果相同。）

首帧至少含底盘常数、夹爪 action=90 等，**不存在** 23 维全零的 episode。

---

**User**

 统计state(排除底盘、夹爪)全为0的episode占百分多少, action(排除底盘、夹爪)全为0的episode占百分多少, state(排除底盘、夹爪)和action(排除底盘、夹爪)同时全为0的episode占百分多少. 补充到 @RLinf/b/test/r1_pro_first_frame_per_episode.md 后面.

---

**Cursor**

已追加到 `b/test/r1_pro_first_frame_per_episode.md` 末尾，并写入 JSON 的 `supplement.arm_only_all_zero`。

**统计范围**：首帧 `frame_index==0`，仅 **dim 0–13**（左/右臂各 7 维），**不含**夹爪 14–15、底盘 16–22。

| 条件 | Episode 数 | 占比 |
|------|------------|------|
| state（手臂）全为 0 | 0 | **0.00%** |
| actions（手臂）全为 0 | 0 | **0.00%** |
| state 与 actions（手臂）同时全为 0 | 0 | **0.00%** |

（`|x|≤1e-6` 判定结果相同；无满足条件的 episode 列表。）

即使去掉夹爪和底盘，首帧手臂关节也**没有**整条 episode 全零的情况（含 episode 35，其手臂维仍有 ~1e-3 量级非零值）。

---

**User**

 统计state(仅底盘速度)全为0的episode占百分多少, actions(仅底盘速度)全为0的episode占百分多少, state(仅底盘速度)和actions(仅底盘速度)同时全为0的episode占百分多少. 补充到 @RLinf/b/test/r1_pro_first_frame_per_episode.md 后面.

---

**Cursor**

已追加到 `b/test/r1_pro_first_frame_per_episode.md` 末尾，并更新 JSON 的 `supplement.chassis_velocity_only_all_zero`。

**统计范围**：首帧，仅 **dim 20–22**（`chassis_velocity`，3 维）。

| 条件 | Episode 数 | 占比 |
|------|------------|------|
| state（底盘速度）全为 0 | **48** | **75.00%** |
| actions（底盘速度）全为 0 | **64** | **100.00%** |
| state 与 actions（底盘速度）同时全为 0 | **48** | **75.00%** |

（`|x|≤1e-6` 判定与上表一致。）

**说明**：
- 所有 episode 首帧 **actions 的底盘速度均为 0**（与全库分析一致）。
- **48/64** 条首帧 **state 底盘速度也为 0**；其余 16 条 state 有微小非零（多为 0.001 量级，如 ep1）。
- 未同时全零的 16 条：state 非零、actions 仍为零 → 即 ep 1, 2, 3, 11, 12, 13, 16, 19, 21, 22, 26, 29, 30, 34, 39, 41（共 16 个）。

---

**User**

我把原 `/mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis/data/merged_all_episodes.parquet` 移动到`/mnt/r/share/zwy/datasets/r1_pro_data_v2/all_cnv_chsis.parquet`了, 请抽取episode 1, 11, 19, 26, 34, 41首帧的底盘速度state看一下.

---

**Cursor**

已从 `/mnt/r/share/zwy/datasets/r1_pro_data_v2/all_cnv_chsis.parquet` 读取各 episode **首帧**（`frame_index=0`）的 **底盘速度 state（dim 20–22）**，对应 `modality.json` 的 `chassis_velocity`。

| episode | global_index | state 底盘速度 [dim20, dim21, dim22] | actions 底盘速度 |
|---------|--------------|-------------------------------------|------------------|
| **1** | 1135 | **[+0.001, +0.001, +0.001]** | [0, 0, 0] |
| **11** | 11422 | **[+0.002, 0, +0.001]** | [0, 0, 0] |
| **19** | 19751 | [0, 0, **+0.001**] | [0, 0, 0] |
| **26** | 26635 | [0, 0, **+0.001**] | [0, 0, 0] |
| **34** | 34463 | [0, 0, **+0.001**] | [0, 0, 0] |
| **41** | 39933 | [0, 0, **+0.001**] | [0, 0, 0] |

**结论**：
- 这 6 条首帧 **actions 底盘速度均为 0**（与全库 100% 一致）。
- **state 并非“明显运动”**：数值多为 **0.001** 或 **0.002**（float32 精确值），只有 ep1 三维都是 0.001，ep11 的 dim20 为 0.002。
- **5/6 条**（19、26、34、41 及同类）仅 **dim22 = 0.001**，dim20/21 为 0 → 在「精确等于 0」判定下仍算 state 非全零，但量级极小，更像是**量化/记录噪声或静止时的残差**，而非真实底盘在动。
- 若用 `|x| ≤ 1e-6` 判零，ep19/26/34/41 仍因 dim22=0.001 而不算全零；若用 `|x| ≤ 0.001` 容差，则 ep19/26/34/41 的 state 可视为全零，ep1/11 仍略超。

