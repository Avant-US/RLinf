# Transform 增强结果 MP4 调试导出（按完整 Episode）

在 `FastWAMProcessor.preprocess` 中，当某个 **episode** 首次被采样到时，会从原始 MP4 读取该 episode 的**全部帧**，施加与训练相同的 per-camera transform，再按 episode 写出 MP4（每个相机一个文件）。

与滑窗训练样本 `idx` 无关；输出命名使用 LeRobot 的 `episode_index`。

## 环境变量

| 变量 | 默认 | 说明 |
|------|------|------|
| `FASTWAM_DUMP_TRANSFORM_MP4` | 未设置 | 设为 `1` 才写文件 |
| `RLINF_TRANSFORM_TEST_DIR` | 本目录 | MP4 输出路径 |
| `FASTWAM_DUMP_TRANSFORM_MAX` | `10` | 每个进程最多 dump 多少个 **episode** |
| `FASTWAM_DUMP_EPISODE_INDICES` | 空 | 可选，逗号分隔的 episode 白名单，如 `0,5,42` |
| `FASTWAM_DUMP_TRANSFORM_FPS` | `14` | 视频帧率 |

## 快速验证

```bash
source /mnt/r/VENV/rlinf_venv/bin/activate
cd /home/Luogang/SRC/RL/RLinf
export PYTHONPATH=/home/Luogang/SRC/RL/RLinf:/home/Luogang/SRC/Robot/FastWAM/src:${PYTHONPATH:-}

export RLINF_TRANSFORM_TEST_DIR=/home/Luogang/SRC/RL/RLinf/b/test/trnsf_tst

python b/test/trnsf_tst/dump_transform_smoke.py --num_episodes 2
```

`dump_transform_smoke.py` 会自动：

- 设置 `EMBODIED_PATH`、`R1PRO_DATA`、`FASTWAM_ROOT` 等 Hydra 所需环境变量
- 开启 `FASTWAM_DUMP_TRANSFORM_MP4=1`
- 顺序读取 dataset 样本，直到 dump 满 `num_episodes` 个**不同 episode**（或遍历 `--max_samples` 上限）

也可手动指定 episode：

```bash
export FASTWAM_DUMP_EPISODE_INDICES=0,1
python b/test/trnsf_tst/dump_transform_smoke.py --max_samples 100
```

输出示例：`episode_000000_head_rgb.mp4`、`episode_000000_left_wrist_rgb.mp4` 等（`episode_{episode_index:06d}_{camera_key}.mp4`）。

## 注意事项

- 未设置 `FASTWAM_DUMP_TRANSFORM_MP4=1` 时**零开销**，不影响训练。
- 调试时建议 `data.num_workers=0`，避免 DataLoader 多 worker 重复导出。
- 每个 episode 在同一进程内只 dump 一次（即使该 episode 对应多个滑窗样本）。
- 训练时请勿长期开启；每个 worker 进程独立计数，易写满磁盘。

## 训练时临时开启（可选）

```bash
export FASTWAM_DUMP_TRANSFORM_MP4=1
export FASTWAM_DUMP_TRANSFORM_MAX=2
export FASTWAM_DUMP_EPISODE_INDICES=0,1
bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  data.num_workers=0 \
  runner.max_steps=1 \
  ...
```
