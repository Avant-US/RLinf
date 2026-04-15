#!/usr/bin/env python3
"""
使用 LeRobot 训练 StarVLA Qwen-GR00T (str_groot) on HuggingFaceVLA/libero.

基于 StarVLA/Qwen3VL-GR00T-Bridge-RT-1 预训练权重，在 libero 数据集上微调。

示例:
  # 小规模测试 (1 step, batch_size 1)
  python bt/str_groot_1/train_str_groot_libero.py --steps 1 --batch-size 1

  # 正式训练
  python bt/str_groot_1/train_str_groot_libero.py --steps 50000 --batch-size 8
  python bt/str_groot_1/train_str_groot_libero.py   --episodes 0 1 3 4 11 12 13 15 16 18 24 25 27 28 29 30 32 35 36 37 --state-indices 0 1 2 3 4 5 7  --steps 100   --batch-size 8 --num-workers 8
"""

from __future__ import annotations

import argparse
import logging
from datetime import datetime
from pathlib import Path

from lerobot.configs.default import DatasetConfig, WandBConfig
from lerobot.configs.train import TrainPipelineConfig
from lerobot.policies.str_groot.configuration_str_groot import StrGrootConfig
from lerobot.scripts.lerobot_train import train

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    force=True,
)
LOGGER = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Train StarVLA Qwen-GR00T (str_groot) on HuggingFaceVLA/libero"
    )

    # --- Dataset ---
    p.add_argument("--dataset-repo", default="HuggingFaceVLA/libero")
    p.add_argument("--dataset-root", default=None, help="本地数据缓存路径")
    p.add_argument(
        "--episodes",
        type=int,
        nargs="*",
        default=None,
        help="指定训练 episode 索引 (不指定则使用全部)",
    )

    # --- Training ---
    p.add_argument("--steps", type=int, default=50000)
    p.add_argument("--batch-size", type=int, default=8)
    p.add_argument("--num-workers", type=int, default=4)
    p.add_argument("--log-freq", type=int, default=50)
    p.add_argument("--save-freq", type=int, default=5000)
    p.add_argument("--output-root", default="outputs/bt/str_groot_1")
    p.add_argument("--job-name", default="str_groot_libero")

    # --- StarVLA Model ---
    p.add_argument(
        "--starvla-checkpoint",
        default="StarVLA/Qwen3VL-GR00T-Bridge-RT-1",
        help="StarVLA 预训练 checkpoint (HF repo id 或本地路径)",
    )
    # p.add_argument(
    #     "--base-vlm",
    #     default="Qwen/Qwen3-VL-4B-Instruct",
    #     help="底层 VLM 模型名",
    # )
    # p.add_argument("--action-dim", type=int, default=7)
    # p.add_argument("--state-dim", type=int, default=7)
    p.add_argument(
        "--state-indices",
        type=int,
        nargs="*",
        default=None,
        help="从数据集 state 中选取的维度索引 (如 0 1 2 3 4 5 7 跳过 pad 维度 6)",
    )
    p.add_argument("--future-action-window-size", type=int, default=7)
    p.add_argument("--action-hidden-dim", type=int, default=1024)

    # --- Tuning flags ---
    p.add_argument("--freeze-vlm", action="store_true", default=False)
    p.add_argument("--use-bf16", action="store_true", default=True)
    p.add_argument("--no-use-bf16", dest="use_bf16", action="store_false")

    # --- Optimizer ---
    p.add_argument("--lr", type=float, default=1e-4)

    # --- Device ---
    p.add_argument("--device", default="cuda:0", help="cuda / cpu / cuda:0")

    # --- WandB ---
    p.add_argument("--wandb", action="store_true", default=False)
    p.add_argument("--wandb-project", default="str_groot_libero")
    p.add_argument("--wandb-entity", default=None)

    p.add_argument("--save-checkpoint", action="store_true", default=True)
    p.add_argument("--no-save-checkpoint", dest="save_checkpoint", action="store_false")
    p.add_argument("--dry-run", action="store_true", help="只构建配置不训练")

    return p.parse_args()


def build_train_config(args: argparse.Namespace, output_dir: Path) -> TrainPipelineConfig:
    policy_cfg = StrGrootConfig(
        push_to_hub=False,
        repo_id=None,
        # VLM
        # base_vlm=args.base_vlm,
        attn_implementation="flash_attention_2",
        # Action model
        # action_dim=args.action_dim,
        # state_dim=args.state_dim,
        state_indices=tuple(args.state_indices) if args.state_indices else None,
        action_hidden_dim=args.action_hidden_dim,
        future_action_window_size=args.future_action_window_size,
        # Training
        freeze_vlm=args.freeze_vlm,
        use_bf16=args.use_bf16,
        starvla_checkpoint=args.starvla_checkpoint,
        # Optimizer
        optimizer_lr=args.lr,
        # Device
        device=args.device,
    )

    dataset_cfg = DatasetConfig(
        repo_id=args.dataset_repo,
        root=args.dataset_root,
        episodes=args.episodes,
        streaming=False,
    )

    save_freq = max(1, min(args.save_freq, args.steps))

    return TrainPipelineConfig(
        dataset=dataset_cfg,
        policy=policy_cfg,
        output_dir=output_dir,
        job_name=args.job_name,
        batch_size=args.batch_size,
        steps=args.steps,
        num_workers=args.num_workers,
        eval_freq=0,
        log_freq=args.log_freq,
        save_checkpoint=args.save_checkpoint,
        save_freq=save_freq,
        wandb=WandBConfig(
            enable=args.wandb,
            project=args.wandb_project,
            entity=args.wandb_entity,
        ),
    )


def main() -> None:
    args = parse_args()

    ckpt_path = Path(args.starvla_checkpoint)
    if ckpt_path.is_absolute() and not ckpt_path.exists():
        ckpt_path.mkdir(parents=True, exist_ok=True)
        LOGGER.info("starvla_checkpoint 路径不存在，已创建目录: %s", ckpt_path)

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_root) / run_id
    cfg = build_train_config(args, output_dir)

    LOGGER.info("训练输出目录: %s", output_dir)
    LOGGER.info(
        "开始训练: repo=%s, steps=%d, batch_size=%d, starvla_ckpt=%s",
        args.dataset_repo,
        args.steps,
        args.batch_size,
        args.starvla_checkpoint,
    )

    if args.dry_run:
        LOGGER.info("dry-run 模式：仅构建配置，不执行训练。")
        LOGGER.info("Policy config: %s", cfg.policy)
        return

    train(cfg)
    LOGGER.info("训练完成。输出目录: %s", output_dir)


if __name__ == "__main__":
    main()
