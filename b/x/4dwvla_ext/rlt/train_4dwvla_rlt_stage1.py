#!/usr/bin/env python3
"""RLT Stage 1 training entry point for 4DWVLA.

Usage (inside GPU container, venv activated):
    cd /workspace/RLinf
    python b/x/4dwvla_ext/rlt/train_4dwvla_rlt_stage1.py \
        --config b/x/4dwvla_ext/rlt/configs/rlt_stage1_franka_plug.yaml \
        --max_steps 10 \
        --dataset_root /home/nvidia/data/plug_into_socket_lrb_4D_8sml

Reference: 4WVLA/src/lerobot/scripts/lerobot_train.py
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import os
import sys
import time
from pathlib import Path

ext_dir = Path(__file__).resolve().parent
sys.path.insert(0, str(ext_dir))
sys.path.insert(0, str(ext_dir.parent))

import torch
import yaml

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    force=True,
)
logger = logging.getLogger("rlt-stage1")


def load_rlt_config(args):
    from rlt_config import RLTStage1Config
    cfg = RLTStage1Config.from_yaml(args.config)
    if args.max_steps is not None:
        cfg.max_steps = args.max_steps
    if args.dataset_root is not None:
        cfg.dataset_root = args.dataset_root
    if args.save_freq is not None:
        cfg.save_freq = args.save_freq
    if args.output_dir is not None:
        cfg.output_dir = args.output_dir
    if args.log_freq is not None:
        cfg.log_freq = args.log_freq
    cfg.resolve_paths()
    cfg.apply_profile()
    return cfg


def load_train_pipeline_config(ckpt_path: str, dataset_root: str | None, repo_id: str):
    """Load TrainPipelineConfig from checkpoint's train_config.json."""
    import lerobot.policies.internvla_a1_5.configuration_internvla_a1_5
    import lerobot.policies.internvla_a1_5.transform_internvla_a1_5
    from lerobot.configs.train import TrainPipelineConfig
    import draccus

    if os.environ.get("HF_TOKEN"):
        os.environ.setdefault("HUGGING_FACE_HUB_TOKEN", os.environ["HF_TOKEN"])
    os.environ.setdefault("HF_HUB_OFFLINE", "1")
    os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")

    train_config_path = os.path.join(ckpt_path, "train_config.json")
    with open(train_config_path) as f:
        raw = json.load(f)

    # Resolve HF repo ids to local cache paths for offline operation
    hf_hub_cache = os.path.join(os.environ.get("HF_HOME", os.path.expanduser("~/.cache/huggingface")), "hub")
    for tf in raw.get("dataset", {}).get("data_transforms", {}).get("inputs", []):
        for key in ["action_tokenizer_name", "pretrained_model_name_or_path", "qwen35_model_name"]:
            if key in tf and "/" in str(tf[key]):
                hf_repo = tf[key]
                cache_dir = os.path.join(hf_hub_cache, f"models--{hf_repo.replace('/', '--')}", "snapshots")
                if os.path.isdir(cache_dir):
                    snaps = sorted(os.listdir(cache_dir))
                    if snaps:
                        local_path = os.path.join(cache_dir, snaps[-1])
                        tf[key] = local_path
                        logger.info("Resolved %s → %s", hf_repo, local_path)

    cfg = draccus.decode(TrainPipelineConfig, raw)

    cfg.policy.pretrained_path = ckpt_path
    cfg.policy.device = "cpu"
    if dataset_root:
        from lerobot.datasets.lerobot_dataset import HF_LEROBOT_HOME
        link = HF_LEROBOT_HOME / repo_id
        real = Path(dataset_root) / repo_id if not Path(dataset_root, "meta", "info.json").exists() else Path(dataset_root)
        if real.exists():
            HF_LEROBOT_HOME.mkdir(parents=True, exist_ok=True)
            # A previous container run can leave a dangling symlink or a link
            # pointing at a path that only exists inside the container. Replace
            # that link, but never remove a real dataset directory implicitly.
            if link.is_symlink():
                current_target = link.resolve(strict=False)
                if current_target != real.resolve():
                    link.unlink()
                    logger.info("Replaced stale dataset symlink %s → %s", link, real)
            elif link.exists() and link.resolve() != real.resolve():
                raise FileExistsError(
                    f"Dataset cache path exists and is not the requested dataset: "
                    f"{link} (requested {real}). Remove it explicitly before retrying."
                )
            if not link.exists():
                link.symlink_to(real)
                logger.info("Symlinked %s → %s", link, real)
        cfg.dataset.root = None
    if repo_id:
        cfg.dataset.repo_id = repo_id
    cfg.dataset.use_external_stats = False

    return cfg


def build_model(train_cfg, rlt_cfg):
    """Build 4DWVLA policy + RLT wrapper."""
    from rlt_stage1_wrapper import RLTStage1TrainingWrapper
    from lerobot.policies.factory import make_policy

    train_cfg.policy.action_loss_only = rlt_cfg.action_loss_only
    train_cfg.policy.enable_vqa_loss = False

    policy = make_policy(cfg=train_cfg.policy)
    logger.info("Loaded base policy from %s", train_cfg.policy.pretrained_path)

    if rlt_cfg.freeze_vision_encoder:
        for name, param in policy.named_parameters():
            if "vision" in name or "visual" in name:
                param.requires_grad = False
        logger.info("Frozen: vision encoder")

    if rlt_cfg.train_expert_only:
        trainable_keywords = [
            "action_expert", "kpt_expert", "action_in_proj", "action_out_proj",
            "state_proj", "kpt_state_proj", "action_time_mlp", "learnable_tokens",
            "track_encoder",
        ]
        for name, param in policy.named_parameters():
            if not any(kw in name for kw in trainable_keywords):
                param.requires_grad = False
        logger.info("Profile B: only experts + projections trainable")

    if rlt_cfg.freeze_keypoint_modules:
        for name, param in policy.named_parameters():
            if any(kw in name for kw in ["kpt_expert", "kpt_state_proj", "track_encoder", "keypoint"]):
                param.requires_grad = False
        logger.info("Frozen: keypoint modules")

    wrapper = RLTStage1TrainingWrapper(policy, rlt_cfg)
    if rlt_cfg.vla_inference_mode:
        wrapper.vla_inference_mode = True
        logger.info("VLA inference mode: ON (saves VRAM, no VLA gradient)")

    vla_trainable = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    rlt_trainable = sum(p.numel() for p in wrapper.rlt_module.parameters())
    total = sum(p.numel() for p in wrapper.parameters())
    logger.info(
        "Parameters: VLA trainable=%.1fM, RLT=%.1fM, total=%.1fM",
        vla_trainable / 1e6, rlt_trainable / 1e6, total / 1e6,
    )
    return wrapper


def build_dataset(train_cfg):
    """Build dataset + dataloader using 4DWVLA's data pipeline."""
    from lerobot.datasets.factory import make_dataset, make_dataloader

    dataset, data_stats = make_dataset(train_cfg)
    dataloader, dl_self_managed = make_dataloader(train_cfg, dataset)
    logger.info("Dataset: %d frames, dataloader ready", len(dataset))
    return dataset, dataloader, dl_self_managed


def build_optimizers(wrapper, rlt_cfg):
    """Build separate optimizers for VLA and RLT."""
    vla_params = list(wrapper.get_vla_params())
    rlt_params = list(wrapper.get_rlt_params())

    vla_optimizer = None
    if vla_params and rlt_cfg.rlt_alpha > 0 and not rlt_cfg.vla_inference_mode:
        vla_optimizer = torch.optim.AdamW(
            vla_params, lr=rlt_cfg.vla_lr, betas=(0.9, 0.95), weight_decay=0.01,
        )
        logger.info("VLA optimizer: %d param groups, lr=%.2e", len(vla_params), rlt_cfg.vla_lr)
    elif rlt_cfg.vla_inference_mode:
        for p in wrapper.base_policy.parameters():
            p.requires_grad = False
        logger.info("VLA inference mode: all VLA params frozen, no VLA optimizer")

    rlt_optimizer = torch.optim.AdamW(
        rlt_params, lr=rlt_cfg.rlt_lr, betas=(0.9, 0.95), weight_decay=0.01,
    )
    logger.info("RLT optimizer: lr=%.2e", rlt_cfg.rlt_lr)

    return vla_optimizer, rlt_optimizer


def train(rlt_cfg, train_cfg):
    from accelerate import Accelerator
    from accelerate.utils import set_seed

    set_seed(42)
    accelerator = Accelerator(
        mixed_precision="bf16",
        gradient_accumulation_steps=rlt_cfg.gradient_accumulation_steps,
    )

    wrapper = build_model(train_cfg, rlt_cfg)
    dataset, dataloader, dl_self_managed = build_dataset(train_cfg)
    vla_optimizer, rlt_optimizer = build_optimizers(wrapper, rlt_cfg)

    if vla_optimizer:
        wrapper, dataloader, vla_optimizer, rlt_optimizer = accelerator.prepare(
            wrapper, dataloader, vla_optimizer, rlt_optimizer,
        )
    else:
        wrapper, dataloader, rlt_optimizer = accelerator.prepare(
            wrapper, dataloader, rlt_optimizer,
        )

    logger.info("Training: max_steps=%d, batch=%d, accum=%d, profile=%s",
                rlt_cfg.max_steps, rlt_cfg.micro_batch_size, rlt_cfg.gradient_accumulation_steps,
                rlt_cfg.train_profile)

    global_step = 0
    loss_history = []
    peak_vram = 0.0
    step_times = []

    dl_iter = iter(dataloader)

    for step_idx in range(rlt_cfg.max_steps):
        t0 = time.monotonic()

        try:
            batch = next(dl_iter)
        except StopIteration:
            dl_iter = iter(dataloader)
            batch = next(dl_iter)

        if dl_self_managed:
            from lerobot.datasets.factory import send_to_device
            batch = send_to_device(batch, accelerator.device, non_blocking=True)

        with accelerator.accumulate(wrapper):
            with accelerator.autocast():
                total_loss, metrics = wrapper(batch)
            accelerator.backward(total_loss)

            if accelerator.sync_gradients:
                accelerator.clip_grad_norm_(wrapper.parameters(), rlt_cfg.grad_clip_norm)

            if vla_optimizer:
                vla_optimizer.step()
                vla_optimizer.zero_grad()
            rlt_optimizer.step()
            rlt_optimizer.zero_grad()

        global_step += 1
        dt = time.monotonic() - t0
        step_times.append(dt)

        loss_rlt = metrics.get("loss_rlt", 0)
        loss_vla = metrics.get("loss_vla", 0)
        loss_total = metrics.get("loss_total", total_loss.item())
        loss_history.append({"step": global_step, "rlt": loss_rlt, "vla": loss_vla, "total": loss_total})

        if torch.cuda.is_available():
            vram = torch.cuda.max_memory_allocated() / 1024**3
            peak_vram = max(peak_vram, vram)

        nan_detected = any(
            v != v for v in [loss_rlt, loss_vla, loss_total]
        )
        inf_detected = any(
            abs(v) == float("inf") for v in [loss_rlt, loss_vla, loss_total]
        )

        if global_step % rlt_cfg.log_freq == 0 or global_step == 1 or nan_detected or inf_detected:
            z_norm = metrics.get("rlt_z_rl_norm", 0)
            prefix_len = metrics.get("prefix_seq_len", 0)
            logger.info(
                "step=%d loss_total=%.4f loss_rlt=%.4f loss_vla=%.4f "
                "z_rl_norm=%.3f prefix_len=%d dt=%.2fs vram=%.1fGB%s",
                global_step, loss_total, loss_rlt, loss_vla,
                z_norm, prefix_len, dt, peak_vram,
                " NaN!" if nan_detected else (" Inf!" if inf_detected else ""),
            )

        if nan_detected or inf_detected:
            logger.error("NaN/Inf detected at step %d, aborting", global_step)
            break

        if rlt_cfg.save_freq > 0 and global_step % rlt_cfg.save_freq == 0:
            save_dir = Path(rlt_cfg.output_dir) / f"step_{global_step:06d}"
            accelerator.wait_for_everyone()
            if accelerator.is_main_process:
                unwrapped = accelerator.unwrap_model(wrapper)
                vla_dir = save_dir / "vla"
                rlt_dir = save_dir / "rlt"
                unwrapped.base_policy.save_pretrained(str(vla_dir))
                unwrapped.save_rlt_checkpoint(str(rlt_dir))
                with open(save_dir / "rlt_config.yaml", "w") as f:
                    yaml.dump(dataclasses.asdict(rlt_cfg), f, default_flow_style=False)
                logger.info("Saved checkpoint at step %d to %s", global_step, save_dir)

    avg_step_time = sum(step_times) / len(step_times) if step_times else 0

    logger.info("=" * 60)
    logger.info("Training complete: %d steps", global_step)
    logger.info("Peak VRAM: %.2f GB", peak_vram)
    logger.info("Avg step time: %.2f s", avg_step_time)
    if len(loss_history) >= 2:
        logger.info("First loss_rlt: %.4f, Last loss_rlt: %.4f",
                     loss_history[0]["rlt"], loss_history[-1]["rlt"])
    logger.info("=" * 60)

    report = {
        "steps_completed": global_step,
        "peak_vram_gb": round(peak_vram, 2),
        "avg_step_time_s": round(avg_step_time, 2),
        "loss_history": loss_history,
        "nan_detected": nan_detected if "nan_detected" in dir() else False,
        "rlt_config": dataclasses.asdict(rlt_cfg),
    }

    report_path = Path(rlt_cfg.output_dir) / "training_report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, default=str)
    logger.info("Report saved to %s", report_path)

    return report


def main():
    parser = argparse.ArgumentParser(description="RLT Stage 1 Training for 4DWVLA")
    parser.add_argument("--config", required=True, help="Path to RLT YAML config")
    parser.add_argument("--max_steps", type=int, default=None)
    parser.add_argument("--dataset_root", type=str, default=None)
    parser.add_argument("--dataset_repo_id", type=str, default=None)
    parser.add_argument("--save_freq", type=int, default=None)
    parser.add_argument("--output_dir", type=str, default=None)
    parser.add_argument("--log_freq", type=int, default=None)
    args = parser.parse_args()

    rlt_cfg = load_rlt_config(args)
    logger.info("RLT config: profile=%s, embed_dim=%d, alpha=%.1f",
                rlt_cfg.train_profile, rlt_cfg.rlt_embed_dim, rlt_cfg.rlt_alpha)

    train_cfg = load_train_pipeline_config(
        ckpt_path=rlt_cfg.pretrained_path,
        dataset_root=rlt_cfg.dataset_root,
        repo_id=args.dataset_repo_id or rlt_cfg.dataset_repo_id,
    )
    # The checkpoint's train_config.json contains the original SFT batch size
    # (16 for the base run). Stage1 exposes micro_batch_size specifically so a
    # single GPU can trade dataloader batch size for gradient accumulation.
    # Leaving the checkpoint value untouched can OOM before the first step.
    train_cfg.batch_size = rlt_cfg.micro_batch_size
    train_cfg.steps = rlt_cfg.max_steps

    report = train(rlt_cfg, train_cfg)
    return 0 if report["steps_completed"] == rlt_cfg.max_steps else 1


if __name__ == "__main__":
    sys.exit(main())
