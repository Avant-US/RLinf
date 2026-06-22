#!/usr/bin/env python3
"""Smoke test: dump full-episode transform MP4s via FastWAMProcessor.preprocess."""

from __future__ import annotations

import argparse
import importlib
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

FASTWAM_SRC = Path(
    os.environ.get("FASTWAM_PATH", "/home/Luogang/SRC/Robot/FastWAM/src")
)
if str(FASTWAM_SRC) not in sys.path:
    sys.path.insert(0, str(FASTWAM_SRC))


def _ensure_smoke_env() -> None:
    """Set env vars required by Hydra config resolution (``oc.env:*``)."""
    defaults = {
        "EMBODIED_PATH": str(REPO_ROOT / "examples" / "sft"),
        "REPO_ROOT": str(REPO_ROOT),
        "R1PRO_DATA": "/mnt/r/share/zwy/datasets/r1_pro_data_v2",
        "FASTWAM_ROOT": "/home/Luogang/SRC/Robot/FastWAM",
        "DIFFSYNTH_MODEL_BASE_PATH": "/mnt/r/CKPT/VLA/FW",
        "DIFFSYNTH_SKIP_DOWNLOAD": "true",
        "TF_ENABLE_ONEDNN_OPTS": "0",
    }
    for key, value in defaults.items():
        os.environ.setdefault(key, value)
    os.environ.setdefault("FASTWAM_PATH", os.path.join(os.environ["FASTWAM_ROOT"], "src"))
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    fastwam_path = os.environ["FASTWAM_PATH"]
    if fastwam_path not in sys.path:
        sys.path.insert(0, fastwam_path)


# Hydra ``oc.env:EMBODIED_PATH`` must exist before any config compose.
_ensure_smoke_env()


def _manual_instantiate(cfg_val):
    if cfg_val is None:
        return None
    from omegaconf import DictConfig, OmegaConf

    if isinstance(cfg_val, DictConfig):
        cfg_val = OmegaConf.to_container(cfg_val, resolve=True)
    if isinstance(cfg_val, dict) and "_target_" in cfg_val:
        cfg_dict = dict(cfg_val)
        target = cfg_dict.pop("_target_")
        module_path, cls_name = target.rsplit(".", 1)
        mod = importlib.import_module(module_path)
        cls = getattr(mod, cls_name)
        return cls(**cfg_dict)
    return cfg_val


def _instantiate_transforms(cfg_list):
    from omegaconf import DictConfig, OmegaConf

    if cfg_list is None:
        return None
    if isinstance(cfg_list, DictConfig):
        cfg_list = OmegaConf.to_container(cfg_list, resolve=True)
    if isinstance(cfg_list, dict) and "_target_" not in cfg_list:
        return {key: _instantiate_transforms(val) for key, val in cfg_list.items()}
    result = []
    for item in cfg_list:
        if isinstance(item, DictConfig):
            item = OmegaConf.to_container(item, resolve=True)
        result.append(_manual_instantiate(item))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Dump full-episode transform MP4 smoke test")
    parser.add_argument(
        "--num_episodes",
        type=int,
        default=2,
        help="Stop after this many distinct episodes are dumped",
    )
    parser.add_argument(
        "--start_idx",
        type=int,
        default=0,
        help="First dataset sample index to iterate from",
    )
    parser.add_argument(
        "--max_samples",
        type=int,
        default=500,
        help="Upper bound on samples to scan while collecting episodes",
    )
    parser.add_argument(
        "--episode_indices",
        type=str,
        default="",
        help="Comma-separated episode_index list for direct dump (skips sample scan)",
    )
    args = parser.parse_args()

    _ensure_smoke_env()
    os.environ["FASTWAM_DUMP_TRANSFORM_MP4"] = "1"
    episode_indices = [
        int(x.strip()) for x in args.episode_indices.split(",") if x.strip()
    ]
    if not episode_indices:
        env_eps = os.environ.get("FASTWAM_DUMP_EPISODE_INDICES", "").strip()
        if env_eps:
            episode_indices = [int(x.strip()) for x in env_eps.split(",") if x.strip()]
    if episode_indices:
        os.environ["FASTWAM_DUMP_EPISODE_INDICES"] = ",".join(str(e) for e in episode_indices)
        os.environ["FASTWAM_DUMP_TRANSFORM_MAX"] = str(len(episode_indices))
        args.num_episodes = len(episode_indices)
    else:
        os.environ.setdefault("FASTWAM_DUMP_TRANSFORM_MAX", str(args.num_episodes))

    from hydra import compose, initialize_config_dir
    from omegaconf import OmegaConf

    import rlinf.data.datasets.fastwam.processor as fwp_module
    from rlinf.data.datasets.fastwam.processor import (
        build_proprio_aug_processor_cls,
        dump_episode_transform_mp4,
        reset_episode_transform_dump_state,
    )
    FastWAMProcessor = build_proprio_aug_processor_cls()
    from fastwam.datasets.lerobot.robot_video_dataset import RobotVideoDataset
    from fastwam.utils.misc import register_work_dir

    reset_episode_transform_dump_state()

    config_dir = str(REPO_ROOT / "examples" / "sft" / "config")
    with initialize_config_dir(version_base="1.1", config_dir=config_dir):
        cfg = compose(config_name="r1_pro_sft_fastwam")

    data_dir = os.environ.get(
        "R1PRO_DATA", "/mnt/r/share/zwy/datasets/r1_pro_data_v2"
    )
    train_path = os.path.join(data_dir, "r1_pro_data_convert_chassis")
    data_cfg = cfg.data
    processor_cfg = OmegaConf.to_container(data_cfg.processor, resolve=True)

    shape_meta = OmegaConf.to_container(data_cfg.shape_meta, resolve=True)
    num_cameras = len(shape_meta["images"])
    expected_mp4 = args.num_episodes * num_cameras

    processor = FastWAMProcessor(
        shape_meta=shape_meta,
        num_obs_steps=int(data_cfg.num_frames),
        num_output_cameras=int(processor_cfg["num_output_cameras"]),
        action_output_dim=int(processor_cfg["action_output_dim"]),
        proprio_output_dim=int(processor_cfg["proprio_output_dim"]),
        action_state_transforms=processor_cfg.get("action_state_transforms"),
        use_stepwise_action_norm=processor_cfg.get("use_stepwise_action_norm", False),
        norm_default_mode=processor_cfg.get("norm_default_mode", "z-score"),
        norm_exception_mode=processor_cfg.get("norm_exception_mode"),
        action_state_merger=_manual_instantiate(processor_cfg["action_state_merger"]),
        train_transforms=_instantiate_transforms(processor_cfg.get("train_transforms")),
        val_transforms=_instantiate_transforms(processor_cfg.get("val_transforms")),
    ).train()

    work_dir = str(REPO_ROOT / "b" / "test" / "trnsf_tst")
    register_work_dir(work_dir)

    text_cache = cfg.actor.model.get(
        "text_embedding_cache_dir",
        os.path.join(
            os.environ.get("FASTWAM_ROOT", "/home/Luogang/SRC/Robot/FastWAM"),
            "data/text_embeds_cache/r1_pro_chassis",
        ),
    )

    stats_cache = out_dir_candidate = REPO_ROOT / "b" / "test" / "trnsf_tst" / "dataset_stats.json"
    pretrained_norm_stats = data_cfg.get("pretrained_norm_stats")
    if not pretrained_norm_stats and stats_cache.is_file():
        pretrained_norm_stats = str(stats_cache)
        print(f"Using cached norm stats: {pretrained_norm_stats}")

    dataset = RobotVideoDataset(
        dataset_dirs=[train_path],
        shape_meta=OmegaConf.create(shape_meta),
        processor=processor,
        num_frames=int(data_cfg.num_frames),
        action_video_freq_ratio=int(data_cfg.action_video_freq_ratio),
        video_size=list(data_cfg.video_size),
        text_embedding_cache_dir=text_cache,
        context_len=int(cfg.actor.model.get("context_len", 128)),
        concat_multi_camera=data_cfg.get("concat_multi_camera"),
        pretrained_norm_stats=pretrained_norm_stats,
        is_training_set=True,
        val_set_proportion=float(data_cfg.get("val_set_proportion", 0.0)),
        global_sample_stride=int(data_cfg.get("global_sample_stride", 1)),
        skip_padding_as_possible=data_cfg.get("skip_padding_as_possible", False),
    )

    out_dir = Path(
        os.environ.get(
            "RLINF_TRANSFORM_TEST_DIR",
            REPO_ROOT / "b" / "test" / "trnsf_tst",
        )
    )
    print(f"Output dir: {out_dir}")

    if episode_indices:
        print(f"Direct dump for episode_index: {episode_indices}")
        for ep in episode_indices:
            print(f"  dumping episode {ep}...")
            dump_episode_transform_mp4(processor, ep, force=True)
    else:
        print(
            f"Scanning samples from idx={args.start_idx}, "
            f"target episodes={args.num_episodes}, max_samples={args.max_samples}"
        )
        end_idx = min(args.start_idx + args.max_samples, len(dataset))
        last_dumped = 0
        for i in range(args.start_idx, end_idx):
            _ = dataset[i]
            dumped = fwp_module._dump_episode_count
            if dumped > last_dumped:
                print(f"  sample idx={i} -> dumped episodes so far: {dumped}")
                last_dumped = dumped
            if dumped >= args.num_episodes:
                break

    mp4_files = sorted(out_dir.glob("episode_*.mp4"))
    print(f"Wrote {len(mp4_files)} MP4 file(s) (expected up to {expected_mp4}):")
    show = mp4_files if len(mp4_files) <= 48 else mp4_files[-48:]
    for p in show:
        print(f"  {p.name} ({p.stat().st_size // 1024} KB)")
    if len(mp4_files) < expected_mp4:
        print(
            "WARNING: fewer MP4s than expected. "
            "Check FASTWAM_DUMP_TRANSFORM_MAX, FASTWAM_DUMP_EPISODE_INDICES, or max_samples."
        )


if __name__ == "__main__":
    main()
