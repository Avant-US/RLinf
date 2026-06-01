import importlib

from omegaconf import DictConfig, OmegaConf
from torch.utils.data.distributed import DistributedSampler
from torchdata.stateful_dataloader import StatefulDataLoader

from rlinf.data.datasets.fastwam.collate import fastwam_collate_fn


def _ensure_dict(val):
    if isinstance(val, DictConfig):
        return OmegaConf.to_container(val, resolve=True)
    if isinstance(val, dict):
        return val
    return val


def _manual_instantiate(cfg_val):
    if cfg_val is None:
        return None
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
    if cfg_list is None:
        return None
    result = []
    for item in cfg_list:
        if isinstance(item, DictConfig):
            item = OmegaConf.to_container(item, resolve=True)
        result.append(_manual_instantiate(item))
    return result


def build_fastwam_sft_dataloader(cfg, world_size, rank, data_paths, eval_dataset=False):
    import os

    from fastwam.datasets.lerobot.robot_video_dataset import RobotVideoDataset
    from fastwam.datasets.lerobot.processors.fastwam_processor import FastWAMProcessor
    from fastwam.utils.misc import register_work_dir

    log_path = cfg.runner.logger.get("log_path", "./runs")
    register_work_dir(log_path)

    model_cfg = cfg.actor.model
    data_cfg = cfg.data

    if isinstance(data_paths, str):
        dataset_dirs = [data_paths]
    elif isinstance(data_paths, (list, tuple)):
        dataset_dirs = list(data_paths)
    else:
        dataset_dirs = [str(data_paths)]

    shape_meta = _ensure_dict(data_cfg.get("shape_meta", model_cfg.get("shape_meta")))

    raw_processor_cfg = data_cfg.get("processor", {})
    processor_cfg = _ensure_dict(raw_processor_cfg) or {}

    action_state_merger_obj = _manual_instantiate(processor_cfg.get("action_state_merger", None))
    train_transforms_obj = _instantiate_transforms(processor_cfg.get("train_transforms", None))
    val_transforms_obj = _instantiate_transforms(processor_cfg.get("val_transforms", None))

    proprio_output_dim = processor_cfg.get("proprio_output_dim", None)
    delta_mask = _ensure_dict(processor_cfg.get("delta_action_dim_mask", None))

    processor = FastWAMProcessor(
        shape_meta=shape_meta,
        num_obs_steps=int(data_cfg.get("num_frames", 33)),
        num_output_cameras=int(processor_cfg.get("num_output_cameras", 2)),
        action_output_dim=int(
            processor_cfg.get(
                "action_output_dim",
                _ensure_dict(model_cfg.get("action_dit_config", {})).get("action_dim", 7),
            )
        ),
        proprio_output_dim=int(proprio_output_dim) if proprio_output_dim else None,
        action_state_transforms=processor_cfg.get("action_state_transforms", None),
        use_stepwise_action_norm=processor_cfg.get("use_stepwise_action_norm", False),
        norm_default_mode=processor_cfg.get("norm_default_mode", "min/max"),
        norm_exception_mode=processor_cfg.get("norm_exception_mode", None),
        action_state_merger=action_state_merger_obj,
        train_transforms=train_transforms_obj,
        val_transforms=val_transforms_obj,
        delta_action_dim_mask=delta_mask,
    )

    shape_meta_cfg = OmegaConf.create(shape_meta)

    dataset = RobotVideoDataset(
        dataset_dirs=dataset_dirs,
        shape_meta=shape_meta_cfg,
        processor=processor,
        num_frames=int(data_cfg.get("num_frames", 33)),
        action_video_freq_ratio=int(data_cfg.get("action_video_freq_ratio", 4)),
        video_size=list(data_cfg.get("video_size", [224, 448])),
        text_embedding_cache_dir=model_cfg.get("text_embedding_cache_dir"),
        context_len=int(model_cfg.get("context_len", 128)),
        concat_multi_camera=data_cfg.get("concat_multi_camera", None),
        pretrained_norm_stats=data_cfg.get("pretrained_norm_stats", None),
        is_training_set=not eval_dataset,
        val_set_proportion=float(data_cfg.get("val_set_proportion", 0.0)),
        global_sample_stride=int(data_cfg.get("global_sample_stride", 1)),
        skip_padding_as_possible=data_cfg.get("skip_padding_as_possible", False),
    )

    sampler = DistributedSampler(
        dataset,
        num_replicas=world_size,
        rank=rank,
        shuffle=not eval_dataset,
    )

    loader = StatefulDataLoader(
        dataset,
        batch_size=cfg.actor.micro_batch_size,
        sampler=sampler,
        collate_fn=fastwam_collate_fn,
        num_workers=int(data_cfg.get("num_workers", 8)),
        pin_memory=True,
        persistent_workers=True,
        prefetch_factor=int(data_cfg.get("prefetch_factor", 2)),
    )

    data_config = {
        "num_samples": len(dataset),
        "num_frames": int(data_cfg.get("num_frames", 33)),
    }

    return loader, data_config
