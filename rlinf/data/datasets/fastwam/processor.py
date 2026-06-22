"""RLinf-side extension of FastWAM's ``FastWAMProcessor``.

FastWAM's ``FastWAMProcessor`` does not support proprioceptive (state/action)
augmentation. The proprio augmentations live entirely on the RLinf side (see
:mod:`rlinf.data.aug.augmentation`) and operate on the raw,
per-key ``state``/``action`` dict before normalization.

This module provides a thin subclass that applies the configured proprio
augmentations during training, right before the base ``preprocess`` runs its
action/state transforms, normalization and merging. Keeping it here avoids
forking the external FastWAM library while still wiring the feature end to end.
"""
import os
from pathlib import Path
from typing import Callable, Any, Dict, List, Optional, Set
import torch
from torchvision.io import write_video
from torchvision.io import read_video
from copy import deepcopy

from rlinf.utils.logging import get_logger

logger = get_logger()

def load_full_episode_camera_frames(
    lerobot_dataset,
    episode_index: int,
    lerobot_key: str,
    dataset_index: int = 0,
) -> torch.Tensor:
    """Load all frames of one camera for a full episode as uint8 ``[T, C, H, W]``."""
    dataset = lerobot_dataset.multi_dataset._datasets[dataset_index]
    video_path = dataset.root / dataset.meta.get_video_file_path(
        int(episode_index), lerobot_key
    )
    video, _, _ = read_video(str(video_path), pts_unit="sec")
    if video.numel() == 0:
        raise RuntimeError(f"No frames decoded from {video_path}")
    return video.permute(0, 3, 1, 2).contiguous()

_DEFAULT_TRANSFORM_TEST_DIR = (
    "../../../../b/test/trnsf_tst/"
)
EpisodeFrameLoader = Callable[[int, str, int], torch.Tensor]
_episode_frame_loader: Optional[EpisodeFrameLoader] = None
_dumped_episodes: Set[int] = set()
_dump_episode_count = 0


def register_episode_frame_loader(loader: Optional[EpisodeFrameLoader]) -> None:
    """Register a callback that loads full-episode camera frames for MP4 dump."""
    global _episode_frame_loader
    _episode_frame_loader = loader


def reset_episode_transform_dump_state() -> None:
    """Reset dump counters (for smoke tests / repeated runs in one process)."""
    global _dumped_episodes, _dump_episode_count
    _dumped_episodes = set()
    _dump_episode_count = 0


def _to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, torch.Tensor):
        return int(value.item())
    return int(value)


def _lerobot_image_key(meta: Dict[str, Any]) -> str:
    key = meta["key"]
    return meta.get("lerobot_key") or (
        f"observation.images.{key}" if key != "default" else "observation.images"
    )


def _tensor_to_video_u8(image: torch.Tensor) -> torch.Tensor:
    """Convert ``[T, C, H, W]`` float or uint8 tensor to ``[T, H, W, C]`` uint8."""
    image = image.detach().cpu()
    if image.dtype == torch.uint8:
        return image.permute(0, 2, 3, 1).contiguous()
    return image.clamp(0.0, 1.0).permute(0, 2, 3, 1).mul(255.0).to(torch.uint8)


def dump_episode_transform_mp4(
    processor,
    episode_index: int,
    dataset_index: int = 0,
    *,
    force: bool = False,
) -> None:
    """Dump one full episode (all cameras) with train/val transforms applied.

    Requires ``register_episode_frame_loader`` (wired from ``BaseLerobotDataset``).
    """
    global _dump_episode_count

    if _episode_frame_loader is None:
        raise RuntimeError(
            "Episode frame loader not registered; call BaseLerobotDataset.set_processor first"
        )
    if not force and episode_index in _dumped_episodes:
        return

    out_dir = Path(
        os.environ.get("RLINF_TRANSFORM_TEST_DIR", _DEFAULT_TRANSFORM_TEST_DIR)
    )
    fps = int(round(float(os.environ.get("FASTWAM_DUMP_TRANSFORM_FPS", "14"))))
    transforms = processor.train_transforms if processor.is_train else processor.val_transforms

    out_dir.mkdir(parents=True, exist_ok=True)
    for meta in processor.shape_meta["images"]:
        key = meta["key"]
        lerobot_key = _lerobot_image_key(meta)
        image = _episode_frame_loader(episode_index, lerobot_key, dataset_index)
        if image.ndim != 4:
            raise ValueError(
                f"Episode loader must return [T, C, H, W], got {tuple(image.shape)}"
            )

        current_transforms = transforms[key] if isinstance(transforms, dict) else transforms
        for trans in current_transforms:
            image = trans(image)

        out_path = out_dir / f"episode_{episode_index:06d}_{key}.mp4"
        video_u8 = _tensor_to_video_u8(image)
        write_video(str(out_path), video_u8, fps=fps)
        logger.info("Dumped episode transform MP4: %s (%d frames)", out_path, video_u8.shape[0])

    _dumped_episodes.add(episode_index)
    _dump_episode_count += 1


def _maybe_dump_episode_transform_mp4(processor, data: Dict[str, Any]) -> None:
    """Dump full-episode augmented videos when enabled during ``preprocess``.

    See ``RLinf/b/test/trnsf_tst/README.md``.
    """
    if os.environ.get("FASTWAM_DUMP_TRANSFORM_MP4") != "1":
        return
    if _episode_frame_loader is None:
        return

    episode_index = _to_int(data.get("episode_index"))
    if episode_index is None:
        logger.warning("FASTWAM_DUMP_TRANSFORM_MP4 enabled but episode_index missing in sample")
        return
    if episode_index in _dumped_episodes:
        return

    episode_filter = os.environ.get("FASTWAM_DUMP_EPISODE_INDICES", "").strip()
    if episode_filter:
        allowed = {int(x.strip()) for x in episode_filter.split(",") if x.strip()}
        if episode_index not in allowed:
            return

    max_episodes = int(os.environ.get("FASTWAM_DUMP_TRANSFORM_MAX", "10"))
    if _dump_episode_count >= max_episodes:
        return

    dataset_index = _to_int(data.get("dataset_index")) or 0
    try:
        dump_episode_transform_mp4(processor, episode_index, dataset_index)
    except Exception as exc:
        logger.warning(
            "Failed to dump episode transform MP4 for episode %s: %s", episode_index, exc
        )



def build_proprio_aug_processor_cls():
    """Build the proprio-aug-aware processor class.

    The base class is imported lazily so that importing this module does not
    require the heavy ``fastwam`` package to be available.
    """
    from fastwam.datasets.lerobot.processors.fastwam_processor import (
        FastWAMProcessor,
    )

    class ProprioAugFastWAMProcessor(FastWAMProcessor):
        """``FastWAMProcessor`` that also applies proprio augmentations.

        Args:
            proprio_augmentations: Optional list of callables taking and
                returning the raw sample dict (see
                ``rlinf.data.aug.augmentation.ProprioAugmentation``).
                Applied only when the processor is in training mode.
        """

        def __init__(
            self,
            *args: Any,
            proprio_augmentations: Optional[List[Any]] = None,
            **kwargs: Any,
        ):
            super().__init__(*args, **kwargs)
            self.proprio_augmentations = proprio_augmentations or []

        def preprocess(self, data: Dict[str, Any]) -> Dict[str, Any]:
            """
            Preprocess the data for the policy model.
            
            Args:
                Data: Dict[str, Any], lerobot sample in raw mcap obtained from dataset __getitem__:
                    - "action": Optional, Dict[str, torch.Tensor] -> [action_horizon, action_dim]
                    - "state": Dict[str, torch.Tensor] -> [num_obs_steps, state_dim]
                    - "images": Dict[str, torch.Tensor] -> [num_obs_steps, C, H, W]
                    - "action_is_pad": Optional, torch.Tensor -> [action_horizon,]
                    - "state_is_pad": torch.Tensor -> [num_obs_steps,]
                    - "image_is_pad": torch.Tensor -> [num_obs_steps,]
                    - "idx": int, sample index
                    
            Returns:
                Sample: Dict[str, Any], which can collated:
                    - "input_ids": torch.Tensor -> [max_image_text_tokens,]
                    - "attention_mask": torch.Tensor -> [max_image_text_tokens,]
                    - "pixel_values": torch.Tensor -> [num_input_cameras, C, H, W]
                    - "image_is_pad": torch.Tensor -> [num_obs_steps,]
                    - "proprio": torch.Tensor -> [num_obs_steps, proprio_dim]
                    - "state_is_pad": torch.Tensor -> [num_obs_steps,]
                    - "action": Optional, torch.Tensor -> [action_horizon, action_dim]
                    - "action_is_pad": Optional, torch.Tensor -> [action_horizon,]
                    - "gt_action: Optional, deepcopy of input action for open loop eval, which is left untouched
                    - "idx": int, sample index
            """
            sample = {}
            # 1. instruction
            sample["instruction"] = self.augment_instruction(data)
            sample["image_is_pad"] = data["image_is_pad"]

            # 2. image
            processed_images = []
            for meta in self.shape_meta["images"]:
                key, shape = meta["key"], meta["shape"]
                image = data["images"][key]  # [num_obs_steps, C, H, W]
                assert image.ndim == 4, f"Expected 4 dimensions (num_obs_steps, C, H, W), got shape {image.shape}"
                
                # Apply transforms efficiently on the merged batch
                transforms = self.train_transforms if self.is_train else self.val_transforms
                current_transforms = transforms[key] if isinstance(transforms, dict) else transforms
                for trans in current_transforms:
                    image = trans(image)

                meta_shape = [self.num_obs_steps] + shape
                assert list(image.shape) == meta_shape, \
                    f"Expected shape {meta_shape}, got {image.shape} after transforms for key {key}"

                processed_images.append(image)
            _maybe_dump_episode_transform_mp4(self, data) #@#按episode保存训练时被数据增强的图片
            pixel_values = torch.stack(processed_images, dim=0) # [num_input_cameras, T, C, H, W]
            
            if self.num_output_cameras > pixel_values.shape[0]:
                out = torch.zeros((self.num_output_cameras,) + pixel_values.shape[1:], device=pixel_values.device, dtype=pixel_values.dtype)
                out[0: pixel_values.shape[0]] = pixel_values
                sample["pixel_values"] = out
            elif self.num_output_cameras < pixel_values.shape[0]:
                logger.warning(f"num_output_cameras {self.num_output_cameras} is less than the number of cameras in data {pixel_values.shape[0]}, "
                            f"truncating the input to the first {self.num_output_cameras} cameras.")
                sample["pixel_values"] = pixel_values[:self.num_output_cameras]
            else:
                sample["pixel_values"] = pixel_values

            # Copy action before transform for open-loop evaluation, 
            # disabled for training dataset as it may cause collating key problem.
            if not self.is_train and "action" in data:
                sample["gt_action"] = deepcopy(data["action"])

            # 3. action & state
            if "action" in data and self.delta_action_dim_mask is not None:
                action_is_pad = torch.as_tensor(data["action_is_pad"], dtype=torch.bool)
                if bool(action_is_pad.any().item()):
                    for key, dim_mask in self.delta_action_dim_mask.items():
                        cur_action = data["action"][key]
                        cur_action_is_pad = action_is_pad.to(device=cur_action.device)
                        cur_dim_mask = dim_mask.to(device=cur_action.device)
                        pad_delta_mask = cur_action_is_pad.unsqueeze(1) & cur_dim_mask.unsqueeze(0)
                        cur_action[pad_delta_mask] = 0.0
            data = self.action_state_transform(data)
            if self.is_train and self.proprio_augmentations is not None:
                for aug in self.proprio_augmentations:
                    data = aug(data)
            data = self.normalizer.forward(data)
            data = self.action_state_merger.forward(data)

            if "action" in data:
                sample["action"] = data["action"] # [action_horizon, action_dim]
                sample["action_is_pad"] = data["action_is_pad"] # [action_horizon,]
                sample["action_dim_is_pad"] = data["action_dim_is_pad"] # [action_dim,]
                assert sample["action"].shape[-1] == self.action_output_dim
                # sample["action"][sample["action_is_pad"], :-1] = 0.0 # NOTE: we assume use delta_eef_pose + gripper， so pad action is 0

            
            # TODO: rename all "state" into "proprio"
            sample["proprio"] = data["state"] # [num_obs_steps, proprio_dim]
            sample["proprio_is_pad"] = data["state_is_pad"] # [num_obs_steps,]
            sample["proprio_dim_is_pad"] = data["state_dim_is_pad"] # [proprio_dim,]
            assert sample["proprio"].shape[-1] == self.proprio_output_dim

            sample["idx"] = data["idx"]

            # sample = self.tokenizer(sample)
            
            return sample

    return ProprioAugFastWAMProcessor
