#!/usr/bin/env python3
"""
可视化 FastWAM 数据增强效果。

基于 r1_pro_chassis_uncond_3cam_384_1e-4 任务的真实数据，展示不同增强方法
和预设对视频帧的效果。生成对比图保存到 b/test/augmentation_vis/。

无需 GPU、Ray 或 FSDP。仅需数据集路径和 FastWAM src 在 PYTHONPATH 中。

用法:
    export FASTWAM_ROOT=/home/Luogang/SRC/Robot/FastWAM
    export FASTWAM_PATH=${FASTWAM_ROOT}/src
    export DIFFSYNTH_MODEL_BASE_PATH=/mnt/r/CKPT/VLA/FW
    export DIFFSYNTH_SKIP_DOWNLOAD=true
    export R1PRO_DATA=/mnt/r/share/zwy/datasets/r1_pro_data_v2
    export PYTHONPATH=/home/Luogang/SRC/RL/RLinf:${FASTWAM_PATH}
    python b/test/visualize_augmentations.py --sample_idx 0 --output_dir b/test/augmentation_vis2
    python b/test/visualize_augmentations.py [--sample_idx 0] [--output_dir b/test/augmentation_vis]
"""
import argparse
import os
import sys

import torch
import torchvision.transforms.v2 as T2
import numpy as np

# ---------------------------------------------------------------------------
# 增强类定义（与 fw_sft_dtaaug_op46.md §5 一致）
# 这些类将来会放入 rlinf/data/datasets/fastwam/augmentation.py
# 这里内联定义以便脚本独立运行
# ---------------------------------------------------------------------------

class VideoAugmentation(torch.nn.Module):
    def __init__(self, p: float = 1.0):
        super().__init__()
        self.p = p

    def forward(self, video: torch.Tensor) -> torch.Tensor:
        assert video.ndim == 4, f"Expected [T, C, H, W], got {video.shape}"
        if torch.rand(1).item() > self.p:
            return video
        return self._apply(video)

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError


class VideoColorJitter(VideoAugmentation):
    def __init__(self, brightness=0.0, contrast=0.0, saturation=0.0, hue=0.0, p=1.0):
        super().__init__(p=p)
        self._jitter = T2.ColorJitter(
            brightness=brightness, contrast=contrast,
            saturation=saturation, hue=hue,
        )

    def _apply(self, video):
        return self._jitter(video).clamp(0.0, 1.0)


class VideoRandomGrayscale(VideoAugmentation):
    def __init__(self, p=0.1):
        super().__init__(p=p)
        self._gray = T2.Grayscale(num_output_channels=3)

    def _apply(self, video):
        return self._gray(video)


class VideoRandomCrop(VideoAugmentation):
    """Frame-consistent random crop.

    Crops to ``int(H * scale) x int(W * scale)``. A subsequent ``Resize``
    in the transform chain restores the target dimensions.

    Args:
        scale: Fraction of height/width to keep (0, 1].
        p: Probability of applying.
    """

    def __init__(self, scale: float = 0.95, p: float = 0.5):
        super().__init__(p=p)
        self.scale = scale

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        _, _, H, W = video.shape
        crop_h, crop_w = int(H * self.scale), int(W * self.scale)
        return T2.RandomCrop(size=(crop_h, crop_w))(video)


class VideoRandomResizedCrop(VideoAugmentation):
    def __init__(self, size=(224, 224), scale=(0.8, 1.0), ratio=(0.75, 1.333), p=0.5):
        super().__init__(p=p)
        self._crop = T2.RandomResizedCrop(
            size=size, scale=scale, ratio=ratio,
            interpolation=T2.InterpolationMode.BILINEAR, antialias=True,
        )

    def _apply(self, video):
        return self._crop(video)


class VideoRandomHorizontalFlip(VideoAugmentation):
    def __init__(self, p=0.5):
        super().__init__(p=p)

    def _apply(self, video):
        return T2.RandomHorizontalFlip(p=1.0)(video)


class VideoRandomRotation(VideoAugmentation):
    def __init__(self, degrees=5.0, p=0.3):
        super().__init__(p=p)
        self._rot = T2.RandomRotation(
            degrees=degrees, interpolation=T2.InterpolationMode.BILINEAR,
        )

    def _apply(self, video):
        return self._rot(video)


class VideoGaussianNoise(VideoAugmentation):
    def __init__(self, std=0.02, per_frame=False, p=0.3):
        super().__init__(p=p)
        self.std = std
        self.per_frame = per_frame

    def _apply(self, video):
        if self.per_frame:
            noise = torch.randn_like(video) * self.std
        else:
            noise = torch.randn_like(video[0:1]) * self.std
            noise = noise.expand_as(video)
        return (video + noise).clamp(0.0, 1.0)


class VideoRandomErasing(VideoAugmentation):
    def __init__(self, scale=(0.02, 0.15), ratio=(0.3, 3.3), value=0, p=0.3):
        super().__init__(p=p)
        self._erase = T2.RandomErasing(p=1.0, scale=scale, ratio=ratio, value=value)

    def _apply(self, video):
        return self._erase(video)


# ---------------------------------------------------------------------------
# 预设
# ---------------------------------------------------------------------------

PRESETS = {
    "none": [],
    "light": [
        VideoColorJitter(brightness=0.1, contrast=0.1, saturation=0.1, hue=0.03, p=1.0),
    ],
    "medium": [
        VideoRandomCrop(scale=0.95, p=1.0),
        VideoColorJitter(brightness=0.2, contrast=0.3, saturation=0.3, hue=0.05, p=1.0),
        VideoGaussianNoise(std=0.01, p=1.0),
    ],
    "strong": [
        VideoRandomCrop(scale=0.90, p=1.0),
        VideoColorJitter(brightness=0.3, contrast=0.4, saturation=0.5, hue=0.08, p=1.0),
        VideoRandomGrayscale(p=1.0),
        VideoGaussianNoise(std=0.02, p=1.0),
        VideoRandomErasing(p=1.0),
    ],
    "dreamzero": [
        VideoRandomCrop(scale=0.95, p=1.0),
        VideoColorJitter(brightness=0.3, contrast=0.4, saturation=0.5, hue=0.08, p=1.0),
    ],
}

INDIVIDUAL_AUGMENTS = {
    "ColorJitter": VideoColorJitter(brightness=0.3, contrast=0.4, saturation=0.5, hue=0.08, p=1.0),
    "RandomCrop(0.90)": VideoRandomCrop(scale=0.95, p=1.0),
    "Grayscale": VideoRandomGrayscale(p=1.0),
    "GaussianNoise": VideoGaussianNoise(std=0.03, p=1.0),
    # "RandomErasing": VideoRandomErasing(scale=(0.01, 0.01), p=1.0),
    "RandomErasingH": VideoRandomErasing(scale=(0.001, 0.01), ratio=(0.4, 4.0), p=1.0),
    "RandomRotation(5°)": VideoRandomRotation(degrees=5.0, p=1.0),
    "HorizontalFlip": VideoRandomHorizontalFlip(p=1.0),
}


# ---------------------------------------------------------------------------
# 数据加载（绕过 Ray/FSDP，直接用 FastWAM 的数据集类）
# ---------------------------------------------------------------------------

def load_sample(data_dir, fastwam_root, sample_idx=0):
    """加载一个 r1_pro 样本的 3 个相机视频帧。

    返回 per-camera 的 float32 [T, C, H, W] 张量（值域 [0, 1]）和元数据。
    """
    from fastwam.datasets.lerobot.transforms.image import ToTensor
    from torchvision.transforms import Resize
    from omegaconf import OmegaConf

    shape_meta = {
        "images": [
            {"key": "head_rgb", "lerobot_key": "head_rgb",
             "raw_shape": [3, 360, 640], "shape": [3, 240, 320]},
            {"key": "left_wrist_rgb", "lerobot_key": "left_wrist_rgb",
             "raw_shape": [3, 480, 640], "shape": [3, 240, 320]},
            {"key": "right_wrist_rgb", "lerobot_key": "right_wrist_rgb",
             "raw_shape": [3, 480, 640], "shape": [3, 240, 320]},
        ],
        "action": [{"key": "default", "lerobot_key": "actions",
                     "raw_shape": 23, "shape": 23}],
        "state": [{"key": "default", "lerobot_key": "state",
                    "raw_shape": 23, "shape": 23}],
    }

    from rlinf.data.datasets.fastwam.processor import build_proprio_aug_processor_cls
    FastWAMProcessor = build_proprio_aug_processor_cls()
    from fastwam.datasets.lerobot.transforms.action_state_merger import ConcatLeftAlign

    processor = FastWAMProcessor(
        shape_meta=shape_meta,
        num_obs_steps=33,
        num_output_cameras=3,
        action_output_dim=23,
        proprio_output_dim=23,
        action_state_transforms=None,
        use_stepwise_action_norm=False,
        norm_default_mode="z-score",
        norm_exception_mode=None,
        action_state_merger=ConcatLeftAlign(),
        train_transforms=[ToTensor(), Resize([240, 320])],
        val_transforms=[ToTensor(), Resize([240, 320])],
    )

    from fastwam.datasets.lerobot.robot_video_dataset import RobotVideoDataset
    from fastwam.utils.misc import register_work_dir

    work_dir = os.path.join(os.path.dirname(__file__), "augmentation_vis")
    os.makedirs(work_dir, exist_ok=True)
    register_work_dir(work_dir)

    text_cache_dir = os.path.join(fastwam_root, "data", "text_embeds_cache", "r1_pro_chassis")
    shape_meta_cfg = OmegaConf.create(shape_meta)

    dataset = RobotVideoDataset(
        dataset_dirs=[data_dir],
        shape_meta=shape_meta_cfg,
        processor=processor,
        num_frames=33,
        action_video_freq_ratio=4,
        video_size=[384, 320],
        text_embedding_cache_dir=text_cache_dir,
        context_len=128,
        concat_multi_camera="robotwin",
        pretrained_norm_stats=None,
        is_training_set=True,
        val_set_proportion=0.0,
        global_sample_stride=1,
        skip_padding_as_possible=False,
    )

    print(f"Dataset loaded: {len(dataset)} samples")

    # 获取已处理的样本 — pixel_values 已包含 per-camera 帧（经过 ToTensor + Resize）
    raw_sample = dataset.lerobot_dataset[sample_idx]
    pixel_values = raw_sample["pixel_values"]  # [num_cameras=3, T=33, C=3, H=240, W=320]

    camera_keys = [m["key"] for m in shape_meta["images"]]
    cameras = {}
    video_indices = list(range(0, 33, 4))  # [0, 4, 8, ..., 32] → 9 帧
    for i, key in enumerate(camera_keys):
        if i < pixel_values.shape[0]:
            cam = pixel_values[i]  # [T=33, C=3, H=240, W=320] float32 [0,1]
            cameras[key] = cam[video_indices]  # [9, C, H, W]

    task = raw_sample.get("instruction", "unknown task")
    print(f"Sample {sample_idx}: task='{task}'")
    for key, v in cameras.items():
        print(f"  {key}: shape={list(v.shape)}, range=[{v.min():.3f}, {v.max():.3f}]")

    return cameras, task


# ---------------------------------------------------------------------------
# robotwin 拼接（复制 RobotVideoDataset._get 中的逻辑）
# ---------------------------------------------------------------------------

def robotwin_concat(cameras):
    """将 3 个相机按 robotwin 模式拼接为单幅图。

    输入: dict of [T, C, H, W] float32 [0, 1]
    输出: [T, C, 384, 320] float32 [0, 1]
    """
    import torchvision.transforms.functional as F

    cam_top = F.resize(cameras["head_rgb"], [256, 320],
                       interpolation=F.InterpolationMode.BILINEAR, antialias=True)
    cam_left = F.resize(cameras["left_wrist_rgb"], [128, 160],
                        interpolation=F.InterpolationMode.BILINEAR, antialias=True)
    cam_right = F.resize(cameras["right_wrist_rgb"], [128, 160],
                         interpolation=F.InterpolationMode.BILINEAR, antialias=True)
    bottom = torch.cat([cam_left, cam_right], dim=-1)  # [T, C, 128, 320]
    return torch.cat([cam_top, bottom], dim=-2)          # [T, C, 384, 320]


def apply_augment_per_camera(cameras, augment):
    """对每个相机独立应用增强（与 FastWAMProcessor.preprocess 行为一致）。"""
    result = {}
    for key, video in cameras.items():
        result[key] = augment(video.clone())
    return result


def apply_augment_list(cameras, aug_list):
    """依次应用增强列表。"""
    result = {k: v.clone() for k, v in cameras.items()}
    for aug in aug_list:
        result = apply_augment_per_camera(result, aug)
    return result


def tensor_to_image(video_tensor, frame_idx=0):
    """从 [T, C, H, W] float32 [0,1] 取一帧转为 numpy [H, W, 3] uint8。"""
    frame = video_tensor[frame_idx].permute(1, 2, 0).clamp(0, 1).numpy()
    return (frame * 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# 可视化
# ---------------------------------------------------------------------------

def visualize_presets(cameras, output_dir):
    """生成预设对比图：每行一个预设，每列一个帧（frame 0, 4, 8）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    frame_indices = [0, 4, 8] if cameras["head_rgb"].shape[0] > 8 else [0]
    n_frames = len(frame_indices)
    preset_names = list(PRESETS.keys())
    n_presets = len(preset_names)

    fig, axes = plt.subplots(n_presets, n_frames, figsize=(5 * n_frames, 4 * n_presets))
    if n_presets == 1:
        axes = axes[np.newaxis, :]
    if n_frames == 1:
        axes = axes[:, np.newaxis]

    for row, name in enumerate(preset_names):
        torch.manual_seed(42)
        aug_list = PRESETS[name]
        if aug_list:
            augmented = apply_augment_list(cameras, aug_list)
        else:
            augmented = cameras
        concat = robotwin_concat(augmented)

        for col, fi in enumerate(frame_indices):
            img = tensor_to_image(concat, fi)
            axes[row, col].imshow(img)
            axes[row, col].set_title(f"{name} | frame {fi}", fontsize=10)
            axes[row, col].axis("off")

    fig.suptitle("Data Augmentation Presets — robotwin layout (3 cameras)", fontsize=14, y=1.01)
    fig.tight_layout()
    path = os.path.join(output_dir, "presets_comparison.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {path}")


def visualize_individual(cameras, output_dir):
    """生成单个增强对比图：每行一个增强类型，每列一个帧。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    frame_indices = [0, 4, 8] if cameras["head_rgb"].shape[0] > 8 else [0]
    n_frames = len(frame_indices)

    aug_names = ["original"] + list(INDIVIDUAL_AUGMENTS.keys())
    n_rows = len(aug_names)

    fig, axes = plt.subplots(n_rows, n_frames, figsize=(5 * n_frames, 4 * n_rows))
    if n_rows == 1:
        axes = axes[np.newaxis, :]
    if n_frames == 1:
        axes = axes[:, np.newaxis]

    for row, name in enumerate(aug_names):
        torch.manual_seed(42)
        if name == "original":
            augmented = cameras
        else:
            aug = INDIVIDUAL_AUGMENTS[name]
            augmented = apply_augment_per_camera(cameras, aug)
        concat = robotwin_concat(augmented)

        for col, fi in enumerate(frame_indices):
            img = tensor_to_image(concat, fi)
            axes[row, col].imshow(img)
            axes[row, col].set_title(f"{name} | frame {fi}", fontsize=10)
            axes[row, col].axis("off")

    fig.suptitle("Individual Augmentations — robotwin layout (3 cameras)", fontsize=14, y=1.01)
    fig.tight_layout()
    path = os.path.join(output_dir, "individual_augmentations.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {path}")


def visualize_per_camera(cameras, output_dir):
    """生成 per-camera 对比：展示每个相机增强前后的效果。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    aug = VideoColorJitter(brightness=0.3, contrast=0.4, saturation=0.5, hue=0.08, p=1.0)
    camera_keys = list(cameras.keys())
    n_cameras = len(camera_keys)

    fig, axes = plt.subplots(2, n_cameras, figsize=(5 * n_cameras, 8))

    for col, key in enumerate(camera_keys):
        original = tensor_to_image(cameras[key], 0)
        torch.manual_seed(42 + col)
        augmented = aug(cameras[key].clone())
        aug_img = tensor_to_image(augmented, 0)

        axes[0, col].imshow(original)
        axes[0, col].set_title(f"{key}\noriginal", fontsize=10)
        axes[0, col].axis("off")

        axes[1, col].imshow(aug_img)
        axes[1, col].set_title(f"{key}\nColorJitter", fontsize=10)
        axes[1, col].axis("off")

    fig.suptitle("Per-Camera Augmentation (frame 0)", fontsize=14)
    fig.tight_layout()
    path = os.path.join(output_dir, "per_camera_comparison.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {path}")


def visualize_frame_consistency(cameras, output_dir):
    """验证帧间一致性：展示同一增强下不同帧的效果。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n_frames = min(cameras["head_rgb"].shape[0], 9)
    aug = VideoColorJitter(brightness=0.4, contrast=0.4, saturation=0.4, hue=0.08, p=1.0)

    torch.manual_seed(42)
    augmented = apply_augment_per_camera(cameras, aug)
    concat_aug = robotwin_concat(augmented)
    concat_orig = robotwin_concat(cameras)

    fig, axes = plt.subplots(2, n_frames, figsize=(3 * n_frames, 6))
    for col in range(n_frames):
        axes[0, col].imshow(tensor_to_image(concat_orig, col))
        axes[0, col].set_title(f"orig f{col}", fontsize=9)
        axes[0, col].axis("off")

        axes[1, col].imshow(tensor_to_image(concat_aug, col))
        axes[1, col].set_title(f"aug f{col}", fontsize=9)
        axes[1, col].axis("off")

    fig.suptitle("Frame Consistency Check — same ColorJitter params across all frames", fontsize=12)
    fig.tight_layout()
    path = os.path.join(output_dir, "frame_consistency.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Visualize FastWAM data augmentations")
    parser.add_argument("--sample_idx", type=int, default=0, help="Dataset sample index")
    parser.add_argument("--output_dir", type=str, default="b/test/augmentation_vis",
                        help="Output directory for PNG images")
    parser.add_argument("--data_dir", type=str, default=None,
                        help="R1 Pro data directory (default: $R1PRO_DATA/r1_pro_data_convert_chassis)")
    args = parser.parse_args()

    fastwam_root = os.environ.get("FASTWAM_ROOT", "/home/Luogang/SRC/Robot/FastWAM")
    data_dir = args.data_dir or os.path.join(
        os.environ.get("R1PRO_DATA", "/mnt/r/share/zwy/datasets/r1_pro_data_v2"),
        "r1_pro_data_convert_chassis",
    )

    if not os.path.isdir(data_dir):
        print(f"ERROR: Data directory not found: {data_dir}")
        print("Set R1PRO_DATA or pass --data_dir")
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    print("=" * 60)
    print("FastWAM Data Augmentation Visualizer")
    print("=" * 60)
    print(f"Data dir:   {data_dir}")
    print(f"Sample idx: {args.sample_idx}")
    print(f"Output dir: {args.output_dir}")
    print()

    cameras, task = load_sample(data_dir, fastwam_root, args.sample_idx)

    print("\nGenerating visualizations...")
    # visualize_presets(cameras, args.output_dir)
    visualize_individual(cameras, args.output_dir)
    # visualize_per_camera(cameras, args.output_dir)
    # visualize_frame_consistency(cameras, args.output_dir)

    print(f"\nDone. All images saved to {args.output_dir}/")
    print("Files:")
    for f in sorted(os.listdir(args.output_dir)):
        if f.endswith(".png"):
            path = os.path.join(args.output_dir, f)
            size_kb = os.path.getsize(path) / 1024
            print(f"  {f} ({size_kb:.0f} KB)")


if __name__ == "__main__":
    main()
