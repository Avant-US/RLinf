#!/usr/bin/env python3
"""Apply convex bulging fisheye distortion to video frames.

Supports:
  1. MP4 / image-sequence input -> MP4 output
  2. Synthetic demo grid (no external data)
  3. R1 Pro LeRobot sample via FastWAM dataset (optional)

The core transform lives in ``rlinf.data.aug.augmentation``.

Examples:
    # Synthetic before/after PNG
    python b/test/apply_fisheye_augment.py --demo --output_dir b/test/fisheye_vis

    # MP4 file (k in (0,1); higher = stronger edge-to-center bulge)
    python b/test/dtaaug/apply_fisheye_augment.py \\
        --input /path/to/clip.mp4 \\
        --output /path/to/clip_fisheye.mp4 \\
        --k 0.35

    # Sweep multiple strengths into output_dir/k_0.25/, k_0.40/, ...
    python b/test/dtaaug/apply_fisheye_augment.py \\
        --input /path/to/clip.mp4 \\
        --output_dir b/test/dtaaug/fisheye_sweep \\
        --k_sweep 0.20 0.35 0.50 0.65

    # R1 Pro sample (needs FASTWAM_PATH + R1PRO_DATA)
    python b/test/apply_fisheye_augment.py \\
        --r1_sample --sample_idx 0 --output_dir b/test/fisheye_vis
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import torch

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _load_augmentation_module():
    """Load augmentation.py without importing fastwam package __init__ (torchdata)."""
    import importlib.util

    module_path = REPO_ROOT / "rlinf" / "data" / "datasets" / "fastwam" / "augmentation.py"
    spec = importlib.util.spec_from_file_location("fastwam_augmentation", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load augmentation module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_aug = _load_augmentation_module()
VideoRandomFisheye = _aug.VideoRandomFisheye


def _require_cv2():
    try:
        import cv2
    except ImportError as exc:
        raise SystemExit("OpenCV is required for video I/O. Install with: pip install opencv-python") from exc
    return cv2


def read_video(path: str) -> tuple[torch.Tensor, float]:
    """Read video into float tensor ``[T, C, H, W]`` in ``[0, 1]``."""
    cv2 = _require_cv2()
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Cannot open video: {path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frames = []
    while True:
        ok, frame_bgr = cap.read()
        if not ok:
            break
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frames.append(frame_rgb)
    cap.release()

    if not frames:
        raise ValueError(f"No frames read from {path}")

    video = torch.from_numpy(np.stack(frames, axis=0)).permute(0, 3, 1, 2).float() / 255.0
    return video, fps


def write_video(path: str, video: torch.Tensor, fps: float) -> None:
    """Write ``[T, C, H, W]`` float tensor to MP4."""
    cv2 = _require_cv2()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

    video_u8 = (video.clamp(0, 1).permute(0, 2, 3, 1).cpu().numpy() * 255).astype(np.uint8)
    height, width = video_u8.shape[1:3]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(path, fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Cannot create video writer: {path}")

    for frame in video_u8:
        writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    writer.release()


def save_comparison_png(
    original: torch.Tensor,
    augmented: torch.Tensor,
    output_path: str,
    frame_idx: int = 0,
    title: str = "Fisheye Augmentation",
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def _to_img(tensor: torch.Tensor) -> np.ndarray:
        return (tensor[frame_idx].permute(1, 2, 0).clamp(0, 1).numpy() * 255).astype(np.uint8)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].imshow(_to_img(original))
    axes[0].set_title("Original")
    axes[0].axis("off")
    axes[1].imshow(_to_img(augmented))
    axes[1].set_title("Fisheye")
    axes[1].axis("off")
    fig.suptitle(title, fontsize=13)
    fig.tight_layout()
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def make_checkerboard_video(num_frames: int = 9, height: int = 384, width: int = 320) -> torch.Tensor:
    """Synthetic video with grid lines to make warp visible."""
    yy, xx = torch.meshgrid(
        torch.linspace(0, 1, height),
        torch.linspace(0, 1, width),
        indexing="ij",
    )
    checker = (((yy * 16).floor() + (xx * 16).floor()) % 2).float()
    rgb = torch.stack([checker, yy, xx], dim=0)
    return rgb.unsqueeze(0).expand(num_frames, -1, -1, -1).clone()


def apply_fisheye(
    video: torch.Tensor,
    *,
    k: float | None = None,
    k_range: tuple[float, float] = (0.20, 0.45),
    center_jitter: float = 0.02,
    seed: int | None = 42,
) -> torch.Tensor:
    """Apply fisheye with optional fixed ``k`` (otherwise sample from ``k_range``)."""
    if seed is not None:
        torch.manual_seed(seed)
    if k is not None:
        k_range = (k, k)
        center_jitter = 0.0
    augment = VideoRandomFisheye(k_range=k_range, center_jitter=center_jitter, p=1.0)
    return augment(video)


def load_r1_sample(data_dir: str, fastwam_root: str, sample_idx: int = 0) -> dict[str, torch.Tensor]:
    """Load per-camera tensors ``[T, C, H, W]`` from R1 Pro dataset."""
    fastwam_path = os.path.join(fastwam_root, "src")
    if fastwam_path not in sys.path:
        sys.path.insert(0, fastwam_path)

    from omegaconf import OmegaConf
    from torchvision.transforms import Resize

    from rlinf.data.datasets.fastwam.processor import build_proprio_aug_processor_cls
    FastWAMProcessor = build_proprio_aug_processor_cls()
    from fastwam.datasets.lerobot.robot_video_dataset import RobotVideoDataset
    from fastwam.datasets.lerobot.transforms.action_state_merger import ConcatLeftAlign
    from fastwam.datasets.lerobot.transforms.image import ToTensor
    from fastwam.utils.misc import register_work_dir

    shape_meta = {
        "images": [
            {"key": "head_rgb", "lerobot_key": "head_rgb", "raw_shape": [3, 360, 640], "shape": [3, 240, 320]},
            {
                "key": "left_wrist_rgb",
                "lerobot_key": "left_wrist_rgb",
                "raw_shape": [3, 480, 640],
                "shape": [3, 240, 320],
            },
            {
                "key": "right_wrist_rgb",
                "lerobot_key": "right_wrist_rgb",
                "raw_shape": [3, 480, 640],
                "shape": [3, 240, 320],
            },
        ],
        "action": [{"key": "default", "lerobot_key": "actions", "raw_shape": 23, "shape": 23}],
        "state": [{"key": "default", "lerobot_key": "state", "raw_shape": 23, "shape": 23}],
    }

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

    work_dir = os.path.join(os.path.dirname(__file__), "fisheye_vis")
    os.makedirs(work_dir, exist_ok=True)
    register_work_dir(work_dir)

    text_cache_dir = os.path.join(fastwam_root, "data", "text_embeds_cache", "r1_pro_chassis")
    dataset = RobotVideoDataset(
        dataset_dirs=[data_dir],
        shape_meta=OmegaConf.create(shape_meta),
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

    raw_sample = dataset.lerobot_dataset[sample_idx]
    pixel_values = raw_sample["pixel_values"]
    video_indices = list(range(0, 33, 4))
    cameras = {}
    for i, meta in enumerate(shape_meta["images"]):
        key = meta["key"]
        cameras[key] = pixel_values[i][video_indices]
    return cameras


def robotwin_concat(cameras: dict[str, torch.Tensor]) -> torch.Tensor:
    import torchvision.transforms.functional as F

    cam_top = F.resize(
        cameras["head_rgb"],
        [256, 320],
        interpolation=F.InterpolationMode.BILINEAR,
        antialias=True,
    )
    cam_left = F.resize(
        cameras["left_wrist_rgb"],
        [128, 160],
        interpolation=F.InterpolationMode.BILINEAR,
        antialias=True,
    )
    cam_right = F.resize(
        cameras["right_wrist_rgb"],
        [128, 160],
        interpolation=F.InterpolationMode.BILINEAR,
        antialias=True,
    )
    bottom = torch.cat([cam_left, cam_right], dim=-1)
    return torch.cat([cam_top, bottom], dim=-2)


def _run_one(
    video: torch.Tensor,
    *,
    fps: float | None,
    output_dir: str,
    output_mp4: str | None,
    title: str,
    k: float | None,
    k_range: tuple[float, float],
    center_jitter: float,
    seed: int | None,
    frame_idx: int,
) -> None:
    os.makedirs(output_dir, exist_ok=True)
    augmented = apply_fisheye(
        video,
        k=k,
        k_range=k_range,
        center_jitter=center_jitter,
        seed=seed,
    )
    k_label = f"k={k:.2f}" if k is not None else f"k∈[{k_range[0]:.2f},{k_range[1]:.2f}]"
    png_path = os.path.join(output_dir, "fisheye_comparison.png")
    save_comparison_png(
        video,
        augmented,
        png_path,
        frame_idx=frame_idx,
        title=f"{title} ({k_label})",
    )
    print(f"Saved comparison PNG: {png_path}")
    if output_mp4 is not None and fps is not None:
        write_video(output_mp4, augmented, fps)
        print(f"Saved augmented MP4: {output_mp4}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Apply convex bulging fisheye distortion to video frames")
    parser.add_argument("--input", type=str, default=None, help="Input MP4 path")
    parser.add_argument("--output", type=str, default=None, help="Output MP4 path")
    parser.add_argument("--output_dir", type=str, default="b/test/dtaaug/fisheye_vis", help="PNG output directory")
    parser.add_argument("--demo", action="store_true", help="Run synthetic checkerboard demo")
    parser.add_argument("--r1_sample", action="store_true", help="Load R1 Pro sample instead of MP4")
    parser.add_argument("--sample_idx", type=int, default=0)
    parser.add_argument("--data_dir", type=str, default=None)
    parser.add_argument(
        "--k",
        type=float,
        default=None,
        help="Fixed fisheye strength in (0,1); higher = stronger bulge from frame edge",
    )
    parser.add_argument(
        "--k_sweep",
        type=float,
        nargs="+",
        default=None,
        metavar="K",
        help="Run multiple fixed k values; each saved under output_dir/k_XX/",
    )
    parser.add_argument("--k_min", type=float, default=0.20)
    parser.add_argument("--k_max", type=float, default=0.45)
    parser.add_argument("--center_jitter", type=float, default=0.02)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--frame_idx", type=int, default=0, help="Frame index for PNG preview")
    args = parser.parse_args()

    if args.k_sweep and args.k is not None:
        parser.error("Use either --k or --k_sweep, not both.")

    os.makedirs(args.output_dir, exist_ok=True)

    fps: float | None = None
    if args.demo:
        video = make_checkerboard_video()
        title = "Synthetic demo"
    elif args.r1_sample:
        fastwam_root = os.environ.get("FASTWAM_ROOT", "/home/Luogang/SRC/Robot/FastWAM")
        data_dir = args.data_dir or os.path.join(
            os.environ.get("R1PRO_DATA", "/mnt/r/share/zwy/datasets/r1_pro_data_v2"),
            "r1_pro_data_convert_chassis",
        )
        if not os.path.isdir(data_dir):
            raise SystemExit(f"Data directory not found: {data_dir}")
        cameras = load_r1_sample(data_dir, fastwam_root, args.sample_idx)
        video = robotwin_concat(cameras)
        title = f"R1 Pro sample {args.sample_idx} (robotwin layout)"
    elif args.input:
        video, fps = read_video(args.input)
        title = os.path.basename(args.input)
    else:
        parser.error("Specify one of --demo, --r1_sample, or --input")

    if args.k_sweep:
        if not args.input:
            parser.error("--k_sweep requires --input (MP4).")
        stem = Path(args.input).stem
        for k_val in args.k_sweep:
            subdir = os.path.join(args.output_dir, f"k_{k_val:.2f}")
            out_mp4 = os.path.join(subdir, f"{stem}_fisheye_k{k_val:.2f}.mp4")
            _run_one(
                video,
                fps=fps,
                output_dir=subdir,
                output_mp4=out_mp4,
                title=title,
                k=k_val,
                k_range=(args.k_min, args.k_max),
                center_jitter=args.center_jitter,
                seed=args.seed,
                frame_idx=args.frame_idx,
            )
        sweep_png = os.path.join(args.output_dir, "fisheye_strength_sweep.png")
        _save_k_sweep(video, sweep_png, ks=[0.0, *args.k_sweep])
        print(f"Saved strength sweep PNG: {sweep_png}")
        return

    _run_one(
        video,
        fps=fps,
        output_dir=args.output_dir,
        output_mp4=args.output if args.input else None,
        title=title,
        k=args.k,
        k_range=(args.k_min, args.k_max),
        center_jitter=args.center_jitter,
        seed=args.seed,
        frame_idx=args.frame_idx,
    )

    if args.demo or args.r1_sample:
        multi_k_path = os.path.join(args.output_dir, "fisheye_strength_sweep.png")
        _save_k_sweep(video, multi_k_path)
        print(f"Saved strength sweep PNG: {multi_k_path}")


def _save_k_sweep(
    video: torch.Tensor,
    output_path: str,
    *,
    ks: list[float] | None = None,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if ks is None:
        ks = [0.0, 0.15, 0.25, 0.35, 0.45, 0.55]
    fig, axes = plt.subplots(1, len(ks), figsize=(4 * len(ks), 4))
    frame_idx = 0
    for ax, k in zip(axes, ks, strict=True):
        if k == 0.0:
            img = video
        else:
            img = apply_fisheye(video, k=k, seed=0)
        frame = (img[frame_idx].permute(1, 2, 0).clamp(0, 1).numpy() * 255).astype(np.uint8)
        ax.imshow(frame)
        ax.set_title(f"k={k:.2f}")
        ax.axis("off")
    fig.suptitle("Fisheye strength sweep (same frame, frame-consistent across time)", fontsize=12)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
