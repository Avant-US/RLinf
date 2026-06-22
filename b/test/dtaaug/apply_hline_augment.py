#!/usr/bin/env python3
"""Apply a frame-consistent random horizontal line to video frames.

All frames in a clip share the same line row (and color when sampled once).
The transform class lives in ``rlinf.data.aug.augmentation``.

Examples:
    python b/test/apply_hline_augment.py --demo --output_dir b/test/hline_vis
    python b/test/apply_hline_augment.py --input clip.mp4 --output clip_hline.mp4
    python b/test/apply_hline_augment.py --r1_sample --sample_idx 0
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from pathlib import Path

import numpy as np
import torch

REPO_ROOT = Path(__file__).resolve().parents[2]


def _load_augmentation_module():
    module_path = REPO_ROOT / "rlinf" / "data" / "datasets" / "fastwam" / "augmentation.py"
    spec = importlib.util.spec_from_file_location("fastwam_augmentation", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load augmentation module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_aug = _load_augmentation_module()
VideoRandomHorizontalLine = _aug.VideoRandomHorizontalLine


def _require_cv2():
    try:
        import cv2
    except ImportError as exc:
        raise SystemExit("OpenCV required: pip install opencv-python") from exc
    return cv2


def read_video(path: str) -> tuple[torch.Tensor, float]:
    cv2 = _require_cv2()
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise FileNotFoundError(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frames = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    cap.release()
    if not frames:
        raise ValueError(f"No frames in {path}")
    video = torch.from_numpy(np.stack(frames)).permute(0, 3, 1, 2).float() / 255.0
    return video, fps


def write_video(path: str, video: torch.Tensor, fps: float) -> None:
    cv2 = _require_cv2()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    frames = (video.clamp(0, 1).permute(0, 2, 3, 1).cpu().numpy() * 255).astype(np.uint8)
    h, w = frames.shape[1:3]
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for frame in frames:
        writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    writer.release()


def make_checkerboard_video(num_frames: int = 9, height: int = 384, width: int = 320) -> torch.Tensor:
    yy, xx = torch.meshgrid(
        torch.linspace(0, 1, height),
        torch.linspace(0, 1, width),
        indexing="ij",
    )
    checker = (((yy * 16).floor() + (xx * 16).floor()) % 2).float()
    rgb = torch.stack([checker, yy, xx], dim=0)
    return rgb.unsqueeze(0).expand(num_frames, -1, -1, -1).clone()


def apply_hline(
    video: torch.Tensor,
    *,
    line_width: int = 2,
    color: tuple[float, float, float] | None = None,
    row_range: tuple[float, float] = (0.0, 1.0),
    seed: int | None = 42,
) -> torch.Tensor:
    if seed is not None:
        torch.manual_seed(seed)
    augment = VideoRandomHorizontalLine(
        line_width=line_width,
        color=color,
        row_range=row_range,
        p=1.0,
    )
    return augment(video)


def save_comparison(
    original: torch.Tensor,
    augmented: torch.Tensor,
    output_path: str,
    frame_indices: list[int] | None = None,
    title: str = "Horizontal Line Augmentation",
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if frame_indices is None:
        frame_indices = [0, min(4, original.shape[0] - 1), min(8, original.shape[0] - 1)]
    frame_indices = [i for i in frame_indices if i < original.shape[0]]
    n = len(frame_indices)

    fig, axes = plt.subplots(2, n, figsize=(4 * n, 6))
    if n == 1:
        axes = axes.reshape(2, 1)

    for col, fi in enumerate(frame_indices):
        for row, tensor in enumerate([original, augmented]):
            img = (tensor[fi].permute(1, 2, 0).clamp(0, 1).numpy() * 255).astype(np.uint8)
            axes[row, col].imshow(img)
            axes[row, col].axis("off")
            if row == 0:
                axes[row, col].set_title(f"orig f{fi}", fontsize=9)
            else:
                axes[row, col].set_title(f"hline f{fi}", fontsize=9)

    fig.suptitle(f"{title}\n(same row on all frames)", fontsize=12)
    fig.tight_layout()
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def load_r1_robotwin_video(data_dir: str, fastwam_root: str, sample_idx: int) -> torch.Tensor:
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
            {"key": "left_wrist_rgb", "lerobot_key": "left_wrist_rgb", "raw_shape": [3, 480, 640], "shape": [3, 240, 320]},
            {"key": "right_wrist_rgb", "lerobot_key": "right_wrist_rgb", "raw_shape": [3, 480, 640], "shape": [3, 240, 320]},
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
        action_state_merger=ConcatLeftAlign(),
        train_transforms=[ToTensor(), Resize([240, 320])],
        val_transforms=[ToTensor(), Resize([240, 320])],
    )
    work_dir = os.path.join(os.path.dirname(__file__), "hline_vis")
    os.makedirs(work_dir, exist_ok=True)
    register_work_dir(work_dir)

    dataset = RobotVideoDataset(
        dataset_dirs=[data_dir],
        shape_meta=OmegaConf.create(shape_meta),
        processor=processor,
        num_frames=33,
        action_video_freq_ratio=4,
        video_size=[384, 320],
        text_embedding_cache_dir=os.path.join(fastwam_root, "data", "text_embeds_cache", "r1_pro_chassis"),
        context_len=128,
        concat_multi_camera="robotwin",
        is_training_set=True,
    )
    pixel_values = dataset.lerobot_dataset[sample_idx]["pixel_values"]
    video_indices = list(range(0, 33, 4))

    import torchvision.transforms.functional as F

    cameras = {shape_meta["images"][i]["key"]: pixel_values[i][video_indices] for i in range(3)}
    top = F.resize(cameras["head_rgb"], [256, 320], antialias=True)
    left = F.resize(cameras["left_wrist_rgb"], [128, 160], antialias=True)
    right = F.resize(cameras["right_wrist_rgb"], [128, 160], antialias=True)
    return torch.cat([top, torch.cat([left, right], dim=-1)], dim=-2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Frame-consistent random horizontal line augmentation")
    parser.add_argument("--input", type=str, default=None)
    parser.add_argument("--output", type=str, default=None)
    parser.add_argument("--output_dir", type=str, default="b/test/hline_vis")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--r1_sample", action="store_true")
    parser.add_argument("--sample_idx", type=int, default=0)
    parser.add_argument("--data_dir", type=str, default=None)
    parser.add_argument("--line_width", type=int, default=2)
    parser.add_argument("--row_min", type=float, default=0.0)
    parser.add_argument("--row_max", type=float, default=1.0)
    parser.add_argument("--color", type=float, nargs=3, default=None, metavar=("R", "G", "B"))
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    row_range = (args.row_min, args.row_max)
    color = tuple(args.color) if args.color is not None else None

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
            raise SystemExit(f"Data not found: {data_dir}")
        video = load_r1_robotwin_video(data_dir, fastwam_root, args.sample_idx)
        title = f"R1 Pro sample {args.sample_idx}"
        fps = 30.0
    elif args.input:
        video, fps = read_video(args.input)
        title = os.path.basename(args.input)
    else:
        parser.error("Use --demo, --r1_sample, or --input")

    augmented = apply_hline(
        video,
        line_width=args.line_width,
        color=color,
        row_range=row_range,
        seed=args.seed,
    )

    out_png = os.path.join(args.output_dir, "hline_comparison.png")
    save_comparison(video, augmented, out_png, title=title)
    print(f"Saved: {out_png}")

    if args.input and args.output:
        write_video(args.output, augmented, fps)
        print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
