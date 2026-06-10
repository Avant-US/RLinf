#!/usr/bin/env python3
"""Pack augmented episode MP4s with originals and side-by-side compare clips."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import zipfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DATA_ROOT = Path(
    os.environ.get(
        "R1PRO_DATA",
        "/mnt/r/share/zwy/datasets/r1_pro_data_v2",
    )
) / "r1_pro_data_convert_chassis"
DEFAULT_SRC_DIR = REPO_ROOT / "b" / "test" / "trnsf_tst"
CAMERAS = ("head_rgb", "left_wrist_rgb", "right_wrist_rgb")
COMPARE_W, COMPARE_H = 240, 320


def _video_info(path: Path) -> dict:
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-count_packets",
        "-show_entries",
        "stream=width,height,nb_read_packets,r_frame_rate",
        "-of",
        "json",
        str(path),
    ]
    out = subprocess.check_output(cmd, text=True)
    stream = json.loads(out)["streams"][0]
    fps_parts = stream["r_frame_rate"].split("/")
    fps = float(fps_parts[0]) / float(fps_parts[1]) if len(fps_parts) == 2 else float(fps_parts[0])
    return {
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "frames": int(stream["nb_read_packets"]),
        "fps": round(fps, 3),
        "size_kb": path.stat().st_size // 1024,
    }


def _original_video_path(data_root: Path, episode_index: int, camera: str) -> Path:
    chunk = episode_index // 1000
    return (
        data_root
        / "videos_backup"
        / f"chunk-{chunk:03d}"
        / f"episode_{episode_index:06d}_{camera}.mp4"
    )


def _make_compare_video(
    original: Path,
    augmented: Path,
    out_path: Path,
    compare_seconds: float,
) -> None:
    filt = (
        f"[0:v]scale={COMPARE_W}:{COMPARE_H}:force_original_aspect_ratio=decrease,"
        f"pad={COMPARE_W}:{COMPARE_H}:(ow-iw)/2:(oh-ih)/2,setsar=1[left];"
        f"[1:v]scale={COMPARE_W}:{COMPARE_H},setsar=1[right];"
        f"[left][right]hstack=inputs=2[v]"
    )
    cmd = [
        "ffmpeg",
        "-y",
        "-loglevel",
        "error",
        "-t",
        str(compare_seconds),
        "-i",
        str(original),
        "-t",
        str(compare_seconds),
        "-i",
        str(augmented),
        "-filter_complex",
        filt,
        "-map",
        "[v]",
        "-c:v",
        "libx264",
        "-preset",
        "ultrafast",
        "-crf",
        "28",
        "-pix_fmt",
        "yuv420p",
        str(out_path),
    ]
    subprocess.run(cmd, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Pack augmented vs original episode MP4s")
    parser.add_argument(
        "--episode_indices",
        type=str,
        default="18,27,32,41,53,61",
    )
    parser.add_argument("--src_dir", type=Path, default=DEFAULT_SRC_DIR)
    parser.add_argument("--data_root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument(
        "--zip_name",
        type=str,
        default="trnsf_tst_compare.zip",
    )
    parser.add_argument(
        "--compare_seconds",
        type=float,
        default=15.0,
        help="Side-by-side preview length (full videos still included separately)",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=6,
        help="Parallel ffmpeg workers for compare previews",
    )
    parser.add_argument(
        "--skip_compare",
        action="store_true",
        help="Only pack augmented + original full videos",
    )
    args = parser.parse_args()

    episodes = [int(x.strip()) for x in args.episode_indices.split(",") if x.strip()]
    staging = args.src_dir / "_pack_staging"
    if staging.exists():
        shutil.rmtree(staging)
    aug_dir = staging / "augmented"
    orig_dir = staging / "original"
    cmp_dir = staging / f"compare_preview_{int(args.compare_seconds)}s"
    for d in (aug_dir, orig_dir, cmp_dir):
        d.mkdir(parents=True)

    manifest: list[dict] = []
    missing: list[str] = []
    compare_jobs: list[tuple[Path, Path, Path]] = []

    for ep in episodes:
        for cam in CAMERAS:
            name = f"episode_{ep:06d}_{cam}.mp4"
            aug_src = args.src_dir / name
            orig_src = _original_video_path(args.data_root, ep, cam)
            if not aug_src.is_file():
                missing.append(f"augmented missing: {aug_src}")
                continue
            if not orig_src.is_file():
                missing.append(f"original missing: {orig_src}")
                continue

            aug_dst = aug_dir / name
            orig_dst = orig_dir / name
            cmp_dst = cmp_dir / f"episode_{ep:06d}_{cam}_orig_vs_aug.mp4"
            shutil.copy2(aug_src, aug_dst)
            shutil.copy2(orig_src, orig_dst)
            if not args.skip_compare:
                compare_jobs.append((orig_dst, aug_dst, cmp_dst))

            entry = {
                "episode_index": ep,
                "camera": cam,
                "original": _video_info(orig_dst),
                "augmented": _video_info(aug_dst),
            }
            manifest.append(entry)
            print(f"copied {name}")

    if compare_jobs:
        print(f"building {len(compare_jobs)} compare previews ({args.compare_seconds}s)...")

        def _run_compare(job: tuple[Path, Path, Path]) -> str:
            orig_dst, aug_dst, cmp_dst = job
            _make_compare_video(orig_dst, aug_dst, cmp_dst, args.compare_seconds)
            return cmp_dst.name

        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = [pool.submit(_run_compare, job) for job in compare_jobs]
            for fut in as_completed(futures):
                print(f"  compare done: {fut.result()}")

        by_key = {(m["episode_index"], m["camera"]): m for m in manifest}
        for ep in episodes:
            for cam in CAMERAS:
                cmp_path = cmp_dir / f"episode_{ep:06d}_{cam}_orig_vs_aug.mp4"
                key = (ep, cam)
                if key in by_key and cmp_path.is_file():
                    by_key[key]["compare_preview"] = _video_info(cmp_path)

    compare_line = (
        f"  {cmp_dir.name}/ - left=original, right=augmented "
        f"(first {args.compare_seconds}s, both scaled to 240x320)"
        if not args.skip_compare
        else "  (compare previews skipped)"
    )
    readme = staging / "README.txt"
    readme.write_text(
        "\n".join(
            [
                "Transform augment comparison package",
                "====================================",
                "",
                "Config: examples/sft/config/r1_pro_sft_fastwam.yaml train_transforms",
                f"Episodes: {', '.join(str(e) for e in episodes)}",
                f"Cameras: {', '.join(CAMERAS)}",
                "",
                "Folders:",
                "  augmented/     - full episode after train_transforms (single random draw)",
                "  original/      - raw dataset MP4 (videos_backup, no augmentation)",
                compare_line,
                "",
                "Note: train_transforms are stochastic (crop/color jitter/erasing/...).",
                "      Augmented full episodes reflect one random application per episode.",
                "",
                f"Missing files: {len(missing)}",
                *missing,
                "",
                "See manifest.json for resolution / frame count / file size.",
            ]
        ),
        encoding="utf-8",
    )
    (staging / "manifest.json").write_text(
        json.dumps(manifest, indent=2),
        encoding="utf-8",
    )

    zip_path = args.src_dir / args.zip_name
    if zip_path.exists():
        zip_path.unlink()
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(staging.rglob("*")):
            if path.is_file():
                zf.write(path, path.relative_to(staging).as_posix())

    shutil.rmtree(staging)
    print(f"\nCreated: {zip_path} ({zip_path.stat().st_size // (1024 * 1024)} MB)")
    print(f"Entries: {len(manifest)} camera-episodes x 3 variants")
    if missing:
        print("WARNING: some files were missing; see README inside zip.")


if __name__ == "__main__":
    main()
