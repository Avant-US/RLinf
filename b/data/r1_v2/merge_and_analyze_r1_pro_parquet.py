#!/usr/bin/env python3
"""Merge chunk-000 episode parquets and analyze per-field distributions."""

from __future__ import annotations

import argparse
import io
import json
import time
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq
from PIL import Image


def merge_parquets(input_dir: Path, output_path: Path) -> dict:
    files = sorted(input_dir.glob("episode_*.parquet"))
    if not files:
        raise FileNotFoundError(f"No episode_*.parquet in {input_dir}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        output_path.unlink()

    writer = None
    total_rows = 0
    t0 = time.time()
    for i, fp in enumerate(files):
        table = pq.read_table(fp)
        total_rows += table.num_rows
        if writer is None:
            writer = pq.ParquetWriter(
                output_path,
                table.schema,
                compression="snappy",
                use_dictionary=True,
            )
        else:
            if table.schema != writer.schema:
                table = table.cast(writer.schema, safe=False)
        writer.write_table(table)
        print(f"  [{i + 1}/{len(files)}] {fp.name}: {table.num_rows} rows (cum {total_rows})")

    if writer is not None:
        writer.close()

    elapsed = time.time() - t0
    size_gb = output_path.stat().st_size / (1024**3)
    meta = {
        "num_files": len(files),
        "total_rows": total_rows,
        "output_path": str(output_path),
        "size_gb": round(size_gb, 3),
        "merge_seconds": round(elapsed, 1),
    }
    print(f"Merged {len(files)} files -> {output_path} ({size_gb:.2f} GB, {elapsed:.1f}s)")
    return meta


def _percentiles(arr: np.ndarray, ps=(1, 5, 25, 50, 75, 95, 99)) -> dict:
    if arr.size == 0:
        return {}
    return {f"p{p}": float(np.percentile(arr, p)) for p in ps}


class _RunningStats:
    """Welford + min/max; optional reservoir for percentiles."""

    def __init__(self, reservoir_size: int = 200_000, seed: int = 42):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_v = np.inf
        self.max_v = -np.inf
        self._reservoir: list[float] = []
        self._reservoir_size = reservoir_size
        self._rng = np.random.default_rng(seed)

    def update(self, x: np.ndarray) -> None:
        x = np.asarray(x, dtype=np.float64).ravel()
        x = x[np.isfinite(x)]
        if x.size == 0:
            return
        self.min_v = min(self.min_v, float(x.min()))
        self.max_v = max(self.max_v, float(x.max()))
        for v in x:
            self.n += 1
            d = v - self.mean
            self.mean += d / self.n
            self.m2 += d * (v - self.mean)
            if len(self._reservoir) < self._reservoir_size:
                self._reservoir.append(float(v))
            else:
                j = int(self._rng.integers(0, self.n))
                if j < self._reservoir_size:
                    self._reservoir[j] = float(v)

    def finalize(self) -> dict:
        std = float(np.sqrt(self.m2 / self.n)) if self.n > 1 else 0.0
        out = {
            "count": self.n,
            "mean": float(self.mean) if self.n else None,
            "std": std,
            "min": float(self.min_v) if self.n else None,
            "max": float(self.max_v) if self.n else None,
        }
        if self._reservoir:
            out["percentiles"] = _percentiles(np.array(self._reservoir))
        return out


class _PerDimStats:
    def __init__(self, dim: int, reservoir_size: int = 50_000, seed: int = 42):
        self.dim = dim
        self.stats = [_RunningStats(reservoir_size=reservoir_size, seed=seed + d) for d in range(dim)]

    def update_batch(self, mat: np.ndarray) -> None:
        for d in range(self.dim):
            self.stats[d].update(mat[:, d])

    def finalize(self) -> list[dict]:
        rows = []
        for d, s in enumerate(self.stats):
            f = s.finalize()
            rows.append({"dim": d, **f})
        return rows


def _actions_minus_state_report(diff_stats: _PerDimStats) -> dict:
    per_dim = diff_stats.finalize()
    return {
        "per_dimension_mean_abs_diff": [float(d["mean"]) for d in per_dim],
        "per_dimension": per_dim,
    }


def analyze_merged(merged_path: Path, sample_pixels: int, seed: int) -> dict:
    print(f"Analyzing {merged_path} (streaming batches) ...")
    t0 = time.time()
    pf = pq.ParquetFile(merged_path)
    schema = pf.schema_arrow
    rng = np.random.default_rng(seed)

    scalar_stats = {c: _RunningStats() for c in ["timestamp", "frame_index", "index"]}
    state_stats = _PerDimStats(23, seed=seed)
    action_stats = _PerDimStats(23, seed=seed + 100)
    global_vec_stats = {"state": _RunningStats(), "actions": _RunningStats()}
    diff_stats = _PerDimStats(23, seed=seed + 200)

    episode_frame_counts: dict[int, int] = {}
    task_counts: dict[int, int] = {}
    prev_index: int | None = None
    index_gaps = 0

    img_byte_lens: dict[str, list[int]] = {
        k: [] for k in ["head_rgb", "left_wrist_rgb", "right_wrist_rgb"]
    }
    img_reservoir: dict[str, list[tuple[int, bytes]]] = {
        k: [] for k in img_byte_lens
    }
    img_reservoir_cap = sample_pixels
    total_rows = 0
    row_offset = 0

    for batch_idx, batch in enumerate(pf.iter_batches(batch_size=512)):
        n = batch.num_rows
        total_rows += n
        ts = batch.column("timestamp").to_numpy(zero_copy_only=False)
        fi = batch.column("frame_index").to_numpy(zero_copy_only=False)
        idx = batch.column("index").to_numpy(zero_copy_only=False)
        ep = batch.column("episode_index").to_numpy(zero_copy_only=False)
        task = batch.column("task_index").to_numpy(zero_copy_only=False)

        scalar_stats["timestamp"].update(ts)
        scalar_stats["frame_index"].update(fi.astype(np.float64))
        scalar_stats["index"].update(idx.astype(np.float64))

        if prev_index is not None:
            if int(idx[0]) != prev_index + 1:
                index_gaps += 1
        for j in range(n - 1):
            if int(idx[j + 1]) != int(idx[j]) + 1:
                index_gaps += 1
        prev_index = int(idx[-1])

        for e in ep:
            episode_frame_counts[int(e)] = episode_frame_counts.get(int(e), 0) + 1
        for t in task:
            task_counts[int(t)] = task_counts.get(int(t), 0) + 1

        states = np.stack(
            [np.asarray(batch.column("state")[i].as_py(), dtype=np.float32) for i in range(n)]
        )
        actions = np.stack(
            [np.asarray(batch.column("actions")[i].as_py(), dtype=np.float32) for i in range(n)]
        )
        state_stats.update_batch(states)
        action_stats.update_batch(actions)
        global_vec_stats["state"].update(states)
        global_vec_stats["actions"].update(actions)
        diff_stats.update_batch(np.abs(actions - states))

        for cam in img_byte_lens:
            col = batch.column(cam)
            for i in range(n):
                b = col[i].as_py()["bytes"]
                blen = len(b)
                img_byte_lens[cam].append(blen)
                gidx = row_offset + i
                if len(img_reservoir[cam]) < img_reservoir_cap:
                    img_reservoir[cam].append((gidx, b))
                else:
                    j = int(rng.integers(0, gidx + 1))
                    if j < img_reservoir_cap:
                        img_reservoir[cam][j] = (gidx, b)

        row_offset += n
        if (batch_idx + 1) % 20 == 0:
            print(f"    batch {batch_idx + 1}, rows so far {total_rows:,}")

    def _image_report(cam: str) -> dict:
        lens = np.array(img_byte_lens[cam], dtype=np.int64)
        out = {
            "storage": "PNG bytes embedded in struct{bytes, path}",
            "count": int(len(lens)),
            "png_bytes": {
                "mean_kb": float(lens.mean() / 1024),
                "std_kb": float(lens.std() / 1024),
                "min_kb": float(lens.min() / 1024),
                "max_kb": float(lens.max() / 1024),
                "total_gb": float(lens.sum() / (1024**3)),
                "percentiles_kb": {
                    k: v / 1024 for k, v in _percentiles(lens.astype(np.float64)).items()
                },
            },
        }
        if img_reservoir[cam]:
            pixels = []
            widths, heights = [], []
            for _, b in img_reservoir[cam]:
                img = Image.open(io.BytesIO(b))
                widths.append(img.size[0])
                heights.append(img.size[1])
                pixels.append(np.asarray(img, dtype=np.float32) / 255.0)
            pix = np.stack(pixels)
            out["decoded_sample"] = {
                "n_sampled": len(img_reservoir[cam]),
                "resolution_wh": {
                    "unique": list({(w, h) for w, h in zip(widths, heights)}),
                },
                "pixel_stats_rgb": {
                    "mean_per_channel": [float(pix[:, :, :, c].mean()) for c in range(3)],
                    "std_per_channel": [float(pix[:, :, :, c].std()) for c in range(3)],
                    "min": float(pix.min()),
                    "max": float(pix.max()),
                    "percentiles": _percentiles(pix.reshape(-1)),
                },
            }
        return out

    ep_counts = np.array(list(episode_frame_counts.values()), dtype=np.int64)
    report = {
        "schema": str(schema),
        "num_rows": total_rows,
        "num_columns": len(schema.names),
        "columns": list(schema.names),
        "fields": {
            "timestamp": scalar_stats["timestamp"].finalize(),
            "frame_index": {
                **scalar_stats["frame_index"].finalize(),
                "note": "0..(episode_len-1) within each episode",
            },
            "episode_index": {
                "episodes": len(episode_frame_counts),
                "min": int(min(episode_frame_counts)),
                "max": int(max(episode_frame_counts)),
                "frames_per_episode": {
                    **{
                        k: float(v)
                        for k, v in _percentiles(ep_counts.astype(np.float64)).items()
                    },
                    "min": int(ep_counts.min()),
                    "max": int(ep_counts.max()),
                    "mean": float(ep_counts.mean()),
                    "std": float(ep_counts.std()),
                },
                "per_episode_lengths": {
                    str(k): v for k, v in sorted(episode_frame_counts.items())
                },
            },
            "index": {
                **scalar_stats["index"].finalize(),
                "index_discontinuities": index_gaps,
            },
            "task_index": {"value_counts": task_counts},
            "state": {
                "dtype": "fixed_size_list<float>[23]",
                "global_flat": global_vec_stats["state"].finalize(),
                "per_dimension": state_stats.finalize(),
            },
            "actions": {
                "dtype": "fixed_size_list<float>[23]",
                "global_flat": global_vec_stats["actions"].finalize(),
                "per_dimension": action_stats.finalize(),
            },
            "head_rgb": _image_report("head_rgb"),
            "left_wrist_rgb": _image_report("left_wrist_rgb"),
            "right_wrist_rgb": _image_report("right_wrist_rgb"),
        },
        "sanity": {
            "global_index_monotonic": index_gaps == 0,
            "index_gap_count": index_gaps,
            "episode_index_range": [
                int(min(episode_frame_counts)),
                int(max(episode_frame_counts)),
            ],
            "num_episodes": len(episode_frame_counts),
        },
        "actions_minus_state": _actions_minus_state_report(diff_stats),
        "analysis_seconds": round(time.time() - t0, 1),
    }
    return report


def render_markdown(merge_meta: dict, report: dict, modality_path: Path | None) -> str:
    lines = [
        "# R1 Pro chunk-000 合并 Parquet 数据分布分析",
        "",
        "## 合并结果",
        "",
        f"| 项 | 值 |",
        f"|----|-----|",
        f"| 源文件数 | {merge_meta['num_files']} |",
        f"| 总行数 | {merge_meta['total_rows']:,} |",
        f"| 输出路径 | `{merge_meta['output_path']}` |",
        f"| 文件大小 | {merge_meta['size_gb']:.2f} GB |",
        f"| 合并耗时 | {merge_meta['merge_seconds']:.1f} s |",
        "",
        "## Schema",
        "",
        f"```",
        f"{report['schema']}",
        f"```",
        "",
        f"共 **{report['num_rows']:,}** 行 × **{report['num_columns']}** 列。",
        "",
    ]

    if modality_path and modality_path.exists():
        mod = json.loads(modality_path.read_text())
        lines.extend(["## 23 维 state/actions 语义 (modality.json)", "", "```json"])
        lines.append(json.dumps(mod, indent=2)[:2000])
        lines.extend(["```", ""])

    for col_name, stats in report["fields"].items():
        lines.append(f"## 字段: `{col_name}`")
        lines.append("")
        if "png_bytes" in stats:
            pb = stats["png_bytes"]
            lines.extend(
                [
                    f"- 存储: {stats['storage']}",
                    f"- 帧数: {stats['count']:,}",
                    f"- PNG 体积: 均值 {pb['mean_kb']:.1f} KB, 标准差 {pb['std_kb']:.1f} KB",
                    f"  - 范围: [{pb['min_kb']:.1f}, {pb['max_kb']:.1f}] KB",
                    f"  - 合计: {pb['total_gb']:.2f} GB",
                    f"  - 分位数 (KB): " + ", ".join(f"{k}={v:.1f}" for k, v in pb["percentiles_kb"].items()),
                    "",
                ]
            )
            if "decoded_sample" in stats:
                ds = stats["decoded_sample"]
                lines.extend(
                    [
                        f"### 像素采样统计 (n={ds['n_sampled']})",
                        f"- 分辨率: {ds['resolution_wh']['unique']}",
                        f"- 通道均值 (RGB): {[round(x, 4) for x in ds['pixel_stats_rgb']['mean_per_channel']]}",
                        f"- 通道标准差: {[round(x, 4) for x in ds['pixel_stats_rgb']['std_per_channel']]}",
                        f"- 像素范围: [{ds['pixel_stats_rgb']['min']:.4f}, {ds['pixel_stats_rgb']['max']:.4f}]",
                        "",
                    ]
                )
        elif "per_dimension" in stats:
            g = stats.get("global_flat", {})
            lines.extend(
                [
                    f"- dtype: {stats.get('dtype', '')}",
                    f"- 全局展平: mean={g.get('mean', 0):.4f}, std={g.get('std', 0):.4f}, "
                    f"min={g.get('min', 0):.4f}, max={g.get('max', 0):.4f}",
                    f"- 分位数: {g.get('percentiles', {})}",
                    "",
                    "### 逐维统计",
                    "",
                    "| dim | mean | std | min | max | p5 | p50 | p95 |",
                    "|-----|------|-----|-----|-----|----|----|-----|",
                ]
            )
            for d in stats["per_dimension"]:
                p = d.get("percentiles") or {}
                lines.append(
                    f"| {d['dim']} | {d.get('mean', 0):.4f} | {d.get('std', 0):.4f} | "
                    f"{d.get('min', 0):.4f} | {d.get('max', 0):.4f} | "
                    f"{p.get('p5', 0):.4f} | {p.get('p50', 0):.4f} | {p.get('p95', 0):.4f} |"
                )
            lines.append("")
        else:
            lines.append("```json")
            lines.append(json.dumps(stats, indent=2, ensure_ascii=False))
            lines.append("```")
            lines.append("")

    lines.extend(
        [
            "## 交叉检查",
            "",
            "```json",
            json.dumps(
                {
                    "sanity": report.get("sanity"),
                    "actions_minus_state": report.get("actions_minus_state"),
                },
                indent=2,
            ),
            "```",
            "",
            f"*分析耗时: {report.get('analysis_seconds', '?')} s*",
        ]
    )
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path(
            "/mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis/data/chunk-000"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Merged parquet path (default: input-dir/merged_all_episodes.parquet)",
    )
    parser.add_argument("--skip-merge", action="store_true")
    parser.add_argument("--pixel-sample", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    output = args.output or (args.input_dir / "merged_all_episodes.parquet")
    report_json = output.with_suffix(".analysis.json")
    report_md = output.with_suffix(".analysis.md")
    modality = args.input_dir.parent.parent / "meta" / "modality.json"

    if not args.skip_merge:
        merge_meta = merge_parquets(args.input_dir, output)
    else:
        if not output.exists():
            raise FileNotFoundError(output)
        pf = pq.ParquetFile(output)
        merge_meta = {
            "num_files": len(list(args.input_dir.glob("episode_*.parquet"))),
            "total_rows": pf.metadata.num_rows,
            "output_path": str(output),
            "size_gb": round(output.stat().st_size / (1024**3), 3),
            "merge_seconds": 0,
        }

    report = analyze_merged(output, sample_pixels=args.pixel_sample, seed=args.seed)
    report_json.write_text(json.dumps({"merge": merge_meta, "analysis": report}, indent=2))
    report_md.write_text(render_markdown(merge_meta, report, modality))
    print(f"Wrote {report_json}")
    print(f"Wrote {report_md}")


if __name__ == "__main__":
    main()
