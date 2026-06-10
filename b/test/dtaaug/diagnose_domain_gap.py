#!/usr/bin/env python3
"""Diagnose domain gap between old and new R1 Pro robot data.

Analyzes action/state distributions, z-score normalization risks, visual
statistics, and episode anomalies to pinpoint why a model trained on old
robot data fails on a new robot.

Modes:
  A) Old data only (default): detect inherent risks in training data
  B) Old vs New: compare distributions if new robot data is available

Examples:
    # Mode A: analyze old data
    python b/test/dtaaug/diagnose_domain_gap.py \\
        --old_data /mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis \\
        --output_dir b/test/dtaaug/domain_gap_report

    # Mode B: compare old vs new
    python b/test/dtaaug/diagnose_domain_gap.py \\
        --old_data /path/to/old_lerobot \\
        --new_data /path/to/new_lerobot \\
        --output_dir b/test/dtaaug/domain_gap_report
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys
from pathlib import Path

import numpy as np

DIM_NAMES_ACTION = [
    "left_arm_j0", "left_arm_j1", "left_arm_j2", "left_arm_j3",
    "left_arm_j4", "left_arm_j5", "left_arm_j6",
    "right_arm_j0", "right_arm_j1", "right_arm_j2", "right_arm_j3",
    "right_arm_j4", "right_arm_j5", "right_arm_j6",
    "left_gripper", "right_gripper",
    "chassis_pose_x", "chassis_pose_y", "chassis_pose_z", "chassis_pose_yaw",
    "chassis_vel_x", "chassis_vel_y", "chassis_vel_yaw",
]

DIM_GROUPS = {
    "left_arm": list(range(0, 7)),
    "right_arm": list(range(7, 14)),
    "grippers": [14, 15],
    "chassis_pose": list(range(16, 20)),
    "chassis_vel": list(range(20, 23)),
}

CAMERA_KEYS = ["head_rgb", "left_wrist_rgb", "right_wrist_rgb"]


def load_parquet_data(data_dir: str, max_rows: int | None = None) -> dict:
    """Load state/actions from LeRobot parquet files."""
    import pyarrow.parquet as pq

    data_path = Path(data_dir) / "data" / "chunk-000"
    parquet_files = sorted(data_path.glob("episode_*.parquet"))
    if not parquet_files:
        raise FileNotFoundError(f"No episode_*.parquet in {data_path}")

    all_actions = []
    all_states = []
    episode_lengths = {}
    rows_loaded = 0

    for fp in parquet_files:
        table = pq.read_table(fp, columns=["actions", "state", "episode_index", "frame_index"])
        n = table.num_rows

        actions_col = table.column("actions")
        state_col = table.column("state")
        ep_col = table.column("episode_index").to_pylist()

        for i in range(n):
            all_actions.append(actions_col[i].as_py())
            all_states.append(state_col[i].as_py())

        for ep_idx in set(ep_col):
            count = ep_col.count(ep_idx)
            episode_lengths[ep_idx] = episode_lengths.get(ep_idx, 0) + count

        rows_loaded += n
        if max_rows and rows_loaded >= max_rows:
            break

    return {
        "actions": np.array(all_actions, dtype=np.float32),
        "states": np.array(all_states, dtype=np.float32),
        "episode_lengths": episode_lengths,
        "num_rows": rows_loaded,
    }


def load_video_frames(data_dir: str, camera: str, num_samples: int = 500, seed: int = 42) -> np.ndarray:
    """Sample frames from video backup for visual statistics."""
    video_dir = Path(data_dir) / "videos_backup" / "chunk-000"
    if not video_dir.exists():
        video_dir = Path(data_dir) / "videos" / "chunk-000"
    if not video_dir.exists():
        return np.array([])

    video_files = sorted(video_dir.glob(f"*_{camera}.mp4"))
    if not video_files:
        return np.array([])

    try:
        import cv2
    except ImportError:
        print(f"  [WARN] OpenCV not available, skipping video analysis for {camera}")
        return np.array([])

    rng = np.random.default_rng(seed)
    selected_files = rng.choice(video_files, size=min(len(video_files), num_samples // 5), replace=False)

    frames = []
    for vf in selected_files:
        cap = cv2.VideoCapture(str(vf))
        if not cap.isOpened():
            continue
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if total <= 0:
            cap.release()
            continue
        indices = rng.choice(total, size=min(5, total), replace=False)
        for idx in sorted(indices):
            cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ok, frame = cap.read()
            if ok:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frames.append(frame_rgb.astype(np.float32) / 255.0)
                if len(frames) >= num_samples:
                    break
        cap.release()
        if len(frames) >= num_samples:
            break

    return np.array(frames) if frames else np.array([])


def compute_dim_stats(arr: np.ndarray) -> list[dict]:
    """Compute per-dimension statistics for [N, D] array."""
    stats = []
    for d in range(arr.shape[1]):
        col = arr[:, d]
        stats.append({
            "dim": d,
            "name": DIM_NAMES_ACTION[d] if d < len(DIM_NAMES_ACTION) else f"dim_{d}",
            "mean": float(np.mean(col)),
            "std": float(np.std(col)),
            "min": float(np.min(col)),
            "max": float(np.max(col)),
            "p5": float(np.percentile(col, 5)),
            "p50": float(np.percentile(col, 50)),
            "p95": float(np.percentile(col, 95)),
            "range": float(np.max(col) - np.min(col)),
            "unique_ratio": len(np.unique(np.round(col, 4))) / len(col),
        })
    return stats


def analyze_zscore_danger(stats: list[dict], field_name: str) -> list[dict]:
    """Assess z-score normalization risk per dimension."""
    results = []
    for s in stats:
        std = s["std"]
        sensitivity = 1.0 / (std + 1e-8)
        shift_001 = 0.01 * sensitivity

        if std < 1e-4:
            risk = "CRITICAL"
            reason = f"std={std:.2e} → constant dim, z-score divides by ~0"
        elif std < 0.01:
            risk = "HIGH"
            reason = f"std={std:.4f} → near-constant, 0.01 shift → {shift_001:.0f}σ in z-score"
        elif s["unique_ratio"] < 0.01:
            risk = "HIGH"
            reason = f"only {s['unique_ratio']*100:.1f}% unique values → discrete/bimodal, z-score inappropriate"
        elif std < 0.1:
            risk = "MEDIUM"
            reason = f"std={std:.4f}, moderate sensitivity"
        else:
            risk = "LOW"
            reason = f"std={std:.4f}, well-distributed"

        results.append({
            "dim": s["dim"],
            "name": s["name"],
            "std": std,
            "sensitivity": sensitivity,
            "shift_001_zscore": shift_001,
            "risk": risk,
            "reason": reason,
        })
    return results


def detect_episode_anomalies(episode_lengths: dict, actions: np.ndarray, states: np.ndarray) -> list[dict]:
    """Detect anomalous episodes."""
    anomalies = []
    lengths = list(episode_lengths.values())
    if not lengths:
        return anomalies

    median_len = float(np.median(lengths))
    std_len = float(np.std(lengths))

    for ep_idx, length in episode_lengths.items():
        issues = []
        if length < 50:
            issues.append(f"very short ({length} frames, median={median_len:.0f})")
        elif length < median_len - 3 * std_len:
            issues.append(f"abnormally short ({length} frames, median={median_len:.0f})")

        if issues:
            anomalies.append({
                "episode": ep_idx,
                "length": length,
                "issues": issues,
            })

    return anomalies


def compute_visual_stats(frames: np.ndarray) -> dict:
    """Compute per-channel pixel statistics from [N, H, W, 3] array."""
    if frames.size == 0:
        return {}
    means = frames.mean(axis=(0, 1, 2))
    stds = frames.std(axis=(0, 1, 2))
    brightness = frames.mean(axis=-1).flatten()
    hist, bin_edges = np.histogram(brightness, bins=50, range=(0, 1))
    return {
        "num_frames": len(frames),
        "resolution": f"{frames.shape[2]}x{frames.shape[1]}",
        "channel_means_rgb": [float(m) for m in means],
        "channel_stds_rgb": [float(s) for s in stds],
        "brightness_mean": float(brightness.mean()),
        "brightness_std": float(brightness.std()),
        "brightness_hist": hist.tolist(),
        "brightness_bins": bin_edges.tolist(),
    }


def generate_plots(
    action_stats: list[dict],
    state_stats: list[dict],
    zscore_action: list[dict],
    visual_stats: dict,
    output_dir: str,
    new_action_stats: list[dict] | None = None,
    new_state_stats: list[dict] | None = None,
) -> list[str]:
    """Generate diagnostic PNG plots."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    os.makedirs(output_dir, exist_ok=True)
    generated = []

    # --- 1. Action dimension stats ---
    fig, axes = plt.subplots(2, 1, figsize=(18, 10))

    dims = [s["dim"] for s in action_stats]
    means = [s["mean"] for s in action_stats]
    stds = [s["std"] for s in action_stats]
    names = [s["name"] for s in action_stats]

    ax = axes[0]
    ax.bar(dims, means, color="steelblue", alpha=0.7, label="Old robot mean")
    ax.errorbar(dims, means, yerr=stds, fmt="none", color="steelblue", alpha=0.5, capsize=2)
    if new_action_stats:
        new_means = [s["mean"] for s in new_action_stats]
        new_stds = [s["std"] for s in new_action_stats]
        ax.bar([d + 0.35 for d in dims], new_means, width=0.35, color="coral", alpha=0.7, label="New robot mean")
        ax.errorbar([d + 0.35 for d in dims], new_means, yerr=new_stds, fmt="none", color="coral", alpha=0.5, capsize=2)
    ax.set_xticks(dims)
    ax.set_xticklabels(names, rotation=45, ha="right", fontsize=7)
    ax.set_title("Action per-dimension: mean ± std")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    ax = axes[1]
    ranges = [s["range"] for s in action_stats]
    ax.bar(dims, ranges, color="steelblue", alpha=0.7)
    ax.set_xticks(dims)
    ax.set_xticklabels(names, rotation=45, ha="right", fontsize=7)
    ax.set_title("Action per-dimension: range (max - min)")
    ax.set_yscale("log")
    ax.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    path = os.path.join(output_dir, "action_dim_stats.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    generated.append(path)

    # --- 2. Z-score danger heatmap ---
    risk_colors = {"CRITICAL": 3, "HIGH": 2, "MEDIUM": 1, "LOW": 0}
    risk_vals = [risk_colors[z["risk"]] for z in zscore_action]

    fig, ax = plt.subplots(figsize=(18, 2.5))
    cmap = ListedColormap(["#2ecc71", "#f1c40f", "#e67e22", "#e74c3c"])
    data = np.array(risk_vals).reshape(1, -1)
    im = ax.imshow(data, cmap=cmap, aspect="auto", vmin=0, vmax=3)
    ax.set_xticks(range(len(zscore_action)))
    ax.set_xticklabels([z["name"] for z in zscore_action], rotation=45, ha="right", fontsize=7)
    ax.set_yticks([])
    ax.set_title("Z-score Normalization Risk (Action Dimensions)")
    for i, z in enumerate(zscore_action):
        ax.text(i, 0, z["risk"][0], ha="center", va="center", fontsize=7,
                color="white" if z["risk"] in ("CRITICAL", "HIGH") else "black", fontweight="bold")
    cbar = fig.colorbar(im, ax=ax, ticks=[0, 1, 2, 3], orientation="vertical", shrink=0.8)
    cbar.ax.set_yticklabels(["LOW", "MEDIUM", "HIGH", "CRITICAL"], fontsize=7)

    fig.tight_layout()
    path = os.path.join(output_dir, "zscore_danger.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    generated.append(path)

    # --- 3. Visual stats ---
    if visual_stats:
        cameras_with_data = [c for c in CAMERA_KEYS if c in visual_stats and visual_stats[c]]
        if cameras_with_data:
            fig, axes = plt.subplots(1, len(cameras_with_data), figsize=(5 * len(cameras_with_data), 4))
            if len(cameras_with_data) == 1:
                axes = [axes]
            for ax, cam in zip(axes, cameras_with_data):
                vs = visual_stats[cam]
                hist = vs["brightness_hist"]
                bins = vs["brightness_bins"]
                centers = [(bins[i] + bins[i + 1]) / 2 for i in range(len(bins) - 1)]
                ax.bar(centers, hist, width=(bins[1] - bins[0]) * 0.9, color="steelblue", alpha=0.7, label="Old")
                ax.set_title(f"{cam}\nmean_rgb={[f'{m:.3f}' for m in vs['channel_means_rgb']]}")
                ax.set_xlabel("Brightness")
                ax.set_ylabel("Pixel count")
                ax.legend(fontsize=7)
            fig.suptitle("Per-camera brightness distribution", fontsize=12)
            fig.tight_layout()
            path = os.path.join(output_dir, "visual_stats.png")
            fig.savefig(path, dpi=150, bbox_inches="tight")
            plt.close(fig)
            generated.append(path)

    return generated


def generate_report(
    action_stats: list[dict],
    state_stats: list[dict],
    zscore_action: list[dict],
    zscore_state: list[dict],
    visual_stats: dict,
    anomalies: list[dict],
    episode_lengths: dict,
    num_rows: int,
    output_path: str,
    new_action_stats: list[dict] | None = None,
) -> None:
    """Generate markdown diagnostic report."""
    lines = []
    lines.append("# R1 Pro Domain Gap Diagnostic Report\n")

    # Dataset overview
    lines.append("## 1. Dataset Overview\n")
    lines.append(f"| Item | Value |")
    lines.append(f"|------|-------|")
    lines.append(f"| Total frames | {num_rows:,} |")
    lines.append(f"| Episodes | {len(episode_lengths)} |")
    ep_lens = list(episode_lengths.values())
    lines.append(f"| Episode length (median) | {np.median(ep_lens):.0f} |")
    lines.append(f"| Episode length (min/max) | {min(ep_lens)} / {max(ep_lens)} |")
    lines.append(f"| Action dims | {len(action_stats)} |")
    lines.append(f"| State dims | {len(state_stats)} |")
    lines.append("")

    # Z-score danger summary
    lines.append("## 2. Z-score Normalization Risk Assessment\n")
    lines.append("**Risk levels**: CRITICAL = std≈0 (z-score explodes), HIGH = near-constant or bimodal, MEDIUM = low variance, LOW = safe\n")

    critical = [z for z in zscore_action if z["risk"] == "CRITICAL"]
    high = [z for z in zscore_action if z["risk"] == "HIGH"]

    if critical:
        lines.append(f"### CRITICAL ({len(critical)} dims) — 这些维度的 z-score 归一化会爆炸\n")
        lines.append("| Dim | Name | std | sensitivity (1/std) | Reason |")
        lines.append("|-----|------|-----|---------------------|--------|")
        for z in critical:
            lines.append(f"| {z['dim']} | {z['name']} | {z['std']:.2e} | {z['sensitivity']:.2e} | {z['reason']} |")
        lines.append("")

    if high:
        lines.append(f"### HIGH ({len(high)} dims) — 高风险\n")
        lines.append("| Dim | Name | std | shift=0.01 → Δz-score | Reason |")
        lines.append("|-----|------|-----|----------------------|--------|")
        for z in high:
            lines.append(f"| {z['dim']} | {z['name']} | {z['std']:.4f} | {z['shift_001_zscore']:.1f} | {z['reason']} |")
        lines.append("")

    # Full action stats table
    lines.append("### Action dimension statistics (all 23 dims)\n")
    lines.append("| Dim | Name | mean | std | min | max | p5 | p50 | p95 | Risk |")
    lines.append("|-----|------|------|-----|-----|-----|----|----|-----|------|")
    for s, z in zip(action_stats, zscore_action):
        risk_emoji = {"CRITICAL": "🔴", "HIGH": "🟠", "MEDIUM": "🟡", "LOW": "🟢"}[z["risk"]]
        lines.append(
            f"| {s['dim']} | {s['name']} | {s['mean']:.4f} | {s['std']:.4f} | "
            f"{s['min']:.4f} | {s['max']:.4f} | {s['p5']:.4f} | {s['p50']:.4f} | "
            f"{s['p95']:.4f} | {risk_emoji} {z['risk']} |"
        )
    lines.append("")

    # State stats
    lines.append("### State dimension statistics\n")
    lines.append("| Dim | Name | mean | std | min | max | Risk |")
    lines.append("|-----|------|------|-----|-----|-----|------|")
    for s, z in zip(state_stats, zscore_state):
        risk_emoji = {"CRITICAL": "🔴", "HIGH": "🟠", "MEDIUM": "🟡", "LOW": "🟢"}[z["risk"]]
        lines.append(
            f"| {s['dim']} | {s['name']} | {s['mean']:.4f} | {s['std']:.4f} | "
            f"{s['min']:.4f} | {s['max']:.4f} | {risk_emoji} {z['risk']} |"
        )
    lines.append("")

    # Visual stats
    lines.append("## 3. Visual Statistics\n")
    for cam in CAMERA_KEYS:
        if cam in visual_stats and visual_stats[cam]:
            vs = visual_stats[cam]
            lines.append(f"### {cam}\n")
            lines.append(f"- Frames sampled: {vs['num_frames']}")
            lines.append(f"- Resolution: {vs['resolution']}")
            lines.append(f"- Channel means (RGB): {[f'{m:.4f}' for m in vs['channel_means_rgb']]}")
            lines.append(f"- Channel stds (RGB): {[f'{s:.4f}' for s in vs['channel_stds_rgb']]}")
            lines.append(f"- Brightness: mean={vs['brightness_mean']:.4f}, std={vs['brightness_std']:.4f}")
            lines.append("")
        else:
            lines.append(f"### {cam}\n- (no video data available)\n")

    # Episode anomalies
    lines.append("## 4. Episode Anomalies\n")
    if anomalies:
        lines.append("| Episode | Length | Issues |")
        lines.append("|---------|--------|--------|")
        for a in anomalies:
            lines.append(f"| {a['episode']} | {a['length']} | {'; '.join(a['issues'])} |")
        lines.append("")
    else:
        lines.append("No episode anomalies detected.\n")

    # Recommendations
    lines.append("## 5. Recommendations\n")
    lines.append("### 紧急（影响新机器人推理正确性）\n")

    if critical:
        dim_list = ", ".join(f"dim {z['dim']} ({z['name']})" for z in critical)
        lines.append(f"1. **修复常数维度归一化**：{dim_list} 的 std≈0，z-score 归一化在这些维度上无效。")
        lines.append(f"   - 方案 A：在 `norm_exception_mode` 中对这些维度使用 `min/max` 或 `const` 模式")
        lines.append(f"   - 方案 B：在推理时对这些维度直接输出固定常数，跳过 denormalize")
        lines.append("")

    if high:
        dim_list = ", ".join(f"dim {z['dim']} ({z['name']})" for z in high)
        lines.append(f"2. **处理离散/低方差维度**：{dim_list}")
        lines.append(f"   - Gripper (dim 14, 15)：二值 {{0, 90}}，建议用 `min/max` 归一化或离散化处理")
        lines.append("")

    lines.append("### 短期（改善泛化）\n")
    lines.append("3. **在新机器人上采集少量标定数据**：即使 5-10 个 episode，也能重新计算 norm_stats，消除归一化偏移")
    lines.append("4. **加强视觉增强**：当前 ColorJitter 范围可能不足以覆盖新环境的光照差异")
    lines.append("5. **验证关节零位**：对比新旧机器人 home position 的 state 值是否一致")
    lines.append("")
    lines.append("### 中长期\n")
    lines.append("6. **多机器人数据混合训练**：从根本上解决个体差异")
    lines.append("7. **Domain randomization**：在训练时对 proprio/action 加噪声")
    lines.append("")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w") as f:
        f.write("\n".join(lines))


def main() -> None:
    parser = argparse.ArgumentParser(description="Diagnose domain gap for R1 Pro robot data")
    parser.add_argument("--old_data", type=str, required=True, help="Path to old robot LeRobot dataset")
    parser.add_argument("--new_data", type=str, default=None, help="Path to new robot data (optional)")
    parser.add_argument("--output_dir", type=str, default="b/test/dtaaug/domain_gap_report")
    parser.add_argument("--max_rows", type=int, default=None, help="Max rows to load (for speed)")
    parser.add_argument("--visual_samples", type=int, default=500, help="Number of video frames to sample")
    parser.add_argument("--skip_visual", action="store_true", help="Skip video frame analysis")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # --- Load old data ---
    print(f"[1/5] Loading old robot data from {args.old_data} ...")
    old = load_parquet_data(args.old_data, max_rows=args.max_rows)
    print(f"  Loaded {old['num_rows']:,} rows, {len(old['episode_lengths'])} episodes")

    # --- Action/State stats ---
    print("[2/5] Computing per-dimension statistics ...")
    action_stats = compute_dim_stats(old["actions"])
    state_stats = compute_dim_stats(old["states"])

    # --- Z-score danger ---
    print("[3/5] Assessing z-score normalization risk ...")
    zscore_action = analyze_zscore_danger(action_stats, "action")
    zscore_state = analyze_zscore_danger(state_stats, "state")

    critical_count = sum(1 for z in zscore_action if z["risk"] == "CRITICAL")
    high_count = sum(1 for z in zscore_action if z["risk"] == "HIGH")
    print(f"  Action: {critical_count} CRITICAL, {high_count} HIGH risk dimensions")

    # --- Visual stats ---
    visual_stats_dict: dict = {}
    if not args.skip_visual:
        print(f"[4/5] Sampling video frames (n={args.visual_samples}) ...")
        for cam in CAMERA_KEYS:
            print(f"  Loading {cam} ...")
            frames = load_video_frames(args.old_data, cam, num_samples=args.visual_samples)
            if frames.size > 0:
                visual_stats_dict[cam] = compute_visual_stats(frames)
                print(f"    {visual_stats_dict[cam]['num_frames']} frames, "
                      f"RGB mean={[f'{m:.3f}' for m in visual_stats_dict[cam]['channel_means_rgb']]}")
            else:
                visual_stats_dict[cam] = {}
                print(f"    (no frames loaded)")
    else:
        print("[4/5] Skipping visual analysis")

    # --- Episode anomalies ---
    print("[5/5] Detecting episode anomalies ...")
    anomalies = detect_episode_anomalies(old["episode_lengths"], old["actions"], old["states"])
    if anomalies:
        for a in anomalies:
            print(f"  Episode {a['episode']}: {'; '.join(a['issues'])}")
    else:
        print("  No anomalies detected")

    # --- Load new data (if provided) ---
    new_action_stats = None
    if args.new_data:
        print(f"\n[+] Loading new robot data from {args.new_data} ...")
        new = load_parquet_data(args.new_data, max_rows=args.max_rows)
        print(f"  Loaded {new['num_rows']:,} rows")
        new_action_stats = compute_dim_stats(new["actions"])

    # --- Generate report ---
    report_path = os.path.join(args.output_dir, "domain_gap_report.md")
    generate_report(
        action_stats, state_stats,
        zscore_action, zscore_state,
        visual_stats_dict, anomalies,
        old["episode_lengths"], old["num_rows"],
        report_path,
        new_action_stats=new_action_stats,
    )
    print(f"\nReport: {report_path}")

    # --- Generate plots ---
    try:
        plots = generate_plots(
            action_stats, state_stats,
            zscore_action, visual_stats_dict,
            args.output_dir,
            new_action_stats=new_action_stats,
        )
        for p in plots:
            print(f"Plot: {p}")
    except Exception as e:
        print(f"[WARN] Plot generation failed: {e}")

    # --- Summary ---
    print("\n" + "=" * 60)
    print("DIAGNOSTIC SUMMARY")
    print("=" * 60)
    for z in zscore_action:
        if z["risk"] in ("CRITICAL", "HIGH"):
            marker = "🔴" if z["risk"] == "CRITICAL" else "🟠"
            print(f"  {marker} dim {z['dim']:2d} ({z['name']:20s}): {z['risk']:8s} — {z['reason']}")
    print("=" * 60)


if __name__ == "__main__":
    main()
