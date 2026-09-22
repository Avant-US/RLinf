#!/usr/bin/env python3
"""Figures for b/d/frk1/grperr_1.md -- why the gripper never closes.

Stage 1 (extract) needs pandas/pyarrow and reads the demo parquet plus the
client log. Stage 2 (plot) needs matplotlib. On this host the two live in
different interpreters, so the stages are split and joined by an npz cache:

    # host (pandas, no usable matplotlib)
    python3 grperr_1_figs.py --stage extract

    # GPU container venv (matplotlib 3.11 + numpy 1.26)
    docker exec rlinf-4dwvla-gpu bash -lc \
      'source /opt/venv/4dwvla/bin/activate && \
       python /workspace/RLinf/b/d/frk1/asset/grperr_1_figs.py --stage plot'
"""
from __future__ import annotations

import argparse
import ast
import re
from pathlib import Path

import numpy as np

ASSET_DIR = Path(__file__).resolve().parent
CACHE = ASSET_DIR / "grperr_1_data.npz"

DEMO_PARQUET = "/home/nvidia/bt/dt/plug_into_socket_lrb_4D_8sml/data/chunk-000/file-000.parquet"
CLIENT_LOG = ASSET_DIR.parents[2] / "x/4dwvla_ext/logs/client_20260918_021841_2745.log"

HISTORY_MAX_LEN = 200  # InternVLAA15Config.keypoint_history_max_len for this checkpoint
GRIPPER_CLOSE_THRESHOLD = 0.5
MAX_GRIPPER_WIDTH_M = 0.08


def extract() -> None:
    import pandas as pd

    df = pd.read_parquet(DEMO_PARQUET)
    demo = {
        "demo_grip_action": df["action.gripper"].to_numpy(dtype=float),
        "demo_grip_width": df["observation.state.gripper"].to_numpy(dtype=float),
        "demo_frame_index": df["frame_index"].to_numpy(dtype=int),
        "demo_episode_index": df["episode_index"].to_numpy(dtype=int),
        "demo_arm": np.stack(df["observation.state.arm"].to_numpy()),
    }

    text = Path(CLIENT_LOG).read_text(encoding="utf-8", errors="replace")
    pattern = re.compile(
        r"\[step (\d+)\] execute: action=(\[[^\]]*\]) state_before=(\[[^\]]*\])"
    )
    steps, actions, states = [], [], []
    for match in pattern.finditer(text):
        steps.append(int(match.group(1)))
        actions.append(ast.literal_eval(match.group(2).replace(" ", "")))
        states.append(ast.literal_eval(match.group(3).replace(" ", "")))

    his_len = [
        int(v) for v in re.findall(r"keypoints: history_shape=\[[^\]]*\] history_len=(\d+)", text)
    ]

    np.savez_compressed(
        CACHE,
        eval_step=np.asarray(steps, dtype=int),
        eval_action=np.asarray(actions, dtype=float),
        eval_state=np.asarray(states, dtype=float),
        eval_his_len=np.asarray(his_len, dtype=int),
        **demo,
    )
    print(f"wrote {CACHE} ({CACHE.stat().st_size / 1024:.0f} KiB)")


def plot() -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = np.load(CACHE)
    demo_act = d["demo_grip_action"]
    demo_frame = d["demo_frame_index"]
    demo_arm = d["demo_arm"]
    eval_act = d["eval_action"]
    eval_state = d["eval_state"]

    demo_his_len = np.minimum(demo_frame, HISTORY_MAX_LEN)
    eval_his_len_max = int(np.minimum(len(eval_act) // 10, HISTORY_MAX_LEN))
    closed = demo_act >= GRIPPER_CLOSE_THRESHOLD

    # ── Figure 1: his_len gates the close command ───────────────────────────
    fig, ax = plt.subplots(figsize=(9, 4.2))
    ax.scatter(demo_his_len[~closed], demo_act[~closed], s=4, alpha=0.25,
               color="#1f77b4", label="demo frame, gripper OPEN")
    ax.scatter(demo_his_len[closed], demo_act[closed], s=4, alpha=0.25,
               color="#d62728", label="demo frame, gripper CLOSED")
    ax.axhline(GRIPPER_CLOSE_THRESHOLD, color="k", ls="--", lw=1,
               label="close threshold 0.5")
    ax.axvspan(0, eval_his_len_max, color="#ffb347", alpha=0.35,
               label=f"his_len reached on-robot (max {eval_his_len_max})")
    ax.axvline(int(demo_his_len[closed].min()), color="#2ca02c", lw=2,
               label=f"earliest close in demos (his_len={int(demo_his_len[closed].min())})")
    ax.set_xlabel("observation.his_len  =  min(frame_index, 200)   [keypoint-history clock]")
    ax.set_ylabel("action.gripper   (1.0 = closed)")
    ax.set_title("The keypoint-history clock never enters the range where demos close the gripper")
    ax.set_ylim(-0.05, 1.08)
    ax.legend(loc="center left", fontsize=8, framealpha=0.95)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(ASSET_DIR / "fig1_hislen_gate.png", dpi=150)
    plt.close(fig)

    # ── Figure 2: commanded gripper, demos vs on-robot run ──────────────────
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for episode in np.unique(d["demo_episode_index"]):
        mask = d["demo_episode_index"] == episode
        axes[0].plot(demo_frame[mask] / 30.0, demo_act[mask], lw=1, alpha=0.8)
    axes[0].axhline(GRIPPER_CLOSE_THRESHOLD, color="k", ls="--", lw=1)
    axes[0].set_title("Demonstrations (8 episodes, 30 Hz)")
    axes[0].set_xlabel("episode time [s]")
    axes[0].set_ylabel("action.gripper   (1.0 = closed)")
    axes[0].grid(alpha=0.25)

    axes[1].plot(np.arange(len(eval_act)), eval_act[:, 7], lw=1.2, color="#d62728")
    axes[1].axhline(GRIPPER_CLOSE_THRESHOLD, color="k", ls="--", lw=1,
                    label="close threshold 0.5")
    axes[1].set_title("On-robot run 2026-09-18 02:19 (700 steps, ~2.5 Hz)")
    axes[1].set_xlabel("control step")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.25)
    axes[1].set_ylim(-0.05, 1.08)
    fig.suptitle("Policy output stays pinned at 'open' for the entire on-robot episode")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(ASSET_DIR / "fig2_grip_cmd.png", dpi=150)
    plt.close(fig)

    # ── Figure 3: q7 leaves the demonstrated joint range ────────────────────
    fig, ax = plt.subplots(figsize=(9, 4))
    q7_lo, q7_hi = demo_arm[:, 6].min(), demo_arm[:, 6].max()
    ax.axhspan(q7_lo, q7_hi, color="#2ca02c", alpha=0.18,
               label=f"q7 range seen in demos [{q7_lo:.3f}, {q7_hi:.3f}]")
    ax.plot(eval_state[:, 6], lw=1.4, color="#d62728", label="q7 measured on-robot")
    ax.axhline(q7_lo, color="#2ca02c", lw=1.2, ls="--")
    ax.set_xlabel("control step")
    ax.set_ylabel("q7 (wrist roll) [rad]")
    below = int((eval_state[:, 6] < q7_lo).sum())
    ax.set_title(
        "Wrist roll leaves the demonstrated range almost immediately\n"
        f"({below}/{len(eval_state)} steps below the demo minimum)"
    )
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(ASSET_DIR / "fig3_q7_ood.png", dpi=150)
    plt.close(fig)

    print("wrote fig1_hislen_gate.png, fig2_grip_cmd.png, fig3_q7_ood.png")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", choices=("extract", "plot", "all"), default="all")
    args = parser.parse_args()
    if args.stage in ("extract", "all"):
        extract()
    if args.stage in ("plot", "all"):
        plot()


if __name__ == "__main__":
    main()
