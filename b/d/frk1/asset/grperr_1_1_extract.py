#!/usr/bin/env python3
"""Extract numbers for grperr_1.1.md (gripper still never closes after A-F).

Host has pandas/pyarrow; matplotlib lives in the GPU container venv.
This script only writes npz + json. Plot stage is grperr_1_1_figs.py.
"""
from __future__ import annotations

import ast
import json
import re
from pathlib import Path

import numpy as np
import pandas as pd

ASSET = Path(__file__).resolve().parent
DEMO_PARQUET = "/home/nvidia/bt/dt/plug_into_socket_lrb_4D_8sml/data/chunk-000/file-000.parquet"
LOG_DIR = ASSET.parents[2] / "x/4dwvla_ext/logs"

L2A = LOG_DIR / "client_20260918_035441_1926.log"  # 300 steps, 10 Hz, first
L2B = LOG_DIR / "client_20260918_040708_3189.log"  # 300 steps, 10 Hz, repeat
L2_120 = LOG_DIR / "client_20260918_035000_1185.log"  # 120 steps, 5 Hz
DRY = LOG_DIR / "client_20260918_034208_376.log"  # 250-step dry-run
DRY25 = LOG_DIR / "client_20260918_034123_177.log"
PRE = LOG_DIR / "client_20260918_021841_2745.log"  # pre-fix A, 700 steps

CLOSE_THR = 0.5
WMAX = 0.08
CHUNK = 50
N_EXEC = 10
HMAX = 200
Q7_TRAIN_MIN = 0.484343558549881
Q7_CLIP = 0.4343  # ACTION_LIMIT_LOWER[6] after fix C
HOME_Q = np.array(
    [-0.2575865685939789, -0.020942620933055878, 0.16744767129421234,
     -1.884966492652893, -0.05506148934364319, 1.9063893556594849,
     0.6435725092887878]
)
GRIP_MEAN = 0.033716946840286255
GRIP_STD = 0.032375022768974304
ACTION_G_MEAN = 0.5785381197929382
ACTION_G_STD = 0.4046877324581146
ARM_MEAN = np.array(
    [-0.24057792127132416, 0.14573757350444794, 0.1872396618127823,
     -0.0, 0.0, 0.0, 0.6998188495635986]  # filled below from stats if needed
)
STATE_ARM_MEAN = np.array(
    [-0.24057792127132416, 0.14573757350444794, 0.1872396618127823,
     -2.059950351715088, -0.05527758225798607, 2.201139450073242,
     0.6998188495635986]
)
STATE_ARM_STD = np.array(
    [0.1206311509013176, 0.08049428462982178, 0.1463828980922699,
     0.08539758622646332, 0.0428556464612484, 0.12847457826137543,
     0.09684077650308609]
)


def parse_array(s: str) -> np.ndarray:
    s = s.replace("\n", " ")
    return np.asarray(ast.literal_eval(s), dtype=float)


def parse_client(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    steps = {}
    inf = {}
    for m in re.finditer(
        r"\[step (\d+)\] execute: action=(\[[^\]]*\]) state_before=(\[[^\]]*\])",
        text,
    ):
        i = int(m.group(1))
        steps[i] = {
            "action": parse_array(m.group(2)),
            "state_before": parse_array(m.group(3)),
        }
    for m in re.finditer(
        r"\[step (\d+)\] result: state_after=(\[[^\]]*\])",
        text,
    ):
        i = int(m.group(1))
        steps.setdefault(i, {})["state_after"] = parse_array(m.group(2))
    for m in re.finditer(
        r"\[inference (\d+)\] request: state=(\[[^\]]*\]) state_history_len=(\d+) "
        r"image_meta=(\{.*?\}) task=",
        text,
    ):
        i = int(m.group(1))
        inf[i] = {
            "state": parse_array(m.group(2)),
            "hist_len": int(m.group(3)),
            "image_meta": ast.literal_eval(m.group(4)),
        }
    for m in re.finditer(
        r"\[inference (\d+)\] response: shape=\[(\d+), (\d+)\] full_actions=(\[\[.*?\]\]) "
        r"delta_from_previous=",
        text,
        flags=re.S,
    ):
        i = int(m.group(1))
        arr = parse_array(m.group(4))
        inf.setdefault(i, {})["chunk"] = arr

    n = max(steps) + 1 if steps else 0
    action = np.full((n, 8), np.nan)
    state_b = np.full((n, 8), np.nan)
    state_a = np.full((n, 8), np.nan)
    for i, d in steps.items():
        if "action" in d:
            action[i] = d["action"]
        if "state_before" in d:
            state_b[i] = d["state_before"]
        if "state_after" in d:
            state_a[i] = d["state_after"]

    n_inf = max(inf) + 1 if inf else 0
    chunks = np.full((n_inf, 10, 8), np.nan)
    img_mean = {"global": [], "wrist": []}
    img_max = {"global": [], "wrist": []}
    for i, d in inf.items():
        if "chunk" in d and d["chunk"].ndim == 2:
            c = d["chunk"]
            chunks[i, : c.shape[0], : c.shape[1]] = c[:10]
        meta = d.get("image_meta") or {}
        for cam in ("global", "wrist"):
            if cam in meta:
                img_mean[cam].append(float(meta[cam]["mean"]))
                img_max[cam].append(float(meta[cam]["max"]))

    hz = None
    m = re.search(r"achieved_control_hz=([0-9.]+)", text)
    if m:
        hz = float(m.group(1))
    width0 = None
    m = re.search(r"holding=\w+ width=([0-9.]+)m", text)
    if m:
        width0 = float(m.group(1))
    maxw = None
    m = re.search(r"max_width=([0-9.]+)m", text)
    if m:
        maxw = float(m.group(1))

    grip = action[:, 7]
    return {
        "n": n,
        "action": action,
        "state_before": state_b,
        "state_after": state_a,
        "grip": grip,
        "chunks": chunks,
        "img_mean_g": np.array(img_mean["global"], dtype=float),
        "img_mean_w": np.array(img_mean["wrist"], dtype=float),
        "img_max_g": np.array(img_max["global"], dtype=float),
        "img_max_w": np.array(img_max["wrist"], dtype=float),
        "hz": hz,
        "width0": width0,
        "max_width": maxw,
        "n_close": int(np.nansum(grip >= CLOSE_THR)),
        "grip_min": float(np.nanmin(grip)) if n else None,
        "grip_max": float(np.nanmax(grip)) if n else None,
        "grip_mean": float(np.nanmean(grip)) if n else None,
        "peak_step": int(np.nanargmax(grip)) if n else None,
    }


def digitize_state(x: np.ndarray) -> np.ndarray:
    """Match InternVLAA15ChatProcessorTransformFn._encode_state after mean_std.

    Training pipeline: NormalizeTransformFn then /3 then 256-bin digitize on [-1,1].
    """
    bins = np.linspace(-1, 1, 257)[:-1]
    return np.digitize(x / 3.0, bins=bins) - 1


def main() -> None:
    df = pd.read_parquet(DEMO_PARQUET)
    ag = np.stack(df["action.gripper"].to_numpy()).reshape(-1)
    wg = np.stack(df["observation.state.gripper"].to_numpy()).reshape(-1)
    arm = np.stack(df["observation.state.arm"].to_numpy())
    ee = np.stack(df["observation.state.ee_pos"].to_numpy())
    fi = df["frame_index"].to_numpy().astype(int)
    ei = df["episode_index"].to_numpy().astype(int)
    his = np.minimum(fi, HMAX)

    # Per-episode first close (action.gripper >= 0.5)
    episodes = []
    close_trans = []  # frames from last <0.2 to first >=0.5
    n_exec_miss = 0
    n_exec_hit = 0
    n_horizon_has_close = 0
    n_pre_close_frames = 0
    delay_in_chunk = []
    for e in sorted(set(ei.tolist())):
        m = ei == e
        a_e = ag[m]
        w_e = wg[m]
        arm_e = arm[m]
        ee_e = ee[m]
        fi_e = fi[m]
        closed = a_e >= CLOSE_THR
        if not closed.any():
            episodes.append({"ep": int(e), "n": int(m.sum()), "first_close": None})
            continue
        k = int(np.argmax(closed))
        # transition length: last index before k with a<0.2
        pre = np.where(a_e[:k] < 0.2)[0]
        t0 = int(pre[-1]) if len(pre) else 0
        trans = k - t0
        close_trans.append(trans)
        # receding-horizon: for frames 0..k, where in next-50 does close appear?
        for t in range(max(0, k - 80), k + 1):
            fut = a_e[t : t + CHUNK]
            if fut.size == 0:
                continue
            n_pre_close_frames += 1
            hits = np.where(fut >= CLOSE_THR)[0]
            if hits.size:
                n_horizon_has_close += 1
                d = int(hits[0])
                delay_in_chunk.append(d)
                if d < N_EXEC:
                    n_exec_hit += 1
                else:
                    n_exec_miss += 1
        episodes.append({
            "ep": int(e),
            "n": int(m.sum()),
            "first_close_frame": int(fi_e[k]),
            "first_close_idx": k,
            "his_len_at_close": int(min(fi_e[k], HMAX)),
            "q_at_close": arm_e[k].tolist(),
            "ee_at_close": ee_e[k].tolist(),
            "w_at_close": float(w_e[k]),
            "a_at_close": float(a_e[k]),
            "q_home": arm_e[0].tolist(),
            "ee_home": ee_e[0].tolist(),
            "w_open_p90": float(np.quantile(w_e[a_e < 0.2], 0.9)) if (a_e < 0.2).any() else None,
            "trans_frames": trans,
            "q7_at_close": float(arm_e[k, 6]),
            "q7_min": float(arm_e[:, 6].min()),
            "q7_max": float(arm_e[:, 6].max()),
        })

    # Gripper action histogram (bimodal?)
    hist_bins = np.linspace(0, 1, 21)
    hist_counts, _ = np.histogram(ag, bins=hist_bins)

    # Mixture-mean interpretation: P(close)*1 + P(open)*0.02
    p_close = float((ag >= CLOSE_THR).mean())
    mix_mean = p_close * 1.0 + (1 - p_close) * float(np.median(ag[ag < 0.2])) if (ag < 0.2).any() else None

    l2a = parse_client(L2A)
    l2b = parse_client(L2B)
    l2120 = parse_client(L2_120)
    dry = parse_client(DRY)
    dry25 = parse_client(DRY25)
    pre = parse_client(PRE)

    def nn_demo(q7: np.ndarray) -> dict:
        """Nearest demo frame in joint space (L2 of 7 joints)."""
        valid = ~np.isnan(q7).any(axis=-1) if q7.ndim == 2 else True
        if q7.ndim == 1:
            d = np.linalg.norm(arm - q7[None], axis=1)
            j = int(d.argmin())
            return {
                "dist": float(d[j]),
                "ep": int(ei[j]),
                "frame": int(fi[j]),
                "ag": float(ag[j]),
                "w": float(wg[j]),
                "ee": ee[j].tolist(),
                "q": arm[j].tolist(),
                "his": int(his[j]),
            }
        out = []
        for row in q7:
            if np.isnan(row).any():
                out.append(None)
                continue
            d = np.linalg.norm(arm - row[None], axis=1)
            j = int(d.argmin())
            out.append({
                "dist": float(d[j]),
                "ep": int(ei[j]),
                "frame": int(fi[j]),
                "ag": float(ag[j]),
                "ee": ee[j].tolist(),
            })
        return out

    # Sample NN every 20 steps for L2A
    q_l2a = l2a["state_after"][:, :7]
    nn_series = []
    for i in range(0, l2a["n"], 20):
        row = q_l2a[i]
        if np.isnan(row).any():
            row = l2a["state_before"][i, :7]
        nn_series.append({"step": i, **nn_demo(row)})
    nn_end_a = nn_demo(q_l2a[-1] if not np.isnan(q_l2a[-1]).any() else l2a["state_before"][-1, :7])
    nn_end_b = nn_demo(l2b["state_after"][-1, :7])
    nn_peak_a = nn_demo(q_l2a[l2a["peak_step"]])

    # Within-chunk grip: first vs last of the 10, and max
    def chunk_grip_stats(chunks: np.ndarray) -> dict:
        g = chunks[:, :, 7]
        valid = ~np.isnan(g[:, 0])
        g = g[valid]
        if g.size == 0:
            return {}
        return {
            "per_slot_mean": np.nanmean(g, axis=0).tolist(),
            "per_slot_max": np.nanmax(g, axis=0).tolist(),
            "first_lt_last_frac": float(np.mean(g[:, -1] > g[:, 0])),
            "any_ge_0_5": int(np.sum(np.nanmax(g, axis=1) >= 0.5)),
            "any_ge_0_3": int(np.sum(np.nanmax(g, axis=1) >= 0.3)),
            "any_ge_0_2": int(np.sum(np.nanmax(g, axis=1) >= 0.2)),
            "n": int(g.shape[0]),
        }

    # Threshold sweep on L2A executed grip
    sweep = {}
    g = l2a["grip"]
    for thr in [0.10, 0.12, 0.15, 0.18, 0.20, 0.22, 0.25, 0.30, 0.40, 0.50]:
        hits = np.where(g >= thr)[0]
        sweep[str(thr)] = {
            "n_steps": int(hits.size),
            "first_step": int(hits[0]) if hits.size else None,
            "last_step": int(hits[-1]) if hits.size else None,
            "n_crossings": int(np.sum(np.diff((g >= thr).astype(int)) == 1)) if hits.size else 0,
        }

    # Tokenize-state bins for gripper width
    def grip_token(w: float) -> dict:
        z = (w - GRIP_MEAN) / GRIP_STD
        tok = int(digitize_state(np.array([z]))[0])
        return {"w": w, "z": float(z), "token": tok}

    # q7 vs training
    q7_a = l2a["state_after"][:, 6]
    q7_cmd = l2a["action"][:, 6]

    # Image brightness
    def img_stats(run):
        return {
            "global_mean": float(np.mean(run["img_mean_g"])) if run["img_mean_g"].size else None,
            "wrist_mean": float(np.mean(run["img_mean_w"])) if run["img_mean_w"].size else None,
            "global_max": float(np.mean(run["img_max_g"])) if run["img_max_g"].size else None,
            "wrist_max": float(np.mean(run["img_max_w"])) if run["img_max_w"].size else None,
        }

    # Analog width implied by peak action
    peak_a = l2a["grip_max"]
    implied_w = WMAX * (1.0 - peak_a)

    summary = {
        "demo": {
            "n_frames": int(len(df)),
            "n_ep": int(ei.max() + 1),
            "fps": 30,
            "p_action_ge_0_5": float((ag >= 0.5).mean()),
            "p_action_ge_0_8": float((ag >= 0.8).mean()),
            "p_action_lt_0_1": float((ag < 0.1).mean()),
            "action_hist_20bins": hist_counts.tolist(),
            "hist_bins": hist_bins.tolist(),
            "median_open": float(np.median(ag[ag < 0.2])),
            "median_closed": float(np.median(ag[ag >= 0.5])),
            "mixture_mean_if_pclose": mix_mean,
            "w_open_max": float(wg.max()),
            "w_open_q90": float(np.quantile(wg[ag < 0.2], 0.9)),
            "w_closed_median": float(np.median(wg[ag >= 0.5])),
            "close_transition_frames": close_trans,
            "close_transition_median": float(np.median(close_trans)),
            "close_transition_mean": float(np.mean(close_trans)),
            "episodes": episodes,
            "receding_horizon": {
                "n_pre_close_frames_window": n_pre_close_frames,
                "n_horizon_has_close": n_horizon_has_close,
                "n_exec10_would_hit": n_exec_hit,
                "n_exec10_would_miss": n_exec_miss,
                "delay_in_chunk_median": float(np.median(delay_in_chunk)) if delay_in_chunk else None,
                "delay_in_chunk_p10": float(np.quantile(delay_in_chunk, 0.1)) if delay_in_chunk else None,
                "delay_in_chunk_p90": float(np.quantile(delay_in_chunk, 0.9)) if delay_in_chunk else None,
                "frac_delay_ge_10": float(np.mean(np.array(delay_in_chunk) >= 10)) if delay_in_chunk else None,
            },
            "q7_all_min": float(arm[:, 6].min()),
            "q7_all_max": float(arm[:, 6].max()),
            "q7_at_close_mean": float(np.mean([e["q7_at_close"] for e in episodes if "q7_at_close" in e])),
            "ee_z_at_close": [e["ee_at_close"][2] for e in episodes if e.get("ee_at_close")],
            "ee_y_at_close": [e["ee_at_close"][1] for e in episodes if e.get("ee_at_close")],
        },
        "l2a": {
            "n": l2a["n"], "hz": l2a["hz"], "width0": l2a["width0"],
            "max_width": l2a["max_width"],
            "grip_min": l2a["grip_min"], "grip_max": l2a["grip_max"],
            "grip_mean": l2a["grip_mean"], "peak_step": l2a["peak_step"],
            "n_close": l2a["n_close"],
            "q_end": l2a["state_after"][-1, :7].tolist(),
            "q_peak": q_l2a[l2a["peak_step"]].tolist(),
            "q7_cmd_min": float(np.nanmin(q7_cmd)),
            "q7_cmd_median": float(np.nanmedian(q7_cmd)),
            "q7_exec_min": float(np.nanmin(q7_a)),
            "frac_q7_clipped": float(np.mean(np.abs(q7_a - 0.4344) < 0.002)),
            "chunk_grip": chunk_grip_stats(l2a["chunks"]),
            "threshold_sweep": sweep,
            "nn_end": nn_end_a,
            "nn_peak": nn_peak_a,
            "nn_series": nn_series,
            "img": img_stats(l2a),
            "implied_width_at_peak_m": implied_w,
            "n_ge_0_15": int(np.sum(g >= 0.15)),
            "n_ge_0_20": int(np.sum(g >= 0.20)),
            "n_ge_0_22": int(np.sum(g >= 0.22)),
        },
        "l2b": {
            "n": l2b["n"], "hz": l2b["hz"],
            "grip_min": l2b["grip_min"], "grip_max": l2b["grip_max"],
            "peak_step": l2b["peak_step"], "n_close": l2b["n_close"],
            "q_end": l2b["state_after"][-1, :7].tolist(),
            "nn_end": nn_end_b,
            "chunk_grip": chunk_grip_stats(l2b["chunks"]),
            "img": img_stats(l2b),
            "n_ge_0_20": int(np.sum(l2b["grip"] >= 0.20)),
        },
        "l2_120": {
            "n": l2120["n"], "hz": l2120["hz"],
            "grip_min": l2120["grip_min"], "grip_max": l2120["grip_max"],
            "n_close": l2120["n_close"],
        },
        "dry250": {
            "n": dry["n"],
            "grip_min": dry["grip_min"], "grip_max": dry["grip_max"],
            "grip_mean": dry["grip_mean"],
            "n_close": dry["n_close"],
            "img": img_stats(dry),
            "state0_grip": float(dry["state_before"][0, 7]) if dry["n"] else None,
        },
        "dry25": {
            "grip_min": dry25["grip_min"], "grip_max": dry25["grip_max"],
            "n_close": dry25["n_close"],
            "img": img_stats(dry25),
        },
        "pre_fix": {
            "n": pre["n"],
            "grip_min": pre["grip_min"], "grip_max": pre["grip_max"],
            "n_close": pre["n_close"],
        },
        "tokens": {
            "demo_open_0p079": grip_token(0.07940476387739182),
            "eval_open_0p0664": grip_token(0.066406),
            "driver_open_0p06": grip_token(0.06),
            "dry_dummy_0p04": grip_token(0.04),
            "demo_closed_0p000": grip_token(0.0),
            "demo_mean": grip_token(GRIP_MEAN),
        },
        "z_action": {
            "peak_l2a": float((l2a["grip_max"] - ACTION_G_MEAN) / ACTION_G_STD),
            "need_for_0_5": float((0.5 - ACTION_G_MEAN) / ACTION_G_STD),
            "need_for_0_8": float((0.8 - ACTION_G_MEAN) / ACTION_G_STD),
            "dry_mean": float((dry["grip_mean"] - ACTION_G_MEAN) / ACTION_G_STD) if dry["grip_mean"] else None,
            "open_typical": float((0.02 - ACTION_G_MEAN) / ACTION_G_STD),
        },
        "home_q": HOME_Q.tolist(),
        "q7_train_min": Q7_TRAIN_MIN,
        "q7_clip": Q7_CLIP,
    }

    out_json = ASSET / "grperr_1_1_summary.json"
    out_json.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    np.savez(
        ASSET / "grperr_1_1_data.npz",
        demo_ag=ag, demo_w=wg, demo_arm=arm, demo_ee=ee, demo_fi=fi, demo_ei=ei,
        l2a_action=l2a["action"], l2a_state=l2a["state_after"], l2a_chunks=l2a["chunks"],
        l2b_action=l2b["action"], l2b_state=l2b["state_after"],
        dry_action=dry["action"],
        pre_action=pre["action"],
        hist_bins=hist_bins, hist_counts=hist_counts,
        delay_in_chunk=np.array(delay_in_chunk, dtype=float),
    )
    print("wrote", out_json)
    print("L2A grip", l2a["grip_min"], l2a["grip_max"], "peak_step", l2a["peak_step"], "n>=0.5", l2a["n_close"])
    print("DRY grip", dry["grip_min"], dry["grip_max"], "n>=0.5", dry["n_close"])
    print("close trans frames", close_trans)
    print("receding", summary["demo"]["receding_horizon"])
    print("nn_end", nn_end_a)
    print("nn_peak", nn_peak_a)
    print("tokens", summary["tokens"])
    print("sweep", sweep)
    print("chunk", summary["l2a"]["chunk_grip"])
    print("episodes first close", [(e["ep"], e.get("first_close_frame"), e.get("ee_at_close"), e.get("q7_at_close")) for e in episodes])


if __name__ == "__main__":
    main()
