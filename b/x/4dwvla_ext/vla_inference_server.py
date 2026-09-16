#!/usr/bin/env python3
"""VLA inference server — runs inside the GPU container.

Loads the 4DWVLA (InternVLA-A1.5) model, builds the transform pipeline
identical to evaluation/RoboTwin/inference.py, and serves inference
requests over multiprocessing.connection on TCP port 5555.

4D Keypoint support: when the checkpoint has ``enable_keypoint_predictor=True``
(the default for Franka plug), the server computes forward-kinematics from
joint angles at each step, maintains a sliding-window keypoint history, and
feeds ``observation.his_kpts`` / ``observation.his_len`` into the model —
matching the training-time 3-path MoT architecture.

Usage (inside GPU container):
    source /opt/venv/4dwvla/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/vla_inference_server.py \
        --ckpt-path /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420 \
        --schema-path /workspace/4WVLA/b/s/Frk/cfg/franka_plug.yaml \
        --kpt-meta-path /workspace/RLinf/b/d/frk1/plug/keypoints_meta.json \
        --urdf-path /workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf \
        --port 5555
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from multiprocessing.connection import Listener
from pathlib import Path

import numpy as np
import torch

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    force=True,
)
logger = logging.getLogger("vla-server")

# ── 4DWVLA imports (available after `pip install -e /workspace/4WVLA`) ───────

from lerobot.configs.policies import PreTrainedConfig
from lerobot.dataset_schemas import DatasetSchema, register_schema, load_schemas_from_path
from lerobot.dataset_schemas import get_schema
from lerobot.datasets.utils import load_json
from lerobot.policies.factory import get_policy_class
from lerobot.policies.internvla_a1_5.configuration_internvla_a1_5 import InternVLAA15Config
from lerobot.policies.internvla_a1_5.transform_internvla_a1_5 import (
    InternVLAA15ChatProcessorTransformFn,
)
from lerobot.transforms.core import (
    NormalizeTransformFn,
    PadStateAndActionTransformFn,
    RemapImageKeyTransformFn,
    ReorderStateActionTransform,
    ResizeImagesWithPadFn,
    UnNormalizeTransformFn,
    compose,
)
from lerobot.utils.constants import ACTION, OBS_IMAGES, OBS_STATE

# ── Constants ────────────────────────────────────────────────────────────────

STATS_KEY = "franka_plug"
RESIZE_SIZE = 224
DEFAULT_PORT = 5555
AUTHKEY = b"4dwvla-eval"
OBS_HIS_KPTS = "observation.his_kpts"
OBS_HIS_LEN = "observation.his_len"

# ── Stats loading ────────────────────────────────────────────────────────────

def load_stats(ckpt_path: Path, schema: DatasetSchema) -> tuple[dict, dict]:
    """Load state and action normalization stats from checkpoint.

    Training computes stats per sub-field (e.g. ``observation.state.arm``,
    ``observation.state.gripper``).  Eval normalizes the composed vector
    (``observation.state``).  Since mean/std normalization is element-wise,
    composing stats by concatenation is mathematically identical to
    normalizing per-field then concatenating.

    When the composed key exists in stats.json we use it directly;
    otherwise we compose from sub-field stats via the schema's
    ``feature_mapping``.
    """
    stats = load_json(ckpt_path / "stats.json")
    if STATS_KEY not in stats:
        raise KeyError(f"stats_key '{STATS_KEY}' not in {ckpt_path / 'stats.json'}")
    selected = stats[STATS_KEY]

    def pick(feature_key: str) -> dict:
        fs = selected[feature_key]
        picked = {}
        for k in ("mean", "std", "min", "max", "q01", "q99"):
            if k in fs:
                picked[k] = np.asarray(fs[k])
        if "mean" not in picked or "std" not in picked:
            raise KeyError(f"{feature_key} must have mean/std")
        if "min" not in picked and "q01" in picked:
            picked["min"] = picked["q01"]
        if "max" not in picked and "q99" in picked:
            picked["max"] = picked["q99"]
        return picked

    def compose_sub_field_stats(composed_key: str) -> dict:
        sub_keys = schema.feature_mapping.get(composed_key, [])
        if not sub_keys:
            raise KeyError(
                f"'{composed_key}' not in stats and schema has no "
                f"feature_mapping for it"
            )
        sub_stats = [pick(k) for k in sub_keys]
        composed: dict[str, np.ndarray] = {}
        for stat_name in ("mean", "std", "min", "max"):
            arrays = [s[stat_name] for s in sub_stats if stat_name in s]
            if len(arrays) == len(sub_stats):
                composed[stat_name] = np.concatenate(arrays)
        if "mean" not in composed or "std" not in composed:
            raise KeyError(
                f"Cannot compose '{composed_key}' from {sub_keys}: "
                f"sub-fields lack mean/std"
            )
        logger.info(
            "Composed %s stats from sub-fields %s (%dD)",
            composed_key, sub_keys, composed["mean"].shape[0],
        )
        return composed

    def pick_or_compose(composed_key: str) -> dict:
        if composed_key in selected:
            return pick(composed_key)
        return compose_sub_field_stats(composed_key)

    state_stat = {OBS_STATE: pick_or_compose(OBS_STATE)}
    action_stat = {ACTION: pick_or_compose(ACTION)}
    return state_stat, action_stat

# ── Schema registration ─────────────────────────────────────────────────────

def ensure_schema(schema_path: Path | None) -> DatasetSchema:
    """Register franka_plug schema if not already known."""
    try:
        return get_schema(STATS_KEY)
    except (KeyError, ValueError):
        pass

    if schema_path is not None and schema_path.exists():
        load_schemas_from_path(str(schema_path))
        return get_schema(STATS_KEY)

    schema = DatasetSchema(
        robot_type=STATS_KEY,
        action_mask_spec=[7, -1],
        feature_mapping={
            "observation.state": [
                "observation.state.arm",
                "observation.state.gripper",
            ],
            "action": ["action.arm", "action.gripper"],
        },
        image_mapping={
            "observation.images.global": "observation.images.image0",
            "observation.images.wrist": "observation.images.image1",
        },
    )
    register_schema(schema)
    return schema

# ── Transform pipeline ───────────────────────────────────────────────────────

def build_transforms(state_stat: dict, action_stat: dict,
                     schema: DatasetSchema, config: InternVLAA15Config):
    """Build the exact same transform pipeline as RoboTwin/R1Pro inference."""
    input_transforms = compose([
        ResizeImagesWithPadFn(
            height=RESIZE_SIZE, width=RESIZE_SIZE,
            mapping=schema.image_mapping,
        ),
        RemapImageKeyTransformFn(mapping=schema.image_mapping),
        NormalizeTransformFn(
            selected_keys=[OBS_STATE],
            norm_stats=state_stat,
        ),
        InternVLAA15ChatProcessorTransformFn(
            mode="eval",
            tokenize_state=getattr(config, "tokenize_state", True),
            max_state_dim=getattr(config, "max_state_dim", 32),
        ),
        PadStateAndActionTransformFn(
            max_state_dim=getattr(config, "max_state_dim", 32),
            max_action_dim=getattr(config, "max_action_dim", 32),
        ),
        ReorderStateActionTransform(
            state_reorder=schema.state_reorder,
            action_reorder=schema.action_reorder,
        ),
    ])

    unnormalize_fn = UnNormalizeTransformFn(
        selected_keys=[ACTION],
        mode="mean_std",
        norm_stats=action_stat,
    )
    return input_transforms, unnormalize_fn

# ── Model loading ────────────────────────────────────────────────────────────

def load_model(ckpt_path: Path, dtype: torch.dtype):
    """Load InternVLA-A1.5.

    Uses the *standard* backend when ``enable_keypoint_predictor=True``
    (the optimized backend has no keypoint path).  ``action_loss_only``
    is always True so that the WAN video branch is not loaded.
    """
    config = PreTrainedConfig.from_pretrained(ckpt_path)
    if not isinstance(config, InternVLAA15Config):
        raise ValueError(f"Expected internvla_a1_5 policy, got {config.type!r}")

    config.action_loss_only = True
    if getattr(config, "enable_keypoint_predictor", False):
        config.inference_backend = "standard"
    else:
        config.inference_backend = "optimized"
    config.device = "cuda" if torch.cuda.is_available() else "cpu"

    policy_cls = get_policy_class(config.type)
    policy = policy_cls.from_pretrained(ckpt_path, config=config)
    device = torch.device(config.device)
    policy.to(device=device, dtype=dtype)
    policy.eval()
    logger.info(
        "Model loaded: device=%s dtype=%s action_loss_only=%s backend=%s kpt=%s",
        device, dtype, config.action_loss_only, config.inference_backend,
        config.enable_keypoint_predictor,
    )
    return policy, device, config

# ── Sample building ──────────────────────────────────────────────────────────

def build_sample(
    images: dict,
    state: dict,
    task: str,
    dtype: torch.dtype,
    kpt_data: tuple[np.ndarray, int] | None = None,
) -> dict:
    """Build a sample dict from raw observations, matching training format.

    Args:
        images: {"global": np.ndarray (H,W,3) uint8, "wrist": np.ndarray}
        state: {"arm": list[7 floats], "gripper": list[1 float]}
        task: instruction string
        dtype: torch dtype for images
        kpt_data: (his_kpts [H, J, D], his_len) from FKKeypointComputer,
                  or None when keypoint predictor is disabled.
    """
    arm = np.asarray(state["arm"], dtype=np.float32)
    gripper = np.asarray(state["gripper"], dtype=np.float32)
    full_state = np.concatenate([arm, gripper])

    chunk_size = 50
    action_dim = 8

    sample = {
        OBS_STATE: torch.from_numpy(full_state).float(),
        ACTION: torch.zeros(chunk_size, action_dim, dtype=torch.float32),
        "task": task,
    }

    for cam_name, img_np in images.items():
        key = f"{OBS_IMAGES}.{cam_name}"
        img_t = torch.as_tensor(img_np).contiguous().to(dtype=dtype) / 255.0
        if img_t.ndim == 3 and img_t.shape[-1] == 3:
            img_t = img_t.permute(2, 0, 1)  # HWC → CHW
        sample[key] = img_t

    if kpt_data is not None:
        his_kpts, his_len = kpt_data
        sample[OBS_HIS_KPTS] = torch.from_numpy(his_kpts).float()
        sample[OBS_HIS_LEN] = torch.tensor(his_len, dtype=torch.long)

    return sample

# ── Batch creation ───────────────────────────────────────────────────────────

def to_batch(sample: dict, device: torch.device, dtype: torch.dtype) -> dict:
    """Add batch dimension and move to device."""
    batch = {}
    for key, value in sample.items():
        if isinstance(value, torch.Tensor):
            value = value.unsqueeze(0)
            if value.dtype.is_floating_point:
                value = value.to(device=device, dtype=dtype)
            else:
                value = value.to(device=device)
            batch[key] = value
        else:
            batch[key] = [value]
    return batch

# ── Server main loop ─────────────────────────────────────────────────────────

def serve(args: argparse.Namespace):
    dtype = torch.float32 if args.dtype == "float32" else torch.bfloat16

    # 1. Register schema
    schema = ensure_schema(
        Path(args.schema_path) if args.schema_path else None
    )
    logger.info("Schema registered: %s", schema.robot_type)

    # 2. Load stats (compose per-field stats if needed — D10 fix)
    ckpt = Path(args.ckpt_path)
    state_stat, action_stat = load_stats(ckpt, schema)
    actual_action_dim = action_stat[ACTION]["mean"].shape[0]
    logger.info(
        "Stats loaded from %s (state=%dD, action=%dD)",
        ckpt / "stats.json",
        state_stat[OBS_STATE]["mean"].shape[0],
        actual_action_dim,
    )

    # 3. Load model
    policy, device, config = load_model(ckpt, dtype)

    # 4. Build transforms
    input_transforms, unnormalize_fn = build_transforms(
        state_stat, action_stat, schema, config,
    )
    logger.info("Transform pipeline built (7 steps + unnormalize)")

    # 5. Set up FK keypoint computer (if enabled)
    fk_computer = None
    if getattr(config, "enable_keypoint_predictor", False):
        if not args.kpt_meta_path or not args.urdf_path:
            raise ValueError(
                "Checkpoint has enable_keypoint_predictor=True. "
                "Provide --kpt-meta-path and --urdf-path."
            )
        from fk_keypoints import FKKeypointComputer

        fk_computer = FKKeypointComputer(
            urdf_path=args.urdf_path,
            kpt_meta_path=args.kpt_meta_path,
            history_max_len=getattr(config, "keypoint_history_max_len", 200),
        )
        logger.info(
            "FK keypoint computer ready: %d joints, dim=%d, history=%d",
            fk_computer.num_joints,
            fk_computer.kpt_dim,
            fk_computer.history_max_len,
        )

    # 6. Start listening
    n_exec = args.n_exec
    address = ("0.0.0.0", args.port)
    listener = Listener(address, authkey=AUTHKEY)
    logger.info("Inference server listening on port %d (n_exec=%d)", args.port, n_exec)

    while True:
        logger.info("Waiting for client connection...")
        conn = listener.accept()
        logger.info("Client connected from %s", listener.last_accepted)
        policy.reset()
        if fk_computer is not None:
            fk_computer.reset()

        try:
            while True:
                msg = conn.recv()
                if msg is None or msg.get("command") == "shutdown":
                    logger.info("Client requested shutdown")
                    break
                if msg.get("command") == "reset":
                    policy.reset()
                    if fk_computer is not None:
                        fk_computer.reset()
                    conn.send({"status": "ok", "actions": []})
                    continue

                t0 = time.perf_counter()

                arm_q = np.asarray(msg["state"]["arm"], dtype=np.float32)
                kpt_data = None
                if fk_computer is not None:
                    kpt_data = fk_computer.step(arm_q)

                sample = build_sample(
                    images={
                        "global": np.asarray(msg["images"]["global"]),
                        "wrist": np.asarray(msg["images"]["wrist"]),
                    },
                    state=msg["state"],
                    task=msg["task"],
                    dtype=dtype,
                    kpt_data=kpt_data,
                )

                sample = input_transforms(sample)
                batch = to_batch(sample, device, dtype)

                with torch.no_grad():
                    action_pred = policy.predict_action_chunk(batch)

                if action_pred.ndim == 3:
                    action_pred = action_pred[0]

                action_pred = action_pred[:n_exec, :actual_action_dim]
                action_pred = unnormalize_fn({ACTION: action_pred})[ACTION]
                actions = action_pred.detach().float().cpu().numpy().tolist()

                t_ms = (time.perf_counter() - t0) * 1000
                logger.info(
                    "Inference: %.1fms, %d actions, "
                    "q1_range=[%.3f,%.3f]",
                    t_ms, len(actions),
                    min(a[0] for a in actions),
                    max(a[0] for a in actions),
                )

                conn.send({"status": "ok", "actions": actions})

        except EOFError:
            logger.info("Client disconnected")
        except Exception as exc:
            logger.error("Error in server loop: %s: %s", type(exc).__name__, exc)
            try:
                conn.send({"status": f"error: {exc}", "actions": []})
            except Exception:
                pass
        finally:
            conn.close()

def main():
    parser = argparse.ArgumentParser(description="4DWVLA Inference Server")
    parser.add_argument("--ckpt-path", type=str, required=True,
                        help="Path to 4DWVLA checkpoint directory")
    parser.add_argument("--schema-path", type=str, default=None,
                        help="Path to franka_plug.yaml schema")
    parser.add_argument("--kpt-meta-path", type=str, default=None,
                        help="Path to keypoints_meta.json (required when checkpoint has enable_keypoint_predictor)")
    parser.add_argument("--urdf-path", type=str, default=None,
                        help="Path to Franka URDF for FK keypoint computation")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--n-exec", type=int, default=10,
                        help="Number of actions per inference (from chunk of 50)")
    parser.add_argument("--dtype", choices=("float32", "bfloat16"),
                        default="bfloat16")
    args = parser.parse_args()
    serve(args)

if __name__ == "__main__":
    main()
