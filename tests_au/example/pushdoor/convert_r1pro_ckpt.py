"""Convert the GCS ``pi05_r1pro_chassis_alig_newnorm`` checkpoint (a LeRobot
``PI05Policy`` bundle) into the minimal RLinf/openpi-native layout that
``rlinf.models.embodiment.openpi_au.get_model()`` expects.

Why this is needed (see the "pushdoor" section of
``b/gcp/demo3/gcp_rlinf_pi05_au.md`` for the full investigation): the checkpoint at

    gs://physical-ai-data-eu/CKPT/VLA/PI/pi05_r1pro_chassis_alig_newnorm/R1/checkpoints/001000/pretrained_model/

was produced by LeRobot's own training script, not openpi's. Its
``model.safetensors`` stores every tensor under a ``model.`` prefix (e.g.
``model.action_in_proj.bias``, 813 tensors, with the gemma_expert half in bf16),
and ships a LeRobot-native ``config.json``/``policy_preprocessor.json`` describing
a ``PI05Config`` + a normalizer processor -- neither of which
``rlinf/models/embodiment/openpi_au/__init__.py::get_model()`` reads. That
function only does ``safetensors.torch.load_file(...)`` followed by
``model.load_state_dict(state_dict, strict=False)`` against an RLinf-native
``OpenPi0ForRLActionPrediction`` (no ``model.`` prefix, e.g. plain
``action_in_proj.bias``), exactly like the locally-converted
``tests_au/example/libero/_ckpt/pi05_base_pt`` / ``pi05_base_pt_fp32`` (812
tensors, no prefix, all F32) produced by
``rlinf/utils/ckpt_convertor/convert_openpi_jax_to_python.py``.

So this script does the small, RLinf-core-untouched translation: strip the
``model.`` prefix, cast every tensor to float32 (lossless roundtrip even for the
already-bf16 gemma_expert half; ``to_bfloat16_for_selected_params`` in
``get_model()`` re-casts the relevant submodules back down at load time anyway),
and write a minimal ``config.json`` mirroring ``pi05_base_pt_fp32``'s schema
(purely documentary -- ``get_model()`` never reads this file; the real model
architecture comes from the Hydra/``Pi0Config`` side via ``--config-name``).

norm_stats.json is deliberately NOT derived from this checkpoint's own
``policy_preprocessor_step_2_normalizer_processor.safetensors``: those stats were
computed on a *different* dataset (``local/r1_pro_chassis_v30``, per
``train_config.json``) with a different (23-dim) state layout, not on pushdoor
directly. Use ``compute_norm_stats_au.py`` for that instead.

Usage:
    /mnt/r/VENV/openpi_venv/bin/python tests_au/example/pushdoor/convert_r1pro_ckpt.py \\
        --output_dir tests_au/example/pushdoor/_ckpt/pi05_r1pro_pt
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess

DEFAULT_GCS_SRC = (
    "gs://physical-ai-data-eu/CKPT/VLA/PI/pi05_r1pro_chassis_alig_newnorm/"
    "R1/checkpoints/001000/pretrained_model/model.safetensors"
)
# Populated by an earlier investigation pass (see manual); reused here to avoid
# re-downloading the ~8.8GB file if it's still sitting in the local cache.
DEFAULT_LOCAL_CACHE = "/tmp/ckpt_probe/r1pro_model.safetensors"

# Mirrors tests_au/example/pushdoor/_ckpt/pi05_base_pt_fp32/config.json's schema.
# Purely documentary: get_model() never reads this file (see module docstring).
CONFIG_JSON = {
    "action_dim": 32,
    "action_horizon": 10,
    "paligemma_variant": "gemma_2b",
    "action_expert_variant": "gemma_300m",
    "precision": "float32",
}


def _fetch_source(src: str, cache: str) -> pathlib.Path:
    cache_path = pathlib.Path(cache)
    if cache_path.is_file():
        print(f"[convert_r1pro_ckpt] reusing cached download at {cache_path}")
        return cache_path
    if src.startswith("gs://"):
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        print(f"[convert_r1pro_ckpt] downloading {src} -> {cache_path}")
        subprocess.run(["gsutil", "cp", src, str(cache_path)], check=True)
        return cache_path
    return pathlib.Path(src)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=DEFAULT_GCS_SRC)
    ap.add_argument("--local_cache", default=DEFAULT_LOCAL_CACHE)
    ap.add_argument(
        "--output_dir", default="tests_au/example/pushdoor/_ckpt/pi05_r1pro_pt"
    )
    args = ap.parse_args()

    import safetensors.torch as st_torch

    src_path = _fetch_source(args.src, args.local_cache)

    print(f"[convert_r1pro_ckpt] loading {src_path}")
    state_dict = st_torch.load_file(str(src_path), device="cpu")
    print(f"[convert_r1pro_ckpt] {len(state_dict)} tensors loaded")

    converted = {}
    n_stripped, n_kept = 0, 0
    for key, tensor in state_dict.items():
        if key.startswith("model."):
            new_key = key[len("model.") :]
            n_stripped += 1
        else:
            new_key = key
            n_kept += 1
        converted[new_key] = tensor.float()
    print(
        f"[convert_r1pro_ckpt] stripped 'model.' prefix from {n_stripped} keys "
        f"({n_kept} left as-is); cast all to float32"
    )

    out_dir = pathlib.Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_weights = out_dir / "model.safetensors"
    st_torch.save_file(converted, str(out_weights))
    print(f"[convert_r1pro_ckpt] wrote {out_weights}")

    out_config = out_dir / "config.json"
    out_config.write_text(json.dumps(CONFIG_JSON, indent=4) + "\n")
    print(f"[convert_r1pro_ckpt] wrote {out_config}")
    print("[convert_r1pro_ckpt] done.")


if __name__ == "__main__":
    main()
