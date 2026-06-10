#!/usr/bin/env python3
# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Convert an RLinf FSDP checkpoint directory to Hugging Face format.

This script wraps the official RLinf converters:

* ``rlinf.utils.ckpt_convertor.fsdp_convertor.convert_dcp_to_pt`` — optional
* ``rlinf.utils.ckpt_convertor.fsdp_convertor.convert_pt_to_hf`` — core logic

It accepts a **checkpoint root** (e.g. ``.../checkpoints/global_step_1000``),
the **actor** subdirectory, a ``full_weights.pt`` file, or a ``dcp_checkpoint``
directory, then writes HF ``safetensors`` (+ model-specific side artifacts such as
``fastwam_native.pt`` for FastWAM).

References
----------
* docs: ``docs/source-zh/rst_source/tutorials/advance/convertor.rst``
* docs: ``docs/source-zh/rst_source/tutorials/advance/resume.rst``
* config: ``rlinf/utils/ckpt_convertor/fsdp_convertor/config/fsdp_dreamzero_convertor.yaml``

Evn Var:
export DIFFSYNTH_MODEL_BASE_PATH=/mnt/r/CKPT/VLA/FW
export DIFFSYNTH_SKIP_DOWNLOAD=true

Examples
--------
From an SFT ``global_step_*`` directory (auto-find ``tensorboard/config.yaml``)::

    export REPO_PATH=/path/to/RLinf
    export RLINF_EXT_MODULE=...   # if model_type is registered via extension only
    python b/scripts/rlinf_ckpt_to_hf.py \\
        --checkpoint /path/to/logs/exp/checkpoints/global_step_3000 \\
        --output /path/to/hf_fastwam_step3000 \\
        --torch-dtype bf16

From ``full_weights.pt`` with an explicit training yaml (Hydra ``defaults``
are merged automatically, same as during training)::

    export EMBODIED_PATH=/path/to/RLinf/examples/sft   # if config uses searchpath
    python b/scripts/rlinf_ckpt_to_hf.py \\
        --checkpoint /path/to/actor/model_state_dict/full_weights.pt \\
        --train-config examples/sft/config/r1_pro_sft_fastwam.yaml \\
        --output /path/to/hf_out

When only ``dcp_checkpoint/`` exists (older runs)::

    python b/scripts/rlinf_ckpt_to_hf.py \\
        --checkpoint /path/to/checkpoints/global_step_100/actor \\
        --output /path/to/hf_out \\
        --from-dcp

export EMBODIED_PATH=/home/Luogang/SRC/RL/RLinf/examples/sft
python b/scripts/rlinf_ckpt_to_hf.py --checkpoint /mnt/r/CKPT/VLA/FW/RUN/R1PR/R1/checkpoints/global_step_1000/actor   --output /mnt/r/CKPT/VLA/FW/RUN/R1PR/R1/checkpoints/global_step_1000_HF  --train-config examples/sft/config/r1_pro_sft_fastwam.yaml  --torch-dtype bf16  --from-dcp

"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

# Allow running as ``python b/scripts/rlinf_ckpt_to_hf.py`` without installing the package.
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


_FULL_WEIGHTS_CANDIDATES = (
    "model_state_dict/full_weights.pt",
    "actor/model_state_dict/full_weights.pt",
    "full_weights.pt",
    "actor/model.pt",  # legacy examples in yaml comments
    "model.pt",
)

_DCP_CANDIDATES = (
    "dcp_checkpoint",
    "actor/dcp_checkpoint",
)


def _is_pt_file(path: Path) -> bool:
    return path.is_file() and path.suffix == ".pt"


def _find_first_existing(base: Path, relatives: tuple[str, ...]) -> Path | None:
    for rel in relatives:
        candidate = base / rel
        if candidate.exists():
            return candidate
    return None


def resolve_weights_pt(checkpoint: str | Path) -> Path:
    """Resolve ``full_weights.pt`` from common RLinf checkpoint layouts."""
    path = Path(checkpoint).expanduser().resolve()
    if _is_pt_file(path):
        return path

    if not path.is_dir():
        raise FileNotFoundError(f"Checkpoint path does not exist: {path}")

    found = _find_first_existing(path, _FULL_WEIGHTS_CANDIDATES)
    if found is not None:
        return found

    raise FileNotFoundError(
        "Could not locate full_weights.pt under checkpoint path. Tried:\n"
        + "\n".join(f"  - {path / rel}" for rel in _FULL_WEIGHTS_CANDIDATES)
        + "\nIf only dcp_checkpoint exists, pass --from-dcp."
    )


def resolve_dcp_dir(checkpoint: str | Path) -> Path:
    path = Path(checkpoint).expanduser().resolve()
    if path.is_dir() and any(p.suffix == ".distcp" for p in path.iterdir()):
        return path

    found = _find_first_existing(path, _DCP_CANDIDATES)
    if found is not None:
        return found

    raise FileNotFoundError(
        f"No dcp_checkpoint directory found under {path}. "
        f"Expected one of: {', '.join(_DCP_CANDIDATES)}"
    )


def convert_dcp_to_pt(dcp_path: Path, output_pt: Path) -> Path:
    """Mirror ``convert_dcp_to_pt.py`` (no distributed init required)."""
    import torch
    from torch.distributed.checkpoint import FileSystemReader
    from torch.distributed.checkpoint.format_utils import _EmptyStateDictLoadPlanner
    from torch.distributed.checkpoint.state_dict_loader import _load_state_dict

    output_pt.parent.mkdir(parents=True, exist_ok=True)
    checkpoint: dict = {}
    _load_state_dict(
        checkpoint,
        storage_reader=FileSystemReader(str(dcp_path)),
        planner=_EmptyStateDictLoadPlanner(keys={"fsdp_checkpoint.model"}),
        no_dist=True,
    )
    try:
        model_state_dict = checkpoint["fsdp_checkpoint"]["model"]
    except KeyError as exc:
        raise KeyError(
            "Could not find 'fsdp_checkpoint.model' in the DCP checkpoint. "
            f"Loaded top-level keys: {list(checkpoint.keys())}"
        ) from exc

    torch.save(model_state_dict, output_pt)
    print(f"Converted DCP -> PT: {dcp_path} -> {output_pt}")
    return output_pt


def resolve_train_config(
    checkpoint: str | Path,
    train_config: str | None,
    *,
    auto_find: bool,
) -> str | None:
    from rlinf.utils.ckpt_convertor.fsdp_convertor.convert_pt_to_hf import (
        _find_rlinf_train_config_near_ckpt,
    )

    if train_config:
        cfg_path = Path(train_config).expanduser().resolve()
        if not cfg_path.is_file():
            raise FileNotFoundError(f"Training config not found: {cfg_path}")
        return str(cfg_path)

    if not auto_find:
        return None

    # Search near weights path or checkpoint directory.
    path = Path(checkpoint).expanduser().resolve()
    search_seed = path if path.is_file() else path
    found = _find_rlinf_train_config_near_ckpt(str(search_seed))
    if found:
        print(f"Auto-discovered training config: {found}")
    return found


def materialize_train_config(
    train_config_path: str | Path,
    *,
    cache_dir: Path,
    compose_defaults: bool = True,
) -> str:
    """Return a training config path with Hydra defaults merged when needed."""
    from omegaconf import OmegaConf

    from rlinf.utils.ckpt_convertor.fsdp_convertor.convert_pt_to_hf import (
        load_train_config,
    )

    path = Path(train_config_path).expanduser().resolve()
    if not compose_defaults:
        return str(path)

    raw_cfg = OmegaConf.load(path)
    if "defaults" not in raw_cfg:
        return str(path)

    composed_cfg = load_train_config(str(path), compose_defaults=True)
    cache_dir.mkdir(parents=True, exist_ok=True)
    resolved_path = cache_dir / f"{path.stem}.resolved.yaml"
    OmegaConf.save(composed_cfg, resolved_path, resolve=True)
    print(f"Cached resolved training config: {resolved_path}")
    return str(resolved_path)


def _maybe_copy_dataset_stats(checkpoint: Path, output_dir: Path) -> None:
    """Copy ``dataset_stats.json`` for FastWAM deploy if found near the run."""
    search_dirs = []
    if checkpoint.is_file():
        search_dirs.extend(checkpoint.parents)
    else:
        search_dirs.extend([checkpoint, *checkpoint.parents])

    for directory in search_dirs[:12]:
        candidate = directory / "dataset_stats.json"
        if candidate.is_file():
            dst = output_dir / "dataset_stats.json"
            shutil.copy2(candidate, dst)
            print(f"Copied dataset stats: {candidate} -> {dst}")
            return


def build_convertor_cfg(
    *,
    ckpt_path: str,
    save_path: str,
    train_config_path: str | None,
    auto_find_train_config: bool,
    compose_train_config: bool,
    torch_dtype: str | None,
    merge_lora: bool,
    strict_load: bool,
    model_overrides: dict | None,
):
    from omegaconf import OmegaConf

    return OmegaConf.create(
        {
            "convertor": {
                "ckpt_path": ckpt_path,
                "save_path": save_path,
                "train_config_path": train_config_path,
                "auto_find_train_config": auto_find_train_config,
                "compose_train_config": compose_train_config,
                "torch_dtype": torch_dtype,
                "merge_lora_weighs": merge_lora,
                "strict_load": strict_load,
                "model_overrides": model_overrides,
            }
        }
    )


def convert_pt_to_hf(cfg) -> None:
    """Run the same flow as ``convert_pt_to_hf.main`` without Hydra."""
    import torch

    from rlinf.models import get_model
    from rlinf.scheduler.cluster import load_user_extension_module
    from rlinf.utils.ckpt_convertor.fsdp_convertor.convert_pt_to_hf import (
        _extract_state_dict,
        _normalize_state_dict_keys,
        _prepare_model_cfg,
        _resolve_model_cfg,
        _save_hf_checkpoint,
    )

    load_user_extension_module()

    model_cfg = _prepare_model_cfg(_resolve_model_cfg(cfg))
    model = get_model(model_cfg)

    checkpoint = torch.load(cfg.convertor.ckpt_path, map_location="cpu")
    model_dict = _normalize_state_dict_keys(_extract_state_dict(checkpoint))
    strict_load = bool(cfg.convertor.get("strict_load", True))
    missing_keys, unexpected_keys = model.load_state_dict(model_dict, strict=strict_load)
    if missing_keys or unexpected_keys:
        print(
            f"Loaded checkpoint with {len(missing_keys)} missing keys and "
            f"{len(unexpected_keys)} unexpected keys."
        )
        if missing_keys:
            print(f"First missing keys: {missing_keys[:20]}")
        if unexpected_keys:
            print(f"First unexpected keys: {unexpected_keys[:20]}")

    save_path = str(cfg.convertor.save_path)
    os.makedirs(save_path, exist_ok=True)
    _save_hf_checkpoint(model, model_cfg, cfg, save_path)
    print(f"Saved Hugging Face checkpoint to {save_path}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert RLinf FSDP checkpoint to Hugging Face safetensors format.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--checkpoint",
        required=True,
        help=(
            "RLinf checkpoint: global_step_* dir, actor/ dir, full_weights.pt, "
            "or dcp_checkpoint/ (with --from-dcp)."
        ),
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output directory for Hugging Face weights and config files.",
    )
    parser.add_argument(
        "--train-config",
        default=None,
        help=(
            "Training Hydra yaml with actor.model (e.g. examples/sft/config/*.yaml "
            "or {log}/tensorboard/config.yaml). Hydra defaults are merged "
            "automatically. Auto-discovered when omitted."
        ),
    )
    parser.add_argument(
        "--no-auto-train-config",
        action="store_true",
        help="Disable auto-discovery of tensorboard/config.yaml near checkpoint.",
    )
    parser.add_argument(
        "--no-compose-train-config",
        action="store_true",
        help=(
            "Disable Hydra defaults merging for --train-config. Use only when the "
            "yaml is already fully resolved (e.g. tensorboard/config.yaml)."
        ),
    )
    parser.add_argument(
        "--from-dcp",
        action="store_true",
        help="Convert actor/dcp_checkpoint to PT first when full_weights.pt is missing.",
    )
    parser.add_argument(
        "--pt-cache",
        default=None,
        help="Where to write intermediate PT converted from DCP (default: temp dir).",
    )
    parser.add_argument(
        "--torch-dtype",
        default=None,
        choices=["bf16", "bfloat16", "fp16", "float16", "fp32", "float32", "null"],
        help="Cast saved weights to this dtype (default: keep checkpoint dtypes).",
    )
    parser.add_argument(
        "--merge-lora",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Merge LoRA into base weights when model.is_lora is true (default: true).",
    )
    parser.add_argument(
        "--strict-load",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use strict=True in load_state_dict (default: true).",
    )
    parser.add_argument(
        "--copy-dataset-stats",
        action="store_true",
        help="Copy dataset_stats.json from the training run into --output if present.",
    )
    parser.add_argument(
        "--model-override",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Override actor.model fields, e.g. model_path=/path/to/base",
    )
    return parser.parse_args(argv)


def _parse_model_overrides(pairs: list[str]) -> dict | None:
    if not pairs:
        return None
    overrides: dict = {}
    for item in pairs:
        if "=" not in item:
            raise ValueError(f"Invalid --model-override (expected KEY=VALUE): {item}")
        key, value = item.split("=", 1)
        overrides[key] = value
    return overrides


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    checkpoint_path = Path(args.checkpoint).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()

    torch_dtype = args.torch_dtype
    if torch_dtype == "null":
        torch_dtype = None

    model_overrides = _parse_model_overrides(args.model_override)

    # Step 1: resolve PT weights (optionally from DCP).
    pt_path: Path
    try:
        pt_path = resolve_weights_pt(checkpoint_path)
    except FileNotFoundError:
        if not args.from_dcp:
            raise
        dcp_dir = resolve_dcp_dir(checkpoint_path)
        if args.pt_cache:
            pt_path = Path(args.pt_cache).expanduser().resolve()
        else:
            cache_dir = output_dir / ".rlinf_ckpt_cache"
            cache_dir.mkdir(parents=True, exist_ok=True)
            pt_path = cache_dir / "full_weights_from_dcp.pt"
        convert_dcp_to_pt(dcp_dir, pt_path)

    print(f"Using weights: {pt_path}")

    # Step 2: resolve training config for actor.model.
    train_config = resolve_train_config(
        pt_path,
        args.train_config,
        auto_find=not args.no_auto_train_config,
    )
    if train_config is None and not model_overrides:
        raise SystemExit(
            "Could not resolve actor.model config. Pass --train-config or place "
            "tensorboard/config.yaml near the checkpoint, or supply --model-override."
        )

    compose_train_config = not args.no_compose_train_config
    if train_config is not None:
        train_config = materialize_train_config(
            train_config,
            cache_dir=output_dir / ".rlinf_ckpt_cache",
            compose_defaults=compose_train_config,
        )

    cfg = build_convertor_cfg(
        ckpt_path=str(pt_path),
        save_path=str(output_dir),
        train_config_path=train_config,
        auto_find_train_config=not args.no_auto_train_config and train_config is None,
        compose_train_config=compose_train_config,
        torch_dtype=torch_dtype,
        merge_lora=args.merge_lora,
        strict_load=args.strict_load,
        model_overrides=model_overrides,
    )

    # Step 3: PT -> HF (reuses official converter helpers).
    convert_pt_to_hf(cfg)

    if args.copy_dataset_stats:
        _maybe_copy_dataset_stats(checkpoint_path, output_dir)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
