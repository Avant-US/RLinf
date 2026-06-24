# Acceptance scripts (`tests_au/scripts/`)

Executable acceptance pipeline for "RLinf openpi_au reproduces openpi π₀.₅". Implements
the three-layer protocol from [`b/d/pi/rlinfpi_accept_1.md`](../../b/d/pi/rlinfpi_accept_1.md):
L1 forward numerical alignment → L2 training-curve alignment → L3 inference/eval.

## Files

| Script | Layer | What it does |
| --- | --- | --- |
| `compare_utils.py` | — | Pure-numpy comparison helpers (tolerances, spearman, magnitude, report, plot). |
| `extract_libero_subset.py` | — | Deterministically extract N LIBERO frames → `batch.npz` + `meta.json`. |
| `forward_align.py` | L1 | Same Observation + injected (noise,time) → compare `v_t`/`loss` (openpi-JAX vs openpi_au). |
| `train_compare.py` | L2 | Real optax LR vs RLinf `_build_openpi_cosine`; real few-step PyTorch training (loss↓, grad_norm, lr). |
| `eval_compare.py` | L3 | Load trained weights, run `sample_actions` smoke; defer to LIBERO sim for full eval. |
| `ablation_runner.py` | §D | Few-step ablations: `precision` (fp32-master vs pure-bf16), `lr` (openpi_cosine vs constant). |
| `run_acceptance.sh` | all | One-click L1→L2→L3. |

## Prerequisites

```bash
# editable RLinf install + deps already present in the venv
export PY=/mnt/r/VENV/openpi_venv/bin/python
export PYTHONPATH=$PYTHONPATH:/home/physical/SRC/Robot/openpi05/src

# weights
export JAX_CKPT=$HOME/.cache/openpi/openpi-assets/checkpoints/pi05_base   # has params/
export PT_CKPT=/mnt/r/CKPT/VLA/pi05_base_pt_fp32                          # has model.safetensors
# (PT_CKPT produced by rlinf/utils/ckpt_convertor/convert_openpi_jax_to_python.py)
```

## Run

```bash
# one-click
PY=$PY JAX_CKPT=$JAX_CKPT PT_CKPT=$PT_CKPT bash tests_au/scripts/run_acceptance.sh

# or step by step
$PY tests_au/scripts/extract_libero_subset.py --num_samples 4 --out_dir tests_au/scripts/_data/libero_subset
$PY tests_au/scripts/forward_align.py  --side all --jax_ckpt $JAX_CKPT --pt_ckpt $PT_CKPT --precision bf16 --num_samples 4
$PY tests_au/scripts/train_compare.py  --side all --jax_ckpt $JAX_CKPT --pt_ckpt $PT_CKPT --num_steps 8 --batch_size 2 --warmup 3
$PY tests_au/scripts/eval_compare.py   --rlinf_ckpt $PT_CKPT --mode smoke
$PY tests_au/scripts/ablation_runner.py --ablation lr --pt_ckpt $PT_CKPT --num_steps 4 --repeats 1
```

Reports land in `tests_au/scripts/_out/`. See `rlinfpi_accept_1.md` §H for the pass criteria
and §G for the empirical findings (notably: openpi-JAX computes in bf16, so the canonical
acceptance precision is **bf16** with `loss` as the primary tight metric).
