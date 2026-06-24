"""Layer-1 forward numerical alignment: openpi-JAX vs RLinf openpi_au (PyTorch).

Strategy (per rlinfpi_accept_1.md §A): build ONE shared Observation from a LIBERO
subset, sample (noise, time) on the JAX side, inject the SAME (noise, time) into the
PyTorch side, and compare v_t / loss. Sides run as isolated subprocesses to avoid
loading two 3B models at once.

Usage (orchestrated):
  python tests_au/scripts/forward_align.py \
      --subset_path tests_au/scripts/_data/libero_subset \
      --jax_config pi05_libero \
      --jax_ckpt <pi05_base JAX dir> \
      --pt_ckpt  <pi05_base PT dir> \
      --precision bf16 --num_samples 4 \
      --out_report tests_au/scripts/_out/forward_align_bf16.json

Sub-modes (used internally): --side {jax,pt,compare}
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import compare_utils as cu  # noqa: E402
from extract_libero_subset import load_subset  # noqa: E402


# --------------------------------------------------------------------------------------
# Shared input construction (identical on both sides)
# --------------------------------------------------------------------------------------
def build_inputs(subset_path: str, num_samples: int, action_dim: int, action_horizon: int,
                 max_token_len: int, image_res: int = 224):
    """Build a deterministic shared Observation as numpy arrays from the LIBERO subset.

    Returns a dict with images (3 keys), image_masks, state, tokenized_prompt,
    tokenized_prompt_mask, and actions. Both JAX and PyTorch sides call this so the
    inputs are bit-identical.
    """
    data = load_subset(subset_path, num_samples)
    n = data["image"].shape[0]

    def _prep_img(arr_uint8):
        # resize to (image_res, image_res), scale uint8[0,255] -> float32[-1,1]
        from PIL import Image

        out = np.empty((n, image_res, image_res, 3), dtype=np.float32)
        for i in range(n):
            im = Image.fromarray(arr_uint8[i]).resize((image_res, image_res), Image.BILINEAR)
            out[i] = (np.asarray(im, dtype=np.float32) / 127.5) - 1.0
        return out

    base = _prep_img(data["image"])
    wrist = _prep_img(data["wrist_image"])
    right = np.zeros_like(base)

    images = {
        "base_0_rgb": base,
        "left_wrist_0_rgb": wrist,
        "right_wrist_0_rgb": right,
    }
    image_masks = {
        "base_0_rgb": np.ones((n,), dtype=bool),
        "left_wrist_0_rgb": np.ones((n,), dtype=bool),
        "right_wrist_0_rgb": np.zeros((n,), dtype=bool),
    }

    # state: pad real libero state [8] -> [action_dim]
    state = np.zeros((n, action_dim), dtype=np.float32)
    s = data["state"]
    state[:, : s.shape[1]] = s

    # tokenized prompt: tokenize the real prompt (pi05 format includes discretized state)
    tok, tok_mask = _tokenize_prompts(data["prompts"], state, max_token_len)

    # actions: deterministic, derived from real libero actions padded to [ah, action_dim]
    actions = np.zeros((n, action_horizon, action_dim), dtype=np.float32)
    a = data["actions"]  # [n, 7]
    for h in range(action_horizon):
        actions[:, h, : a.shape[1]] = a  # tile across horizon (deterministic, identical both sides)

    return {
        "images": images,
        "image_masks": image_masks,
        "state": state,
        "tokenized_prompt": tok,
        "tokenized_prompt_mask": tok_mask,
        "actions": actions,
    }


def _tokenize_prompts(prompts, state, max_token_len):
    from openpi.models.tokenizer import PaligemmaTokenizer

    tok = PaligemmaTokenizer(max_len=max_token_len)
    n = len(prompts)
    out_tok = np.zeros((n, max_token_len), dtype=np.int32)
    out_mask = np.zeros((n, max_token_len), dtype=bool)
    for i, p in enumerate(prompts):
        t, m = tok.tokenize(p, state=state[i])
        out_tok[i] = t.astype(np.int32)
        out_mask[i] = m.astype(bool)
    return out_tok, out_mask


# --------------------------------------------------------------------------------------
# JAX side
# --------------------------------------------------------------------------------------
def run_jax_side(args):
    import jax
    import jax.numpy as jnp
    import openpi.models.model as _model
    import openpi.training.config as _config
    from openpi.models.pi0 import make_attn_mask

    dtype = jnp.float32 if args.precision == "fp32" else jnp.bfloat16
    train_config = _config.get_config(args.jax_config)
    mcfg = train_config.model
    model = train_config.model.load(
        _model.restore_params(Path(args.jax_ckpt) / "params", dtype=dtype)
    )

    inp = build_inputs(args.subset_path, args.num_samples, mcfg.action_dim,
                       mcfg.action_horizon, mcfg.max_token_len)

    obs = _model.Observation(
        images={k: jnp.asarray(v) for k, v in inp["images"].items()},
        image_masks={k: jnp.asarray(v) for k, v in inp["image_masks"].items()},
        state=jnp.asarray(inp["state"]),
        tokenized_prompt=jnp.asarray(inp["tokenized_prompt"]),
        tokenized_prompt_mask=jnp.asarray(inp["tokenized_prompt_mask"]),
    )
    actions = jnp.asarray(inp["actions"])

    # sample noise/time exactly as compute_loss does
    rng = jax.random.key(0)
    _, noise_rng, time_rng = jax.random.split(rng, 3)
    batch_shape = actions.shape[:-2]
    noise = jax.random.normal(noise_rng, actions.shape)
    time = jax.random.beta(time_rng, 1.5, 1, batch_shape) * 0.999 + 0.001

    obs = _model.preprocess_observation(None, obs, train=False)
    time_expanded = time[..., None, None]
    x_t = time_expanded * noise + (1 - time_expanded) * actions
    u_t = noise - actions

    prefix_tokens, prefix_mask, prefix_ar_mask = model.embed_prefix(obs)
    suffix_tokens, suffix_mask, suffix_ar_mask, adarms_cond = model.embed_suffix(obs, x_t, time)
    input_mask = jnp.concatenate([prefix_mask, suffix_mask], axis=1)
    ar_mask = jnp.concatenate([prefix_ar_mask, suffix_ar_mask], axis=0)
    attn_mask = make_attn_mask(input_mask, ar_mask)
    positions = jnp.cumsum(input_mask, axis=1) - 1
    (_, suffix_out), _ = model.PaliGemma.llm(
        [prefix_tokens, suffix_tokens], mask=attn_mask, positions=positions,
        adarms_cond=[None, adarms_cond],
    )
    v_t = model.action_out_proj(suffix_out[:, -model.action_horizon:])
    loss = jnp.mean(jnp.square(v_t - u_t), axis=-1)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    np.savez(
        args.out,
        noise=np.asarray(noise, dtype=np.float32),
        time=np.asarray(time, dtype=np.float32),
        v_t=np.asarray(v_t, dtype=np.float32),
        loss=np.asarray(loss, dtype=np.float32),
    )
    print(f"[jax] dumped v_t{tuple(v_t.shape)} loss{tuple(loss.shape)} -> {args.out}")


# --------------------------------------------------------------------------------------
# PyTorch side (openpi_au)
# --------------------------------------------------------------------------------------
def run_pt_side(args):
    import torch
    from openpi.models import model as _model
    import openpi.training.config as _config

    from rlinf.models.embodiment.openpi_au.openpi_action_model import (
        OpenPi0Config,
        OpenPi0ForRLActionPrediction,
    )

    device = "cuda" if torch.cuda.is_available() else "cpu"
    dtype = torch.float32 if args.precision == "fp32" else torch.bfloat16

    train_config = _config.get_config(args.jax_config)
    mcfg = train_config.model
    cfg_kwargs = dict(mcfg.__dict__)
    cfg_kwargs["config_name"] = args.jax_config
    cfg_kwargs["faithful_augmentation"] = False  # deterministic alignment
    au_cfg = OpenPi0Config(**cfg_kwargs)

    model = OpenPi0ForRLActionPrediction(au_cfg)
    import safetensors.torch as st

    sd = st.load_file(os.path.join(args.pt_ckpt, "model.safetensors"), device="cpu")
    model.load_state_dict(sd, strict=False)
    model = model.to(device=device).eval()
    # Mimic openpi mixed precision: only selected params -> bf16 (action_out_proj etc.
    # stay fp32, matching pi0_pytorch.forward which casts suffix_out to fp32 first).
    if args.precision == "bf16":
        model.paligemma_with_expert.to_bfloat16_for_selected_params("bfloat16")

    inp = build_inputs(args.subset_path, args.num_samples, mcfg.action_dim,
                       mcfg.action_horizon, mcfg.max_token_len)

    # Feed fp32 inputs; the model casts internally to bf16 where its weights are bf16
    # (pi0_pytorch.forward casts prefix/suffix embeddings based on llm weight dtype).
    def _t(x, d=torch.float32):
        return torch.as_tensor(x, device=device, dtype=d)

    # PyTorch SigLIP expects channels-first NCHW; build_inputs returns NHWC (JAX-native).
    def _img(v):
        return _t(v).permute(0, 3, 1, 2).contiguous()  # [B,H,W,C] -> [B,C,H,W]

    obs = _model.Observation(
        images={k: _img(v) for k, v in inp["images"].items()},
        image_masks={k: torch.as_tensor(v, device=device) for k, v in inp["image_masks"].items()},
        state=_t(inp["state"]),
        tokenized_prompt=torch.as_tensor(inp["tokenized_prompt"], device=device),
        tokenized_prompt_mask=torch.as_tensor(inp["tokenized_prompt_mask"], device=device),
    )

    jax_data = np.load(args.in_noise)
    noise = _t(jax_data["noise"])
    time = _t(jax_data["time"])
    actions = _t(inp["actions"])

    # force train=False preprocessing (no augmentation) for deterministic alignment
    orig_prep = model._preprocess_observation
    model._preprocess_observation = lambda observation, train=True: orig_prep(observation, train=False)

    captured = {}

    def _hook(_m, _i, output):
        captured["v_t"] = output.detach().float().cpu().numpy()

    handle = model.action_out_proj.register_forward_hook(_hook)
    # Call the base PI0Pytorch flow-matching forward directly (the RL subclass overrides
    # forward() to dispatch by ForwardType, which would not accept noise/time).
    from openpi.models_pytorch.pi0_pytorch import PI0Pytorch

    with torch.no_grad():
        loss_elem = PI0Pytorch.forward(model, obs, actions, noise=noise, time=time)
    handle.remove()

    loss = loss_elem.float().mean(dim=-1).cpu().numpy()  # [B, ah] to match JAX
    v_t = captured.get("v_t")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    np.savez(args.out, v_t=v_t.astype(np.float32), loss=loss.astype(np.float32))
    print(f"[pt] dumped v_t{None if v_t is None else tuple(v_t.shape)} "
          f"loss{tuple(loss.shape)} -> {args.out}")


# --------------------------------------------------------------------------------------
# Compare
# --------------------------------------------------------------------------------------
def run_compare(args):
    jax_d = np.load(args.jax)
    pt_d = np.load(args.pt)
    entries = {}
    for name in ["v_t", "loss"]:
        if name in jax_d and name in pt_d:
            t = cu.tol(name, args.precision)
            entries[name] = cu.compare_pointwise(jax_d[name], pt_d[name], t)
    report = cu.make_report(entries)
    report["precision"] = args.precision
    cu.dump_report(report, args.out_report)
    print(cu.pretty_print_report(report))
    return 0 if report["overall_pass"] else 1


# --------------------------------------------------------------------------------------
# Orchestrator
# --------------------------------------------------------------------------------------
def run_orchestrate(args):
    out_dir = Path(args.out_report).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    jax_npz = str(out_dir / f"_jax_{args.precision}.npz")
    pt_npz = str(out_dir / f"_pt_{args.precision}.npz")
    py = sys.executable
    script = str(Path(__file__).resolve())

    cmd_jax = [py, script, "--side", "jax", "--subset_path", args.subset_path,
               "--jax_config", args.jax_config, "--jax_ckpt", args.jax_ckpt,
               "--precision", args.precision, "--num_samples", str(args.num_samples),
               "--out", jax_npz]
    cmd_pt = [py, script, "--side", "pt", "--subset_path", args.subset_path,
              "--jax_config", args.jax_config, "--pt_ckpt", args.pt_ckpt,
              "--precision", args.precision, "--num_samples", str(args.num_samples),
              "--in_noise", jax_npz, "--out", pt_npz]

    print(f"[orchestrate] JAX side: {' '.join(cmd_jax)}")
    subprocess.run(cmd_jax, check=True)
    print(f"[orchestrate] PT side: {' '.join(cmd_pt)}")
    subprocess.run(cmd_pt, check=True)

    args.jax = jax_npz
    args.pt = pt_npz
    return run_compare(args)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["jax", "pt", "compare", "all"], default="all")
    ap.add_argument("--subset_path", default="tests_au/scripts/_data/libero_subset")
    ap.add_argument("--jax_config", default="pi05_libero")
    ap.add_argument("--jax_ckpt", default=None)
    ap.add_argument("--pt_ckpt", default=None)
    ap.add_argument("--precision", choices=["fp32", "bf16"], default="bf16")
    ap.add_argument("--num_samples", type=int, default=4)
    ap.add_argument("--in_noise", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--jax", default=None)
    ap.add_argument("--pt", default=None)
    ap.add_argument("--out_report", default="tests_au/scripts/_out/forward_align_report.json")
    args = ap.parse_args()

    if args.side == "jax":
        run_jax_side(args)
    elif args.side == "pt":
        run_pt_side(args)
    elif args.side == "compare":
        sys.exit(run_compare(args))
    else:
        sys.exit(run_orchestrate(args))


if __name__ == "__main__":
    main()
