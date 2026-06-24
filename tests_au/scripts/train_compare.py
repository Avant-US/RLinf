"""Layer-2 training-curve alignment: openpi-JAX reference vs RLinf openpi_au (PyTorch).

Per rlinfpi_accept_1.md §B, with the scope clarification documented in §G:
  * The LR schedule is compared cross-framework: the *real* openpi optax
    `warmup_cosine_decay_schedule` vs the *real* RLinf worker `_build_openpi_cosine`
    (tight, pointwise < 1e-6).
  * The flow-matching loss MAGNITUDE is anchored to the JAX step-0 forward loss
    (reusing the validated L1 path), compared with a relative tolerance.
  * A REAL few-step PyTorch training loop runs on the system-under-test (openpi_au
    model + AdamW + openpi_cosine LR + bf16 compute / fp32 master). We check that the
    loss decreases (overfitting a fixed tiny batch), grad_norm is finite, and the LR
    follows the schedule.

A full JAX optax training loop (mesh / data-loader / freeze-filter) is intentionally
out of scope for the lightweight acceptance; the JAX reference here is the exact optax
LR schedule + the real JAX forward loss. See §G for rationale.

Sub-modes: --side {jax_ref, pt_train, compare, all}
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import compare_utils as cu  # noqa: E402
import forward_align as fa  # noqa: E402


def _optim_cfg(peak=5e-5, decay_lr=5e-5, warmup=10, decay_steps=1_000_000):
    class Cfg:
        lr = peak

        def get(self, k, d=None):
            return {"decay_lr": decay_lr, "lr_warmup_steps": warmup,
                    "decay_steps": decay_steps}.get(k, d)

    return Cfg()


# --------------------------------------------------------------------------------------
# JAX reference: optax LR curve + step-0 forward loss
# --------------------------------------------------------------------------------------
def run_jax_ref(args):
    import jax
    import jax.numpy as jnp
    import openpi.models.model as _model
    import openpi.training.config as _config
    import optax
    from openpi.models.pi0 import make_attn_mask

    # --- optax LR curve (the exact openpi schedule) ---
    peak, decay_lr, warmup = args.lr, args.lr, args.warmup
    decay_steps = args.decay_steps
    sched = optax.warmup_cosine_decay_schedule(
        init_value=peak / (warmup + 1), peak_value=peak,
        warmup_steps=warmup, decay_steps=decay_steps, end_value=decay_lr,
    )
    lr_curve = [float(sched(i)) for i in range(args.num_steps)]

    # --- step-0 forward loss (real JAX forward) ---
    train_config = _config.get_config(args.jax_config)
    mcfg = train_config.model
    model = train_config.model.load(
        _model.restore_params(Path(args.jax_ckpt) / "params", dtype=jnp.bfloat16)
    )
    inp = fa.build_inputs(args.subset_path, args.batch_size, mcfg.action_dim,
                          mcfg.action_horizon, mcfg.max_token_len)
    obs = _model.Observation(
        images={k: jnp.asarray(v) for k, v in inp["images"].items()},
        image_masks={k: jnp.asarray(v) for k, v in inp["image_masks"].items()},
        state=jnp.asarray(inp["state"]),
        tokenized_prompt=jnp.asarray(inp["tokenized_prompt"]),
        tokenized_prompt_mask=jnp.asarray(inp["tokenized_prompt_mask"]),
    )
    actions = jnp.asarray(inp["actions"])
    rng = jax.random.key(0)
    _, noise_rng, time_rng = jax.random.split(rng, 3)
    noise = jax.random.normal(noise_rng, actions.shape)
    time = jax.random.beta(time_rng, 1.5, 1, actions.shape[:-2]) * 0.999 + 0.001
    obs = _model.preprocess_observation(None, obs, train=False)
    x_t = time[..., None, None] * noise + (1 - time[..., None, None]) * actions
    u_t = noise - actions
    pt, pm, par = model.embed_prefix(obs)
    st_, sm, sar, ada = model.embed_suffix(obs, x_t, time)
    im = jnp.concatenate([pm, sm], axis=1)
    arm = jnp.concatenate([par, sar], axis=0)
    attn = make_attn_mask(im, arm)
    pos = jnp.cumsum(im, axis=1) - 1
    (_, so), _ = model.PaliGemma.llm([pt, st_], mask=attn, positions=pos, adarms_cond=[None, ada])
    v_t = model.action_out_proj(so[:, -model.action_horizon:])
    loss0 = float(jnp.mean(jnp.square(v_t - u_t)))

    out = {"lr_curve": lr_curve, "loss0": loss0}
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"[jax_ref] loss0={loss0:.4f}, lr[0]={lr_curve[0]:.3e}, lr[-1]={lr_curve[-1]:.3e} -> {args.out}")


# --------------------------------------------------------------------------------------
# PyTorch real few-step training on the system-under-test
# --------------------------------------------------------------------------------------
def run_pt_train(args):
    import torch
    from openpi.models import model as _model
    from openpi.models_pytorch.pi0_pytorch import PI0Pytorch
    import openpi.training.config as _config

    from rlinf.models.embodiment.openpi_au.openpi_action_model import (
        OpenPi0Config,
        OpenPi0ForRLActionPrediction,
    )
    from rlinf.workers.sft.fsdp_vla_sft_worker_au import _build_openpi_cosine

    device = "cuda" if torch.cuda.is_available() else "cpu"
    train_config = _config.get_config(args.jax_config)
    mcfg = train_config.model
    cfg_kwargs = dict(mcfg.__dict__)
    cfg_kwargs["config_name"] = args.jax_config
    cfg_kwargs["faithful_augmentation"] = False
    au_cfg = OpenPi0Config(**cfg_kwargs)

    model = OpenPi0ForRLActionPrediction(au_cfg)
    import safetensors.torch as st

    sd = st.load_file(os.path.join(args.pt_ckpt, "model.safetensors"), device="cpu")
    model.load_state_dict(sd, strict=False)
    model = model.to(device=device).train()
    # Ablation knobs (env-driven; used by ablation_runner.py). Defaults reproduce §9.3.
    pure_bf16 = os.environ.get("AU_PURE_BF16", "0") == "1"
    ablation_seed = int(os.environ.get("AU_SEED", "0"))
    lr_scheduler_kind = os.environ.get("AU_LR_SCHEDULER", "openpi_cosine")
    if pure_bf16:
        model = model.to(dtype=torch.bfloat16)  # pure bf16 master (ablation A6 negative)
    else:
        # fp32 master + bf16 compute (selected params) — matches §9.3
        model.paligemma_with_expert.to_bfloat16_for_selected_params("bfloat16")

    inp = fa.build_inputs(args.subset_path, args.batch_size, mcfg.action_dim,
                          mcfg.action_horizon, mcfg.max_token_len)

    def _t(x):
        return torch.as_tensor(x, device=device, dtype=torch.float32)

    obs = _model.Observation(
        images={k: _t(v).permute(0, 3, 1, 2).contiguous() for k, v in inp["images"].items()},
        image_masks={k: torch.as_tensor(v, device=device) for k, v in inp["image_masks"].items()},
        state=_t(inp["state"]),
        tokenized_prompt=torch.as_tensor(inp["tokenized_prompt"], device=device),
        tokenized_prompt_mask=torch.as_tensor(inp["tokenized_prompt_mask"], device=device),
    )
    actions = _t(inp["actions"])

    # deterministic noise/time so the loss curve is clean (overfitting a fixed batch);
    # seed is ablation-controllable so repeats differ.
    g = torch.Generator(device=device).manual_seed(ablation_seed)
    noise = torch.randn(actions.shape, generator=g, device=device, dtype=torch.float32)
    time = torch.rand(actions.shape[0], generator=g, device=device, dtype=torch.float32) * 0.8 + 0.1

    # force train=False preprocessing (no random augmentation) for a clean trend
    orig_prep = model._preprocess_observation
    model._preprocess_observation = lambda observation, train=True: orig_prep(observation, train=False)

    optimizer = torch.optim.AdamW(
        [p for p in model.parameters() if p.requires_grad],
        lr=args.lr, betas=(0.9, 0.95), eps=1e-8, weight_decay=1e-10,
    )
    if lr_scheduler_kind == "constant":
        from torch.optim.lr_scheduler import LambdaLR

        sched = LambdaLR(optimizer, lambda _step: 1.0)  # ablation A: constant LR
    else:
        sched = _build_openpi_cosine(
            optimizer, _optim_cfg(args.lr, args.lr, args.warmup, args.decay_steps)
        )

    losses, lrs, gnorms, pnorms = [], [], [], []
    for step in range(args.num_steps):
        loss_elem = PI0Pytorch.forward(model, obs, actions, noise=noise, time=time)
        loss = loss_elem.float().mean()
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        gnorm = torch.nn.utils.clip_grad_norm_(
            [p for p in model.parameters() if p.requires_grad], 1.0
        )
        lrs.append(float(optimizer.param_groups[0]["lr"]))
        optimizer.step()
        sched.step()
        with torch.no_grad():
            pnorm = torch.norm(torch.stack(
                [p.detach().float().norm() for p in model.parameters() if p.requires_grad]
            ))
        losses.append(float(loss.item()))
        gnorms.append(float(gnorm))
        pnorms.append(float(pnorm))
        print(f"[pt_train] step {step}: loss={losses[-1]:.4f} lr={lrs[-1]:.3e} "
              f"gnorm={gnorms[-1]:.3f}")

    out = {"loss": losses, "lr": lrs, "grad_norm": gnorms, "param_norm": pnorms}
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"[pt_train] dumped {args.num_steps}-step log -> {args.out}")


# --------------------------------------------------------------------------------------
# Compare
# --------------------------------------------------------------------------------------
def run_compare(args):
    with open(args.jax) as f:
        jax_ref = json.load(f)
    with open(args.pt) as f:
        pt_log = json.load(f)

    entries = {}
    # LR: real optax vs real RLinf worker schedule — the only HARD gate (M2).
    lr_e = cu.compare_pointwise(pt_log["lr"], jax_ref["lr_curve"], tol_val=1e-6)
    lr_e["blocking"] = True
    entries["lr_schedule"] = lr_e

    # loss magnitude: PT step-0 vs JAX step-0 forward loss. Both stacks use DIFFERENT
    # RNG noise/time here (identical-input loss agreement is already proven by L1
    # forward_align), so this is a same-ballpark magnitude check (non-blocking warning).
    mag = cu.compare_magnitude([pt_log["loss"][0]], [jax_ref["loss0"]], ratio=(0.5, 2.0))
    mag["blocking"] = False
    entries["loss_magnitude"] = mag

    # loss trend: PT loss should decrease (overfitting a fixed batch). Non-blocking.
    steps = list(range(len(pt_log["loss"])))
    decreasing_ref = [-s for s in steps]
    trend = cu.compare_spearman(pt_log["loss"], decreasing_ref, min_corr=0.8)
    trend["blocking"] = False
    entries["loss_trend"] = trend

    # grad_norm finite + non-negative. Non-blocking sanity check.
    gn = np.asarray(pt_log["grad_norm"], dtype=np.float64)
    entries["grad_norm_finite"] = {
        "kind": "finite", "all_finite": bool(np.all(np.isfinite(gn))),
        "min": float(gn.min()), "max": float(gn.max()),
        "pass": bool(np.all(np.isfinite(gn)) and np.all(gn >= 0)),
        "blocking": False,
    }

    report = cu.make_report(entries)
    cu.dump_report(report, args.out_report)

    # plot
    series = {
        "loss": {"pt": pt_log["loss"]},
        "lr": {"pt": pt_log["lr"], "optax(jax)": jax_ref["lr_curve"]},
        "grad_norm": {"pt": pt_log["grad_norm"]},
    }
    if args.out_plot:
        cu.plot_curves(series, args.out_plot, title="L2 train compare")

    print(cu.pretty_print_report(report))
    return 0 if report["overall_pass"] else 1


def run_orchestrate(args):
    out_dir = Path(args.out_report).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    jax_json = str(out_dir / "_jax_ref.json")
    pt_json = str(out_dir / "_pt_train.json")
    py = sys.executable
    script = str(Path(__file__).resolve())

    cmd_jax = [py, script, "--side", "jax_ref", "--subset_path", args.subset_path,
               "--jax_config", args.jax_config, "--jax_ckpt", args.jax_ckpt,
               "--num_steps", str(args.num_steps), "--batch_size", str(args.batch_size),
               "--lr", str(args.lr), "--warmup", str(args.warmup),
               "--decay_steps", str(args.decay_steps), "--out", jax_json]
    cmd_pt = [py, script, "--side", "pt_train", "--subset_path", args.subset_path,
              "--jax_config", args.jax_config, "--pt_ckpt", args.pt_ckpt,
              "--num_steps", str(args.num_steps), "--batch_size", str(args.batch_size),
              "--lr", str(args.lr), "--warmup", str(args.warmup),
              "--decay_steps", str(args.decay_steps), "--out", pt_json]

    print(f"[orchestrate] JAX ref: {' '.join(cmd_jax)}")
    subprocess.run(cmd_jax, check=True)
    print(f"[orchestrate] PT train: {' '.join(cmd_pt)}")
    subprocess.run(cmd_pt, check=True)

    args.jax = jax_json
    args.pt = pt_json
    return run_compare(args)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["jax_ref", "pt_train", "compare", "all"], default="all")
    ap.add_argument("--subset_path", default="tests_au/scripts/_data/libero_subset")
    ap.add_argument("--jax_config", default="pi05_libero")
    ap.add_argument("--jax_ckpt", default=None)
    ap.add_argument("--pt_ckpt", default=None)
    ap.add_argument("--num_steps", type=int, default=15)
    ap.add_argument("--batch_size", type=int, default=2)
    ap.add_argument("--lr", type=float, default=5e-5)
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--decay_steps", type=int, default=1_000_000)
    ap.add_argument("--out", default=None)
    ap.add_argument("--jax", default=None)
    ap.add_argument("--pt", default=None)
    ap.add_argument("--out_report", default="tests_au/scripts/_out/train_compare_report.json")
    ap.add_argument("--out_plot", default="tests_au/scripts/_out/curves.png")
    args = ap.parse_args()

    if args.side == "jax_ref":
        run_jax_ref(args)
    elif args.side == "pt_train":
        run_pt_train(args)
    elif args.side == "compare":
        sys.exit(run_compare(args))
    else:
        sys.exit(run_orchestrate(args))


if __name__ == "__main__":
    main()
