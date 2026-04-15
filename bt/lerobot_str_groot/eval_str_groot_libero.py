#!/usr/bin/env python3
"""
Evaluate StrGroot (StarVLA Qwen-GR00T) on the LIBERO benchmark.

Reports success rates for LIBERO-Spatial, LIBERO-Object, LIBERO-Goal, LIBERO-Long
and prints the average score.

Usage:
  # Full benchmark (4 suites, 10 episodes per task, ~40 tasks total)
  python bt/str_groot_1/eval_str_groot_libero.py

  # Quick smoke test (2 episodes per task)
  python bt/str_groot_1/eval_str_groot_libero.py --n-episodes 2

  # Evaluate on specific suites only
  python bt/str_groot_1/eval_str_groot_libero.py --suites libero_goal libero_10

  # Dry-run: verify env & model creation without running rollouts
  python bt/str_groot_1/eval_str_groot_libero.py --dry-run

Prerequisites:
  pip install -e ".[libero]"
  export MUJOCO_GL=egl   # for headless servers
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from contextlib import nullcontext
from datetime import datetime
from pathlib import Path

import numpy as np
import torch

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    force=True,
)
LOGGER = logging.getLogger(__name__)

ALL_SUITES = ["libero_spatial", "libero_object", "libero_goal", "libero_10"]

SUITE_DISPLAY = {
    "libero_spatial": "LIBERO-Spatial",
    "libero_object": "LIBERO-Object",
    "libero_goal": "LIBERO-Goal",
    "libero_10": "LIBERO-Long",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Evaluate StrGroot on the LIBERO benchmark",
    )
    p.add_argument(
        "--starvla-checkpoint",
        default="StarVLA/Qwen3VL-GR00T-Bridge-RT-1",
        help="StarVLA checkpoint (HF repo id or local path)",
    )
    p.add_argument(
        "--dataset-repo",
        default="HuggingFaceVLA/libero",
        help="Dataset repo for loading normalization stats",
    )
    p.add_argument("--state-dim", type=int, default=7)
    p.add_argument(
        "--state-indices",
        type=int,
        nargs="*",
        default=[0, 1, 2, 3, 4, 5, 7],
        help="State dimension indices to select (default: skip pad at index 6)",
    )
    p.add_argument(
        "--suites",
        nargs="*",
        default=None,
        help="Suites to evaluate (default: all 4)",
    )
    p.add_argument("--n-episodes", type=int, default=10, help="Episodes per task")
    p.add_argument("--batch-size", type=int, default=1, help="Parallel envs per task")
    p.add_argument("--seed", type=int, default=1000)
    p.add_argument("--device", default=None)
    p.add_argument("--control-mode", default="relative", choices=["relative", "absolute"])
    p.add_argument("--output-dir", default="outputs/bt/str_groot_1/eval")
    p.add_argument("--dry-run", action="store_true", help="Only build env+model, skip rollouts")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    suites = args.suites or ALL_SUITES
    device = args.device or ("cuda" if torch.cuda.is_available() else "cpu")
    device_type = device.split(":")[0]

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_dir) / run_id
    output_dir.mkdir(parents=True, exist_ok=True)

    # --- Imports (delayed so --help is fast) ---
    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
    from lerobot.envs.configs import LiberoEnv
    from lerobot.envs.factory import make_env, make_env_pre_post_processors
    from lerobot.envs.utils import close_envs
    from lerobot.policies.factory import make_policy, make_pre_post_processors
    from lerobot.policies.str_groot.configuration_str_groot import StrGrootConfig
    from lerobot.scripts.lerobot_eval import eval_policy_all

    # ------------------------------------------------------------------
    # 1. Environment
    # ------------------------------------------------------------------
    LOGGER.info("Suites to evaluate: %s", suites)
    env_cfg = LiberoEnv(
        task=",".join(suites),
        obs_type="pixels_agent_pos",
        render_mode="rgb_array",
        control_mode=args.control_mode,
    )

    LOGGER.info("Creating LIBERO environments (batch_size=%d) ...", args.batch_size)
    envs = make_env(env_cfg, n_envs=args.batch_size)
    LOGGER.info("Environments created: %s", {k: list(v.keys()) for k, v in envs.items()})

    # ------------------------------------------------------------------
    # 2. Policy
    # ------------------------------------------------------------------
    state_indices = tuple(args.state_indices) if args.state_indices else None
    policy_cfg = StrGrootConfig(
        push_to_hub=False,
        repo_id=None,
        starvla_checkpoint=args.starvla_checkpoint,
        state_dim=args.state_dim,
        state_indices=state_indices,
        use_bf16=True,
        device=device,
    )

    LOGGER.info("Creating StrGroot policy (checkpoint=%s) ...", args.starvla_checkpoint)
    policy = make_policy(cfg=policy_cfg, env_cfg=env_cfg)
    policy.eval()
    n_params = sum(p.numel() for p in policy.parameters())
    LOGGER.info("Policy ready: %s params", f"{n_params:,}")

    # ------------------------------------------------------------------
    # 3. Processors
    # ------------------------------------------------------------------
    LOGGER.info("Loading dataset stats from %s ...", args.dataset_repo)
    ds_meta = LeRobotDatasetMetadata(repo_id=args.dataset_repo)
    dataset_stats = ds_meta.stats

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_cfg,
        dataset_stats=dataset_stats,
    )
    env_preprocessor, env_postprocessor = make_env_pre_post_processors(
        env_cfg=env_cfg,
        policy_cfg=policy_cfg,
    )

    # ------------------------------------------------------------------
    # 4. Dry-run gate
    # ------------------------------------------------------------------
    if args.dry_run:
        LOGGER.info("dry-run 模式：env + model 创建成功，跳过 rollout。")
        close_envs(envs)
        return

    # ------------------------------------------------------------------
    # 5. Evaluate
    # ------------------------------------------------------------------
    LOGGER.info(
        "Starting evaluation: %d episodes/task, seed=%d",
        args.n_episodes,
        args.seed,
    )
    t0 = time.time()

    amp_ctx = (
        torch.autocast(device_type=device_type, dtype=torch.bfloat16)
        if policy_cfg.use_bf16 and device_type == "cuda"
        else nullcontext()
    )
    with torch.no_grad(), amp_ctx:
        info = eval_policy_all(
            envs=envs,
            policy=policy,
            env_preprocessor=env_preprocessor,
            env_postprocessor=env_postprocessor,
            preprocessor=preprocessor,
            postprocessor=postprocessor,
            n_episodes=args.n_episodes,
            start_seed=args.seed,
            max_parallel_tasks=1,
        )

    close_envs(envs)
    elapsed = time.time() - t0
    LOGGER.info("Evaluation finished in %.1fs", elapsed)

    # ------------------------------------------------------------------
    # 6. Results
    # ------------------------------------------------------------------
    print_results(info, suites)

    results_path = output_dir / "eval_results.json"
    with open(results_path, "w") as f:
        json.dump(info, f, indent=2, default=str)
    LOGGER.info("Full results saved to: %s", results_path)


# ======================================================================
# Pretty-print
# ======================================================================
def print_results(info: dict, suites: list[str]) -> None:
    per_group = info.get("per_group", {})

    print("\n" + "=" * 56)
    print("  LIBERO Benchmark — StrGroot Evaluation Results")
    print("=" * 56)

    scores: dict[str, float] = {}
    for suite_key in suites:
        display = SUITE_DISPLAY.get(suite_key, suite_key)
        if suite_key in per_group:
            sc = per_group[suite_key]["pc_success"]
            n = per_group[suite_key].get("n_episodes", "?")
            scores[display] = sc
            print(f"  {display:<20s}  {sc:6.1f}%  ({n} episodes)")
        else:
            print(f"  {display:<20s}  {'N/A':>6s}")

    if scores:
        avg = float(np.mean(list(scores.values())))
        print("-" * 56)
        print(f"  {'Average':<20s}  {avg:6.1f}%")
    print("=" * 56)

    overall = info.get("overall", {})
    if "eval_s" in overall:
        print(f"  Total time : {overall['eval_s']:.1f}s")
        print(f"  Per episode: {overall.get('eval_ep_s', 0):.1f}s")
    print()


if __name__ == "__main__":
    main()
