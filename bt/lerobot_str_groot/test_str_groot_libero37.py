#!/usr/bin/env python3
"""
StrGroot 端到端验证脚本：
  1) 从 HuggingFaceVLA/libero 随机抽取 37 个 episode
  2) 分阶段验证 str_groot 适配代码的各个环节
  3) 最后执行 2 步实际训练，确认完整流水线跑通

用法：
  # 完整验证（含模型创建和训练，需要 GPU 和 ~8GB 下载 Qwen3-VL）
  python bt/str_groot_1/test_str_groot_libero37.py

  # 仅验证注册 / 配置 / 数据加载，不构建模型（快速，无需 GPU）
  python bt/str_groot_1/test_str_groot_libero37.py --stage config_only

  # 验证到 dataset 阶段
  python bt/str_groot_1/test_str_groot_libero37.py --stage data
"""

from __future__ import annotations

import argparse
import json
import logging
import random
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    force=True,
)
LOG = logging.getLogger(__name__)

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
SKIP = "\033[93m[SKIP]\033[0m"


def _banner(title: str) -> None:
    LOG.info("=" * 60)
    LOG.info("  %s", title)
    LOG.info("=" * 60)


def _ok(msg: str) -> None:
    LOG.info("%s %s", PASS, msg)


def _fail(msg: str) -> None:
    LOG.error("%s %s", FAIL, msg)


def _skip(msg: str) -> None:
    LOG.warning("%s %s", SKIP, msg)


# ── episode sampling ────────────────────────────────────────────────────
def sample_episodes(repo_id: str, root: str | None, n: int, seed: int):
    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata

    meta = LeRobotDatasetMetadata(repo_id=repo_id, root=root)
    total = int(meta.total_episodes)
    if n > total:
        raise ValueError(f"sample_size({n}) > total_episodes({total})")
    episodes = sorted(random.Random(seed).sample(range(total), n))
    return episodes, total


def save_episodes(path: Path, repo: str, total: int, n: int, seed: int, eps: list[int]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {"dataset_repo": repo, "total_episodes": total, "sample_size": n,
             "sample_seed": seed, "episodes": eps},
            ensure_ascii=False, indent=2,
        ),
        encoding="utf-8",
    )


# ── tests ────────────────────────────────────────────────────────────────
def test_config_and_registration() -> bool:
    """验证 StrGrootConfig 能正确注册到 lerobot。"""
    _banner("Test 1: Config & factory registration")
    try:
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.str_groot.configuration_str_groot import StrGrootConfig
        from lerobot.policies.factory import get_policy_class, make_policy_config

        cfg = StrGrootConfig()
        assert cfg.type == "str_groot", f"Expected type 'str_groot', got '{cfg.type}'"
        assert cfg.chunk_size == cfg.future_action_window_size + 1
        assert cfg.action_delta_indices == list(range(cfg.chunk_size))
        _ok("StrGrootConfig 创建成功，字段正确")

        known = PreTrainedConfig.get_known_choices()
        assert "str_groot" in known, "'str_groot' not in registered choices"
        _ok("'str_groot' 已注册到 PreTrainedConfig")

        cls = get_policy_class("str_groot")
        assert cls.__name__ == "StrGrootPolicy"
        _ok("get_policy_class('str_groot') -> StrGrootPolicy")

        cfg2 = make_policy_config("str_groot")
        assert cfg2.type == "str_groot"
        _ok("make_policy_config('str_groot') 正确返回 StrGrootConfig")

        return True
    except Exception:
        _fail("Config / registration 测试失败")
        traceback.print_exc()
        return False


def test_processor() -> bool:
    """验证 pre / post processor 创建。"""
    _banner("Test 2: Processor pipeline")
    try:
        from lerobot.policies.str_groot.configuration_str_groot import StrGrootConfig
        from lerobot.policies.str_groot.processor_str_groot import make_str_groot_pre_post_processors
        from lerobot.policies.factory import make_pre_post_processors

        cfg = StrGrootConfig()
        pre, post = make_str_groot_pre_post_processors(cfg, dataset_stats=None)
        assert len(pre.steps) > 0
        assert len(post.steps) > 0
        _ok(f"直接创建 processor: pre({len(pre.steps)} steps), post({len(post.steps)} steps)")

        pre2, post2 = make_pre_post_processors(policy_cfg=cfg, dataset_stats=None)
        assert len(pre2.steps) > 0
        _ok("通过 factory.make_pre_post_processors 创建 processor 成功")

        return True
    except Exception:
        _fail("Processor 测试失败")
        traceback.print_exc()
        return False


def test_dataset_loading(repo_id: str, root: str | None, episodes: list[int]) -> bool:
    """验证用抽样 episodes 创建 dataset 和 dataloader。"""
    _banner("Test 3: Dataset loading (37 episodes)")
    try:
        import torch
        from lerobot.configs.default import DatasetConfig
        from lerobot.configs.train import TrainPipelineConfig
        from lerobot.datasets.factory import make_dataset
        from lerobot.policies.str_groot.configuration_str_groot import StrGrootConfig

        policy_cfg = StrGrootConfig(push_to_hub=False, repo_id=None)
        dataset_cfg = DatasetConfig(
            repo_id=repo_id, root=root, episodes=episodes, streaming=False,
        )
        train_cfg = TrainPipelineConfig(
            dataset=dataset_cfg, policy=policy_cfg,
            output_dir=Path("/tmp/str_groot_test"),
            steps=1, batch_size=1,
        )
        dataset = make_dataset(train_cfg)
        LOG.info("  dataset.num_frames  = %d", dataset.num_frames)
        LOG.info("  dataset.num_episodes = %d", dataset.num_episodes)
        LOG.info("  features keys: %s", sorted(dataset.meta.features.keys())[:8])
        assert dataset.num_episodes > 0
        assert dataset.num_frames > 0

        sample = dataset[0]
        LOG.info("  sample keys: %s", sorted(sample.keys())[:10])
        assert "action" in sample
        _ok(f"Dataset 加载成功: {dataset.num_episodes} episodes, {dataset.num_frames} frames")

        dl = torch.utils.data.DataLoader(dataset, batch_size=2, shuffle=True, num_workers=0)
        batch = next(iter(dl))
        LOG.info("  batch keys: %s", sorted(batch.keys())[:10])
        action_shape = batch["action"].shape
        LOG.info("  batch['action'] shape: %s", action_shape)
        assert len(action_shape) >= 2
        _ok(f"DataLoader batch 获取成功, action shape = {tuple(action_shape)}")

        return True
    except Exception:
        _fail("Dataset 加载测试失败")
        traceback.print_exc()
        return False


def test_model_creation() -> bool:
    """验证 StrGrootPolicy 模型实例化（需要下载 Qwen3-VL）。"""
    _banner("Test 4: Model creation (需要 GPU + 网络)")
    try:
        import torch
        from lerobot.policies.str_groot.configuration_str_groot import StrGrootConfig
        from lerobot.policies.str_groot.modeling_str_groot import StrGrootPolicy
        from lerobot.configs.types import FeatureType, PolicyFeature

        cfg = StrGrootConfig(
            push_to_hub=False, repo_id=None,
            starvla_checkpoint=None,
        )
        cfg.input_features = {
            "observation.images.image_0": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 224, 224)),
        }
        cfg.output_features = {
            "action": PolicyFeature(type=FeatureType.ACTION, shape=(7,)),
        }

        LOG.info("  正在实例化 StrGrootPolicy（将下载 %s）...", cfg.base_vlm)
        t0 = time.time()
        policy = StrGrootPolicy(config=cfg)
        elapsed = time.time() - t0
        LOG.info("  模型创建用时: %.1f s", elapsed)

        n_params = sum(p.numel() for p in policy.parameters())
        n_trainable = sum(p.numel() for p in policy.parameters() if p.requires_grad)
        LOG.info("  总参数: %s, 可训练: %s", f"{n_params:,}", f"{n_trainable:,}")

        assert hasattr(policy, "_starvla_model")
        assert hasattr(policy._starvla_model, "qwen_vl_interface")
        assert hasattr(policy._starvla_model, "action_model")
        _ok(f"StrGrootPolicy 创建成功 ({n_params:,} params, {elapsed:.1f}s)")

        return True
    except Exception:
        _fail("Model creation 测试失败")
        traceback.print_exc()
        return False


def test_forward_pass() -> bool:
    """构造 fake batch 验证 forward 能跑通。"""
    _banner("Test 5: Forward pass (fake data)")
    try:
        import torch
        from lerobot.policies.str_groot.configuration_str_groot import StrGrootConfig
        from lerobot.policies.str_groot.modeling_str_groot import StrGrootPolicy
        from lerobot.configs.types import FeatureType, PolicyFeature

        cfg = StrGrootConfig(
            push_to_hub=False, repo_id=None,
            starvla_checkpoint=None,
        )
        cfg.input_features = {
            "observation.images.image_0": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 224, 224)),
        }
        cfg.output_features = {
            "action": PolicyFeature(type=FeatureType.ACTION, shape=(7,)),
        }
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        cfg.device = str(device)

        policy = StrGrootPolicy(config=cfg)
        policy.to(device)
        policy.train()

        B, C, H, W = 2, 3, 224, 224
        chunk = cfg.chunk_size
        fake_batch = {
            "observation.images.image_0": torch.rand(B, C, H, W, device=device),
            "action": torch.rand(B, chunk, cfg.action_dim, device=device) * 2 - 1,
            "task": ["pick up the red block"] * B,
        }
        LOG.info("  fake batch: images=%s, action=%s",
                 fake_batch["observation.images.image_0"].shape,
                 fake_batch["action"].shape)

        LOG.info("  Running forward ...")
        t0 = time.time()
        loss, out_dict = policy.forward(fake_batch)
        elapsed = time.time() - t0
        LOG.info("  loss = %.6f  (%.2f s)", loss.item(), elapsed)
        assert torch.isfinite(loss), f"loss is not finite: {loss}"
        _ok(f"Forward pass 成功, loss = {loss.item():.6f}")

        return True
    except Exception:
        _fail("Forward pass 测试失败")
        traceback.print_exc()
        return False


def test_predict_action() -> bool:
    """验证 predict_action_chunk。"""
    _banner("Test 6: Predict action (inference)")
    try:
        import torch
        from lerobot.policies.str_groot.configuration_str_groot import StrGrootConfig
        from lerobot.policies.str_groot.modeling_str_groot import StrGrootPolicy
        from lerobot.configs.types import FeatureType, PolicyFeature

        cfg = StrGrootConfig(
            push_to_hub=False, repo_id=None,
            starvla_checkpoint=None,
        )
        cfg.input_features = {
            "observation.images.image_0": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 224, 224)),
        }
        cfg.output_features = {
            "action": PolicyFeature(type=FeatureType.ACTION, shape=(7,)),
        }
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        cfg.device = str(device)

        policy = StrGrootPolicy(config=cfg)
        policy.to(device)
        policy.eval()

        B = 1
        fake_batch = {
            "observation.images.image_0": torch.rand(B, 3, 224, 224, device=device),
            "task": ["pick up the red block"],
        }

        LOG.info("  Running predict_action_chunk ...")
        t0 = time.time()
        actions = policy.predict_action_chunk(fake_batch)
        elapsed = time.time() - t0
        LOG.info("  predicted actions shape: %s  (%.2f s)", tuple(actions.shape), elapsed)
        assert actions.shape[0] == B
        assert actions.shape[2] == cfg.action_dim
        _ok(f"predict_action_chunk 成功, output shape = {tuple(actions.shape)}")

        policy.reset()
        action = policy.select_action(fake_batch)
        LOG.info("  select_action output shape: %s", tuple(action.shape))
        _ok(f"select_action 成功, output shape = {tuple(action.shape)}")

        return True
    except Exception:
        _fail("Predict action 测试失败")
        traceback.print_exc()
        return False


def test_e2e_train(repo_id: str, root: str | None, episodes: list[int]) -> bool:
    """端到端训练 2 步。"""
    _banner("Test 7: End-to-end training (2 steps, 37 episodes)")
    try:
        from lerobot.configs.default import DatasetConfig, WandBConfig
        from lerobot.configs.train import TrainPipelineConfig
        from lerobot.policies.str_groot.configuration_str_groot import StrGrootConfig
        from lerobot.scripts.lerobot_train import train

        output_dir = Path(f"/tmp/str_groot_e2e_{datetime.now():%Y%m%d_%H%M%S}")

        policy_cfg = StrGrootConfig(
            push_to_hub=False, repo_id=None,
            starvla_checkpoint=None,
            freeze_vlm=True,
        )

        dataset_cfg = DatasetConfig(
            repo_id=repo_id, root=root, episodes=episodes, streaming=False,
        )

        train_cfg = TrainPipelineConfig(
            dataset=dataset_cfg,
            policy=policy_cfg,
            output_dir=output_dir,
            job_name="str_groot_e2e_test",
            batch_size=1,
            steps=2,
            num_workers=0,
            eval_freq=0,
            log_freq=1,
            save_checkpoint=False,
            save_freq=999,
            wandb=WandBConfig(enable=False),
        )

        LOG.info("  output_dir = %s", output_dir)
        LOG.info("  正在启动训练 (2 steps, batch_size=1, freeze_vlm=True) ...")
        t0 = time.time()
        train(train_cfg)
        elapsed = time.time() - t0
        _ok(f"端到端训练 2 步完成 ({elapsed:.1f}s)")
        return True
    except Exception:
        _fail("端到端训练测试失败")
        traceback.print_exc()
        return False


# ── main ─────────────────────────────────────────────────────────────────
STAGES = {
    "config_only": ["config", "processor"],
    "data": ["config", "processor", "dataset"],
    "model": ["config", "processor", "dataset", "model"],
    "forward": ["config", "processor", "dataset", "model", "forward"],
    "predict": ["config", "processor", "dataset", "model", "forward", "predict"],
    "full": ["config", "processor", "dataset", "model", "forward", "predict", "e2e"],
}


def parse_args():
    p = argparse.ArgumentParser(description="StrGroot 分阶段验证脚本")
    p.add_argument("--stage", choices=list(STAGES.keys()), default="full",
                    help="验证到哪个阶段 (default: full)")
    p.add_argument("--dataset-repo", default="HuggingFaceVLA/libero")
    p.add_argument("--dataset-root", default=None)
    p.add_argument("--sample-size", type=int, default=37)
    p.add_argument("--sample-seed", type=int, default=20260314)
    p.add_argument("--episodes-file", default="bt/str_groot_1/test_episodes_37.json")
    return p.parse_args()


def main():
    args = parse_args()
    tests_to_run = STAGES[args.stage]

    LOG.info("")
    LOG.info("StrGroot 验证脚本  stage=%s  tests=%s", args.stage, tests_to_run)
    LOG.info("")

    results: dict[str, bool | None] = {}

    # ── 1. config ──
    if "config" in tests_to_run:
        results["config"] = test_config_and_registration()
    if not results.get("config", True):
        LOG.error("基础注册测试失败, 终止后续测试。")
        _summary(results)
        return

    # ── 2. processor ──
    if "processor" in tests_to_run:
        results["processor"] = test_processor()

    # ── 3. dataset ──
    episodes = None
    if "dataset" in tests_to_run:
        _banner("Sampling 37 episodes from libero")
        episodes, total = sample_episodes(
            args.dataset_repo, args.dataset_root, args.sample_size, args.sample_seed,
        )
        ep_path = Path(args.episodes_file)
        save_episodes(ep_path, args.dataset_repo, total, args.sample_size, args.sample_seed, episodes)
        LOG.info("  已抽样 %d/%d episodes (seed=%d) -> %s",
                 len(episodes), total, args.sample_seed, ep_path)
        LOG.info("  前 10: %s", episodes[:10])
        results["dataset"] = test_dataset_loading(args.dataset_repo, args.dataset_root, episodes)

    # ── 4. model ──
    if "model" in tests_to_run:
        results["model"] = test_model_creation()

    # ── 5. forward ──
    if "forward" in tests_to_run:
        results["forward"] = test_forward_pass()

    # ── 6. predict ──
    if "predict" in tests_to_run:
        results["predict"] = test_predict_action()

    # ── 7. e2e ──
    if "e2e" in tests_to_run:
        if episodes is None:
            episodes, _ = sample_episodes(
                args.dataset_repo, args.dataset_root, args.sample_size, args.sample_seed,
            )
        results["e2e"] = test_e2e_train(args.dataset_repo, args.dataset_root, episodes)

    _summary(results)


def _summary(results: dict[str, bool | None]):
    LOG.info("")
    _banner("Summary")
    all_pass = True
    for name, ok in results.items():
        if ok is True:
            LOG.info("  %s %s", PASS, name)
        elif ok is False:
            LOG.info("  %s %s", FAIL, name)
            all_pass = False
        else:
            LOG.info("  %s %s", SKIP, name)

    LOG.info("")
    if all_pass:
        LOG.info("%s 所有测试通过!", PASS)
    else:
        LOG.error("%s 存在失败的测试", FAIL)
        sys.exit(1)


if __name__ == "__main__":
    main()
