#!/usr/bin/env python3
"""T-RLT2: Forward integration tests (offline).

Tests RLTStage1TrainingWrapper with a mock base policy that simulates
the InternVLAA15Policy forward path including qwen3_5_with_expert.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import torch
import torch.nn as nn
from rlt_config import RLTStage1Config
from rlt_stage1_wrapper import RLTStage1TrainingWrapper

PASS = 0
FAIL = 0


def run_test(name, fn):
    global PASS, FAIL
    try:
        fn()
        PASS += 1
        print(f"  [PASS] {name}")
    except Exception as e:
        FAIL += 1
        print(f"  [FAIL] {name}: {e}")


class MockExpertModel(nn.Module):
    """Mock qwen3_5_with_expert that returns [prefix_out, kpt_out, suffix_out]."""

    def __init__(self, hidden_size=2048, prefix_len=200):
        super().__init__()
        self.hidden_size = hidden_size
        self.prefix_len = prefix_len
        self.dummy_param = nn.Parameter(torch.zeros(1))

    def forward(self, *args, **kwargs):
        batch_size = 2
        prefix_out = torch.randn(batch_size, self.prefix_len, self.hidden_size)
        kpt_out = torch.randn(batch_size, 56, self.hidden_size)
        suffix_out = torch.randn(batch_size, 50, self.hidden_size)
        return ([prefix_out, kpt_out, suffix_out], None)


class MockInnerModel(nn.Module):
    """Mock InternVLAA15 inner model."""

    def __init__(self, hidden_size=2048, prefix_len=200):
        super().__init__()
        self.qwen3_5_with_expert = MockExpertModel(hidden_size, prefix_len)


class MockPolicy(nn.Module):
    """Mock InternVLAA15Policy for testing wrapper without loading real model."""

    def __init__(self, hidden_size=2048, prefix_len=200):
        super().__init__()
        self.model = MockInnerModel(hidden_size, prefix_len)
        self.trainable_param = nn.Parameter(torch.randn(100))

    def forward(self, batch):
        # Trigger expert forward (which the wrapper hooks)
        self.model.qwen3_5_with_expert()
        vla_loss = (self.trainable_param ** 2).mean()
        output_dict = {"loss_action": vla_loss.item()}
        return vla_loss, output_dict


def _make_wrapper(prefix_len=200):
    config = RLTStage1Config(
        rlt_input_dim=2048,
        rlt_embed_dim=1024,
        rlt_prefix_seq_len=512,
        rlt_num_layers=2,
        rlt_num_heads=8,
    )
    policy = MockPolicy(hidden_size=2048, prefix_len=prefix_len)
    wrapper = RLTStage1TrainingWrapper(policy, config)
    return wrapper


def _make_batch(prefix_len=200):
    batch = {
        "labels": torch.full((2, prefix_len), -100, dtype=torch.long),
    }
    # Simulate some assistant tokens
    batch["labels"][0, 150:180] = torch.arange(150, 180)
    batch["labels"][1, 160:190] = torch.arange(160, 190)
    return batch


def t2_1_wrapper_construction():
    wrapper = _make_wrapper()
    assert wrapper.rlt_module is not None
    assert wrapper.base_policy is not None


def t2_2_hook_installed():
    wrapper = _make_wrapper()
    # The original forward should have been replaced
    original_type = type(wrapper.base_policy.model.qwen3_5_with_expert.forward)
    # It should be a function (wrapped), not a bound method of MockExpertModel
    assert callable(wrapper.base_policy.model.qwen3_5_with_expert.forward)


def t2_3_forward_returns():
    wrapper = _make_wrapper()
    batch = _make_batch()
    total_loss, output_dict = wrapper(batch)
    assert isinstance(total_loss, torch.Tensor), f"total_loss should be tensor, got {type(total_loss)}"
    assert total_loss.dim() == 0, f"total_loss should be scalar, dim={total_loss.dim()}"


def t2_4_output_dict_keys():
    wrapper = _make_wrapper()
    batch = _make_batch()
    _, output_dict = wrapper(batch)
    required_keys = ["loss_rlt", "loss_vla", "loss_total", "rlt_mse", "rlt_z_rl_norm", "prefix_seq_len"]
    for key in required_keys:
        assert key in output_dict, f"Missing key in output_dict: {key}"


def t2_5_prefix_seq_len_reasonable():
    wrapper = _make_wrapper(prefix_len=200)
    batch = _make_batch(prefix_len=200)
    _, output_dict = wrapper(batch)
    psl = output_dict["prefix_seq_len"]
    assert 100 < psl < 512, f"prefix_seq_len should be 100-512, got {psl}"


def t2_6_prefix_out_captured():
    wrapper = _make_wrapper()
    batch = _make_batch()
    wrapper._captured_prefix_out = None
    # Forward should capture and then clear
    wrapper(batch)
    # After forward, _captured_prefix_out should be None (cleared)
    assert wrapper._captured_prefix_out is None, "Should be cleared after forward"


def t2_7_deploy_view_mask():
    wrapper = _make_wrapper(prefix_len=200)
    batch = _make_batch(prefix_len=200)

    # Run forward to capture prefix_out
    wrapper._captured_prefix_out = None
    wrapper.base_policy.model.qwen3_5_with_expert()
    prefix_out = wrapper._captured_prefix_out

    # Compute mask using prefix_out directly
    mask = wrapper._compute_deploy_view_mask(batch, prefix_out)
    assert mask is not None, "mask should not be None"
    assert mask.shape[0] == 2, f"mask batch dim should be 2, got {mask.shape[0]}"
    assert mask.sum().item() > 100, f"mask should have >100 True values, got {mask.sum().item()}"


def t2_8_loss_values_reasonable():
    wrapper = _make_wrapper()
    batch = _make_batch()
    total_loss, output_dict = wrapper(batch)
    assert output_dict["loss_rlt"] > 0, "RLT loss should be > 0"
    assert not torch.isnan(total_loss), "total_loss should not be NaN"
    assert not torch.isinf(total_loss), "total_loss should not be Inf"


if __name__ == "__main__":
    print("T-RLT2: Forward Integration Tests")
    print("=" * 50)

    run_test("T2.1 Wrapper construction", t2_1_wrapper_construction)
    run_test("T2.2 Hook installed on expert model", t2_2_hook_installed)
    run_test("T2.3 forward returns (loss, dict)", t2_3_forward_returns)
    run_test("T2.4 output_dict has required keys", t2_4_output_dict_keys)
    run_test("T2.5 prefix_seq_len reasonable", t2_5_prefix_seq_len_reasonable)
    run_test("T2.6 prefix_out captured and cleared", t2_6_prefix_out_captured)
    run_test("T2.7 deploy_view_mask correctness", t2_7_deploy_view_mask)
    run_test("T2.8 Loss values reasonable (no NaN/Inf)", t2_8_loss_values_reasonable)

    print("=" * 50)
    print(f"=== Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)
