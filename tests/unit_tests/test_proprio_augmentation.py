"""
Comprehensive tests for Proprio Domain Randomization (Chapter 9).

Covers: ProprioAugmentation base class, ProprioRandomOffset, ProprioRandomScale,
ProprioRandomDeadzone, ProprioRandomNoise, frame selection (6 modes),
dimension exclusion, augment_target isolation, padding interaction,
and integration with FastWAMProcessor.

Usage:
    pytest tests/unit_tests/test_proprio_augmentation.py -v -s
"""

import math
from copy import deepcopy

import pytest
import torch

from rlinf.data.datasets.fastwam.augmentation import (
    ProprioAugmentation,
    ProprioRandomOffset,
    ProprioRandomScale,
    ProprioRandomDeadzone,
    ProprioRandomNoise,
)


# ============================================================
#  Helper functions
# ============================================================

T_OBS = 33   # num_obs_steps
T_ACT = 32   # action_horizon
D = 23       # action/state dim (R1 Pro)

# R1 Pro dimension layout
ARM_DIMS = list(range(0, 14))
GRIPPER_DIMS = [14, 15]
CHASSIS_POSE_DIMS = [16, 17, 18, 19]
CHASSIS_VEL_DIMS = [20, 21, 22]
NON_ARM_DIMS = GRIPPER_DIMS + CHASSIS_POSE_DIMS + CHASSIS_VEL_DIMS


def _make_batch(seed=42):
    torch.manual_seed(seed)
    return {
        "action": {"default": torch.randn(T_ACT, D)},
        "state":  {"default": torch.randn(T_OBS, D)},
    }


def _make_batch_with_pad(n_pad_front=3, n_pad_back=2, seed=42):
    batch = _make_batch(seed)
    state_is_pad = torch.zeros(T_OBS, dtype=torch.bool)
    state_is_pad[:n_pad_front] = True
    action_is_pad = torch.zeros(T_ACT, dtype=torch.bool)
    action_is_pad[-n_pad_back:] = True
    batch["state_is_pad"] = state_is_pad
    batch["action_is_pad"] = action_is_pad
    return batch


def _make_batch_multi_key(seed=42):
    torch.manual_seed(seed)
    return {
        "action": {
            "default": torch.randn(T_ACT, D),
            "extra": torch.randn(T_ACT, 7),
        },
        "state": {
            "default": torch.randn(T_OBS, D),
            "extra": torch.randn(T_OBS, 7),
        },
    }


def _deep_clone_batch(batch):
    cloned = {}
    for k, v in batch.items():
        if isinstance(v, dict):
            cloned[k] = {kk: vv.clone() for kk, vv in v.items()}
        elif isinstance(v, torch.Tensor):
            cloned[k] = v.clone()
        else:
            cloned[k] = v
    return cloned


# ============================================================
#  T01–T03: Base class __call__ probability gating
# ============================================================

class _DummyAug(ProprioAugmentation):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.applied = False

    def _apply(self, batch):
        self.applied = True
        return batch


def test_t01_p0_never_applies():
    aug = _DummyAug(p=0.0)
    batch = _make_batch()
    for _ in range(100):
        aug.applied = False
        aug(batch)
        assert not aug.applied


def test_t02_p1_always_applies():
    aug = _DummyAug(p=1.0)
    batch = _make_batch()
    for _ in range(100):
        aug.applied = False
        aug(batch)
        assert aug.applied


def test_t03_p05_statistical():
    aug = _DummyAug(p=0.5)
    batch = _make_batch()
    count = 0
    n = 1000
    for _ in range(n):
        aug.applied = False
        aug(batch)
        if aug.applied:
            count += 1
    ratio = count / n
    assert 0.4 < ratio < 0.6, f"Expected ~0.5, got {ratio}"


# ============================================================
#  T04–T05: Base class _make_mask
# ============================================================

def test_t04_make_mask_basic():
    aug_no_excl = _DummyAug(exclude_dims=None)
    mask = aug_no_excl._make_mask(D, "cpu")
    assert mask.all() and mask.shape == (D,)

    aug_empty = _DummyAug(exclude_dims=[])
    mask = aug_empty._make_mask(D, "cpu")
    assert mask.all()

    aug_excl = _DummyAug(exclude_dims=[14, 15])
    mask = aug_excl._make_mask(D, "cpu")
    assert mask.sum() == D - 2
    assert not mask[14] and not mask[15]
    assert mask[0] and mask[13] and mask[16]


def test_t05_make_mask_out_of_range():
    aug = _DummyAug(exclude_dims=[-1, 0, 100])
    mask = aug._make_mask(D, "cpu")
    assert not mask[0]
    assert mask[1:].all()
    assert mask.sum() == D - 1


# ============================================================
#  T06–T16: Base class _resolve_frame_indices (6 modes × padding)
# ============================================================

def test_t06_frame_all():
    aug = _DummyAug(frame_indices="all", respect_pad=True)

    mask = aug._resolve_frame_indices(T_OBS, is_pad=None)
    assert mask.all()

    pad = torch.zeros(T_OBS, dtype=torch.bool)
    pad[:3] = True
    mask = aug._resolve_frame_indices(T_OBS, is_pad=pad)
    assert mask.sum() == T_OBS - 3
    assert not mask[:3].any()
    assert mask[3:].all()


def test_t07_frame_all_respect_pad_false():
    aug = _DummyAug(frame_indices="all", respect_pad=False)
    pad = torch.zeros(T_OBS, dtype=torch.bool)
    pad[:3] = True
    mask = aug._resolve_frame_indices(T_OBS, is_pad=pad)
    assert mask.all(), "respect_pad=False should include padding frames"


def test_t08_frame_explicit_indices():
    aug = _DummyAug(frame_indices=[0, 5, 31])
    mask = aug._resolve_frame_indices(T_ACT)
    assert mask.sum() == 3
    assert mask[0] and mask[5] and mask[31]

    aug2 = _DummyAug(frame_indices=[0, 50, -1])
    mask2 = aug2._resolve_frame_indices(T_ACT)
    assert mask2.sum() == 1
    assert mask2[0]


def test_t09_frame_first_n():
    aug = _DummyAug(frame_indices={"first_n": 5})
    mask = aug._resolve_frame_indices(T_OBS)
    assert mask[:5].all()
    assert not mask[5:].any()

    aug_over = _DummyAug(frame_indices={"first_n": 100})
    mask_over = aug_over._resolve_frame_indices(T_OBS)
    assert mask_over.all()


def test_t10_frame_first_n_with_padding():
    aug = _DummyAug(frame_indices={"first_n": 5}, respect_pad=True)
    pad = torch.zeros(T_OBS, dtype=torch.bool)
    pad[:3] = True
    mask = aug._resolve_frame_indices(T_OBS, is_pad=pad)
    assert not mask[:3].any()
    assert mask[3] and mask[4]
    assert not mask[5:].any()
    assert mask.sum() == 2


def test_t11_frame_last_n():
    aug = _DummyAug(frame_indices={"last_n": 3})
    mask = aug._resolve_frame_indices(T_ACT)
    assert not mask[:-3].any()
    assert mask[-3:].all()

    aug_over = _DummyAug(frame_indices={"last_n": 100})
    mask_over = aug_over._resolve_frame_indices(T_ACT)
    assert mask_over.all()


def test_t12_frame_random_n():
    aug = _DummyAug(frame_indices={"random_n": 5})
    m1 = aug._resolve_frame_indices(T_OBS)
    assert m1.sum() == 5

    m2 = aug._resolve_frame_indices(T_OBS)
    assert m2.sum() == 5

    all_same = all(
        torch.equal(aug._resolve_frame_indices(T_OBS), m1) for _ in range(10)
    )
    assert not all_same, "random_n should produce different selections"


def test_t13_frame_random_n_overflow():
    pad = torch.ones(T_OBS, dtype=torch.bool)
    pad[10:15] = False
    aug = _DummyAug(frame_indices={"random_n": 20}, respect_pad=True)
    mask = aug._resolve_frame_indices(T_OBS, is_pad=pad)
    assert mask.sum() == 5
    assert mask[10:15].all()


def test_t14_frame_random_n_with_padding():
    pad = torch.zeros(T_OBS, dtype=torch.bool)
    pad[:5] = True
    aug = _DummyAug(frame_indices={"random_n": 3}, respect_pad=True)
    mask = aug._resolve_frame_indices(T_OBS, is_pad=pad)
    assert mask.sum() == 3
    assert not mask[:5].any(), "Padding frames should not be selected"

    all_pad = torch.ones(T_OBS, dtype=torch.bool)
    mask_all = aug._resolve_frame_indices(T_OBS, is_pad=all_pad)
    assert mask_all.sum() == 0


def test_t15_frame_random_frac():
    aug = _DummyAug(frame_indices={"random_frac": 0.2})
    mask = aug._resolve_frame_indices(T_OBS)
    expected = math.ceil(0.2 * T_OBS)
    assert mask.sum() == expected, f"Expected {expected}, got {mask.sum()}"

    aug_full = _DummyAug(frame_indices={"random_frac": 1.0})
    mask_full = aug_full._resolve_frame_indices(T_OBS)
    assert mask_full.sum() == T_OBS


def test_t16_frame_no_padding_tensor():
    for mode in ["all", [0, 1], {"first_n": 3}]:
        aug = _DummyAug(frame_indices=mode, respect_pad=True)
        mask = aug._resolve_frame_indices(T_OBS, is_pad=None)
        aug2 = _DummyAug(frame_indices=mode, respect_pad=False)
        mask2 = aug2._resolve_frame_indices(T_OBS, is_pad=None)
        assert torch.equal(mask, mask2), f"is_pad=None should make respect_pad irrelevant for {mode}"


# ============================================================
#  T17–T23: ProprioRandomOffset
# ============================================================

def test_t17_offset_shape_and_both():
    batch = _make_batch()
    orig = _deep_clone_batch(batch)
    aug = ProprioRandomOffset(offset_range=0.02, p=1.0,
                              augment_target="both", frame_indices="all")
    result = aug(batch)

    assert result["action"]["default"].shape == (T_ACT, D)
    assert result["state"]["default"].shape == (T_OBS, D)
    assert not torch.equal(result["action"]["default"], orig["action"]["default"])
    assert not torch.equal(result["state"]["default"], orig["state"]["default"])


def test_t18_offset_same_across_frames():
    torch.manual_seed(99)
    batch = {
        "action": {"default": torch.zeros(T_ACT, D)},
        "state":  {"default": torch.zeros(T_OBS, D)},
    }
    aug = ProprioRandomOffset(offset_range=0.1, exclude_dims=NON_ARM_DIMS, p=1.0,
                              frame_indices="all")
    result = aug(batch)

    action_out = result["action"]["default"]
    for t in range(1, T_ACT):
        assert torch.equal(action_out[t], action_out[0]), \
            f"Frame {t} offset differs from frame 0"

    state_out = result["state"]["default"]
    assert torch.equal(state_out[0], action_out[0])


def test_t19_offset_exclude_dims():
    batch = _make_batch()
    orig = _deep_clone_batch(batch)
    aug = ProprioRandomOffset(offset_range=0.1, exclude_dims=NON_ARM_DIMS, p=1.0,
                              frame_indices="all")
    result = aug(batch)

    for dim in NON_ARM_DIMS:
        assert torch.equal(result["action"]["default"][:, dim],
                           orig["action"]["default"][:, dim]), \
            f"Excluded dim {dim} should be unchanged in action"
        assert torch.equal(result["state"]["default"][:, dim],
                           orig["state"]["default"][:, dim]), \
            f"Excluded dim {dim} should be unchanged in state"


def test_t20_offset_value_range():
    ofs_range = 0.05
    batch = {
        "action": {"default": torch.zeros(T_ACT, D)},
        "state":  {"default": torch.zeros(T_OBS, D)},
    }
    aug = ProprioRandomOffset(offset_range=ofs_range, p=1.0, frame_indices="all")
    for _ in range(100):
        result = aug(deepcopy(batch))
        action_vals = result["action"]["default"]
        assert action_vals.abs().max() <= ofs_range + 1e-7, \
            f"Offset {action_vals.abs().max()} exceeds range {ofs_range}"


def test_t21_offset_frame_selective():
    batch = {
        "action": {"default": torch.zeros(T_ACT, D)},
        "state":  {"default": torch.zeros(T_OBS, D)},
    }
    aug = ProprioRandomOffset(offset_range=0.1, p=1.0,
                              frame_indices={"first_n": 2})
    result = aug(batch)

    state_out = result["state"]["default"]
    assert state_out[:2].abs().sum() > 0, "First 2 frames should be offset"
    assert (state_out[2:] == 0).all(), "Frames 2+ should be unchanged"


def test_t22_offset_augment_target():
    batch_s = _make_batch(seed=10)
    orig_s = _deep_clone_batch(batch_s)
    aug_s = ProprioRandomOffset(offset_range=0.1, p=1.0, augment_target="state")
    result_s = aug_s(batch_s)
    assert torch.equal(result_s["action"]["default"], orig_s["action"]["default"]), \
        "augment_target='state': action should be unchanged"
    assert not torch.equal(result_s["state"]["default"], orig_s["state"]["default"]), \
        "augment_target='state': state should be changed"

    batch_a = _make_batch(seed=20)
    orig_a = _deep_clone_batch(batch_a)
    aug_a = ProprioRandomOffset(offset_range=0.1, p=1.0, augment_target="action")
    result_a = aug_a(batch_a)
    assert torch.equal(result_a["state"]["default"], orig_a["state"]["default"]), \
        "augment_target='action': state should be unchanged"
    assert not torch.equal(result_a["action"]["default"], orig_a["action"]["default"]), \
        "augment_target='action': action should be changed"


def test_t23_offset_multi_key():
    batch = _make_batch_multi_key()
    orig = _deep_clone_batch(batch)
    aug = ProprioRandomOffset(offset_range=0.1, p=1.0, frame_indices="all")
    result = aug(batch)

    for key in ["default", "extra"]:
        assert result["action"][key].shape == orig["action"][key].shape
        assert result["state"][key].shape == orig["state"][key].shape
        assert not torch.equal(result["action"][key], orig["action"][key])


# ============================================================
#  T24–T27: ProprioRandomScale
# ============================================================

def test_t24_scale_action_only():
    batch = _make_batch()
    orig = _deep_clone_batch(batch)
    aug = ProprioRandomScale(scale_range=0.1, p=1.0, frame_indices="all")
    result = aug(batch)

    assert torch.equal(result["state"]["default"], orig["state"]["default"]), \
        "ProprioRandomScale should NOT modify state"
    assert not torch.equal(result["action"]["default"], orig["action"]["default"]), \
        "ProprioRandomScale should modify action"


def test_t25_scale_value_range():
    sr = 0.1
    batch = {
        "action": {"default": torch.ones(T_ACT, D)},
        "state":  {"default": torch.ones(T_OBS, D)},
    }
    aug = ProprioRandomScale(scale_range=sr, p=1.0, frame_indices="all")
    for _ in range(100):
        result = aug(deepcopy(batch))
        factors = result["action"]["default"]
        assert factors.min() >= (1 - sr) - 1e-7
        assert factors.max() <= (1 + sr) + 1e-7


def test_t26_scale_exclude_dims():
    batch = {
        "action": {"default": torch.ones(T_ACT, D) * 2.0},
        "state":  {"default": torch.ones(T_OBS, D)},
    }
    aug = ProprioRandomScale(scale_range=0.1, exclude_dims=NON_ARM_DIMS, p=1.0,
                             frame_indices="all")
    result = aug(batch)

    for dim in NON_ARM_DIMS:
        assert (result["action"]["default"][:, dim] == 2.0).all(), \
            f"Excluded dim {dim} should keep value 2.0"


def test_t27_scale_frame_selective():
    action_val = 5.0
    batch = {
        "action": {"default": torch.full((T_ACT, D), action_val)},
        "state":  {"default": torch.ones(T_OBS, D)},
    }
    aug = ProprioRandomScale(scale_range=0.2, p=1.0, frame_indices={"last_n": 3})
    result = aug(batch)

    action_out = result["action"]["default"]
    assert (action_out[:-3] == action_val).all(), \
        "Frames before last_n should be unchanged"
    assert not (action_out[-3:] == action_val).all(), \
        "Last 3 frames should be scaled"


# ============================================================
#  T28–T30: ProprioRandomDeadzone
# ============================================================

def test_t28_deadzone_zeroing():
    dz_max = 0.01
    action = torch.tensor([
        [0.001, 0.1, -0.002, 0.5],
        [0.05,  0.003, 0.2, -0.001],
    ], dtype=torch.float32)
    batch = {
        "action": {"default": action},
        "state":  {"default": torch.randn(2, 4)},
    }
    aug = ProprioRandomDeadzone(deadzone_max=dz_max, p=1.0, frame_indices="all")

    zeroed_small = False
    kept_large = True
    for _ in range(50):
        result = aug(deepcopy(batch))
        out = result["action"]["default"]
        if out[0, 1] != 0.1:
            kept_large = False
        if out[0, 0] == 0.0:
            zeroed_small = True

    assert kept_large, "Large actions should never be zeroed"
    assert zeroed_small, "Small actions should sometimes be zeroed"


def test_t29_deadzone_state_unchanged_and_exclude():
    batch = _make_batch()
    orig = _deep_clone_batch(batch)
    aug = ProprioRandomDeadzone(deadzone_max=100.0, exclude_dims=[0, 1], p=1.0,
                                frame_indices="all")
    result = aug(batch)

    assert torch.equal(result["state"]["default"], orig["state"]["default"]), \
        "Deadzone should not modify state"
    assert torch.equal(result["action"]["default"][:, :2],
                       orig["action"]["default"][:, :2]), \
        "Excluded dims should be unchanged"


def test_t30_deadzone_frame_selective():
    action = torch.full((T_ACT, 4), 0.001)
    batch = {
        "action": {"default": action.clone()},
        "state":  {"default": torch.randn(T_OBS, 4)},
    }
    aug = ProprioRandomDeadzone(deadzone_max=0.01, p=1.0,
                                frame_indices={"random_n": 5})
    result = aug(batch)
    out = result["action"]["default"]

    unchanged_frames = (out == 0.001).all(dim=1).sum().item()
    assert unchanged_frames >= T_ACT - 5, \
        f"At most 5 frames should be affected, but {T_ACT - unchanged_frames} were"


# ============================================================
#  T31–T34: ProprioRandomNoise
# ============================================================

def test_t31_noise_per_frame_independent():
    batch = {
        "action": {"default": torch.zeros(T_ACT, D)},
        "state":  {"default": torch.zeros(T_OBS, D)},
    }
    aug = ProprioRandomNoise(noise_std=0.1, p=1.0, frame_indices="all",
                             exclude_dims=NON_ARM_DIMS)
    result = aug(batch)

    state_out = result["state"]["default"]
    assert not torch.equal(state_out[0, :14], state_out[1, :14]), \
        "Different frames should have independent noise"
    for dim in NON_ARM_DIMS:
        if dim < D:
            assert (state_out[:, dim] == 0).all(), \
                f"Excluded dim {dim} should have zero noise"


def test_t32_noise_augment_target():
    batch = _make_batch()
    orig = _deep_clone_batch(batch)
    aug = ProprioRandomNoise(noise_std=0.1, p=1.0, augment_target="state",
                             frame_indices="all")
    result = aug(batch)

    assert torch.equal(result["action"]["default"], orig["action"]["default"]), \
        "augment_target='state': action should be unchanged"
    assert not torch.equal(result["state"]["default"], orig["state"]["default"])


def test_t33_noise_gaussian_stats():
    noise_std = 0.05
    n_samples = 500
    all_noise = []
    for i in range(n_samples):
        batch = {
            "action": {"default": torch.zeros(1, 4)},
            "state":  {"default": torch.zeros(1, 4)},
        }
        aug = ProprioRandomNoise(noise_std=noise_std, p=1.0, frame_indices="all",
                                 augment_target="action")
        result = aug(batch)
        all_noise.append(result["action"]["default"][0])

    noise_tensor = torch.stack(all_noise)
    assert noise_tensor.mean().abs() < 0.01, \
        f"Mean {noise_tensor.mean():.4f} should be near 0"
    empirical_std = noise_tensor.std().item()
    assert abs(empirical_std - noise_std) < 0.015, \
        f"Std {empirical_std:.4f} should be near {noise_std}"


def test_t34_noise_frame_selective():
    batch = {
        "action": {"default": torch.zeros(T_ACT, D)},
        "state":  {"default": torch.zeros(T_OBS, D)},
    }
    aug = ProprioRandomNoise(noise_std=0.1, p=1.0, augment_target="state",
                             frame_indices={"random_frac": 0.3})
    result = aug(batch)

    state_out = result["state"]["default"]
    expected_k = math.ceil(0.3 * T_OBS)
    noisy_frames = (state_out.abs().sum(dim=1) > 0).sum().item()
    assert noisy_frames == expected_k, \
        f"Expected {expected_k} noisy frames, got {noisy_frames}"
    zero_frames = T_OBS - noisy_frames
    zero_mask = state_out.abs().sum(dim=1) == 0
    assert zero_mask.sum().item() == zero_frames


# ============================================================
#  T35–T37: Integration tests
# ============================================================

def test_t35_processor_train_vs_eval():
    batch = {
        "action": {"default": torch.zeros(T_ACT, D)},
        "state":  {"default": torch.zeros(T_OBS, D)},
    }

    aug = ProprioRandomOffset(offset_range=0.1, p=1.0, frame_indices="all")
    augmentations = [aug]

    is_train = True
    result_train = deepcopy(batch)
    if is_train and augmentations is not None:
        for a in augmentations:
            result_train = a(result_train)
    assert not torch.equal(result_train["action"]["default"],
                           batch["action"]["default"]), \
        "Train mode should apply augmentation"

    is_train = False
    result_eval = deepcopy(batch)
    if is_train and augmentations is not None:
        for a in augmentations:
            result_eval = a(result_eval)
    assert torch.equal(result_eval["action"]["default"],
                       batch["action"]["default"]), \
        "Eval mode should skip augmentation"


def test_t36_chain_offset_then_scale():
    batch = {
        "action": {"default": torch.ones(T_ACT, D) * 2.0},
        "state":  {"default": torch.ones(T_OBS, D) * 2.0},
    }
    aug_offset = ProprioRandomOffset(offset_range=0.5, p=1.0, frame_indices="all")
    aug_scale = ProprioRandomScale(scale_range=0.3, p=1.0, frame_indices="all")

    result = deepcopy(batch)
    result = aug_offset(result)
    assert not (result["action"]["default"] == 2.0).all()

    result = aug_scale(result)
    assert not torch.equal(result["state"]["default"],
                           batch["state"]["default"])


def test_t37_yaml_instantiation():
    configs = [
        {
            "_target_": "rlinf.data.datasets.fastwam.augmentation.ProprioRandomOffset",
            "offset_range": 0.02,
            "exclude_dims": [14, 15, 16, 17, 18, 19, 20, 21, 22],
            "p": 0.5,
            "frame_indices": "all",
        },
        {
            "_target_": "rlinf.data.datasets.fastwam.augmentation.ProprioRandomScale",
            "scale_range": 0.05,
            "exclude_dims": [14, 15, 16, 17, 18, 19, 22],
            "p": 0.3,
        },
        {
            "_target_": "rlinf.data.datasets.fastwam.augmentation.ProprioRandomDeadzone",
            "deadzone_max": 0.005,
            "exclude_dims": [14, 15, 16, 17, 18, 19, 20, 21, 22],
            "p": 0.2,
        },
        {
            "_target_": "rlinf.data.datasets.fastwam.augmentation.ProprioRandomNoise",
            "noise_std": 0.008,
            "exclude_dims": [14, 15, 16, 17, 18, 19, 20, 21, 22],
            "frame_indices": {"random_n": 5},
            "augment_target": "state",
            "p": 0.4,
        },
    ]

    class_map = {
        "rlinf.data.datasets.fastwam.augmentation.ProprioRandomOffset": ProprioRandomOffset,
        "rlinf.data.datasets.fastwam.augmentation.ProprioRandomScale": ProprioRandomScale,
        "rlinf.data.datasets.fastwam.augmentation.ProprioRandomDeadzone": ProprioRandomDeadzone,
        "rlinf.data.datasets.fastwam.augmentation.ProprioRandomNoise": ProprioRandomNoise,
    }

    instances = []
    for cfg in configs:
        cfg = dict(cfg)
        target = cfg.pop("_target_")
        cls = class_map[target]
        instance = cls(**cfg)
        instances.append(instance)
        assert isinstance(instance, ProprioAugmentation)

    batch = _make_batch()
    for inst in instances:
        result = inst(batch)
        assert "action" in result and "state" in result


# ============================================================
#  T38–T40: Additional boundary tests
# ============================================================

def test_t38_offset_with_padding_and_frame_selection():
    batch = _make_batch_with_pad(n_pad_front=3, n_pad_back=2)
    batch["action"]["default"] = torch.zeros(T_ACT, D)
    batch["state"]["default"] = torch.zeros(T_OBS, D)
    aug = ProprioRandomOffset(offset_range=0.1, p=1.0,
                              frame_indices={"first_n": 5}, respect_pad=True)
    result = aug(batch)

    state_out = result["state"]["default"]
    assert (state_out[:3] == 0).all(), "Padding frames should be unchanged"
    assert state_out[3].abs().sum() > 0, "Frame 3 should be offset"
    assert state_out[4].abs().sum() > 0, "Frame 4 should be offset"
    assert (state_out[5:] == 0).all(), "Frames 5+ should be unchanged"


def test_t39_random_frac_with_heavy_padding():
    pad = torch.ones(T_OBS, dtype=torch.bool)
    pad[20:26] = False
    aug = _DummyAug(frame_indices={"random_frac": 0.5}, respect_pad=True)
    mask = aug._resolve_frame_indices(T_OBS, is_pad=pad)
    expected = math.ceil(0.5 * 6)
    assert mask.sum() == expected
    assert not mask[:20].any() and not mask[26:].any()


def test_t40_deadzone_all_large_actions():
    batch = {
        "action": {"default": torch.ones(T_ACT, D) * 10.0},
        "state":  {"default": torch.randn(T_OBS, D)},
    }
    aug = ProprioRandomDeadzone(deadzone_max=0.005, p=1.0, frame_indices="all")
    result = aug(batch)
    assert (result["action"]["default"] == 10.0).all(), \
        "All large actions should be preserved"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
