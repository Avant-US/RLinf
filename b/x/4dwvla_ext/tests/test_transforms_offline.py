#!/usr/bin/env python3
"""Offline test: verify the transform pipeline matches RoboTwin inference.

Run inside the GPU container with the 4dwvla venv activated.

    source /opt/venv/4dwvla/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/tests/test_transforms_offline.py \
        --ckpt-path /home/nvidia/ckpts/4wvlaFrkPlugCkp010420 \
        --schema-path /workspace/4WVLA/b/s/Frk/cfg/franka_plug.yaml
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path("/workspace/4WVLA/src")))

from lerobot.transforms.core import (
    NormalizeTransformFn, UnNormalizeTransformFn,
    ResizeImagesWithPadFn, RemapImageKeyTransformFn,
    PadStateAndActionTransformFn, ReorderStateActionTransform,
    compose,
)
from lerobot.policies.internvla_a1_5.transform_internvla_a1_5 import (
    InternVLAA15ChatProcessorTransformFn,
)
from lerobot.utils.constants import ACTION, OBS_IMAGES, OBS_STATE

PASS = 0
FAIL = 0

def check(name: str, condition: bool, detail: str = ""):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}: {detail}")

def test_state_normalization():
    print("\n=== T1.1: State Normalization ===")
    state_mean = np.array([-0.2406, 0.1457, 0.1872, -2.0600, -0.0553, 2.2011, 0.6998, 0.0337])
    state_std  = np.array([0.1206, 0.0805, 0.1464, 0.0854, 0.0429, 0.1285, 0.0968, 0.0324])
    state_stat = {OBS_STATE: {"mean": state_mean, "std": state_std}}
    norm_fn = NormalizeTransformFn(selected_keys=[OBS_STATE], norm_stats=state_stat)

    sample = {OBS_STATE: torch.from_numpy(state_mean).float()}
    result = norm_fn(sample)
    check("mean->zero", np.allclose(result[OBS_STATE].numpy(), 0.0, atol=1e-5))

    sample = {OBS_STATE: torch.from_numpy(state_mean + state_std).float()}
    result = norm_fn(sample)
    # NormalizeTransformFn uses eps=1e-6 in denominator: (x-mean)/(std+eps)
    # For small std (e.g. gripper std=0.0324), result = std/(std+1e-6) ≈ 0.999969
    expected = state_std / (state_std + 1e-6)
    check("mean+std->one", np.allclose(result[OBS_STATE].numpy(), expected, atol=1e-5))

    q4_raw = -2.06
    q4_norm = (q4_raw - state_mean[3]) / state_std[3]
    q4_bin = int(np.clip(np.round((q4_norm / 3.0 + 1) / 2 * 255), 0, 255))
    check("q4 tokenization bin~128", abs(q4_bin - 128) <= 2, f"bin={q4_bin}")

def test_action_unnormalization():
    print("\n=== T1.2: Action UnNormalization ===")
    action_mean = np.array([-0.2381, 0.1417, 0.1886, -2.0560, -0.0617, 2.2639, 0.7208, 0.5785])
    action_std  = np.array([0.1218, 0.0852, 0.1472, 0.0867, 0.0559, 0.1419, 0.1672, 0.4047])
    action_stat = {ACTION: {"mean": action_mean, "std": action_std}}
    unnorm_fn = UnNormalizeTransformFn(selected_keys=[ACTION], mode="mean_std", norm_stats=action_stat)

    zero_action = torch.zeros(1, 8)
    recovered = unnorm_fn({ACTION: zero_action})[ACTION].numpy()[0]
    check("zero->mean", np.allclose(recovered, action_mean, atol=1e-4))

    JOINT_LOWER = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
    JOINT_UPPER = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])
    arm = recovered[:7]
    check("arm within FR3 limits", np.all(arm >= JOINT_LOWER) and np.all(arm <= JOINT_UPPER))

def test_image_transforms():
    print("\n=== T1.3: Image Transforms ===")
    mapping = {"observation.images.global": "observation.images.image0",
               "observation.images.wrist": "observation.images.image1"}
    resize_fn = ResizeImagesWithPadFn(height=224, width=224, mapping=mapping)
    remap_fn = RemapImageKeyTransformFn(mapping=mapping)

    sample = {f"{OBS_IMAGES}.global": torch.rand(3, 480, 640),
              f"{OBS_IMAGES}.wrist": torch.rand(3, 480, 640),
              OBS_STATE: torch.zeros(8), ACTION: torch.zeros(50, 8), "task": "test"}

    result = resize_fn(sample)
    for k in [f"{OBS_IMAGES}.global", f"{OBS_IMAGES}.wrist"]:
        if k in result:
            check(f"resize {k}", result[k].shape == (3, 224, 224), f"shape={result[k].shape}")

    result = remap_fn(result)
    check("image0 exists", f"{OBS_IMAGES}.image0" in result)
    check("image1 exists", f"{OBS_IMAGES}.image1" in result)
    check("image2 exists (padded)", f"{OBS_IMAGES}.image2" in result)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt-path", type=str, default=None)
    parser.add_argument("--schema-path", type=str, default=None)
    args = parser.parse_args()

    test_state_normalization()
    test_action_unnormalization()
    test_image_transforms()

    print(f"\n=== Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)

if __name__ == "__main__":
    main()
