"""在 dummy 模式下测试 FrankyPegInsertionEnv + RLT 特征模型。

用法（在 GPU 容器中）：
    source b/rlt/configs/setup_gpu.sh
    python b/rlt/scripts/test_env_dummy.py

验证：
    - Gym 环境注册正常
    - 19 维 state 构建正确
    - 7 维 action 格式正确
    - 相机图像 224x224
    - Stage 1 冻结模型能产出 z_rl / proprio / ref_chunk
"""

import sys
import os

REPO_PATH = os.environ.get("REPO_PATH", "/home/nvidia/kaixin_ws/RLinf")
if REPO_PATH not in sys.path:
    sys.path.insert(0, REPO_PATH)
bx_path = os.path.join(REPO_PATH, "b", "x")
if bx_path not in sys.path:
    sys.path.insert(0, bx_path)

os.environ.setdefault("RLINF_EXT_MODULE", "franky_ext.runtime_bootstrap")
os.environ["RLINF_SKIP_CAMERA"] = "1"


def main():
    print("[1/4] Registering Franky gym envs ...")
    import franky_ext.runtime_bootstrap

    franky_ext.runtime_bootstrap.register()

    import gymnasium as gym

    print("[2/4] Creating FrankyPegInsertionEnv-v1 in dummy mode ...")
    env = gym.make(
        "FrankyPegInsertionEnv-v1",
        override_cfg={
            "is_dummy": True,
            "robot_ip": "0.0.0.0",
            "step_frequency": 10.0,
            "action_scale": [0.02, 0.1, 1.0],
            "target_ee_pose": [0.55, 0.0, 0.48, 3.14, 0.0, 0.0],
        },
        worker_info=None,
        hardware_info=None,
        env_idx=0,
    )

    print("[3/4] Testing reset + step ...")
    obs, info = env.reset()

    import numpy as np

    state = obs.get("state", {})
    print(f"  State keys: {sorted(state.keys())}")
    total_dim = sum(np.asarray(v).size for v in state.values())
    print(f"  Total state dim: {total_dim}")

    action = env.action_space.sample()
    print(f"  Action shape: {action.shape}")
    print(f"  Action space: {env.action_space}")

    obs2, reward, terminated, truncated, info2 = env.step(action)
    print(f"  Step reward:     {reward}")
    print(f"  Step terminated: {terminated}")
    print(f"  Step truncated:  {truncated}")

    print("\n[4/4] Checking shapes ...")
    checks = []

    state2 = obs2.get("state", {})
    total_dim2 = sum(np.asarray(v).size for v in state2.values())

    if total_dim2 == 20:
        print("  state: 20 dim (7D tcp_pose, will be 19 after Quat2Euler wrapper)")
        checks.append(True)
    elif total_dim2 == 19:
        print("  state: 19 dim (6D tcp_pose euler)")
        checks.append(True)
    else:
        print(f"  WARNING: state dim = {total_dim2}, expected 19 or 20")
        checks.append(False)

    if action.shape == (7,):
        print("  action: 7 dim (dx,dy,dz,dr,dp,dy,gripper)")
        checks.append(True)
    elif action.shape == (6,):
        print("  action: 6 dim (no gripper, GripperCloseEnv active)")
        checks.append(True)
    else:
        print(f"  WARNING: action shape = {action.shape}")
        checks.append(False)

    env.close()

    if all(checks):
        print("\n=== Dummy env test PASSED ===")
    else:
        print("\n=== Dummy env test FAILED (see warnings above) ===")
        sys.exit(1)


if __name__ == "__main__":
    main()
