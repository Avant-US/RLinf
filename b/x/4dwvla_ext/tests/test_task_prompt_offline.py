#!/usr/bin/env python3
"""Offline test: verify task prompt and inference config consistency.

Validates that all eval-side task prompts match the training dataset's
tasks.parquet, and that key inference parameters align with training config.

Run on any machine with Python 3.10+ and pyarrow/pandas:

    python /path/to/4dwvla_ext/tests/test_task_prompt_offline.py

Or in the GPU container (4dwvla venv has pyarrow):

    source /opt/venv/4dwvla/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/tests/test_task_prompt_offline.py
"""
import json
import re
import sys
from pathlib import Path

PASS = 0
FAIL = 0

_ext_dir = Path(__file__).resolve().parent.parent
_rlinf_root = _ext_dir.parent.parent.parent  # RLmm/

TASKS_PARQUET = Path("/home/nvidia/bt/dt/plug_into_socket_lrb_4D_8sml/meta/tasks.parquet")
TRAIN_CONFIG = Path("/home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/train_config.json")

CLIENT_PY = _ext_dir / "franka_vla_client.py"
SERVER_PY = _ext_dir / "vla_inference_server.py"
TEST_IPC_PY = _ext_dir / "tests" / "test_ipc_offline.py"
EVAL_DOC = _rlinf_root / "b" / "d" / "frk1" / "4wvla_rlinf_eval_3A3.md"


def check(name: str, condition: bool, detail: str = ""):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}: {detail}")


def get_training_task() -> str | None:
    """Read task string from training dataset's tasks.parquet."""
    if not TASKS_PARQUET.exists():
        return None
    try:
        import pandas as pd
        df = pd.read_parquet(TASKS_PARQUET)
        # task string is the index (row label), not a column
        return str(df.index[0])
    except ImportError:
        try:
            import pyarrow.parquet as pq
            table = pq.read_table(TASKS_PARQUET)
            pdf = table.to_pandas()
            return str(pdf.index[0])
        except ImportError:
            return None


def test_task_prompt_sources():
    """T11.1: Verify task string in all source files matches training data."""
    print("\n=== T11.1: Task Prompt Source Consistency ===")

    training_task = get_training_task()
    if training_task is None:
        print("  [SKIP] tasks.parquet not accessible (missing file or pyarrow)")
        return

    check("training task is 'plug into socket'",
          training_task == "plug into socket",
          f"got '{training_task}'")

    if CLIENT_PY.exists():
        client_src = CLIENT_PY.read_text()
        task_matches = re.findall(r'--task\s+"([^"]+)"', client_src)
        for m in task_matches:
            check(f"franka_vla_client.py --task matches training",
                  m == training_task,
                  f"found '{m}', expected '{training_task}'")

    if TEST_IPC_PY.exists():
        test_src = TEST_IPC_PY.read_text()
        task_matches = re.findall(r'"task":\s*"([^"]+)"', test_src)
        real_tasks = [m for m in task_matches if m != "test task"]
        for m in real_tasks:
            check(f"test_ipc_offline.py task matches training",
                  m == training_task,
                  f"found '{m}', expected '{training_task}'")


def test_no_wrong_prompt_in_doc():
    """T11.2: Ensure document has no stale task prompt."""
    print("\n=== T11.2: Document Task Prompt Consistency ===")

    if not EVAL_DOC.exists():
        print("  [SKIP] eval document not found")
        return

    doc_text = EVAL_DOC.read_text()
    # Only flag the old prompt in executable contexts (--task arguments),
    # not in D9 analysis prose that intentionally documents the old value.
    wrong_in_task_arg = len(re.findall(
        r'--task\s+"plug the charger[^"]*"', doc_text))
    check("no '--task \"plug the charger...\"' in document commands",
          wrong_in_task_arg == 0,
          f"found {wrong_in_task_arg} occurrences in --task arguments")

    correct_count = doc_text.count('"plug into socket"')
    check("document uses correct task prompt (>= 5 occurrences)",
          correct_count >= 5,
          f"found {correct_count}")


def test_train_config_flags():
    """T11.3: Verify key training config flags match inference assumptions."""
    print("\n=== T11.3: Training Config Consistency ===")

    if not TRAIN_CONFIG.exists():
        print("  [SKIP] train_config.json not accessible")
        return

    with open(TRAIN_CONFIG) as f:
        cfg = json.load(f)

    policy = cfg.get("policy", cfg)

    check("tokenize_state == True",
          policy.get("tokenize_state") is True,
          f"got {policy.get('tokenize_state')}")

    check("enable_keypoint_predictor == True",
          policy.get("enable_keypoint_predictor") is True,
          f"got {policy.get('enable_keypoint_predictor')}")

    check("block_action_attend_fast_tokens == True",
          policy.get("block_action_attend_fast_tokens") is True,
          f"got {policy.get('block_action_attend_fast_tokens')}")

    action_mode = None
    for dt in cfg.get("dataset", {}).get("data_transforms", {}).get("inputs", []):
        if dt.get("type") == "internvla_a1_5_chat_processor":
            action_mode = dt.get("action_mode")
            break
    if action_mode is not None:
        check("action_mode == 'joint'",
              action_mode == "joint",
              f"got '{action_mode}'")

    use_fast = None
    for dt in cfg.get("dataset", {}).get("data_transforms", {}).get("inputs", []):
        if dt.get("type") == "internvla_a1_5_chat_processor":
            use_fast = dt.get("use_fast_action_tokens")
            break
    if use_fast is not None:
        check("use_fast_action_tokens == True",
              use_fast is True,
              f"got {use_fast}")


def test_server_eval_mode():
    """T11.4: Verify inference server uses mode='eval'."""
    print("\n=== T11.4: Server Eval Mode ===")

    if not SERVER_PY.exists():
        print("  [SKIP] vla_inference_server.py not found")
        return

    src = SERVER_PY.read_text()
    check("server uses mode='eval'",
          'mode="eval"' in src or "mode='eval'" in src,
          "mode='eval' not found in server code")

    check("server uses tokenize_state from config",
          "tokenize_state" in src,
          "tokenize_state not referenced in server code")

    check("server handles keypoint (fk_computer)",
          "fk_computer" in src or "FKKeypointComputer" in src,
          "no FK keypoint handling in server")


def main():
    test_task_prompt_sources()
    test_no_wrong_prompt_in_doc()
    test_train_config_flags()
    test_server_eval_mode()
    print(f"\n=== T11 Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)


if __name__ == "__main__":
    main()
