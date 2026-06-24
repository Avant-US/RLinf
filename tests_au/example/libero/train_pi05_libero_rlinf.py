"""RLinf-style pi0.5 LIBERO fine-tuning using OpenPI's JAX training infrastructure.

Replicates the openpi JAX training pipeline (flow matching loss, FSDP, EMA, etc.)
from within RLinf's example directory. All configs match the reference openpi
implementation at /home/physical/SRC/Robot/openpi05/b/tst/libero/.

Usage:
    /mnt/r/VENV/openpi_venv/bin/python train_pi05_libero_rlinf.py [overrides...]

Overrides (tyro CLI):
    --batch-size=128
    --num-train-steps=1000
    --save-interval=200
    etc.
"""

import sys
from pathlib import Path
from unittest.mock import patch

OPENPI_ROOT = Path("/home/physical/SRC/Robot/openpi05")
sys.path.insert(0, str(OPENPI_ROOT / "src"))
sys.path.insert(0, str(OPENPI_ROOT / "scripts"))

import openpi.training.config as _config
import openpi.training.data_loader as _data_loader
import openpi.transforms as transforms


def get_local_episode_count(root: Path) -> int:
    """Count locally available parquet episode files."""
    count = 0
    data_dir = root / "data"
    if data_dir.exists():
        for chunk_dir in sorted(data_dir.iterdir()):
            if chunk_dir.is_dir() and chunk_dir.name.startswith("chunk-"):
                count += sum(1 for f in chunk_dir.iterdir() if f.suffix == ".parquet")
    return count


def make_patched_create_torch_dataset(available_episodes: int):
    """Create a patched dataset loader that only uses locally cached episodes."""

    def patched_create_torch_dataset(data_config, action_horizon, model_config):
        from lerobot.common.datasets import lerobot_dataset

        repo_id = data_config.repo_id
        if repo_id is None:
            raise ValueError("Repo ID is not set.")
        if repo_id == "fake":
            return _data_loader.FakeDataset(model_config, num_samples=1024)

        dataset_meta = lerobot_dataset.LeRobotDatasetMetadata(repo_id)
        dataset = lerobot_dataset.LeRobotDataset(
            repo_id,
            episodes=list(range(available_episodes)),
            delta_timestamps={
                key: [t / dataset_meta.fps for t in range(action_horizon)]
                for key in data_config.action_sequence_keys
            },
        )

        if data_config.prompt_from_task:
            dataset = _data_loader.TransformedDataset(
                dataset, [transforms.PromptFromLeRobotTask(dataset_meta.tasks)]
            )
        return dataset

    return patched_create_torch_dataset


def main():
    from lerobot.common.constants import HF_LEROBOT_HOME

    config = _config.cli()

    data_config = config.data.create(config.assets_dirs, config.model)
    local_root = HF_LEROBOT_HOME / data_config.repo_id
    available_episodes = get_local_episode_count(local_root)
    print(f"[RLinf pi05_libero] Found {available_episodes} locally cached episodes at {local_root}")

    if available_episodes == 0:
        print("ERROR: No local LIBERO data found.")
        print("Please download the dataset first:")
        print("  python -c \"from lerobot.common.datasets.lerobot_dataset import LeRobotDataset; "
              "LeRobotDataset('physical-intelligence/libero')\"")
        sys.exit(1)

    patched_fn = make_patched_create_torch_dataset(available_episodes)

    with patch.object(_data_loader, "create_torch_dataset", patched_fn):
        from train import main as train_main
        train_main(config)


if __name__ == "__main__":
    main()
