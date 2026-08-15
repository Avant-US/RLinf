#!/bin/bash
# One-time (per container session) deps for Step 7 dummy SAC in franky venv.
set -euo pipefail

INSTALL_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${INSTALL_SCRIPT_DIR}/../configs/setup_before_ray_5090.sh"

if python - <<'PY'
import franky_ext.runtime_bootstrap  # noqa: F401
import peft, transformers, timm
from rlinf.workers.reward.reward_worker import EmbodiedRewardWorker  # noqa: F401
print("step7 deps ok")
PY
then
  echo "Step7 deps already present in venv"
  exit 0
fi

echo "Installing Step7 embodied deps into franky venv..."
pip install -q "torch==2.5.1+cpu" "torchvision==0.20.1+cpu" \
  --index-url https://download.pytorch.org/whl/cpu --force-reinstall
pip install -q peft transformers accelerate timm hydra-core omegaconf einops filelock

python - <<'PY'
import franky_ext.runtime_bootstrap  # noqa: F401
import peft, transformers, timm
from rlinf.workers.reward.reward_worker import EmbodiedRewardWorker
print("step7 deps installed OK")
PY
