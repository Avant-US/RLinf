#!/usr/bin/env bash
# =============================================================================
# Vertex AI 多机容器入口：把 CLUSTER_SPEC 转换成一个 Ray 集群并启动 RLinf VLM SFT。
#
# 运行位置：每个 worker pool 副本（共 3 个）都会执行本脚本。
# 它本身放在 GCS 上，容器命令为：  bash /gcs/<BUCKET>/rlinf/bootstrap.sh
# （/gcs 是 Vertex AI 自动挂载的 Cloud Storage FUSE，见官方文档 code-requirements#fuse）
#
# 依赖的环境变量（在 vertex_sft_3node.yaml 的 containerSpec.env 中注入）：
#   GCS_BUCKET   形如 rlinf-sft-europe-west4（不带 gs:// 前缀）
#   EXP_NAME     实验名，输出目录用，例如 qwen2_5_vl_sft_3node
#   RAY_PORT     Ray head 端口，默认 6379
#   VENV_NAME    镜像内 venv 名，reason 镜像为 reason
#   SMOKE_MODE   1 时使用本地 tiny 数据快速验证；EOS token 重映射始终执行
# =============================================================================
set -euo pipefail
export PYTHONUNBUFFERED=1   # 让 print/日志实时进入 Cloud Logging

# ----------------------------- 0. 参数与默认值 -------------------------------
GCS_BUCKET="${GCS_BUCKET:?must set GCS_BUCKET}"
EXP_NAME="${EXP_NAME:-qwen2_5_vl_sft_3node}"
RAY_PORT="${RAY_PORT:-6379}"
VENV_NAME="${VENV_NAME:-reason}"
SMOKE_MODE="${SMOKE_MODE:-0}"
SMOKE_TRAIN_SAMPLES="${SMOKE_TRAIN_SAMPLES:-288}"
SMOKE_VAL_SAMPLES="${SMOKE_VAL_SAMPLES:-96}"

GCS_ROOT="/gcs/${GCS_BUCKET}/rlinf"     # 预存区（FUSE）：代码、模型、数据、产物
CODE_TAR="${GCS_ROOT}/code/RLinf.tar.gz"
LOCAL_REPO="/workspace/RLinf"           # 代码解压到本地（FUSE 不适合做 Python import）
MODEL_SRC_DIR="${GCS_ROOT}/models/Qwen2.5-VL-3B-Instruct"   # 原始模型目录（只读）
MODEL_DIR="${MODEL_SRC_DIR}"                                # 可在 smoke 模式改成临时目录
TRAIN_DIR="${GCS_ROOT}/data/Robo2VLM-1/train_data"
VAL_DIR="${GCS_ROOT}/data/Robo2VLM-1/test_data"
OUTPUT_DIR="${GCS_ROOT}/runs/${EXP_NAME}"               # checkpoint/tensorboard 直接落 GCS

log() { echo "[bootstrap][$(date +%H:%M:%S)] $*"; }

# ----------------------------- 1. 激活 venv ----------------------------------
# 官方 RLinf 镜像把 venv 放在 /opt/venv/<name>；reason 镜像只有一个 reason venv。
set +u
source "/opt/venv/${VENV_NAME}/bin/activate"
set -u
log "python = $(which python), $(python --version 2>&1)"

# ------------------------- 2. 解析 CLUSTER_SPEC ------------------------------
# 形如：
# {"cluster":{"workerpool0":["...workerpool0-..-0:2222"],
#             "workerpool1":["...-0:2222","...-1:2222"]},
#  "task":{"type":"workerpool0","index":0}, "environment":"cloud"}
log "CLUSTER_SPEC=${CLUSTER_SPEC:-<empty>}"
read -r NODE_RANK NUM_NODES HEAD_HOST < <(python - <<'PY'
import json, os
spec = json.loads(os.environ.get("CLUSTER_SPEC") or "{}")
cluster = spec.get("cluster", {})
task = spec.get("task", {"type": "workerpool0", "index": 0})
pool0 = cluster.get("workerpool0", ["localhost:2222"])
pool1 = cluster.get("workerpool1", [])
n0, n1 = len(pool0), len(pool1)
# 全局 node rank：workerpool0 永远是 0（primary/chief，副本数恒为 1），
# workerpool1 的第 i 个副本 => rank = n0 + i = i + 1。
ttype, tindex = task.get("type", "workerpool0"), int(task.get("index", 0))
rank = 0 if ttype == "workerpool0" else n0 + tindex
head_host = pool0[0].rsplit(":", 1)[0]      # 去掉 :2222，端口我们自己用 RAY_PORT
print(rank, n0 + n1, head_host)
PY
)
log "NODE_RANK=${NODE_RANK}  NUM_NODES=${NUM_NODES}  HEAD_HOST=${HEAD_HOST}  RAY_PORT=${RAY_PORT}"

# RLinf 多机靠这个变量识别节点序号；必须在 `ray start` 之前 export（Ray 启动时抓取环境）。
export RLINF_NODE_RANK="${NODE_RANK}"

# --------------------------- 3. 等待 GCS FUSE 挂载就绪 & 准备代码到本地 -------
log "Waiting for GCS FUSE mount to become ready..."
for i in $(seq 1 30); do
  if [ -f "${CODE_TAR}" ]; then
    log "GCS FUSE mount is ready. Code tarball found."
    break
  fi
  sleep 1
  if [ "${i}" -eq 30 ]; then
    log "ERROR: GCS FUSE mount failed or code tarball not found at ${CODE_TAR} after 30s."
    ls -la /gcs || true
    exit 1
  fi
done

# 官方镜像默认不安装 rlinf 本体（install.sh 用 --no-install-project），
# 因此需要在运行时提供仓库源码并加入 PYTHONPATH。
log "Staging code from ${CODE_TAR} -> ${LOCAL_REPO}"
mkdir -p "${LOCAL_REPO}"
tar -xzf "${CODE_TAR}" -C "${LOCAL_REPO}" --strip-components=1
export REPO_PATH="${LOCAL_REPO}"
export PYTHONPATH="${LOCAL_REPO}:${PYTHONPATH:-}"
cd "${LOCAL_REPO}"

# --------------------------- 4. 健康检查（快速失败）-------------------------
nvidia-smi -L || { log "nvidia-smi failed"; exit 1; }
ls "${MODEL_SRC_DIR}" >/dev/null || { log "model dir missing: ${MODEL_SRC_DIR}"; exit 1; }
ls "${TRAIN_DIR}" >/dev/null || { log "train dir missing: ${TRAIN_DIR}"; exit 1; }
ls "${VAL_DIR}"   >/dev/null || { log "val dir missing: ${VAL_DIR}"; exit 1; }
mkdir -p "${OUTPUT_DIR}"

CONFIG_DIR="${LOCAL_REPO}/b/gcp/demo3"
CONFIG_NAME="qwen2_5_vl_sft_vlm_3node"

# --------------------------- 4.1 Smoke 模式：临时数据/模型元数据 -------------
if [ "${SMOKE_MODE}" = "1" ]; then
  log "SMOKE_MODE=1: preparing tiny local train/eval parquet subsets."
  SMOKE_DATA_DIR="/workspace/smoke_data/Robo2VLM-1"
  SMOKE_TRAIN_DIR="${SMOKE_DATA_DIR}/train_data"
  SMOKE_VAL_DIR="${SMOKE_DATA_DIR}/test_data"
  mkdir -p "${SMOKE_TRAIN_DIR}" "${SMOKE_VAL_DIR}"
  python - "${TRAIN_DIR}" "${VAL_DIR}" "${SMOKE_TRAIN_DIR}" "${SMOKE_VAL_DIR}" "${SMOKE_TRAIN_SAMPLES}" "${SMOKE_VAL_SAMPLES}" <<'PY'
import glob
import os
import sys

import pandas as pd


def build_subset(src_dir: str, dst_dir: str, prefix: str, sample_count: int) -> None:
    files = sorted(glob.glob(os.path.join(src_dir, "*.parquet")))
    if not files:
        raise RuntimeError(f"No parquet files found in {src_dir}")

    frames = []
    remaining = sample_count
    for path in files:
        if remaining <= 0:
            break
        frame = pd.read_parquet(path)
        if len(frame) == 0:
            continue
        take = min(remaining, len(frame))
        frames.append(frame.head(take))
        remaining -= take

    if remaining > 0:
        raise RuntimeError(
            f"Requested {sample_count} samples from {src_dir}, "
            f"but only collected {sample_count - remaining}."
        )

    out_path = os.path.join(dst_dir, f"{prefix}-00000-of-00001.parquet")
    pd.concat(frames, ignore_index=True).to_parquet(out_path, index=False)
    print(f"[bootstrap-smoke] wrote {sample_count} rows -> {out_path}", flush=True)


_, train_src, val_src, train_dst, val_dst, train_n, val_n = sys.argv
build_subset(train_src, train_dst, "train", int(train_n))
build_subset(val_src, val_dst, "test", int(val_n))
PY
  TRAIN_DIR="${SMOKE_TRAIN_DIR}"
  VAL_DIR="${SMOKE_VAL_DIR}"
fi

# --------------------------- 4.2 EOS token 重映射 (Bug 6 修复，始终执行) --------
log "Preparing runtime model dir with eval-safe tokenizer EOS (Bug 6 fix)."
MODEL_RUNTIME_DIR="/workspace/model_runtime/Qwen2.5-VL-3B-Instruct-eval-safe-eos"
mkdir -p "${MODEL_RUNTIME_DIR}"
python - "${MODEL_SRC_DIR}" "${MODEL_RUNTIME_DIR}" <<'PY'
import json
import os
import shutil
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
dst.mkdir(parents=True, exist_ok=True)

for child in src.iterdir():
    target = dst / child.name
    if target.exists() or target.is_symlink():
        if target.is_dir() and not target.is_symlink():
            shutil.rmtree(target)
        else:
            target.unlink()
    os.symlink(child, target)

tokenizer_config = dst / "tokenizer_config.json"
if tokenizer_config.exists() or tokenizer_config.is_symlink():
    tokenizer_config.unlink()
with (src / "tokenizer_config.json").open("r", encoding="utf-8") as f:
    cfg = json.load(f)
cfg["eos_token"] = "<|video_pad|>"
with tokenizer_config.open("w", encoding="utf-8") as f:
    json.dump(cfg, f, ensure_ascii=False, indent=2)
    f.write("\n")

from transformers import AutoTokenizer

tokenizer = AutoTokenizer.from_pretrained(str(dst), trust_remote_code=True)
print(
    "[bootstrap] tokenizer "
    f"eos_token={tokenizer.eos_token!r} eos_token_id={tokenizer.eos_token_id!r} "
    f"pad_token={tokenizer.pad_token!r} pad_token_id={tokenizer.pad_token_id!r}",
    flush=True,
)
if tokenizer.eos_token != "<|video_pad|>" or tokenizer.eos_token_id != 151656:
    raise RuntimeError("Failed to remap tokenizer.eos_token to <|video_pad|>")
PY
MODEL_DIR="${MODEL_RUNTIME_DIR}"

# =============================== 5. 分支：head / worker =======================
if [ "${NODE_RANK}" -eq 0 ]; then
  # ---------------------------- HEAD（rank 0）-------------------------------
  log "Starting Ray HEAD on port ${RAY_PORT}"
  ray start --head --port="${RAY_PORT}" --dashboard-host=0.0.0.0 --disable-usage-stats

  log "Launching RLinf SFT entrypoint (will block until all ${NUM_NODES} nodes join Ray)"
  # train_vlm_sft.py 里 Cluster(cfg.cluster) 会 ray.init() 并阻塞等待 num_nodes 台节点就绪。
  python examples/sft/train_vlm_sft.py \
    --config-path "${CONFIG_DIR}" \
    --config-name "${CONFIG_NAME}" \
    runner.logger.log_path="${OUTPUT_DIR}" \
    actor.model.model_path="${MODEL_DIR}" \
    data.train_data_paths="${TRAIN_DIR}" \
    data.val_data_paths="${VAL_DIR}"

  log "Training finished; stopping Ray head."
  ray stop || true
  log "DONE (rank 0). Outputs at gs://${GCS_BUCKET}/rlinf/runs/${EXP_NAME}"

else
  # --------------------------- WORKER（rank > 0）----------------------------
  log "Waiting for Ray head ${HEAD_HOST}:${RAY_PORT} to become reachable..."
  check_port() {
    python3 -c "import socket; s = socket.socket(); s.settimeout(2); s.connect(('$1', int($2)))" >/dev/null 2>&1
  }
  for i in $(seq 1 600); do
    if check_port "${HEAD_HOST}" "${RAY_PORT}"; then
      log "Head reachable after ${i}s"; break
    fi
    sleep 1
    if [ "${i}" -eq 600 ]; then log "ERROR: head not reachable in 600s"; exit 1; fi
  done

  log "Joining Ray cluster at ${HEAD_HOST}:${RAY_PORT}"
  ray start --address="${HEAD_HOST}:${RAY_PORT}" --disable-usage-stats

  # 保活：head 在跑训练；只要 head 端口可达就继续等待，断开即随主节点一起结束。
  log "Worker joined; keeping alive until head exits."
  miss=0
  while true; do
    if check_port "${HEAD_HOST}" "${RAY_PORT}"; then
      miss=0
    else
      miss=$((miss + 1))
      if [ "${miss}" -ge 12 ]; then     # 连续 ~60s 不可达 => head 结束
        log "Head gone; worker exiting."; break
      fi
    fi
    sleep 5
  done
  ray stop || true
  log "DONE (rank ${NODE_RANK})."
fi
