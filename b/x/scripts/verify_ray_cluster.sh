#!/bin/bash
# Host-side acceptance for the two-container Ray cluster of dmo_place_2.md S3.2.
#
# Run on the HOST (needs only docker + ss). Verifies, in order:
#   1. both containers are up, on the expected images
#   2. nothing holds the robot's FCI port (1337) -- the cluster must not grab
#      the arm; only train_async.py's env worker may, later
#   3. a raylet runs in each container, with the RIGHT RLINF_NODE_RANK baked
#      into its environment (Ray captures env at `ray start`; a wrong rank here
#      cannot be fixed without restarting that node's ray)
#   4. `ray status` from the head shows exactly 2 alive nodes
#   5. the python interpreters the training YAML points at actually exist
#   6. franky imports in the franky container; CUDA torch works in the GPU one
#   7. the gym id registers in BOTH containers (RLINF_EXT_MODULE path)
#   8. end-to-end: a Ray task pinned to EACH node reports back that node's
#      RLINF_NODE_RANK / hostname / python -- proves scheduling works on both
#      nodes and that each raylet captured the env we think it did
#   9. the ResNet10 weights are visible inside the GPU container
#
# Exit 0 only if every CHECK passes. Output style matches step8_checks.
set -uo pipefail

FRANKY_CONTAINER="${RLINF_FRANKA_CONTAINER:-rlinf-franky-5090}"
GPU_CONTAINER="${RLINF_GPU_CONTAINER:-rlinf-gpu-5090}"
FRANKY_IMAGE="rlinf/rlinf:agentic-rlinf0.4-franka"
GPU_IMAGE="rlinf/rlinf:agentic-rlinf0.4-maniskill_libero"
FRANKY_VENV_PY="/opt/venv/franky-0.19.0/bin/python"
SAC_YAML="/home/nvidia/bt/s/RLinf/b/x/configs/realworld_cube_place_sac.yaml"
RESNET_PT="/home/nvidia/ckpts/RLinf-ResNet10-pretrained/resnet10_pretrained.pt"

FAILED=0
check() { # check <name> <ok:0/1> [detail]
  local status="OK"
  [[ "$2" == "0" ]] || { status="FAIL"; FAILED=1; }
  if [[ -n "${3:-}" ]]; then
    echo "CHECK $1 ${status}  $3"
  else
    echo "CHECK $1 ${status}"
  fi
}

# --- 1. containers -----------------------------------------------------------
for pair in "${FRANKY_CONTAINER}:${FRANKY_IMAGE}" "${GPU_CONTAINER}:${GPU_IMAGE}"; do
  name="${pair%%:*}"; want_image="${pair#*:}"
  line="$(docker ps --filter "name=^${name}\$" --format '{{.Image}} {{.Status}}' 2>/dev/null)"
  if [[ -z "${line}" ]]; then
    check "container_${name}" 1 "not running (docker ps has no ${name})"
    continue
  fi
  got_image="${line%% *}"
  if [[ "${got_image}" == "${want_image}" ]]; then
    check "container_${name}" 0 "${line}"
  else
    check "container_${name}" 1 "wrong image: ${got_image} (want ${want_image})"
  fi
done

# --- 2. FCI free --------------------------------------------------------------
if ss -tn state established '( dport = :1337 or sport = :1337 )' 2>/dev/null | grep -q 1337; then
  check "fci_free" 1 "something holds :1337 -- cluster must not touch the arm yet"
else
  check "fci_free" 0
fi

# --- 3. raylet per container, with the right captured RLINF_NODE_RANK --------
raylet_rank() { # raylet_rank <container> -> prints rank or empty
  local c="$1" pid
  pid="$(docker exec "${c}" bash -c 'pgrep -x raylet | head -1' 2>/dev/null)"
  [[ -n "${pid}" ]] || return 1
  docker exec "${c}" bash -c \
    "tr '\0' '\n' < /proc/${pid}/environ | grep '^RLINF_NODE_RANK=' | cut -d= -f2" 2>/dev/null
}
declare -A RANK_OF=()
for pair in "${FRANKY_CONTAINER}:1" "${GPU_CONTAINER}:0"; do
  name="${pair%%:*}"; want="${pair#*:}"
  if ! docker ps --format '{{.Names}}' | grep -qx "${name}"; then
    check "raylet_${name}" 1 "container not running"
    continue
  fi
  rank="$(raylet_rank "${name}")"
  if [[ -z "${rank}" ]]; then
    check "raylet_${name}" 1 "no raylet process (ray start not done in this container?)"
  elif [[ "${rank}" == "${want}" ]]; then
    check "raylet_${name}" 0 "RLINF_NODE_RANK=${rank} captured at ray start"
  else
    check "raylet_${name}" 1 "RLINF_NODE_RANK=${rank}, want ${want} -- restart this node's ray with the right export BEFORE ray start"
  fi
  RANK_OF["${name}"]="${rank:-}"
done

# --- 4. ray status from the head ----------------------------------------------
if docker ps --format '{{.Names}}' | grep -qx "${GPU_CONTAINER}"; then
  status_out="$(docker exec "${GPU_CONTAINER}" bash -lc \
    'source b/x/configs/setup_before_ray_gpu_5090.sh >/dev/null 2>&1; ray status 2>&1' || true)"
  alive="$(grep -c 'Alive:' <<<"${status_out}" || true)"
  # `ray status` prints one "Alive:" header per live node group section; count
  # node lines instead: they look like " 1 node_<hex>" under Active.
  nodes="$(python3 - <<'PY' "${status_out}"
import re, sys
text = sys.argv[1]
active = re.search(r"Active:\n((?:\s+\d+\s+node_[0-9a-f]+\n?)*)", text)
print(len(re.findall(r"node_[0-9a-f]+", active.group(1))) if active else 0)
PY
)"
  if [[ "${nodes}" == "2" ]]; then
    check "ray_status_2_nodes" 0
  else
    check "ray_status_2_nodes" 1 "active nodes=${nodes}, want 2; ray status said: $(tail -5 <<<"${status_out}" | tr '\n' '|')"
  fi
else
  check "ray_status_2_nodes" 1 "GPU container not running"
fi

# --- 5. interpreters the YAML points at ---------------------------------------
if docker ps --format '{{.Names}}' | grep -qx "${FRANKY_CONTAINER}"; then
  if docker exec "${FRANKY_CONTAINER}" test -x "${FRANKY_VENV_PY}"; then
    check "interpreter_franky" 0 "${FRANKY_VENV_PY}"
  else
    check "interpreter_franky" 1 "${FRANKY_VENV_PY} not executable in ${FRANKY_CONTAINER}"
  fi
fi
if docker ps --format '{{.Names}}' | grep -qx "${GPU_CONTAINER}"; then
  gpu_py="$(docker exec "${GPU_CONTAINER}" bash -lc \
    'source b/x/configs/setup_before_ray_gpu_5090.sh >/dev/null 2>&1; which python' 2>/dev/null | tail -1)"
  yaml_py="$(grep -A3 'label: gpu' "${SAC_YAML}" 2>/dev/null | grep python_interpreter_path | awk '{print $2}')"
  if [[ -n "${gpu_py}" ]] && docker exec "${GPU_CONTAINER}" test -x "${gpu_py}"; then
    if [[ -n "${yaml_py}" && "${yaml_py}" != "${gpu_py}" ]]; then
      check "interpreter_gpu" 1 "YAML says ${yaml_py} but container has ${gpu_py} -- fix the EDIT ME in ${SAC_YAML}"
    else
      check "interpreter_gpu" 0 "${gpu_py}"
    fi
  else
    check "interpreter_gpu" 1 "no python on PATH in ${GPU_CONTAINER} after setup"
  fi
fi

# --- 6. backend sanity ---------------------------------------------------------
if docker ps --format '{{.Names}}' | grep -qx "${FRANKY_CONTAINER}"; then
  if docker exec "${FRANKY_CONTAINER}" bash -lc \
      'source b/x/configs/setup_before_ray_5090.sh >/dev/null 2>&1; python -c "import franky"' 2>/dev/null; then
    check "franky_import" 0
  else
    check "franky_import" 1 "import franky failed in ${FRANKY_CONTAINER}"
  fi
fi
if docker ps --format '{{.Names}}' | grep -qx "${GPU_CONTAINER}"; then
  cuda_line="$(docker exec "${GPU_CONTAINER}" bash -lc \
    'source b/x/configs/setup_before_ray_gpu_5090.sh >/dev/null 2>&1; python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"\")"' 2>/dev/null | tail -1)"
  if grep -q '^True ' <<<"${cuda_line}"; then
    check "gpu_cuda_torch" 0 "${cuda_line#True }"
  else
    check "gpu_cuda_torch" 1 "torch.cuda.is_available()=${cuda_line:-<no output>}"
  fi
fi

# --- 7. gym id registers in both ------------------------------------------------
for c in "${FRANKY_CONTAINER}" "${GPU_CONTAINER}"; do
  docker ps --format '{{.Names}}' | grep -qx "${c}" || continue
  setup="setup_before_ray_5090.sh"
  [[ "${c}" == "${GPU_CONTAINER}" ]] && setup="setup_before_ray_gpu_5090.sh"
  gid="$(docker exec "${c}" bash -lc \
    "source b/x/configs/${setup} >/dev/null 2>&1; python -c \"import franky_ext.tasks.register, gymnasium as gym; print(gym.spec('FrankyCubePlaceEnv-v1').id)\" 2>/dev/null | tail -1")"
  if [[ "${gid}" == "FrankyCubePlaceEnv-v1" ]]; then
    check "gym_id_${c}" 0
  else
    check "gym_id_${c}" 1 "got '${gid}' (RLINF_EXT_MODULE / PYTHONPATH broken?)"
  fi
done

# --- 8. end-to-end: pinned task on each node ------------------------------------
if docker ps --format '{{.Names}}' | grep -qx "${GPU_CONTAINER}"; then
  e2e="$(docker exec "${GPU_CONTAINER}" bash -lc 'source b/x/configs/setup_before_ray_gpu_5090.sh >/dev/null 2>&1; python - <<'"'"'PY'"'"' 2>&1 | tail -8
import os, socket, sys
import ray
from ray.util.scheduling_strategies import NodeAffinitySchedulingStrategy

ray.init(address="auto", logging_level="ERROR")

@ray.remote
def whoami():
    return {
        "rank": os.environ.get("RLINF_NODE_RANK"),
        "host": socket.gethostname(),
        "python": sys.executable,
        "skip_camera": os.environ.get("RLINF_SKIP_CAMERA"),
    }

nodes = [n for n in ray.nodes() if n["Alive"]]
print("ALIVE_NODES", len(nodes))
for n in nodes:
    info = ray.get(
        whoami.options(
            scheduling_strategy=NodeAffinitySchedulingStrategy(
                node_id=n["NodeID"], soft=False
            )
        ).remote()
    )
    print("NODE", n["NodeID"][:8], info)
PY' 2>/dev/null)"
  if grep -q 'ALIVE_NODES 2' <<<"${e2e}"; then
    ranks="$(grep -o "'rank': '[01]'" <<<"${e2e}" | sort -u | tr '\n' ' ')"
    if grep -q "'rank': '0'" <<<"${e2e}" && grep -q "'rank': '1'" <<<"${e2e}"; then
      check "e2e_pinned_tasks" 0 "ranks seen on the two nodes: ${ranks}"
    else
      check "e2e_pinned_tasks" 1 "nodes alive but ranks wrong/missing: ${e2e}"
    fi
  else
    check "e2e_pinned_tasks" 1 "ray.init/pinned task failed: $(tail -3 <<<"${e2e}" | tr '\n' '|')"
  fi
fi

# --- 9. weights visible in GPU container ----------------------------------------
if docker ps --format '{{.Names}}' | grep -qx "${GPU_CONTAINER}"; then
  if docker exec "${GPU_CONTAINER}" test -f "${RESNET_PT}"; then
    check "resnet10_weights" 0 "${RESNET_PT}"
  else
    check "resnet10_weights" 1 "${RESNET_PT} not visible in ${GPU_CONTAINER}"
  fi
fi

echo
if [[ "${FAILED}" == "0" ]]; then
  echo "RESULT verify_ray_cluster PASS"
  exit 0
else
  echo "RESULT verify_ray_cluster FAIL"
  exit 1
fi
