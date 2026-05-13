#! /bin/bash

set -eo pipefail

TARGET=""

MODEL=""
ENV_NAME=""
VENV_DIR=".venv"
PYTHON_VERSION="3.11.14"
TEST_BUILD=${TEST_BUILD:-0}
# Absolute path to this script (resolves symlinks)
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
# Immutable copies of the script's own location.  Some sourced setup
# scripts (notably Galaxea SDK's mobiman.sh) reassign a bare ``SCRIPT_DIR``
# in the current shell, which would otherwise corrupt our SCRIPT_DIR.
# Use these *_RAW variables anywhere we need the install.sh location after
# sourcing third-party setup scripts.
readonly RLINF_INSTALL_SCRIPT_PATH="$SCRIPT_PATH"
readonly RLINF_INSTALL_SCRIPT_DIR="$SCRIPT_DIR"
USE_MIRRORS=0
GITHUB_PREFIX=""
NO_ROOT=0
NO_INSTALL_RLINF_CMD="--no-install-project"
SUPPORTED_TARGETS=("embodied" "agentic" "docs")
SUPPORTED_MODELS=("openvla" "openvla-oft" "openpi" "gr00t" "dexbotic" "starvla" "lingbotvla" "dreamzero")
SUPPORTED_ENVS=("behavior" "maniskill_libero" "metaworld" "calvin" "isaaclab" "robocasa" "franka" "frankasim" "robotwin" "habitat" "opensora" "wan" "xsquare_turtle2" "liberopro" "liberoplus" "roboverse" "galaxea_r1_pro" "galaxea_r1_pro_orin")

#=======================Utility Functions=======================

print_help() {
        cat <<EOF
Usage: bash install.sh <target> [options]

Targets:
    embodied               Install embodied model and envs (default).
    agentic                Install agentic stack (Megatron etc.).
    docs                   Install documentation requirements.

Options (for target=embodied):
    --model <name>         Embodied model to install: ${SUPPORTED_MODELS[*]}.
    --env <name>           Single environment to install: ${SUPPORTED_ENVS[*]}.

Common options:
    -h, --help             Show this help message and exit.
    --venv <dir>           Virtual environment directory name (default: .venv).
    --use-mirror           Use mirrors for faster downloads.
    --no-root              Avoid system dependency installation for non-root users. Only use this if you are certain system dependencies are already installed.
    --install-rlinf        Install RLinf itself into the python.

Notes:
    --env galaxea_r1_pro       Standard server-side install with full embodied
                               stack (sapien/robosuite/...).  Use on x86_64
                               with a GPU server in the loop.
    --env galaxea_r1_pro_orin  Single-node Jetson Orin (real R1 Pro) V3 install.
                               Purpose-built for:
                                 * RLinf R1 Pro CLI tools
                                   (toolkits/realworld_check/test_galaxea_*.py)
                                 * RLinf local single-process M1 joint-mode SAC
                                   on Orin (no Ray cluster, no FSDP, no torch.distributed)
                               Minimal layout:
                                 * System/user Python pool — NVIDIA Jetson CUDA
                                   PyTorch only.  Reuse an existing working torch
                                   when present; otherwise install the selected
                                   NVIDIA wheel.
                                 * Venv ($VENV_DIR) — RLinf editable plus the
                                   small runtime closure needed by CLI + local
                                   SAC: ray core import surface, gymnasium,
                                   hydra/omegaconf, numpy/scipy, pyyaml,
                                   filelock/cloudpickle/msgpack/attrs/einops.
                                   No simulators, no VLA deps, no flash-attn/apex.
                               The venv uses --system-site-packages so it can see
                               Jetson torch and, after activation, ROS 2 / Galaxea
                               SDK Python bindings.
                               PyTorch wheel via RLINF_ORIN_JP_PYTORCH:
                                 v60  (default) torch 2.4.0a0 nv24.05 — cuDNN 8,
                                                NO cuSPARSELt needed.
                                                Best fit for stock JetPack 6.0 GA.
                                 v60-nv24.06    torch 2.4.0a0 + cuSPARSELt
                                 v60-nv24.07    torch 2.4.0a0 + cuSPARSELt
                                 v61            torch 2.5.0a0 nv24.08 — cuDNN 9
                                                + cuSPARSELt (JetPack 6.1 image
                                                or manually-upgraded cuDNN 9 only)
                               Env overrides:
                                 RLINF_ORIN_PYTORCH_WHEEL_URL=...   custom .whl
                                 RLINF_ORIN_CUSPARSELT=pypi|apt     cusparselt path
                                                                    (default pypi,
                                                                    LD_LIBRARY_PATH)
                                 RLINF_ORIN_SKIP_NV_TORCH=1         skip torch wheel
                                 RLINF_ORIN_SKIP_SYSTEM_POOL=1      skip torch system pool
                                 RLINF_ORIN_SYSTEM_POOL_SUDO=1      sudo install
                                 RLINF_ORIN_INSTALL_DEV=1           pytest-asyncio etc.
                               NO simulator deps; NO apt sys_deps.sh.
EOF
}

parse_args() {
    if [ "$#" -eq 0 ]; then
        print_help
        exit 0
    fi

    while [ "$#" -gt 0 ]; do
        case "$1" in
            -h|--help)
                print_help
                exit 0
                ;;
            --venv)
                if [ -z "${2:-}" ]; then
                    echo "--venv requires a directory name argument." >&2
                    exit 1
                fi
                VENV_DIR="${2:-}"
                shift 2
                ;;
            --model)
                if [ -z "${2:-}" ]; then
                    echo "--model requires a model name argument." >&2
                    exit 1
                fi
                MODEL="${2:-}"
                shift 2
                ;;
            --env)
                if [ -n "$ENV_NAME" ]; then
                    echo "Only one --env can be specified." >&2
                    exit 1
                fi
                ENV_NAME="${2:-}"
                shift 2
                ;;
            --use-mirror)
                USE_MIRRORS=1
                shift
                ;;
            --no-root)
                NO_ROOT=1
                shift
                ;;
            --install-rlinf)
                NO_INSTALL_RLINF_CMD=""
                shift
                ;;
            --*)
                echo "Unknown option: $1" >&2
                echo "Use --help to see available options." >&2
                exit 1
                ;;
            *)
                if [ -z "$TARGET" ]; then
                    TARGET="$1"
                    shift
                else
                    echo "Unexpected positional argument: $1" >&2
                    echo "Use --help to see usage." >&2
                    exit 1
                fi
                ;;
        esac
    done

    if [ -z "$TARGET" ]; then
        TARGET="embodied"
    fi
}

install_uv() {
    # Ensure uv is installed
    if ! command -v uv &> /dev/null; then
        echo "uv command not found. Installing uv..."
        # Check if pip is available
        if ! command -v pip &> /dev/null; then
            echo "pip command not found. Please install pip first." >&2
            exit 1
        fi
        pip_failed=0
        pip install uv || pip_failed=1
        if [ $pip_failed -eq 1 ]; then
            echo "Cannot install uv via pip. Installing uv using installer script..."
            if ! command -v wget &> /dev/null; then
                echo "wget command not found. Please install wget first." >&2
                exit 1
            fi
            
            # If uv already exists in ~/.local/bin, use it
            if [ -f ~/.local/bin/uv ]; then
                echo "uv already exists in ~/.local/bin. Using it..."
            else
                wget -qO- https://astral.sh/uv/install.sh | sh
            fi
            export PATH="$HOME/.local/bin:$PATH"
        fi
    fi
}

setup_mirror() {
    if [ "$USE_MIRRORS" -eq 1 ]; then
        export UV_PYTHON_INSTALL_MIRROR=https://ghfast.top/https://github.com/astral-sh/python-build-standalone/releases/download
        export UV_DEFAULT_INDEX=https://mirrors.aliyun.com/pypi/simple
        export HF_ENDPOINT=https://hf-mirror.com
        export GITHUB_PREFIX="https://ghfast.top/"
        git config --global url."${GITHUB_PREFIX}github.com/".insteadOf "https://github.com/"
    fi
}

unset_mirror() {
    if [ "$USE_MIRRORS" -eq 1 ]; then
        unset UV_PYTHON_INSTALL_MIRROR
        unset UV_DEFAULT_INDEX
        unset HF_ENDPOINT
        git config --global --unset url."${GITHUB_PREFIX}github.com/".insteadOf
    fi
}

create_and_sync_venv() {
    local required_python_mm
    required_python_mm="$(echo "$PYTHON_VERSION" | awk -F. '{print $1"."$2}')"

    if [ -d "$VENV_DIR" ] && [ -f "$VENV_DIR/bin/activate" ]; then
        echo "Found existing venv at $VENV_DIR; validating Python version compatibility..."
        # shellcheck disable=SC1090
        source "$VENV_DIR/bin/activate"

        local active_python_mm
        active_python_mm="$(python - <<'EOF'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
EOF
)"

        if [ "$active_python_mm" != "$required_python_mm" ]; then
            echo "Venv Python version mismatch: required ${required_python_mm}.x (from PYTHON_VERSION=${PYTHON_VERSION}), found ${active_python_mm}.x. Recreating venv..." >&2
            deactivate || true
            rm -rf "$VENV_DIR"

            # Create new venv
            install_uv
            uv venv "$VENV_DIR" --python "$PYTHON_VERSION"
            # shellcheck disable=SC1090
            source "$VENV_DIR/bin/activate"
        else
            echo "Reusing existing venv at $VENV_DIR"
            install_uv
        fi
    else
        # Create new venv
        install_uv
        uv venv "$VENV_DIR" --python "$PYTHON_VERSION"
        # shellcheck disable=SC1090
        source "$VENV_DIR/bin/activate"
    fi
    UV_TORCH_BACKEND=auto uv sync --active $NO_INSTALL_RLINF_CMD
}

install_flash_attn() {
    # Base release info – adjust when bumping flash-attn
    local flash_ver="2.7.4.post1"
    local base_url="${GITHUB_PREFIX}https://github.com/Dao-AILab/flash-attention/releases/download/v${flash_ver}"

    # Detect Python tags
    local py_major py_minor
    py_major=$(python - <<'EOF'
import sys
print(sys.version_info.major)
EOF
)
    py_minor=$(python - <<'EOF'
import sys
print(sys.version_info.minor)
EOF
)
    local py_tag="cp${py_major}${py_minor}"   # e.g. cp311
    local abi_tag="${py_tag}"                 # we assume cpXY-cpXY ABI, adjust if needed

    # Detect torch version (major.minor) and strip dots, e.g. 2.6.0 -> 26
    local torch_mm
    torch_mm=$(python - <<'EOF'
import torch
v = torch.__version__.split("+")[0]
parts = v.split(".")
print(f"{parts[0]}.{parts[1]}")
EOF
)

    # Detect CUDA major, e.g. 12 from 12.4
    local cuda_major
    cuda_major=$(python - <<'EOF'
import torch
from packaging.version import Version
v = Version(torch.version.cuda)
print(v.base_version.split(".")[0])
EOF
)

    local cu_tag="cu${cuda_major}"            # e.g. cu12
    local torch_tag="torch${torch_mm}"        # e.g. torch2.6

    # We currently assume cxx11 abi FALSE and linux x86_64
    local platform_tag="linux_x86_64"
    local cxx_abi="cxx11abiFALSE"

    local wheel_name="flash_attn-${flash_ver}+${cu_tag}${torch_tag}${cxx_abi}-${py_tag}-${abi_tag}-${platform_tag}.whl"
    uv pip uninstall flash-attn || true
    uv pip install "${base_url}/${wheel_name}" || (echo "Flash attn installation via wheel failed. Attempting to install from source..."; uv pip install flash-attn==${flash_ver} --no-build-isolation)
}

install_apex() {
    # Example URL: https://github.com/RLinf/apex/releases/download/25.09/apex-0.1+torch2.6-cp311-cp311-linux_x86_64.whl
    local base_url="${GITHUB_PREFIX}https://github.com/RLinf/apex/releases/download/25.09"

    local py_major py_minor
    py_major=$(python - <<'EOF'
import sys
print(sys.version_info.major)
EOF
)
    py_minor=$(python - <<'EOF'
import sys
print(sys.version_info.minor)
EOF
)

# Detect torch version (major.minor) and strip dots, e.g. 2.6.0 -> 26
    local torch_mm
    torch_mm=$(python - <<'EOF'
import torch
v = torch.__version__.split("+")[0]
parts = v.split(".")
print(f"{parts[0]}.{parts[1]}")
EOF
)
    local torch_tag="torch${torch_mm}"        # e.g. torch2.6
    local py_tag="cp${py_major}${py_minor}"   # e.g. cp311
    local abi_tag="${py_tag}"                 # we assume cpXY-cpXY ABI, adjust if needed
    local platform_tag="linux_x86_64"
    local wheel_name="apex-0.1+${torch_tag}-${py_tag}-${abi_tag}-${platform_tag}.whl"
        
    uv pip uninstall apex || true
    export NUM_THREADS=$(nproc)
    export NVCC_APPEND_FLAGS=${NVCC_APPEND_FLAGS:-"--threads ${NUM_THREADS}"}
    export APEX_PARALLEL_BUILD=${APEX_PARALLEL_BUILD:-${NUM_THREADS}}
    uv pip install "${base_url}/${wheel_name}" || (echo "Apex installation via wheel failed. Attempting to install from source..."; APEX_CPP_EXT=1 APEX_CUDA_EXT=1 uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/apex.git --no-build-isolation)
}

clone_or_reuse_repo() {
    # Usage: clone_or_reuse_repo ENV_VAR_NAME DEFAULT_DIR GIT_URL [GIT_CLONE_ARGS...]
    # - If ENV_VAR_NAME is set, verify it points to an existing directory and reuse it (no pull).
    # - Otherwise, clone GIT_URL (with optional GIT_CLONE_ARGS) into DEFAULT_DIR if it doesn't exist.
    # If env var is not set and the directory already exists as a git repo, check if it is intact and re-clone it if not.
    # The resolved directory path is printed to stdout.
    local env_var_name="$1"
    local default_dir="$2"
    local git_url="$3"
    shift 3

    # Read the value of the environment variable safely under `set -u`.
    local env_value
    env_value="$(printenv "$env_var_name" 2>/dev/null || true)"

    local target_dir
    if [ -n "$env_value" ]; then
        if [ ! -d "$env_value" ]; then
            echo "$env_var_name is set to '$env_value' but the directory does not exist." >&2
            exit 1
        fi
        target_dir="$env_value"
    else
        target_dir="$default_dir"
        if [ ! -d "$target_dir" ]; then
            git clone "$@" "$git_url" "$target_dir" >&2
        elif [ -d "$target_dir/.git" ]; then
            echo "Checking git repo $target_dir..." >&2
            local git_intact=1
            git -C "$target_dir" status --porcelain >/dev/null 2>&1 || git_intact=0
            if [ $git_intact -eq 1 ]; then
                echo "Git repo $target_dir is intact." >&2
            else
                echo "Git repo $target_dir is corrupted. Re-cloning..." >&2
                rm -rf "$target_dir"
                git clone "$@" "$git_url" "$target_dir" >&2
            fi
        fi
    fi

    printf '%s\n' "$(realpath "$target_dir")"
}

#=======================EMBODIED INSTALLERS=======================
install_common_embodied_deps() {
    uv sync --extra embodied --active $NO_INSTALL_RLINF_CMD
    uv pip install -r $SCRIPT_DIR/embodied/envs/common.txt
    if [ "$NO_ROOT" -eq 0 ]; then
        bash $SCRIPT_DIR/embodied/sys_deps.sh
    fi
    {
        echo "export NVIDIA_DRIVER_CAPABILITIES=all"
        echo "export VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json"
        echo "export VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json"
    } >> "$VENV_DIR/bin/activate"
}

install_openvla_model() {
    case "$ENV_NAME" in
        maniskill_libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_maniskill_libero_env
            ;;
        frankasim)
            create_and_sync_venv
            install_common_embodied_deps
            install_frankasim_env
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for OpenVLA model." >&2
            exit 1
            ;;
    esac
    uv pip install git+${GITHUB_PREFIX}https://github.com/openvla/openvla.git --no-build-isolation
    install_flash_attn
    uv pip uninstall pynvml || true
}

install_openvla_oft_model() {
    case "$ENV_NAME" in
        behavior)
            PYTHON_VERSION="3.10"
            create_and_sync_venv
            install_common_embodied_deps
            uv pip install git+${GITHUB_PREFIX}https://github.com/moojink/openvla-oft.git  --no-build-isolation
            install_behavior_env
            ;;
        maniskill_libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_maniskill_libero_env
            install_flash_attn
            uv pip install git+${GITHUB_PREFIX}https://github.com/moojink/openvla-oft.git  --no-build-isolation
            ;;
        metaworld)
            create_and_sync_venv
            install_common_embodied_deps
            install_flash_attn
            install_metaworld_env
            uv pip install git+${GITHUB_PREFIX}https://github.com/moojink/openvla-oft.git  --no-build-isolation
            ;;
        calvin)
            create_and_sync_venv
            install_common_embodied_deps
            install_flash_attn
            install_calvin_env
            uv pip install git+${GITHUB_PREFIX}https://github.com/moojink/openvla-oft.git  --no-build-isolation
            ;;
        robotwin)
            create_and_sync_venv
            install_common_embodied_deps
            install_flash_attn
            uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/openvla-oft.git@RLinf/v0.1  --no-build-isolation
            install_robotwin_env
            ;;
        opensora)
            create_and_sync_venv
            install_common_embodied_deps
            install_maniskill_libero_env
            install_opensora_world_model
            install_flash_attn
            uv pip install git+${GITHUB_PREFIX}https://github.com/moojink/openvla-oft.git
            ;;
        wan)
            create_and_sync_venv
            install_common_embodied_deps
            install_maniskill_libero_env
            install_wan_world_model
            install_flash_attn
            uv pip install git+${GITHUB_PREFIX}https://github.com/moojink/openvla-oft.git
            ;;
        liberopro)
            create_and_sync_venv
            install_common_embodied_deps
            install_liberopro_env
            install_flash_attn
            uv pip install git+${GITHUB_PREFIX}https://github.com/moojink/openvla-oft.git  --no-build-isolation
            ;;
        liberoplus)
            create_and_sync_venv
            install_common_embodied_deps
            install_liberoplus_env
            install_flash_attn
            uv pip install git+${GITHUB_PREFIX}https://github.com/moojink/openvla-oft.git  --no-build-isolation
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for OpenVLA-OFT model." >&2
            exit 1
            ;;
    esac
    uv pip uninstall pynvml || true
}

install_openpi_model() {
    case "$ENV_NAME" in
        behavior)
            PYTHON_VERSION="3.10"
            create_and_sync_venv
            install_common_embodied_deps
            uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/openpi
            install_behavior_env
            uv pip install protobuf==6.33.0
            ;;
        maniskill_libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_maniskill_libero_env
            uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/openpi
            install_flash_attn
            ;;
        metaworld)
            create_and_sync_venv
            install_common_embodied_deps
            uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/openpi
            install_flash_attn
            install_metaworld_env
            ;;
        calvin)
            create_and_sync_venv
            install_common_embodied_deps
            uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/openpi
            install_flash_attn
            install_calvin_env
            ;;
        robocasa)
            create_and_sync_venv
            install_common_embodied_deps
            uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/openpi
            install_flash_attn
            install_robocasa_env
            ;;
        robotwin)
            create_and_sync_venv
            install_common_embodied_deps
            uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/openpi
            install_flash_attn
            install_robotwin_env
            ;;
        isaaclab)
            create_and_sync_venv
            install_common_embodied_deps
            uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/openpi
            install_isaaclab_env
            # Torch is modified in Isaac Lab, install flash-attn afterwards
            install_flash_attn
            uv pip install numpydantic==1.7.0 pydantic==2.11.7 numpy==1.26.0
            ;;
        roboverse)
            create_and_sync_venv
            install_common_embodied_deps
            uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/openpi
            install_flash_attn
            install_roboverse_env
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for OpenPI model." >&2
            exit 1
            ;;
    esac

    # Replace transformers models with OpenPI's modified versions
    local py_major_minor
    py_major_minor=$(python - <<'EOF'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
EOF
)
    cp -r "$VENV_DIR/lib/python${py_major_minor}/site-packages/openpi/models_pytorch/transformers_replace/"* \
        "$VENV_DIR/lib/python${py_major_minor}/site-packages/transformers/"
    
    bash $SCRIPT_DIR/embodied/download_assets.sh --assets openpi
    uv pip uninstall pynvml || true
}

install_starvla_model() {
    case "$ENV_NAME" in
        maniskill_libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_maniskill_libero_env
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for StarVLA model." >&2
            exit 1
            ;;
    esac

    local starvla_path
    starvla_path=$(clone_or_reuse_repo STARVLA_PATH "$VENV_DIR/starVLA" https://github.com/starVLA/starVLA.git -b "${STARVLA_GIT_REF:-starVLA-1.2}" --depth 1)

    # Prefer upstream StarVLA requirements first when available.
    if [ -f "$starvla_path/requirements.txt" ]; then
        uv pip install -r "$starvla_path/requirements.txt"
    fi

    # Enforce RLinf-compatible runtime pins to avoid known breakages.
    uv pip install -r "$SCRIPT_DIR/embodied/models/starvla.txt"
    uv pip install -e "$starvla_path" --no-deps

    # Some StarVLA revisions call logger.log() on an overwatch logger that only
    # provides warning/info/error. Keep this patch guarded and optional.
    local framework_init="$starvla_path/starVLA/model/framework/__init__.py"
    if [ "${STARVLA_SKIP_LOGGER_PATCH:-0}" != "1" ] && [ -f "$framework_init" ]; then
        if grep "logger\\.log\\(" "$framework_init" >/dev/null 2>&1; then
            sed -i 's/logger\.log(/logger.warning(/g' "$framework_init"
        fi
    fi

    install_flash_attn
    uv pip uninstall pynvml || true
}

install_gr00t_model() {
    create_and_sync_venv
    install_common_embodied_deps

    local gr00t_path
    gr00t_path=$(clone_or_reuse_repo GR00T_PATH "$VENV_DIR/gr00t" https://github.com/RLinf/Isaac-GR00T.git)
    uv pip install -e "$gr00t_path" --no-deps
    uv pip install -r $SCRIPT_DIR/embodied/models/gr00t.txt
    case "$ENV_NAME" in
        maniskill_libero)
            install_maniskill_libero_env
            install_flash_attn
            ;;
        isaaclab)
            install_isaaclab_env
            # Torch is modified in Isaac Lab, install flash-attn afterwards
            install_flash_attn
            uv pip install numpydantic==1.7.0 pydantic==2.11.7 numpy==1.26.0
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for Gr00t model." >&2
            exit 1
            ;;
    esac
    uv pip uninstall pynvml || true
}

install_dexbotic_model() {
    case "$ENV_NAME" in
        maniskill_libero)
            create_and_sync_venv
            install_common_embodied_deps

            local dexbotic_path
            dexbotic_path=$(clone_or_reuse_repo DEXBOTIC_PATH "$VENV_DIR/dexbotic" https://github.com/dexmal/dexbotic.git -b 0.2.0)
            uv pip install -e "$dexbotic_path"

            install_maniskill_libero_env
            uv pip install transformers==4.53.2
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for Dexbotic model." >&2
            exit 1
            ;;
    esac
    uv pip uninstall pynvml || true
}

install_lingbot_vla_model() {
    create_and_sync_venv
    install_common_embodied_deps
    local lingbotvla_dir
    lingbotvla_dir=$(clone_or_reuse_repo LINGBOT_PATH "$VENV_DIR/lingbot-vla" ${GITHUB_PREFIX}https://github.com/RLinf/lingbot-vla.git --recurse-submodules)
    uv pip install -e $lingbotvla_dir
    uv pip install -r $lingbotvla_dir/requirements.txt
    uv pip install -e $lingbotvla_dir/lingbotvla/models/vla/vision_models/lingbot-depth/ --no-deps
    uv pip install -e $lingbotvla_dir/lingbotvla/models/vla/vision_models/MoGe --no-deps

    uv pip install git+${GITHUB_PREFIX}https://github.com/huggingface/lerobot.git@0cf864870cf29f4738d3ade893e6fd13fbd7cdb5
    uv pip install -r $SCRIPT_DIR/embodied/models/lingbotvla.txt

    case "$ENV_NAME" in
        robotwin)
            install_robotwin_env
            install_flash_attn
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for Lingbot-VLA model." >&2
            exit 1
            ;;
    esac
    uv pip uninstall pynvml || true
}

install_dreamzero_model() {
    case "$ENV_NAME" in
        maniskill_libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_maniskill_libero_env
            uv pip install -r $SCRIPT_DIR/embodied/models/dreamzero.txt
            install_flash_attn
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for DreamZero model." >&2
    esac
}

install_env_only() {
    SKIP_ROS=${SKIP_ROS:-0}
    # Per-env Python version overrides — must be set BEFORE create_and_sync_venv.
    # galaxea_r1_pro requires Python 3.10 to match the ROS2 Humble rclpy .so ABI
    # on Ubuntu 22.04 (system Python is 3.10; rclpy .so is compiled for 3.10).
    case "$ENV_NAME" in
        galaxea_r1_pro|galaxea_r1_pro_orin)
            PYTHON_VERSION="3.10.12"
            ;;
    esac
    # The galaxea_r1_pro_orin env has its own venv-creation path:
    # reuse Jetson CUDA torch via --system-site-packages, but install the
    # minimal RLinf runtime closure into the venv.  PyPI has no usable
    # aarch64+CUDA torch wheel for Jetson, and the full embodied sync would
    # pull simulators/VLA deps that are irrelevant on the robot.
    if [ "$ENV_NAME" != "galaxea_r1_pro_orin" ]; then
        create_and_sync_venv
    fi
    case "$ENV_NAME" in
        franka)
            uv sync --extra franka --active $NO_INSTALL_RLINF_CMD
            if [ "$SKIP_ROS" -ne 1 ]; then
                if [ "$NO_ROOT" -eq 0 ]; then
                    bash $SCRIPT_DIR/embodied/ros_install.sh
                fi
                install_franka_env
            fi
            ;;
        xsquare_turtle2)
            uv sync --extra xsquare_turtle2 --active $NO_INSTALL_RLINF_CMD
            install_xsquare_turtle2_env
            ;;
        habitat)
            install_common_embodied_deps
            install_habitat_env
            ;;
        galaxea_r1_pro)
            if [ "$SKIP_ROS" -ne 1 ]; then
                if [ "$NO_ROOT" -eq 0 ]; then
                    bash $SCRIPT_DIR/embodied/ros2_humble_install.sh
                fi
            fi
            install_common_embodied_deps
            install_galaxea_r1_pro_env
            ;;
        galaxea_r1_pro_orin)
            # Single-node / Orin-only V3 profile: CLI + local M1 joint-mode
            # RLinf SAC.  Reuses Jetson CUDA torch, installs non-torch RLinf
            # runtime deps into the venv, skips simulators and VLA deps.
            install_galaxea_r1_pro_orin_env
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for env-only installation." >&2
            exit 1
            ;;
    esac
}

#=======================ENV INSTALLERS=======================

install_maniskill_libero_env() {
    # Prefer an existing checkout if LIBERO_PATH is provided; otherwise clone into the venv.
    local libero_dir
    libero_dir=$(clone_or_reuse_repo LIBERO_PATH "$VENV_DIR/libero" https://github.com/RLinf/LIBERO.git)

    uv pip install -e "$libero_dir"
    echo "export PYTHONPATH=$(realpath "$libero_dir"):\$PYTHONPATH" >> "$VENV_DIR/bin/activate"
    uv pip install git+${GITHUB_PREFIX}https://github.com/haosulab/ManiSkill.git@v3.0.0b22

    # Maniskill assets
    bash $SCRIPT_DIR/embodied/download_assets.sh --assets maniskill
}

install_liberopro_env() {
    # Base LIBERO + ManiSkill required for LIBERO-Pro.
    local libero_dir
    libero_dir=$(clone_or_reuse_repo LIBERO_PATH "$VENV_DIR/libero" https://github.com/RLinf/LIBERO.git)
    uv pip install -e "$libero_dir"

    local libero_pro_dir
    libero_pro_dir=$(clone_or_reuse_repo LIBERO_PRO_PATH "$VENV_DIR/libero_pro" https://github.com/RLinf/LIBERO-PRO.git)
    uv pip install -e "$libero_pro_dir"
}

install_liberoplus_env() {
    local libero_dir
    libero_dir=$(clone_or_reuse_repo LIBERO_PATH "$VENV_DIR/libero" https://github.com/RLinf/LIBERO.git)
    uv pip install -e "$libero_dir"

    local libero_plus_dir
    libero_plus_dir=$(clone_or_reuse_repo LIBERO_PLUS_PATH "$VENV_DIR/libero_plus" https://github.com/RLinf/LIBERO-plus.git)
    uv pip install -r $libero_plus_dir/extra_requirements.txt
    uv pip install -e "$libero_plus_dir"
}

install_behavior_env() {
    # Prefer an existing checkout if BEHAVIOR_PATH is provided; otherwise clone into the venv.
    local behavior_dir
    behavior_dir=$(clone_or_reuse_repo BEHAVIOR_PATH "$VENV_DIR/BEHAVIOR-1K" https://github.com/RLinf/BEHAVIOR-1K.git -b RLinf/v3.7.2 --depth 1)

    pushd "$behavior_dir" >/dev/null
    UV_LINK_MODE=hardlink ./setup.sh --omnigibson --bddl --joylo --confirm-no-conda --accept-nvidia-eula --use-uv
    # OmniGibson's eval deps need another commit of lerobot, which is in conflict with which rlinf needs.
    # We actually does not use OmniGibson's lerobot deps, so just install other deps in OmniGibson's eval deps. 
    uv pip install "dm_tree>=0.1.9" "hydra-core>=1.3.2" "websockets>=15.0.1" "msgpack>=1.1.0" "gspread>=6.2.1" "open3d>=0.19.0" av "numpy<2"
    popd >/dev/null
    uv pip uninstall flash-attn || true
    uv pip install ml_dtypes==0.5.3 protobuf==3.20.3
    uv pip install click==8.2.1
    pushd ~ >/dev/null
    uv pip install torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1
    install_flash_attn
    popd >/dev/null
}

install_metaworld_env() {
    uv pip install metaworld==3.0.0
}

install_calvin_env() {
    local calvin_dir
    calvin_dir=$(clone_or_reuse_repo CALVIN_PATH "$VENV_DIR/calvin" https://github.com/mees/calvin.git --recurse-submodules)

    uv pip install wheel cmake==3.18.4.post1 setuptools==57.5.0 wheel==0.45.1
    # NOTE: Use a fork version of pyfasthash that fixes install on Python 3.11
    uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/pyfasthash.git --no-build-isolation
    uv pip install -e ${calvin_dir}/calvin_env/tacto
    uv pip install -e ${calvin_dir}/calvin_env
    uv pip install -e ${calvin_dir}/calvin_models
    uv pip install --upgrade hydra-core==1.3.2
}

install_isaaclab_env() {
    local isaaclab_dir
    isaaclab_dir=$(clone_or_reuse_repo ISAAC_LAB_PATH "$VENV_DIR/isaaclab" https://github.com/RLinf/IsaacLab)

    pushd ~ >/dev/null
    uv pip install "flatdict==4.0.1" --no-build-isolation
    uv pip install "cuda-toolkit[nvcc]==12.8.0"

    # Force CMake < 4 for egl-probe / robomimic native build compatibility
    uv pip uninstall -y cmake || true
    uv pip install "cmake<4"

    $isaaclab_dir/isaaclab.sh --install
    popd >/dev/null
}

install_robocasa_env() {
    local robocasa_dir
    robocasa_dir=$(clone_or_reuse_repo ROBOCASA_PATH "$VENV_DIR/robocasa" https://github.com/RLinf/robocasa.git)
    
    uv pip install -e "$robocasa_dir"
    uv pip install protobuf==6.33.0
    python -m robocasa.scripts.setup_macros
}

install_franka_env() {
    # Install serl_franka_controller
    # Check if ROS_CATKIN_PATH is set or serl_franka_controllers is already built
    set +euo pipefail
    source /opt/ros/noetic/setup.bash
    set -euo pipefail
    ROS_CATKIN_PATH=$(realpath "$VENV_DIR/franka_catkin_ws")
    LIBFRANKA_VERSION=${LIBFRANKA_VERSION:-0.15.0}
    FRANKA_ROS_VERSION=${FRANKA_ROS_VERSION:-0.10.0}

    mkdir -p "$ROS_CATKIN_PATH/src"

    # Clone necessary repositories
    pushd "$ROS_CATKIN_PATH/src"
    if [ ! -d "$ROS_CATKIN_PATH/src/serl_franka_controllers" ]; then
        git clone https://github.com/rail-berkeley/serl_franka_controllers
    fi
    if [ ! -d "$ROS_CATKIN_PATH/libfranka" ]; then
        git clone -b "${LIBFRANKA_VERSION}" --recurse-submodules https://github.com/frankaemika/libfranka $ROS_CATKIN_PATH/libfranka
    fi
    if [ ! -d "$ROS_CATKIN_PATH/src/franka_ros" ]; then
        # Use a fork version that fixes compile issues with newer libfranka using C++17
        git clone -b "${FRANKA_ROS_VERSION}" --recurse-submodules https://github.com/RLinf/franka_ros
    fi
    popd >/dev/null

    # Build
    pushd "$ROS_CATKIN_PATH"
    # libfranka first
    if [ ! -f "$ROS_CATKIN_PATH/libfranka/build/libfranka.so" ]; then
        mkdir -p "$ROS_CATKIN_PATH/libfranka/build"
        pushd "$ROS_CATKIN_PATH/libfranka/build" >/dev/null
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
        make -j$(nproc)
        popd >/dev/null
    fi
    export LD_LIBRARY_PATH=$ROS_CATKIN_PATH/libfranka/build:/opt/openrobots/lib:$LD_LIBRARY_PATH
    export CMAKE_PREFIX_PATH=$ROS_CATKIN_PATH/libfranka/build:$CMAKE_PREFIX_PATH

    # Then franka_ros
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DFranka_DIR:PATH=$ROS_CATKIN_PATH/libfranka/build

    # Finally serl_franka_controllers
    catkin_make -DCMAKE_CXX_STANDARD=17 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 --pkg serl_franka_controllers
    popd >/dev/null

    echo "export LD_LIBRARY_PATH=$ROS_CATKIN_PATH/libfranka/build:/opt/openrobots/lib:\$LD_LIBRARY_PATH" >> "$VENV_DIR/bin/activate"
    echo "export CMAKE_PREFIX_PATH=$ROS_CATKIN_PATH/libfranka/build:\$CMAKE_PREFIX_PATH" >> "$VENV_DIR/bin/activate"
    echo "source /opt/ros/noetic/setup.bash" >> "$VENV_DIR/bin/activate"
    echo "source $ROS_CATKIN_PATH/devel/setup.bash" >> "$VENV_DIR/bin/activate"
}

install_xsquare_turtle2_env() {
    uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/xsquare_turtle_basics.git
}

# ---------------------------------------------------------------------------
# Galaxea R1 Pro on Jetson AGX Orin — single-node, MINIMAL venv design.
#
# Two-tier package layout:
#
#   Tier A — SYSTEM POOL (system Python or ~/.local, shared across projects)
#     Common AI / robotics packages that any other project on this Orin
#     might reuse: torch (NVIDIA Jetson CUDA wheel), ray, gymnasium,
#     omegaconf / hydra-core, filelock, einops, imageio, msgpack,
#     cloudpickle, fsspec, jinja2, networkx, attrs, typing_extensions.
#     Installed via the SYSTEM Python's pip (default: ``--user`` to avoid
#     sudo; opt into a system-wide install with RLINF_ORIN_SYSTEM_POOL_SUDO=1).
#
#   Tier B — VENV (.venv at $VENV_DIR, the smallest possible footprint)
#     Only RLinf itself (editable, --no-deps), wired to source ROS 2 +
#     Galaxea SDK on every ``source .venv/bin/activate``.  Inherits Tier A
#     via ``uv venv --system-site-packages`` (which also exposes user-site).
#
# Why this layout:
#   * Galaxea R1 Pro real-robot training / inference on a single Jetson Orin
#     is bottlenecked by env / ROS / camera I/O, not by which dir torch lives
#     in.  Keep the venv pure so we can rm -rf .venv freely without losing
#     a 1+ GB CUDA torch.
#   * Other R1 Pro tools and Jetson AI demos can ``import torch`` / ``import
#     ray`` directly from /usr/bin/python3.10 without a venv.
#   * Real RL training (FSDP / Megatron actor) actually runs on a remote
#     GPU server, not Orin.  Orin only runs EnvWorker + lightweight
#     model inference, so we do not need transformers / accelerate /
#     huggingface-hub / wandb / tensorboard / datasets here at all.
# ---------------------------------------------------------------------------

# NVIDIA Jetson PyTorch wheel selection.
#
# IMPORTANT — Jetson-specific caveat: every NVIDIA-published Jetson PyTorch
# wheel (any nv24.xx tag, jp/v60 and jp/v61 alike) is built with
# torch.distributed DISABLED.  i.e.:
#     >>> torch.distributed.is_available()      # False
#     >>> hasattr(torch.distributed, "Work")    # False
# This means ``rlinf.scheduler`` (which imports ``dist.Work`` for type hints
# in collective.async_work / collective_group / multi_channel_pg) cannot be
# imported with these wheels.  The CLI tools (``test_galaxea_r1_pro_cli_*``)
# are still usable thanks to the ``_bootstrap_minimal_env_stubs`` shim, but
# real-robot training/inference that touches RLinf's scheduler will not work
# until either RLinf grows a distributed-disabled fallback for Orin OR you
# replace torch with a distributed-enabled aarch64 build.
#
# Distributed-enabled aarch64 builds exist at https://pypi.jetson-ai-lab.io
# (``jp6/cu126`` torch 2.8/2.9/2.10/2.11), but those link CUDA 12.6 and call
# ``cudaGetDriverEntryPointByVersion`` (CUDA 12.5+).  On stock JetPack 6.0 GA
# (this image: L4T R36.3, CUDA 12.2.140, libcudnn8 8.9.4) those wheels fail
# with ``undefined symbol: cudaGetDriverEntryPointByVersion`` in libc10_cuda.
# Bottom line: distributed-enabled torch on Jetson currently requires a
# JetPack 6.1+ image (CUDA 12.6+, cuDNN 9), and is NOT installable on top
# of stock JP6.0 GA.
#
# Recommended default — ``v60`` (nv24.05, torch 2.4.0a0):
#   * Links cuDNN **8** (matches what JetPack 6.0 GA ships: libcudnn8 8.9.x)
#   * Does NOT depend on cuSPARSELt — no extra NVIDIA repo / sudo dpkg
#   * On this Orin (CUDA 12.2 + cuDNN 8.9.4) it imports cleanly and runs
#     FP16 GEMM 2048×2048 at ~2.4 TFLOPs (verified during development).
#   * Compatible with RLinf's real-robot Orin path: env worker + small-net
#     inference (CLI controllers).  Real RL training stays on the remote
#     GPU server (torch 2.6 pinned in pyproject.toml).
#
# Optional newer variants (see RLINF_ORIN_JP_PYTORCH below):
#   * v60-nv24.07 — torch 2.4.0a0 with cuSPARSELt structured-sparse kernels.
#     Requires libcusparseLt0 (auto-installed via PyPI nvidia-cusparselt-cu12
#     unless you pass RLINF_ORIN_CUSPARSELT=apt for the NVIDIA Tegra deb).
#     Still distributed-disabled (NVIDIA wheel).
#   * v61 — torch 2.5.0a0 (nv24.08).  Links cuDNN **9** and cuSPARSELt.
#     Matches RLinf pyproject.toml's ``torch>=2.5,<=2.9`` version window
#     and picks up newer FSDP / dynamo APIs.  But on stock JetPack 6.0 GA
#     the system cuDNN is 8.9 — you must install libcudnn9 yourself (apt)
#     before this wheel will import.  Recommended ONLY on JetPack 6.1+
#     images.  Still distributed-disabled.
#   * jetson-ai-lab cu126 (RLINF_ORIN_PYTORCH_WHEEL_URL=...): only on
#     JetPack 6.1+ images (CUDA 12.6+).  This is the only distributed-
#     enabled aarch64 path today.
_galaxea_orin_default_torch_wheel_url() {
    case "${RLINF_ORIN_JP_PYTORCH:-v60}" in
        v60|jp60|60|6.0)
            echo "https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.4.0a0+07cecf4168.nv24.05.14710581-cp310-cp310-linux_aarch64.whl"
            ;;
        v60-nv24.06|v60-nv2406)
            echo "https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.4.0a0+f70bd71a48.nv24.06.15634931-cp310-cp310-linux_aarch64.whl"
            ;;
        v60-nv24.07|v60-nv2407)
            echo "https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.4.0a0+3bcc3cddb5.nv24.07.16234504-cp310-cp310-linux_aarch64.whl"
            ;;
        v61|jp61|61|6.1)
            echo "https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl"
            ;;
        *)
            return 1
            ;;
    esac
}

# Whether the chosen wheel needs cuSPARSELt at runtime.
_galaxea_orin_wheel_needs_cusparselt() {
    case "${RLINF_ORIN_JP_PYTORCH:-v60}" in
        v60|jp60|60|6.0) return 1 ;;
        *) return 0 ;;
    esac
}

# Whether the chosen wheel links cuDNN 9 (vs the JP6.0 default cuDNN 8).
_galaxea_orin_wheel_needs_cudnn9() {
    case "${RLINF_ORIN_JP_PYTORCH:-v60}" in
        v61|jp61|61|6.1) return 0 ;;
        *) return 1 ;;
    esac
}

# Install cuSPARSELt for Tegra into the user-pool so torch can dlopen
# libcusparseLt.so.0.  Two paths:
#   * default (no sudo): PyPI ``nvidia-cusparselt-cu12==0.6.3`` aarch64 wheel,
#     wires LD_LIBRARY_PATH into the venv activate hook.
#   * RLINF_ORIN_CUSPARSELT=apt: NVIDIA Tegra local deb installer (sudo).
_galaxea_orin_install_cusparselt() {
    local sys_py="$1"
    if [ "${RLINF_ORIN_CUSPARSELT:-pypi}" = "apt" ]; then
        echo "[install][galaxea_r1_pro_orin] Installing cuSPARSELt via NVIDIA Tegra deb (sudo)..."
        local tmp deb url
        url="https://developer.download.nvidia.com/compute/cusparselt/0.7.0/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb"
        tmp="$(mktemp -d)"; deb="$tmp/cusparselt.deb"
        if ! curl -fL -o "$deb" "$url"; then
            echo "[install][galaxea_r1_pro_orin] ERROR: failed to download cuSPARSELt deb." >&2
            return 1
        fi
        sudo dpkg -i "$deb" || true
        sudo cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.7.0/cusparselt-*-keyring.gpg /usr/share/keyrings/ 2>/dev/null || true
        sudo apt-get update -qq
        sudo apt-get install -y libcusparselt0 libcusparselt-dev
        rm -rf "$tmp"
    else
        echo "[install][galaxea_r1_pro_orin] Installing cuSPARSELt via PyPI nvidia-cusparselt-cu12 wheel (no sudo)..."
        if ! _galaxea_orin_system_pip "$sys_py" install "nvidia-cusparselt-cu12==0.6.3"; then
            echo "[install][galaxea_r1_pro_orin] ERROR: failed to install nvidia-cusparselt-cu12 from PyPI." >&2
            echo "  Re-run with RLINF_ORIN_CUSPARSELT=apt to use the NVIDIA Tegra deb path." >&2
            return 1
        fi
        # Wheel installs to <site>/cusparselt/lib/libcusparseLt.so.0; need to
        # add that to LD_LIBRARY_PATH for torch to find it at import time.
        local _ld_dir
        _ld_dir="$("$sys_py" - <<'PY'
import os, sysconfig, importlib.util
spec = importlib.util.find_spec('cusparselt')
if spec and spec.submodule_search_locations:
    print(os.path.join(spec.submodule_search_locations[0], 'lib'))
PY
)"
        if [ -n "$_ld_dir" ] && [ -f "$_ld_dir/libcusparseLt.so.0" ]; then
            local _line='export LD_LIBRARY_PATH="'"$_ld_dir"':${LD_LIBRARY_PATH:-}"'
            if [ -f "$VENV_DIR/bin/activate" ] && ! grep -qF "$_ld_dir" "$VENV_DIR/bin/activate"; then
                echo "$_line" >> "$VENV_DIR/bin/activate"
            fi
            export LD_LIBRARY_PATH="${_ld_dir}:${LD_LIBRARY_PATH:-}"
            echo "[install][galaxea_r1_pro_orin] cuSPARSELt at: $_ld_dir/libcusparseLt.so.0"
            echo "  (added to venv activate via LD_LIBRARY_PATH)"
        else
            echo "[install][galaxea_r1_pro_orin] WARN: could not locate cusparselt lib dir from PyPI wheel."
            return 1
        fi
    fi
}

# Decide whether to invoke the system pip with ``--user`` or system-wide.
#   RLINF_ORIN_SYSTEM_POOL_SUDO=1    — system-wide via sudo (multi-user)
#   default                          — ``--user`` (writes to ~/.local; no sudo)
_galaxea_orin_system_pip() {
    local sys_py="$1"
    shift
    if [ "${RLINF_ORIN_SYSTEM_POOL_SUDO:-0}" = "1" ]; then
        sudo -E "$sys_py" -m pip "$@"
    else
        "$sys_py" -m pip --no-input "$@" --user
    fi
}

# Install only the CUDA-enabled NVIDIA Jetson PyTorch wheel into the system
# Python (or ~/.local).  V3 deliberately keeps ray/gym/hydra/etc. out of the
# system pool; those live in the venv so this env remains focused on RLinf.
# Idempotent: if a working CUDA torch is already present, this does nothing.
_galaxea_orin_install_system_pool() {
    if [ "${RLINF_ORIN_SKIP_SYSTEM_POOL:-0}" = "1" ]; then
        echo "[install][galaxea_r1_pro_orin] Skipping Jetson torch system pool (RLINF_ORIN_SKIP_SYSTEM_POOL=1)."
        return 0
    fi
    local sys_py="$1"

    echo "[install][galaxea_r1_pro_orin] Tier A — ensuring NVIDIA Jetson CUDA PyTorch in SYSTEM Python pool."
    if [ "${RLINF_ORIN_SYSTEM_POOL_SUDO:-0}" = "1" ]; then
        echo "  Mode: sudo system-wide  (set RLINF_ORIN_SYSTEM_POOL_SUDO=0 to use --user)"
    else
        echo "  Mode: pip --user        (set RLINF_ORIN_SYSTEM_POOL_SUDO=1 to install into /usr/local/lib)"
    fi

    # Ensure modern pip / wheel; pin setuptools <80 so colcon-core 0.20.x
    # (system) keeps importing.
    _galaxea_orin_system_pip "$sys_py" install --upgrade pip wheel "setuptools>=69.5.1,<80" \
        >/dev/null 2>&1 || true

    # NVIDIA Jetson PyTorch (CUDA) — the only package intentionally managed in
    # the system/user pool.  PyPI does not provide aarch64 CUDA torch wheels.
    if [ "${RLINF_ORIN_SKIP_NV_TORCH:-0}" = "1" ]; then
        echo "[install][galaxea_r1_pro_orin] Skipping NVIDIA PyTorch wheel (RLINF_ORIN_SKIP_NV_TORCH=1)."
        return 0
    fi
    if [ "$(uname -m)" != "aarch64" ]; then
        echo "[install][galaxea_r1_pro_orin] Not aarch64; skipping NVIDIA Jetson PyTorch wheel."
        return 0
    fi
    if "$sys_py" -c 'import torch; assert torch.cuda.is_available()' >/dev/null 2>&1; then
        local _v
        _v="$("$sys_py" -c 'import torch;print(torch.__version__)')"
        echo "[install][galaxea_r1_pro_orin] System Python already has CUDA torch (${_v}); skipping NVIDIA wheel."
        return 0
    fi

    local wheel_url
    if [ -n "${RLINF_ORIN_PYTORCH_WHEEL_URL:-}" ]; then
        wheel_url="${RLINF_ORIN_PYTORCH_WHEEL_URL}"
    else
        wheel_url="$(_galaxea_orin_default_torch_wheel_url)" || {
            echo "[install][galaxea_r1_pro_orin] ERROR: unknown RLINF_ORIN_JP_PYTORCH='${RLINF_ORIN_JP_PYTORCH:-}'." >&2
            echo "  Use one of: v60 (default, nv24.05 — no cuSPARSELt needed), v60-nv24.06, v60-nv24.07, v61." >&2
            exit 1
        }
    fi

    # Pre-flight check: cuDNN 9 vs system cuDNN 8.
    if _galaxea_orin_wheel_needs_cudnn9; then
        if [ ! -f /usr/lib/aarch64-linux-gnu/libcudnn.so.9 ] \
           && ! ldconfig -p 2>/dev/null | grep -q 'libcudnn\.so\.9'; then
            echo "[install][galaxea_r1_pro_orin] ERROR: chosen wheel (RLINF_ORIN_JP_PYTORCH=${RLINF_ORIN_JP_PYTORCH}) links cuDNN 9 but the system has only cuDNN 8." >&2
            echo "  Options:" >&2
            echo "    * Recommended: re-run without RLINF_ORIN_JP_PYTORCH (defaults to v60 / nv24.05 / cuDNN 8 — works on this JetPack 6.0 GA image)." >&2
            echo "    * Or upgrade to cuDNN 9: sudo apt-get install -y libcudnn9-cuda-12 libcudnn9-dev-cuda-12 (after adding the matching NVIDIA repo)." >&2
            echo "    * Or override the wheel URL with a cuDNN-8 build: RLINF_ORIN_PYTORCH_WHEEL_URL=..." >&2
            exit 1
        fi
    fi

    # Pre-flight: cuSPARSELt (auto-install if missing).
    if _galaxea_orin_wheel_needs_cusparselt; then
        if [ -f /usr/lib/aarch64-linux-gnu/libcusparseLt.so.0 ] \
           || ldconfig -p 2>/dev/null | grep -q 'libcusparseLt\.so\.0'; then
            echo "[install][galaxea_r1_pro_orin] System cuSPARSELt found via ldconfig — skipping auto-install."
        else
            _galaxea_orin_install_cusparselt "$sys_py" || {
                echo "[install][galaxea_r1_pro_orin] ERROR: cuSPARSELt setup failed." >&2
                exit 1
            }
        fi
    fi

    echo "[install][galaxea_r1_pro_orin] Installing NVIDIA Jetson PyTorch wheel into system Python:"
    echo "  $wheel_url"
    if ! _galaxea_orin_system_pip "$sys_py" install --no-cache-dir --no-deps --force-reinstall "$wheel_url"; then
        echo "[install][galaxea_r1_pro_orin] ERROR: NVIDIA PyTorch wheel install failed." >&2
        echo "  See the JetPack/PyTorch compatibility matrix:" >&2
        echo "    https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html" >&2
        echo "  Override with RLINF_ORIN_PYTORCH_WHEEL_URL=... or RLINF_ORIN_JP_PYTORCH={v60,v60-nv24.06,v60-nv24.07,v61}." >&2
        exit 1
    fi

    # Detect ImportError early with actionable hints.
    local _torch_err
    if ! _torch_err="$("$sys_py" -c 'import torch' 2>&1)"; then
        echo "[install][galaxea_r1_pro_orin] ERROR: torch installed into system pool but fails to import:" >&2
        echo "$_torch_err" >&2
        if echo "$_torch_err" | grep -q 'libcudnn\.so\.9'; then
            echo "  Fix: this wheel needs cuDNN 9; install libcudnn9-cuda-12 / libcudnn9-dev-cuda-12 from the NVIDIA repo, OR re-run with RLINF_ORIN_JP_PYTORCH=v60 (default, cuDNN-8 compatible)." >&2
        elif echo "$_torch_err" | grep -q 'libcudnn\.so\.8'; then
            echo "  Fix: install libcudnn8 (sudo apt-get install -y libcudnn8 libcudnn8-dev)." >&2
        elif echo "$_torch_err" | grep -q cusparseLt; then
            echo "  Fix: rerun this script (cuSPARSELt auto-install will trigger), or install via apt:" >&2
            echo "    RLINF_ORIN_CUSPARSELT=apt bash requirements/install.sh embodied --env galaxea_r1_pro_orin" >&2
        fi
        exit 1
    fi
    echo "[install][galaxea_r1_pro_orin] Tier A torch import OK: $("$sys_py" -c 'import torch;print(torch.__version__,"cuda=",torch.cuda.is_available(),"cudnn=",torch.backends.cudnn.version())')"
}

_galaxea_orin_install_venv_runtime_deps() {
    # V3 runtime closure for the two supported Orin use cases:
    #   1. R1 Pro CLI controller (dummy/rclpy/ray backend)
    #   2. local single-process RLinf M1 joint-mode SAC runner
    #
    # Keep this deliberately smaller than ``uv sync --extra embodied``:
    # no simulators (sapien/robosuite/bddl), no VLA stack, no flash-attn/apex,
    # no torch install from PyPI.  Torch is inherited from the Jetson system
    # pool through --system-site-packages.
    local runtime_deps=(
        # Build / editable-install basics.
        "setuptools>=69.5.1,<80"
        wheel
        pip
        # Numeric + gym API.  NumPy is capped to stay compatible with Ubuntu
        # 22.04's system scipy when scipy comes from apt; scipy wheel is
        # installed when available so scipy.spatial Rotation works reliably.
        "numpy>=1.24,<1.27"
        "scipy>=1.8,<1.12"
        gymnasium
        # RLinf import/config/runtime surface.
        "ray>=2.47.0"
        hydra-core
        omegaconf
        pyyaml
        filelock
        cloudpickle
        msgpack
        attrs
        typing_extensions
        packaging
        psutil
        fsspec
        jinja2
        networkx
        einops
        imageio
        urllib3
    )

    echo "[install][galaxea_r1_pro_orin] Tier B — installing minimal RLinf runtime deps into venv."
    uv pip install --upgrade "${runtime_deps[@]}"

    if [ "${RLINF_ORIN_INSTALL_DEV:-0}" = "1" ]; then
        echo "[install][galaxea_r1_pro_orin] Installing optional dev/test tools into venv."
        uv pip install --upgrade pytest pytest-asyncio lark ruff
    fi
}

install_galaxea_r1_pro_orin_env() {
    # Single-node / Orin-only V3 env for Galaxea R1 Pro.
    #
    # Purpose:
    #   * Run RLinf's R1 Pro CLI tools on Orin.
    #   * Run the local single-process M1 joint-mode SAC guide
    #     (bt/docs/rwRL/r1pro6op47_reach_joint3.md).
    #
    # Layout:
    #   * System/user Python pool — NVIDIA Jetson CUDA PyTorch only.
    #     PyPI has no usable aarch64 CUDA torch wheel for this JetPack.
    #   * This venv — RLinf editable + minimal Python runtime closure
    #     for CLI/local SAC (ray import surface, gymnasium, hydra,
    #     omegaconf, numpy/scipy, pyyaml, filelock/cloudpickle/msgpack).
    #   * ROS2 + Galaxea SDK — sourced into the venv activation script.
    #
    # Explicitly skipped: simulators, VLA deps, sys_deps.sh, flash-attn, apex.

    # ---- 0. Pre-flight: arch + Jetson + Python checks -------------
    local arch
    arch="$(uname -m)"
    if [ "$arch" != "aarch64" ]; then
        echo ""
        echo "[install][galaxea_r1_pro_orin] WARNING: this profile is designed"
        echo "  for Jetson AGX Orin (aarch64).  You are on '$arch'."
        echo "  If you want the full server install instead, use:"
        echo "    bash requirements/install.sh embodied --env galaxea_r1_pro"
        echo ""
    fi
    if [ ! -f /etc/nv_tegra_release ]; then
        echo "[install][galaxea_r1_pro_orin] WARNING: /etc/nv_tegra_release missing -- not a Jetson?  Continuing anyway."
    fi
    local sys_py="/usr/bin/python3.10"
    if [ ! -x "$sys_py" ]; then
        # Fallback: try plain python3 if the version-suffixed binary is
        # absent.
        sys_py="$(command -v python3 || true)"
        if [ -z "$sys_py" ]; then
            echo "[install][galaxea_r1_pro_orin] ERROR: no system Python 3 found." >&2
            exit 1
        fi
    fi
    local sys_py_mm
    sys_py_mm="$("$sys_py" -c 'import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    if [ "$sys_py_mm" != "3.10" ]; then
        echo "[install][galaxea_r1_pro_orin] ERROR: system Python is ${sys_py_mm}, but ROS2 Humble rclpy on Jetson needs 3.10." >&2
        echo "  rclpy .so files at /opt/ros/humble/local/lib/python3.10/dist-packages cannot be loaded by ${sys_py_mm}." >&2
        exit 1
    fi
    if [ ! -f /opt/ros/humble/setup.bash ]; then
        echo ""
        echo "[install][galaxea_r1_pro_orin] WARNING: /opt/ros/humble/setup.bash NOT found."
        echo "  rclpy will NOT be importable until you install ROS2 Humble:"
        echo "    bash requirements/embodied/ros2_humble_install.sh"
        echo "  The Python venv will still be created so you can run --dummy mode."
        echo ""
    fi

    install_uv

    # ---- 1. Tier A — system/user pool (Jetson CUDA torch only) ----
    # Done BEFORE creating the venv so --system-site-packages can expose torch.
    _galaxea_orin_install_system_pool "$sys_py"

    # ---- 2. Tier B — minimal RLinf runtime venv -------------------
    # Uses the SYSTEM Python (not python-build-standalone) so ROS2 Humble
    # rclpy .so ABI matches.  Uses --system-site-packages so ``import torch``
    # sees the Jetson CUDA wheel from Tier A.
    if [ -d "$VENV_DIR" ] && [ -f "$VENV_DIR/bin/activate" ]; then
        local existing_py existing_ssp
        existing_py="$($VENV_DIR/bin/python -c 'import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null || echo unknown)"
        existing_ssp="$(grep -c 'include-system-site-packages = true' "$VENV_DIR/pyvenv.cfg" 2>/dev/null || echo 0)"
        if [ "$existing_py" != "3.10" ] || [ "$existing_ssp" -eq 0 ]; then
            echo "[install][galaxea_r1_pro_orin] Existing venv at $VENV_DIR has wrong Python ($existing_py) or no --system-site-packages; recreating."
            rm -rf "$VENV_DIR"
        else
            echo "[install][galaxea_r1_pro_orin] Reusing existing venv at $VENV_DIR (Python 3.10 + system-site-packages)."
        fi
    fi
    if [ ! -d "$VENV_DIR" ]; then
        echo "[install][galaxea_r1_pro_orin] Creating venv at $VENV_DIR with system Python ($sys_py) and --system-site-packages."
        uv venv "$VENV_DIR" --python "$sys_py" --system-site-packages
    fi
    # shellcheck disable=SC1090
    source "$VENV_DIR/bin/activate"

    # Install the minimal runtime closure into the venv before RLinf editable.
    # Use --no-deps for RLinf itself later so pyproject's torch>=2.5 constraint
    # cannot replace the Jetson CUDA torch.
    _galaxea_orin_install_venv_runtime_deps

    # ---- 3. Source ROS 2 + Galaxea SDK on every venv activate -----
    if [ -f /opt/ros/humble/setup.bash ]; then
        if ! grep -q "source /opt/ros/humble/setup.bash" "$VENV_DIR/bin/activate"; then
            echo "source /opt/ros/humble/setup.bash" >> "$VENV_DIR/bin/activate"
        fi
        set +euo pipefail
        source /opt/ros/humble/setup.bash 2>/dev/null || true
        set -euo pipefail
    fi
    local galaxea_install="${GALAXEA_INSTALL_PATH:-$HOME/galaxea/install}"
    if [ -f "${galaxea_install}/setup.bash" ]; then
        if ! grep -q "source ${galaxea_install}/setup.bash" "$VENV_DIR/bin/activate"; then
            echo "source ${galaxea_install}/setup.bash" >> "$VENV_DIR/bin/activate"
        fi
        set +euo pipefail
        source "${galaxea_install}/setup.bash" 2>/dev/null || true
        set -euo pipefail
    else
        echo "[install][galaxea_r1_pro_orin] WARNING: Galaxea SDK setup.bash not found at $galaxea_install"
        echo "  Set GALAXEA_INSTALL_PATH=/path/to/galaxea/install before re-running, or fix the workspace, then re-run this script."
    fi
    # DDS / RLinf env vars Galaxea SDK expects.
    {
        echo 'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"'
        echo 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-41}"'
    } | while read -r line; do
        if ! grep -qF "$line" "$VENV_DIR/bin/activate"; then
            echo "$line" >> "$VENV_DIR/bin/activate"
        fi
    done

    # ---- 4. RLinf editable install (without dependency resolution) -
    # Walk upward from RLINF_INSTALL_SCRIPT_DIR until we find pyproject.toml.
    # IMPORTANT: we use ``RLINF_INSTALL_SCRIPT_DIR`` (captured at install.sh
    # entry, before any third-party sourcing) instead of ``SCRIPT_DIR``,
    # because Galaxea SDK's mobiman.sh reassigns a bare ``SCRIPT_DIR`` to
    # ``/home/.../mobiman/share/mobiman/environment`` when sourced above,
    # which would corrupt the lookup and yield a spurious "could not find
    # RLinf pyproject.toml" error.
    export UV_LINK_MODE=${UV_LINK_MODE:-hardlink}
    local rlinf_root parent_dir
    rlinf_root="$RLINF_INSTALL_SCRIPT_DIR"
    while [ ! -f "${rlinf_root}/pyproject.toml" ]; do
        if [ "$rlinf_root" = "/" ]; then
            rlinf_root=""
            break
        fi
        parent_dir="$(dirname "$rlinf_root")"
        if [ "$parent_dir" = "$rlinf_root" ]; then
            rlinf_root=""
            break
        fi
        rlinf_root="$parent_dir"
    done
    if [ -z "$rlinf_root" ] || [ ! -f "${rlinf_root}/pyproject.toml" ]; then
        echo "[install][galaxea_r1_pro_orin] ERROR: could not find RLinf pyproject.toml above RLINF_INSTALL_SCRIPT_DIR=${RLINF_INSTALL_SCRIPT_DIR}" >&2
        echo "  Use a full RLinf git checkout (with pyproject.toml at the repo root), not a partial copy." >&2
        exit 1
    fi
    rlinf_root="$(cd "$rlinf_root" && pwd)"
    unset UV_PROJECT UV_WORKSPACE 2>/dev/null || true
    uv pip install -e "$rlinf_root" --no-deps

    # ---- 5. Verify imports + GPU + report sizes -------------------
    echo ""
    echo "[install][galaxea_r1_pro_orin] Verifying imports from inside the venv..."
    local check
    check=$(python <<'PYEOF' 2>&1 || true
import importlib, importlib.metadata as m, sys

# Required: must all import for CLI + local single-process RLinf SAC on Orin.
required = [
    ("torch", "torch"),
    ("numpy", "numpy"),
    ("scipy", "scipy"),
    ("ray", "ray"),
    ("gymnasium", "gymnasium"),
    ("hydra", "hydra"),
    ("omegaconf", "omegaconf"),
    ("filelock", "filelock"),
    ("imageio", "imageio"),
    ("einops", "einops"),
    ("pyyaml", "yaml"),
    ("cloudpickle", "cloudpickle"),
    ("msgpack", "msgpack"),
    ("attrs", "attr"),
    ("packaging", "packaging"),
    ("psutil", "psutil"),
    ("rlinf", "rlinf"),
]
# ROS 2 Humble + standard message bindings.
ros_mods = ["rclpy", "sensor_msgs.msg", "geometry_msgs.msg", "std_msgs.msg"]
# Galaxea SDK message types — warn-only (no SDK on dev machine is OK).
opt_mods = ["hdas_msg.msg"]

ok = True
for label, modname in required:
    try:
        importlib.import_module(modname)
        try:
            ver = m.version(modname)
        except Exception:
            ver = "?"
        print(f"  [OK ] {label}: {ver}")
    except Exception as e:
        ok = False
        print(f"  [FAIL] {label}: {e}")

for modname in ros_mods:
    try:
        importlib.import_module(modname)
        print(f"  [OK ] {modname}")
    except Exception as e:
        ok = False
        print(f"  [FAIL] {modname}: {e}  (ROS2 Humble not sourced?)")

for modname in opt_mods:
    try:
        importlib.import_module(modname)
        print(f"  [OK ] {modname}  (Galaxea SDK)")
    except Exception as e:
        print(f"  [WARN] {modname}: {e}  (Galaxea SDK not sourced — fine for dummy/CI)")

# GPU smoke test (Orin iGPU); warn-only — CI / no-driver still passes.
try:
    import torch
    import torch.distributed as dist
    print(f"  [OK ] torch.distributed.is_available(): {dist.is_available()} (False is expected on NVIDIA Jetson wheels)")
    if torch.cuda.is_available():
        try:
            x = torch.zeros(1, device="cuda")
            x.add_(1.0)
            print(f"  [OK ] torch.cuda: {torch.cuda.get_device_name(0)}  (kernel launch OK)")
        except Exception as e:
            print(f"  [WARN] torch.cuda available but kernel launch failed: {e}")
    else:
        print("  [WARN] torch.cuda.is_available() is False (CPU-only torch or driver missing)")
except Exception as e:
    print(f"  [WARN] torch CUDA probe failed: {e}")

# RLinf local-runner import smoke.  The Orin torch wheel has
# torch.distributed disabled, while some RLinf import paths refer to
# dist.Work / torch.Event in type annotations.  The guide runner installs the
# same import-time shim before importing these modules; verify that path here.
try:
    import torch
    import torch.distributed as dist
    if not hasattr(dist, "Work"):
        class _StubWork:
            def wait(self, *args, **kwargs):
                raise RuntimeError("torch.distributed is disabled on this Orin.")
            def is_completed(self):
                return False
        dist.Work = _StubWork
    if not hasattr(torch, "Event"):
        torch.Event = torch.cuda.Event if torch.cuda.is_available() else object
    from rlinf.envs.realworld.galaxear.r1_pro_safety import build_safety_config
    from rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher import JointStateDispatcher
    from rlinf.envs.realworld.galaxear.tasks.r1_pro_single_arm_reach_joint import GalaxeaR1ProSingleArmReachJointEnv
    from rlinf.models.embodiment.mlp_policy.mlp_policy import MLPPolicy
    build_safety_config({})
    print("  [OK ] RLinf Galaxea local SAC imports: SafetyConfig / JointStateDispatcher / Env / MLPPolicy")
except Exception as e:
    ok = False
    print(f"  [FAIL] RLinf Galaxea local SAC imports: {e}")

print()
print("OK_OVERALL" if ok else "FAILED_OVERALL")
PYEOF
)
    echo "$check"
    if echo "$check" | grep -q "FAILED_OVERALL"; then
        echo ""
        echo "[install][galaxea_r1_pro_orin] Some required imports FAILED — see above."
        echo "  Common causes:"
        echo "    * ROS2 Humble not installed:    bash requirements/embodied/ros2_humble_install.sh"
        echo "    * Jetson CUDA torch missing:    re-run without RLINF_ORIN_SKIP_SYSTEM_POOL=1"
        echo "    * Venv runtime deps missing:    re-run this script; it installs them into $VENV_DIR"
        echo "  The venv is preserved — re-run after fixing."
    fi

    local venv_size local_size
    venv_size=$(du -sh "$VENV_DIR" 2>/dev/null | awk '{print $1}')
    local_size=$(du -sh "$HOME/.local/lib/python3.10/site-packages" 2>/dev/null | awk '{print $1}')
    echo ""
    echo "[install][galaxea_r1_pro_orin] Footprint:"
    echo "    V3 venv runtime ($VENV_DIR):              ${venv_size:-(missing)}"
    echo "    Jetson user site (~/.local, torch etc.):  ${local_size:-(empty / sudo mode)}"
    echo "[install][galaxea_r1_pro_orin] env ready.  To use:"
    echo "    source $VENV_DIR/bin/activate"
    echo "    python toolkits/realworld_check/test_galaxea_r1_pro_cli_controller.py --help"
    echo "    python toolkits/realworld_check/train_r1pro_m1_orin_joint_mode_rlinf_sac.py --help"
    echo "    python -c 'import torch; print(torch.cuda.is_available(), torch.__version__)'"
}

install_galaxea_r1_pro_env() {
    # Galaxea R1 Pro real-world env: rclpy / ROS2 Humble / Galaxea SDK live OUTSIDE
    # the venv (apt + colcon).  Here we install the lightweight Python deps used by
    # the EnvWorker side (camera Mux, USB direct path, JPEG decode) and wire up the
    # venv so that ROS2 is auto-sourced on every `source .venv/bin/activate`.
    # The Orin must be set up separately per bt/docs/rwRL/r1pro5op47_imp1.md.

    # Source ROS2 now so that the rclpy importability check below works.
    # set +euo pipefail: ROS2 setup.bash may reference unbound variables.
    set +euo pipefail
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    set -euo pipefail

    uv pip install icmplib opencv-python pyrealsense2 PyTurboJPEG psutil filelock

    # Write ROS2 source commands into the venv activate script so that
    # `source .venv/bin/activate` automatically injects ROS2 into PYTHONPATH.
    # This mirrors install_franka_env's pattern for ROS1 Noetic (§13.4 of imp1.md).
    if [ -f /opt/ros/humble/setup.bash ]; then
        echo "source /opt/ros/humble/setup.bash" >> "$VENV_DIR/bin/activate"
    fi
    local galaxea_install="${GALAXEA_INSTALL_PATH:-$HOME/galaxea/install}"
    if [ -f "${galaxea_install}/setup.bash" ]; then
        echo "source ${galaxea_install}/setup.bash" >> "$VENV_DIR/bin/activate"
    fi

    if ! python -c "import rclpy" >/dev/null 2>&1; then
        echo ""
        echo "[install][galaxea_r1_pro] rclpy is NOT importable from this venv."
        echo "[install][galaxea_r1_pro] Install ROS2 Humble if not already done:"
        echo "[install][galaxea_r1_pro]   bash requirements/embodied/ros2_humble_install.sh"
        echo "[install][galaxea_r1_pro] Then re-activate the venv to pick up the sourced ROS2:"
        echo "[install][galaxea_r1_pro]   source $VENV_DIR/bin/activate"
        echo "[install][galaxea_r1_pro] Or before \`ray start\` use:"
        echo "[install][galaxea_r1_pro]   source ray_utils/realworld/setup_before_ray_galaxea_r1_pro.sh"
        echo "[install][galaxea_r1_pro] which sources /opt/ros/humble/setup.bash + galaxea workspace"
        echo "[install][galaxea_r1_pro] and exports DDS / RLinf env vars."
        echo ""
    else
        echo "[install][galaxea_r1_pro] rclpy importable OK."
    fi
    echo "[install][galaxea_r1_pro] env ready."
}

install_robotwin_env() {
    # Set TORCH_CUDA_ARCH_LIST based on the CUDA version
    local nvcc_exe
    if [ -x "$(command -v nvcc)" ]; then
        nvcc_exe=$(which nvcc)
    elif [ -x /usr/local/cuda/bin/nvcc ]; then
        nvcc_exe="/usr/local/cuda/bin/nvcc"
    else
        echo "nvcc not found. Cannot build robotwin environment."
        exit 1
    fi
    local cuda_major=$("$nvcc_exe" --version | grep 'Cuda compilation tools' | awk '{print $5}' | tr -d ',' | awk -F '.' '{print $1}')
    local cuda_minor=$("$nvcc_exe" --version | grep 'Cuda compilation tools' | awk '{print $5}' | tr -d ',' | awk -F '.' '{print $2}')
    if [ "$cuda_major" -gt 12 ] || { [ "$cuda_major" -eq 12 ] && [ "$cuda_minor" -ge 8 ]; }; then
        # Include Blackwell support for CUDA 12.8+
        export TORCH_CUDA_ARCH_LIST="7.0;8.0;9.0;10.0"
    else
        export TORCH_CUDA_ARCH_LIST="7.0;8.0;9.0"
    fi

    uv pip install mplib==0.2.1 gymnasium==0.29.1 av open3d zarr openai

    uv pip install git+${GITHUB_PREFIX}https://github.com/facebookresearch/pytorch3d.git@v0.7.9  --no-build-isolation
    uv pip install warp-lang==1.11.1
    uv pip install git+${GITHUB_PREFIX}https://github.com/NVlabs/curobo.git  --no-build-isolation

    # patch sapien and mplib for robotwin
    SAPIEN_LOCATION=$(uv pip show sapien | grep 'Location' | awk '{print $2}')/sapien
    # Adjust some code in wrapper/urdf_loader.py
    URDF_LOADER=$SAPIEN_LOCATION/wrapper/urdf_loader.py
    # ----------- before -----------
    # 667         with open(urdf_file, "r") as f:
    # 668             urdf_string = f.read()
    # 669 
    # 670         if srdf_file is None:
    # 671             srdf_file = urdf_file[:-4] + "srdf"
    # 672         if os.path.isfile(srdf_file):
    # 673             with open(srdf_file, "r") as f:
    # 674                 self.ignore_pairs = self.parse_srdf(f.read())
    # ----------- after  -----------
    # 667         with open(urdf_file, "r", encoding="utf-8") as f:
    # 668             urdf_string = f.read()
    # 669 
    # 670         if srdf_file is None:
    # 671             srdf_file = urdf_file[:-4] + ".srdf"
    # 672         if os.path.isfile(srdf_file):
    # 673             with open(srdf_file, "r", encoding="utf-8") as f:
    # 674                 self.ignore_pairs = self.parse_srdf(f.read())
    sed -i -E 's/("r")(\))( as)/\1, encoding="utf-8") as/g' $URDF_LOADER

    MPLIB_LOCATION=$(uv pip show mplib | grep 'Location' | awk '{print $2}')/mplib
    # Adjust some code in planner.py
    # ----------- before -----------
    # 807             if np.linalg.norm(delta_twist) < 1e-4 or collide or not within_joint_limit:
    # 808                 return {"status": "screw plan failed"}
    # ----------- after  ----------- 
    # 807             if np.linalg.norm(delta_twist) < 1e-4 or not within_joint_limit:
    # 808                 return {"status": "screw plan failed"}
    PLANNER=$MPLIB_LOCATION/planner.py
    sed -i -E 's/(if np.linalg.norm\(delta_twist\) < 1e-4 )(or collide )(or not within_joint_limit:)/\1\3/g' $PLANNER
}

install_frankasim_env() {
    local serldir
    serldir=$(clone_or_reuse_repo SERL_PATH "$VENV_DIR/serl" https://github.com/RLinf/serl.git -b RLinf/franka-sim)
    uv pip install -e "$serldir/franka_sim"
    uv pip install -r "$serldir/franka_sim/requirements.txt"
}

install_habitat_env() {
    local habitat_sim_dir
    habitat_sim_dir=$(clone_or_reuse_repo HABITAT_SIM_PATH "$VENV_DIR/habitat" https://github.com/facebookresearch/habitat-sim.git -b v0.3,3 --recurse-submodules)
    if [ -d "$habitat_sim_dir/build" ]; then
        rm -rf $habitat_sim_dir/build
    fi
    export CMAKE_POLICY_VERSION_MINIMUM=3.5
    uv pip install "$habitat_sim_dir" --config-settings="--build-option=--headless" --config-settings="--build-option=--with-bullet"
    uv pip install $habitat_sim_dir/build/deps/magnum-bindings/src/python/

    local habitat_lab_dir
    # Use a fork version of habitat-lab that fixes Python 3.11 compatibility issues
    habitat_lab_dir=$(clone_or_reuse_repo HABITAT_LAB_PATH "$VENV_DIR/habitat-lab" https://github.com/RLinf/habitat-lab.git -b v0.3.3 --recurse-submodules)
    uv pip install -e $habitat_lab_dir/habitat-lab
    uv pip install -e $habitat_lab_dir/habitat-baselines
}

install_opensora_world_model() {
    # Clone opensora repository
    local opensora_dir
    opensora_dir=$(clone_or_reuse_repo OPENSORA_PATH "$VENV_DIR/opensora" ${GITHUB_PREFIX}https://github.com/RLinf/opensora.git)
    
    uv pip install -e "$opensora_dir"
    
    # Install opensora dependencies
    uv pip install -r $SCRIPT_DIR/embodied/models/opensora.txt
    uv pip install git+${GITHUB_PREFIX}https://github.com/fangqi-Zhu/TensorNVMe.git --no-build-isolation
    echo "export LD_LIBRARY_PATH=~/.tensornvme/lib:\$LD_LIBRARY_PATH" >> "$VENV_DIR/bin/activate"
    install_apex
}

install_wan_world_model() {
    local wan_dir
    wan_dir=$(clone_or_reuse_repo WAN_PATH "$VENV_DIR/wan" https://github.com/RLinf/diffsynth-studio.git)
    uv pip install -e "$wan_dir"
    uv pip install -r $SCRIPT_DIR/embodied/models/wan.txt
}

install_roboverse_env() {
    local roboverse_dir
    roboverse_dir=$(clone_or_reuse_repo ROBOVERSE_PATH "$VENV_DIR/roboverse" https://github.com/tiny-xie/roboverse.git)
    uv pip install -e "${roboverse_dir}[mujoco]"
    uv pip install git+${GITHUB_PREFIX}https://github.com/facebookresearch/pytorch3d.git@v0.7.9 --no-build-isolation
    uv pip install -e "${roboverse_dir}[sapien3]"
    uv pip install -e "${roboverse_dir}[genesis]"
    
    local pyroki_dir
    pyroki_dir=$(clone_or_reuse_repo PYROKI_PATH "$roboverse_dir/pyroki" https://github.com/chungmin99/pyroki.git)
    uv pip install -e "$pyroki_dir"
    uv pip install "numpy==1.26.4" --force-reinstall
}

#=======================AGENTIC INSTALLER=======================

install_agentic() {
    uv sync --extra agentic-vllm --active $NO_INSTALL_RLINF_CMD
    uv sync --extra agentic-sglang --inexact --active $NO_INSTALL_RLINF_CMD

    # Megatron-LM
    # Prefer an existing checkout if MEGATRON_PATH is provided; otherwise clone into the venv.
    local megatron_dir
    megatron_dir=$(clone_or_reuse_repo MEGATRON_PATH "$VENV_DIR/Megatron-LM" https://github.com/NVIDIA/Megatron-LM.git -b core_r0.13.0)

    echo "export PYTHONPATH=$(realpath "$megatron_dir"):\$PYTHONPATH" >> "$VENV_DIR/bin/activate"

    # If TEST_BUILD is 1, skip installing megatron.txt
    if [ "$TEST_BUILD" -ne 1 ]; then
        uv pip install -r $SCRIPT_DIR/agentic/megatron.txt --no-build-isolation
    fi

    install_apex
    install_flash_attn
    uv pip uninstall pynvml || true
}

#=======================DOCUMENTATION INSTALLER=======================

install_docs() {
    uv sync --extra agentic-vllm --active $NO_INSTALL_RLINF_CMD
    uv sync --extra agentic-sglang --inexact --active $NO_INSTALL_RLINF_CMD
    uv sync --extra embodied --active --inexact $NO_INSTALL_RLINF_CMD
    uv pip install -r $SCRIPT_DIR/docs/requirements.txt
    uv pip uninstall pynvml || true
}

main() {
    parse_args "$@"
    setup_mirror

    case "$TARGET" in
        embodied)
            # validate --model
            if [ -n "$MODEL" ]; then
                if [[ ! " ${SUPPORTED_MODELS[*]} " =~ " $MODEL " ]]; then
                    echo "Unknown embodied model: $MODEL. Supported models: ${SUPPORTED_MODELS[*]}" >&2
                    exit 1
                fi
            fi
            # check --env is set and supported
            if [ -n "$ENV_NAME" ]; then
                if [[ ! " ${SUPPORTED_ENVS[*]} " =~ " $ENV_NAME " ]]; then
                    echo "Unknown environment: $ENV_NAME. Supported environments: ${SUPPORTED_ENVS[*]}" >&2
                    exit 1
                fi
            else
                echo "--env must be specified when target=embodied." >&2
                exit 1
            fi

            case "$MODEL" in
                openvla)
                    install_openvla_model
                    ;;
                openvla-oft)
                    install_openvla_oft_model
                    ;;
                openpi)
                    install_openpi_model
                    ;;
                starvla)
                    install_starvla_model
                    ;;
                gr00t)
                    install_gr00t_model
                    ;;
                dexbotic)
                    install_dexbotic_model
                    ;;
                lingbotvla)                  
                    install_lingbot_vla_model 
                    ;;
                dreamzero)
                    install_dreamzero_model
                    ;;
                "")
                    install_env_only
                    ;;
            esac
            ;;
        agentic)
            create_and_sync_venv
            install_agentic
            ;;
        docs)
            create_and_sync_venv
            install_docs
            ;;
        *)
			echo "Unknown target: $TARGET" >&2
			echo "Supported targets: ${SUPPORTED_TARGETS[*]}" >&2
            exit 1
            ;;
    esac

    unset_mirror
}

main "$@"
