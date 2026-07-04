#! /bin/bash

set -eo pipefail
#@# 例子: bash requirements/au_install.sh embodied --model aupi --aupi-path /home/physical/SRC/Robot/aupi05 --venv /mnt/r/VENV/rlinf/
TARGET=""

MODEL=""
ENV_NAME=""
VENV_DIR=".venv"
# Local path to an openpi (au / R1 Pro) checkout, used by --model aupi.
# See /home/physical/SRC/Robot/aupi05/install.sh for that project's own
# installation recipe (uv sync + editable install), which install_aupi_model
# replicates on top of the shared venv. Overridable via --aupi-path.
AUPI_PATH="/home/physical/SRC/Robot/aupi05"
PYTHON_VERSION="3.11.14"
TORCH_VERSION=""
PLATFORM="nvidia"
ROCM_VERSION=""
# PEP 440 local-version segment (including the leading '+') that
# apply_torch_override appends to torch/torchvision/torchaudio overrides so uv
# is forced to fetch the platform-specific wheel instead of the bare PyPI one.
# Empty for nvidia (PyPI CUDA wheels match `==X.Y.Z` directly). Set by the
# per-platform configure_<platform> hooks.
PLATFORM_TORCH_STR=""
# URL of the platform-specific PyTorch wheel index. When non-empty,
# apply_torch_override injects [[tool.uv.index]] + [tool.uv.sources] blocks
# into pyproject.toml so `uv sync` resolves torch/torchvision/torchaudio from
# this index (UV_TORCH_BACKEND alone only affects `uv pip install` /  `uv add`).
PLATFORM_TORCH_INDEX=""
# Package names routed through PLATFORM_TORCH_INDEX. Must include any transitive
# deps that only live on the platform-specific index (e.g. pytorch-triton-rocm
# for ROCm). Set per-platform by configure_<platform>.
PLATFORM_TORCH_PACKAGES=()
# Lines appended to the venv's bin/activate by embodied installers (each is a
# full shell statement, e.g. `export VK_DRIVER_FILES=...`). Populated per-
# platform by configure_<platform>; other targets ignore the array.
PLATFORM_VENV_EXPORTS=()
# Whether the platform supports flash-attn at all. When 0, install_flash_attn
# returns immediately without installing or building anything (e.g. Ascend
# where the kernels are CUDA-only and no NPU equivalent ships in the package).
PLATFORM_FLASH_ATTN_INSTALL=1
# Whether the platform has prebuilt flash-attn wheels available on the
# Dao-AILab GitHub releases. When 0, install_flash_attn skips the wheel and
# does a `uv pip install flash-attn==<ver> --no-build-isolation` source build.
# Only consulted when PLATFORM_FLASH_ATTN_INSTALL=1.
PLATFORM_FLASH_ATTN_PREBUILT=0
# User-level opt-out, set by --no-flash-attn. Wins over the platform default
# so the user can skip flash-attn on platforms where it would otherwise
# install (e.g. when build deps aren't available on the host).
DISABLE_FLASH_ATTN=0
# Whether apply_torch_override should rewrite the pyproject.toml `torchcodec`
# pin from ==0.2 to >=0.5. The ==0.2 line in override-dependencies has wheels
# only for x86_64 + torch 2.5/2.6, so it breaks on AMD (torch 2.8 from rocm
# index) and on Ascend (aarch64). Set per-platform by configure_<platform>.
PLATFORM_RELAX_TORCHCODEC=0
# Extra entries (full PEP 508 specifiers) inserted into the pyproject.toml
# `override-dependencies` array by apply_torch_override. Use this for
# platform-specific transitive pins that aren't in the original file
# (e.g. `"evdev<1.9"` on Ascend where newer evdev fails to build against
# older kernel headers). Set per-platform by configure_<platform>.
PLATFORM_EXTRA_OVERRIDES=()
# Default torch-backend per platform; user can override by exporting
# UV_TORCH_BACKEND before invoking this script.
DEFAULT_BACKEND_NVIDIA="auto"
# AMD composes UV_TORCH_BACKEND=rocm<version>; --rocm picks the version. When
# unset, configure_amd detects the system's ROCm version and auto-picks the
# minimum torch version on https://download.pytorch.org/whl/torch/ that has a
# matching +rocm<version> wheel.
# Add new platforms by extending SUPPORTED_PLATFORMS, defining
# configure_<platform> + install_<platform>_extras, and routing in their
# respective dispatchers below.
SUPPORTED_PLATFORMS=("nvidia" "amd" "ascend")
TEST_BUILD=${TEST_BUILD:-0}
# Absolute path to this script (resolves symlinks)
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
USE_MIRRORS=0
GITHUB_PREFIX=""
NO_ROOT=0
NO_INSTALL_RLINF_CMD="--no-install-project"
SUPPORTED_TARGETS=("embodied" "agentic" "docs")
SUPPORTED_MODELS=("openvla" "openvla-oft" "openpi" "gr00t" "dexbotic" "starvla" "lingbotvla" "dreamzero" "qwen3_vl" "fastwam" "aupi")
SUPPORTED_ENVS=("behavior" "maniskill_libero" "libero" "metaworld" "calvin" "isaaclab" "robocasa" "franka" "franka-dexhand" "frankasim" "robotwin" "habitat" "opensora" "wan" "xsquare_turtle2" "liberopro" "liberoplus" "roboverse" "embodichain" "d4rl" "dosw1" "gim_arm" "dummy")

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
                            Not required for --model aupi (real-world R1 Pro,
                            no simulator env).
    --aupi-path <dir>      Local path to an openpi (au / R1 Pro) checkout, used
                            by --model aupi. Defaults to
                            /home/physical/SRC/Robot/aupi05.

Common options:
    -h, --help             Show this help message and exit.
    --venv <dir>           Virtual environment directory name (default: .venv).
    --torch <version>      Override torch version (e.g., 2.7.0). torchvision/torchaudio are derived
                           automatically (torchvision=0.<minor+15>.<patch>, torchaudio=<torch>).
                           torchcodec is left untouched. Patches pyproject.toml in place for the
                           duration of the install; the original is restored on exit. On
                           --platform amd, defaults to the lowest torch version with a matching
                           +rocm<version> wheel on https://download.pytorch.org/whl/torch/.
    --platform <name>      Hardware platform: nvidia (default, fully tested), amd (experimental,
                           ROCm), or ascend (experimental, NPU). Sets UV_TORCH_BACKEND
                           (auto / rocm<version> / cpu); export UV_TORCH_BACKEND yourself to
                           bypass (e.g. UV_TORCH_BACKEND=cu124). Ascend uses CPU torch from PyPI
                           and adds torch-npu in install_ascend_extras.
    --rocm <version>       ROCm version for --platform amd. When unset, auto-detected from the
                           system (/opt/rocm/.info/version, hipconfig, rocminfo). Composes
                           UV_TORCH_BACKEND=rocm<version>. Ignored on other platforms.
    --python <version>     Python version for the venv (e.g. 3.11.14). Defaults to 3.11.14.
                           Must be >=3.10. Some envs (behavior, d4rl) require 3.10 and will override this.
    --use-mirror           Use mirrors for faster downloads.
    --no-root              Avoid system dependency installation for non-root users. Only use this if you are certain system dependencies are already installed.
    --no-flash-attn        Skip flash-attn install. Useful when the host lacks a CUDA build
                           toolchain or when the platform has no flash-attn support (Ascend).
    --install-rlinf        Install RLinf itself into the python.

Example (install RLinf + the local openpi-au/R1 Pro checkout into one venv):
    bash au_install.sh embodied --model aupi \\
        --aupi-path /home/physical/SRC/Robot/aupi05 \\
        --venv /mnt/r/VENV/rlinf/
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
            --python)
                if [ -z "${2:-}" ]; then
                    echo "--python requires a version argument (e.g. 3.11.14)." >&2
                    exit 1
                fi
                PYTHON_VERSION="${2:-}"
                shift 2
                ;;
            --torch)
                if [ -z "${2:-}" ]; then
                    echo "--torch requires a version argument (e.g. 2.7.0)." >&2
                    exit 1
                fi
                TORCH_VERSION="${2:-}"
                shift 2
                ;;
            --platform)
                if [ -z "${2:-}" ]; then
                    echo "--platform requires one of: ${SUPPORTED_PLATFORMS[*]}." >&2
                    exit 1
                fi
                PLATFORM="${2:-}"
                shift 2
                ;;
            --rocm)
                if [ -z "${2:-}" ]; then
                    echo "--rocm requires a version argument (e.g. 6.3)." >&2
                    exit 1
                fi
                ROCM_VERSION="${2:-}"
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
            --aupi-path)
                if [ -z "${2:-}" ]; then
                    echo "--aupi-path requires a directory argument." >&2
                    exit 1
                fi
                AUPI_PATH="${2:-}"
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
            --no-flash-attn)
                DISABLE_FLASH_ATTN=1
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

validate_python_version() {
    # Reject malformed versions (must be X.Y or X.Y.Z with numeric components).
    if [[ ! "$PYTHON_VERSION" =~ ^[0-9]+\.[0-9]+(\.[0-9]+)?$ ]]; then
        echo "--python must be of form X.Y or X.Y.Z (got '$PYTHON_VERSION')." >&2
        exit 1
    fi

    # Soft-check against pyproject.toml's requires-python = ">=3.10".
    local py_major py_minor _py_patch
    IFS='.' read -r py_major py_minor _py_patch <<< "$PYTHON_VERSION"
    local mm="${py_major}.${py_minor}"
    if [ "$(printf '%s\n3.10\n' "$mm" | sort -V | head -n1)" != "3.10" ]; then
        echo "[install.sh] WARNING: Python ${PYTHON_VERSION} is below the pyproject.toml requires-python minimum (>=3.10). The install may fail." >&2
    fi
}

#=======================PLATFORM CONFIG=======================
# Per-platform runtime env-var configuration. Each configure_<platform> runs
# before any uv operation, so set everything that affects how dependencies
# resolve here (UV_TORCH_BACKEND, indexes, build flags, etc.). All functions
# respect a pre-existing UV_TORCH_BACKEND from the caller's environment.

# Detect installed ROCm version. Prints major.minor on success, returns 1 on
# failure. Probes the standard locations in order of reliability.
detect_rocm_version() {
    local raw=""
    if [ -f /opt/rocm/.info/version ]; then
        raw=$(head -n1 /opt/rocm/.info/version 2>/dev/null)
    fi
    if [ -z "$raw" ] && command -v hipconfig &>/dev/null; then
        raw=$(hipconfig --version 2>/dev/null | head -n1)
    fi
    if [ -z "$raw" ] && command -v rocminfo &>/dev/null; then
        raw=$(rocminfo 2>/dev/null | grep -i 'ROCm Version' | head -n1)
    fi
    [ -z "$raw" ] && return 1

    local mm
    mm=$(echo "$raw" | grep -oE '[0-9]+\.[0-9]+' | head -n1)
    [ -z "$mm" ] && return 1
    echo "$mm"
}

# Find a torch version on the PyTorch wheel index that has a +rocm<rocm_ver>
# Linux x86_64 wheel matching PYTHON_VERSION's cpXY tag. Prefers the smallest
# version >= 2.5; falls back to the highest available wheel if no >= 2.5 wheel
# exists. Uses the NJU mirror (per-ROCm subdir) when --use-mirror is set,
# otherwise the upstream universal index. Echoes X.Y.Z on success, returns 1
# on failure.
detect_torch_for_rocm() {
    local rocm_ver="$1"

    if ! command -v curl &>/dev/null; then
        echo "[install.sh] curl not found; cannot auto-detect torch version." >&2
        return 1
    fi

    local url
    if [ "$USE_MIRRORS" -eq 1 ]; then
        url="https://mirrors.nju.edu.cn/pytorch/whl/rocm${rocm_ver}/torch/"
    else
        url="https://download.pytorch.org/whl/torch/"
    fi

    # Python ABI tag (e.g. 3.11.14 -> cp311). The venv hasn't been created yet
    # at this point, so derive it from the PYTHON_VERSION script global.
    local py_major py_minor _py_patch
    IFS='.' read -r py_major py_minor _py_patch <<< "$PYTHON_VERSION"
    local py_tag="cp${py_major}${py_minor}"

    local html
    html=$(curl -fsSL --max-time 30 "$url" 2>/dev/null) || {
        echo "[install.sh] Failed to fetch ${url}." >&2
        return 1
    }

    # Wheel filenames look like:
    #   torch-2.8.0+rocm6.4-cp311-cp311-manylinux_2_28_x86_64.whl
    # The abi tag may have a trailing 't' (free-threaded build); the platform
    # tag covers manylinux_*_x86_64 / manylinux<digits>_x86_64 / linux_x86_64
    # (NJU and upstream both stick to manylinux_2_28 for recent ROCm wheels,
    # but allow the older tags for forward-compat).
    local rocm_re="${rocm_ver//./\\.}"
    local versions
    versions=$(echo "$html" \
        | grep -oE "torch-[0-9]+\.[0-9]+\.[0-9]+\+rocm${rocm_re}(\.[0-9]+)?-${py_tag}-${py_tag}t?-(manylinux[^-]*|linux)_x86_64\.whl" \
        | sed -E 's/torch-([0-9]+\.[0-9]+\.[0-9]+).*/\1/' \
        | sort -uV)
    [ -z "$versions" ] && return 1

    # Prefer the smallest version >= 2.5.0; otherwise take the highest
    # available wheel (which the project may still reject at install time, but
    # surfaces a usable starting point).
    local picked=""
    while IFS= read -r v; do
        if [ "$(printf '%s\n2.5.0\n' "$v" | sort -V | head -n1)" = "2.5.0" ]; then
            picked="$v"
            break
        fi
    done <<< "$versions"
    if [ -z "$picked" ]; then
        picked=$(echo "$versions" | tail -n1)
    fi
    echo "$picked"
}

# Prints "MAJOR MINOR" (e.g. "12 4") on success, returns 1 if no CUDA is
# available. Probes torch.version.cuda first (safe None check), then falls
# back to nvcc so callers work both before and after the venv is populated.
detect_cuda_major_minor() {
    local mm
    if mm=$(python - <<'EOF' 2>/dev/null
import torch, sys
v = torch.version.cuda
if v is None:
    sys.exit(1)
parts = v.split(".")
print(parts[0], parts[1] if len(parts) > 1 else "0")
EOF
    ); then
        echo "$mm"
        return 0
    fi

    local nvcc_exe=""
    if command -v nvcc &>/dev/null; then
        nvcc_exe=$(command -v nvcc)
    elif [ -x /usr/local/cuda/bin/nvcc ]; then
        nvcc_exe="/usr/local/cuda/bin/nvcc"
    fi
    [ -z "$nvcc_exe" ] && return 1
    local ver
    ver=$("$nvcc_exe" --version | grep 'Cuda compilation tools' | awk '{print $5}' | tr -d ',')
    [ -z "$ver" ] && return 1
    echo "${ver%%.*} ${ver#*.}"
}

configure_nvidia() {
    PLATFORM_TORCH_STR=""
    PLATFORM_TORCH_INDEX=""
    PLATFORM_TORCH_PACKAGES=()
    PLATFORM_VENV_EXPORTS=(
        "export NVIDIA_DRIVER_CAPABILITIES=all"
        "export VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json"
        "export VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json"
    )
    PLATFORM_FLASH_ATTN_INSTALL=1
    PLATFORM_FLASH_ATTN_PREBUILT=1
    PLATFORM_RELAX_TORCHCODEC=0
    PLATFORM_EXTRA_OVERRIDES=()
    if [ -z "${UV_TORCH_BACKEND:-}" ]; then
        export UV_TORCH_BACKEND="$DEFAULT_BACKEND_NVIDIA"
    fi
}

configure_amd() {
    if [ -z "$ROCM_VERSION" ]; then
        ROCM_VERSION=$(detect_rocm_version) || {
            echo "[install.sh] Could not auto-detect ROCm version; pass --rocm explicitly." >&2
            exit 1
        }
        echo "[install.sh] Auto-detected ROCm version: ${ROCM_VERSION}"
    fi

    if [[ ! "$ROCM_VERSION" =~ ^[0-9]+\.[0-9]+(\.[0-9]+)?$ ]]; then
        echo "--rocm must be of form X.Y or X.Y.Z (got '$ROCM_VERSION')." >&2
        exit 1
    fi

    if [ -z "$TORCH_VERSION" ]; then
        TORCH_VERSION=$(detect_torch_for_rocm "$ROCM_VERSION") || {
            echo "[install.sh] No compatible torch wheels found for ROCm ${ROCM_VERSION} (Python ${PYTHON_VERSION}). Pass --torch explicitly." >&2
            exit 1
        }
        echo "[install.sh] Auto-selected torch version for ROCm ${ROCM_VERSION}: ${TORCH_VERSION}"
    fi

    PLATFORM_TORCH_STR="+rocm${ROCM_VERSION}"
    if [ "$USE_MIRRORS" -eq 1 ]; then
        PLATFORM_TORCH_INDEX="https://mirrors.nju.edu.cn/pytorch/whl/rocm${ROCM_VERSION}"
    else
        PLATFORM_TORCH_INDEX="https://download.pytorch.org/whl/rocm${ROCM_VERSION}"
    fi
    # All four packages are routed through the ROCm index (and only that
    # index — see explicit=true on [[tool.uv.index]]). torchvision/torchaudio
    # arrive transitively via vllm/etc.; pytorch-triton-rocm arrives
    # transitively via torch. apply_torch_override promotes them to direct
    # deps in [project.dependencies] so [tool.uv.sources] mappings actually
    # take effect (uv only applies sources to direct deps).
    PLATFORM_TORCH_PACKAGES=("torch" "torchvision" "torchaudio" "pytorch-triton-rocm" "triton-rocm")
    PLATFORM_VENV_EXPORTS=(
        "export AMD_VULKAN_ICD=RADV"
        "export VK_DRIVER_FILES=/usr/share/vulkan/icd.d/radeon_icd.x86_64.json"
        "export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/radeon_icd.x86_64.json"
    )
    PLATFORM_FLASH_ATTN_INSTALL=1
    PLATFORM_FLASH_ATTN_PREBUILT=0
    PLATFORM_RELAX_TORCHCODEC=1
    PLATFORM_EXTRA_OVERRIDES=()
    if [ -z "${UV_TORCH_BACKEND:-}" ]; then
        export UV_TORCH_BACKEND="rocm${ROCM_VERSION}"
    fi
}

configure_ascend() {
    # Ascend NPU uses CPU torch from PyPI plus torch-npu installed via
    # install_ascend_extras. No platform-specific wheel index is needed
    # because there's no ascend-tagged torch on PyTorch's index — torch-npu
    # is the standalone package that adds the NPU backend at runtime.
    PLATFORM_TORCH_STR=""
    PLATFORM_TORCH_INDEX=""
    PLATFORM_TORCH_PACKAGES=()
    PLATFORM_VENV_EXPORTS=()
    # flash-attn is CUDA-only; skip the install entirely on Ascend instead
    # of trying (and failing) to build it from source.
    PLATFORM_FLASH_ATTN_INSTALL=0
    PLATFORM_FLASH_ATTN_PREBUILT=0
    PLATFORM_RELAX_TORCHCODEC=1
    PLATFORM_EXTRA_OVERRIDES=()
    if [ -z "${UV_TORCH_BACKEND:-}" ]; then
        # `cpu` keeps `uv pip install torch ...` calls fetching the CPU build
        # from download.pytorch.org/whl/cpu instead of PyPI's CUDA wheel.
        export UV_TORCH_BACKEND="cpu"
    fi
    # evdev's generated ecodes.c references KEY_* constants without including
    # <linux/input-event-codes.h>. On systems where userspace kernel headers
    # split input-event-codes.h out of input.h, the build fails with
    # "KEY_ALL_APPLICATIONS undeclared" etc. Force-include the header for all
    # C compilations during this install so the constants are always visible.
    if [ -f /usr/include/linux/input-event-codes.h ]; then
        export CFLAGS="${CFLAGS:+$CFLAGS }-include /usr/include/linux/input-event-codes.h"
    fi
}

configure_platform() {
    if [[ ! " ${SUPPORTED_PLATFORMS[*]} " =~ " $PLATFORM " ]]; then
        echo "--platform must be one of: ${SUPPORTED_PLATFORMS[*]} (got '$PLATFORM')." >&2
        exit 1
    fi

    if [ -n "$ROCM_VERSION" ] && [ "$PLATFORM" != "amd" ]; then
        echo "[install.sh] WARNING: --rocm is only meaningful with --platform amd; ignoring on platform=${PLATFORM}." >&2
        ROCM_VERSION=""
    fi

    case "$PLATFORM" in
        nvidia)  configure_nvidia ;;
        amd)     configure_amd ;;
        ascend)  configure_ascend ;;
    esac
    echo "[install.sh] platform=${PLATFORM}, UV_TORCH_BACKEND=${UV_TORCH_BACKEND}"
}

#=======================PLATFORM EXTRAS=======================
# Per-platform post-install hooks. Each install_<platform>_extras runs after
# the target-specific case finishes (venv populated, target deps installed).
# Keep these symmetric — add platform-specific runtime libs / drivers / kernel
# packages here rather than sprinkling them through target installers.

install_nvidia_extras() {
    : # CUDA torch from PyPI works out of the box; flash-attn/apex are wired
      # into target installers where they are actually used.
}

install_amd_extras() {
    # Some downstream packages (vllm and friends) import `triton` directly even
    # when running on ROCm. pytorch-triton-rocm provides the ROCm runtime but
    # is not importable as `triton`, so install the `triton` shim package at
    # the matching version to satisfy `import triton`. Skipping is safe if
    # pytorch-triton-rocm isn't present — that just means torch-based packages
    # haven't been installed for this target.
    local triton_ver
    triton_ver=$(python - <<'EOF' 2>/dev/null || true
try:
    import importlib.metadata as m
    print(m.version("pytorch-triton-rocm"))
except Exception:
    pass
EOF
)
    if [ -z "$triton_ver" ]; then
        echo "[install.sh] pytorch-triton-rocm not installed; skipping matching triton install."
        return 0
    fi
    echo "[install.sh] Installing triton==${triton_ver} to match pytorch-triton-rocm"
    uv pip install "triton==${triton_ver}"
}

install_ascend_extras() {
    # Ascend NPU support comes from torch-npu, a side-car package that
    # registers an NPU backend on torch import. The package version must
    # match the installed torch (torch-npu 2.X.Y → torch 2.X.Y). Skip if
    # torch isn't present (e.g. docs target), so this hook is safe to run
    # for every ascend target.
    local torch_ver
    torch_ver=$(python - <<'EOF' 2>/dev/null || true
try:
    import torch
    print(torch.__version__.split("+")[0])
except Exception:
    pass
EOF
)
    if [ -z "$torch_ver" ]; then
        echo "[install.sh] torch not installed; skipping torch-npu install."
        return 0
    fi
    # torch-npu imports a few packages at runtime (`yaml`, `decorator`) but
    # doesn't declare them in its wheel metadata, so install them explicitly.
    uv pip install pyyaml decorator
    echo "[install.sh] Installing torch-npu==${torch_ver} to match torch"
    uv pip install "torch-npu==${torch_ver}" \
        || (echo "[install.sh] Pinned torch-npu==${torch_ver} failed; falling back to latest compatible build." >&2 \
            && uv pip install torch-npu)
    if [ -f /usr/local/Ascend/ascend-toolkit/set_env.sh ]; then
        echo "source /usr/local/Ascend/ascend-toolkit/set_env.sh" >> "$VENV_DIR/bin/activate"
    fi
}

install_platform_extras() {
    case "$PLATFORM" in
        nvidia)  install_nvidia_extras ;;
        amd)     install_amd_extras ;;
        ascend)  install_ascend_extras ;;
    esac
}

PYPROJECT_FILE="$(dirname "$SCRIPT_DIR")/pyproject.toml"
PYPROJECT_BACKUP=""

restore_pyproject() {
    if [ -n "$PYPROJECT_BACKUP" ] && [ -f "$PYPROJECT_BACKUP" ]; then
        mv -f "$PYPROJECT_BACKUP" "$PYPROJECT_FILE"
        PYPROJECT_BACKUP=""
    fi
}

apply_torch_override() {
    # Fires when --torch is given (rewrite versions), PLATFORM_TORCH_STR is
    # non-empty (append a PEP 440 local segment so uv picks the platform-specific
    # wheel rather than PyPI's CUDA build), PLATFORM_TORCH_INDEX is non-empty
    # (route torch* through a dedicated index for `uv sync`, which doesn't honor
    # UV_TORCH_BACKEND), PLATFORM_RELAX_TORCHCODEC is set (rewrite the
    # torchcodec pin for non-x86_64 / non-CUDA torch combos), or
    # PLATFORM_EXTRA_OVERRIDES has entries (insert extra override pins).

    # torchcodec==0.2 only has wheels for torch<=2.6. Relax the pin whenever
    # the effective torch version exceeds 2.6, regardless of platform.
    local _eff_torch="${TORCH_VERSION}"
    if [ -z "$_eff_torch" ] && [ -f "$PYPROJECT_FILE" ]; then
        _eff_torch=$(sed -nE 's/.*"torch==([^"+]+).*".*/\1/p' "$PYPROJECT_FILE" | head -1)
    fi
    if [ -n "$_eff_torch" ]; then
        local _tmaj _tmin _tpatch
        IFS='.' read -r _tmaj _tmin _tpatch <<< "$_eff_torch"
        if [ "$_tmaj" -gt 2 ] || { [ "$_tmaj" -eq 2 ] && [ "$_tmin" -gt 6 ]; }; then
            PLATFORM_RELAX_TORCHCODEC=1
        fi
    fi

    local needs_torch_rewrite=0
    if [ -n "$TORCH_VERSION" ] || [ -n "$PLATFORM_TORCH_STR" ] || [ -n "$PLATFORM_TORCH_INDEX" ]; then
        needs_torch_rewrite=1
    fi
    if [ "$needs_torch_rewrite" -eq 0 ] \
        && [ "$PLATFORM_RELAX_TORCHCODEC" -ne 1 ] \
        && [ ${#PLATFORM_EXTRA_OVERRIDES[@]} -eq 0 ]; then
        return 0
    fi

    if [ ! -f "$PYPROJECT_FILE" ]; then
        echo "Cannot locate pyproject.toml at $PYPROJECT_FILE" >&2
        exit 1
    fi

    PYPROJECT_BACKUP="${PYPROJECT_FILE}.rlinf-torch-bak.$$"
    cp "$PYPROJECT_FILE" "$PYPROJECT_BACKUP"
    trap 'restore_pyproject' EXIT INT TERM HUP

    if [ "$PLATFORM_RELAX_TORCHCODEC" -eq 1 ]; then
        # The pyproject.toml `torchcodec==0.2` override only has wheels for
        # x86_64 + torch ~2.5/2.6. It breaks on AMD (our torch override pins
        # 2.8 from the rocm index) and on Ascend (typically aarch64, where
        # 0.2.x has no wheels). Relaxing to >=0.5 lets uv pick a wheel for
        # the resolved environment; transitive pins like lerobot==0.1.0's
        # ==0.2 are superseded by override-dependencies.
        sed -i 's/"torchcodec==0\.2"/"torchcodec>=0.5"/' "$PYPROJECT_FILE"
        echo "[install.sh] Relaxed torchcodec override to >=0.5 for ${PLATFORM} compatibility"
    fi

    if [ ${#PLATFORM_EXTRA_OVERRIDES[@]} -gt 0 ]; then
        # Insert each extra override right after the opening bracket of the
        # override-dependencies array. Done in reverse so the final order
        # matches the array order. The trap restores the original on exit.
        local i
        for (( i=${#PLATFORM_EXTRA_OVERRIDES[@]}-1; i>=0; i-- )); do
            local entry="${PLATFORM_EXTRA_OVERRIDES[i]}"
            sed -i "/^override-dependencies = \\[\$/a\\    \"${entry}\"," "$PYPROJECT_FILE"
        done
        echo "[install.sh] Added override-dependencies entries: ${PLATFORM_EXTRA_OVERRIDES[*]}"
    fi

    if [ "$needs_torch_rewrite" -eq 0 ]; then
        echo "[install.sh] Original pyproject.toml will be restored on exit."
        return 0
    fi

    local torch_version torchvision_version torchaudio_version
    if [ -n "$TORCH_VERSION" ]; then
        local torch_major torch_minor torch_patch
        IFS='.' read -r torch_major torch_minor torch_patch <<< "$TORCH_VERSION"
        if [ "$torch_major" != "2" ] || [ -z "$torch_minor" ] || [ -z "$torch_patch" ]; then
            echo "--torch must be of form 2.Y.Z (got '$TORCH_VERSION')." >&2
            exit 1
        fi
        case "$torch_minor$torch_patch" in
            *[!0-9]*)
                echo "--torch components must be numeric (got '$TORCH_VERSION')." >&2
                exit 1
                ;;
        esac
        local tv_minor=$((torch_minor + 15))
        torch_version="$TORCH_VERSION"
        torchvision_version="0.${tv_minor}.${torch_patch}"
        torchaudio_version="$TORCH_VERSION"
    else
        # Reuse the public versions already pinned in pyproject.toml, stripping
        # any pre-existing local segment so PLATFORM_TORCH_STR can be re-applied cleanly.
        torch_version=$(sed -nE 's/.*"torch==([^"+]+).*".*/\1/p' "$PYPROJECT_FILE" | head -1)
        torchvision_version=$(sed -nE 's/.*"torchvision==([^"+]+).*".*/\1/p' "$PYPROJECT_FILE" | head -1)
        torchaudio_version=$(sed -nE 's/.*"torchaudio==([^"+]+).*".*/\1/p' "$PYPROJECT_FILE" | head -1)
        if [ -z "$torch_version" ] || [ -z "$torchvision_version" ] || [ -z "$torchaudio_version" ]; then
            echo "Could not parse existing torch/torchvision/torchaudio pins from $PYPROJECT_FILE" >&2
            exit 1
        fi
    fi

    local torch_pin="${torch_version}${PLATFORM_TORCH_STR}"
    local torchvision_pin="${torchvision_version}${PLATFORM_TORCH_STR}"
    local torchaudio_pin="${torchaudio_version}${PLATFORM_TORCH_STR}"

    sed -i \
        -e "s/\"torch==[^\"]*\"/\"torch==${torch_pin}\"/" \
        -e "s/\"torchvision==[^\"]*\"/\"torchvision==${torchvision_pin}\"/" \
        -e "s/\"torchaudio==[^\"]*\"/\"torchaudio==${torchaudio_pin}\"/" \
        "$PYPROJECT_FILE"

    echo "[install.sh] Patched pyproject.toml override-dependencies: torch==${torch_pin}, torchvision==${torchvision_pin}, torchaudio==${torchaudio_pin}"

    if [ -n "$PLATFORM_TORCH_INDEX" ]; then
        # `uv sync` does not honor UV_TORCH_BACKEND for resolution, so register
        # the platform-specific wheel index and pin every torch-family package
        # to it. `explicit = true` keeps unrelated packages (e.g. cmake) from
        # being shadowed by stale copies on the PyTorch index. Because
        # [tool.uv.sources] only applies to direct deps, also promote each
        # mapped package to a direct dep in [project.dependencies] (skipping
        # any already declared there). Appended to the file so the existing
        # trap restores the original on exit.
        for pkg in "${PLATFORM_TORCH_PACKAGES[@]}"; do
            if grep -qE "^[[:space:]]*\"${pkg}\\b" "$PYPROJECT_FILE"; then
                continue
            fi
            sed -i "/^dependencies = \\[\$/a\\    \"${pkg}\"," "$PYPROJECT_FILE"
        done

        {
            echo ""
            echo "[[tool.uv.index]]"
            echo "name = \"pytorch-platform\""
            echo "url = \"${PLATFORM_TORCH_INDEX}\""
            echo "explicit = true"
            echo ""
            echo "[tool.uv.sources]"
            for pkg in "${PLATFORM_TORCH_PACKAGES[@]}"; do
                echo "${pkg} = { index = \"pytorch-platform\" }"
            done
        } >> "$PYPROJECT_FILE"
        echo "[install.sh] Routed ${PLATFORM_TORCH_PACKAGES[*]} through index ${PLATFORM_TORCH_INDEX} (explicit; promoted to direct deps as needed)"
    fi

    echo "[install.sh] Original pyproject.toml will be restored on exit."
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
    uv sync --active $NO_INSTALL_RLINF_CMD
}

install_flash_attn() {
    # Base release info – adjust when bumping flash-attn
    local flash_ver="2.7.4.post1"

    if [ "$DISABLE_FLASH_ATTN" -eq 1 ]; then
        echo "[install.sh] --no-flash-attn was specified; skipping flash-attn install."
        return 0
    fi
    if [ "$PLATFORM_FLASH_ATTN_INSTALL" -ne 1 ]; then
        echo "[install.sh] flash-attn is unsupported on platform=${PLATFORM}; skipping install."
        return 0
    fi

    local torch_ge_28
    if torch_ge_28=$(python - <<'EOF' 2>/dev/null
import re
import torch

version = torch.__version__.split("+", 1)[0]
match = re.match(r"^(\d+)\.(\d+)", version)
if match is None:
    print("0")
else:
    major, minor = (int(part) for part in match.groups())
    print("1" if (major, minor) >= (2, 8) else "0")
EOF
    ); then
        if [ "$torch_ge_28" = "1" ]; then
            flash_ver="2.8.3"
        fi
    fi

    local prebuilt_flash_versions=("$flash_ver")
    if [ "$flash_ver" != "2.8.3" ]; then
        prebuilt_flash_versions+=("2.8.3")
    fi

    if [ "$PLATFORM_FLASH_ATTN_PREBUILT" -ne 1 ]; then
        echo "[install.sh] Building flash-attn==${flash_ver} from source on platform=${PLATFORM}..."
        uv pip uninstall flash-attn || true
        uv pip install "flash-attn==${flash_ver}" --no-build-isolation
        return 0
    fi
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
    local cuda_mm cuda_major
    cuda_mm=$(detect_cuda_major_minor) || {
        echo "[install.sh] Could not detect CUDA version; falling back to source build." >&2
        uv pip install "flash-attn==${flash_ver}" --no-build-isolation
        return 0
    }
    cuda_major="${cuda_mm%% *}"

    local cu_tag="cu${cuda_major}"            # e.g. cu12
    local torch_tag="torch${torch_mm}"        # e.g. torch2.6

    # We currently assume cxx11 abi FALSE and linux x86_64
    local platform_tag="linux_x86_64"
    local cxx_abi="cxx11abiFALSE"

    uv pip uninstall flash-attn || true
    local prebuilt_ver base_url wheel_name
    for prebuilt_ver in "${prebuilt_flash_versions[@]}"; do
        base_url="${GITHUB_PREFIX}https://github.com/Dao-AILab/flash-attention/releases/download/v${prebuilt_ver}"
        wheel_name="flash_attn-${prebuilt_ver}+${cu_tag}${torch_tag}${cxx_abi}-${py_tag}-${abi_tag}-${platform_tag}.whl"
        echo "[install.sh] Installing flash-attn prebuilt wheel from v${prebuilt_ver}..."
        if uv pip install "${base_url}/${wheel_name}"; then
            return 0
        fi
        echo "[install.sh] flash-attn prebuilt wheel v${prebuilt_ver} was unavailable or failed to install."
    done
    echo "Flash attn installation via prebuilt wheels failed. Attempting to install from source..."
    uv pip install "flash-attn==${flash_ver}" --no-build-isolation
}

install_apex() {
    if [ "$PLATFORM" != "nvidia" ]; then
        echo "[install.sh] Skipping apex install on platform=${PLATFORM} (CUDA-only)."
        return 0
    fi
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
        bash $SCRIPT_DIR/embodied/sys_deps.sh "$PLATFORM"
    fi
    if [ ${#PLATFORM_VENV_EXPORTS[@]} -gt 0 ]; then
        printf '%s\n' "${PLATFORM_VENV_EXPORTS[@]}" >> "$VENV_DIR/bin/activate"
    fi
}

install_openvla_model() {
    case "$ENV_NAME" in
        maniskill_libero|libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_${ENV_NAME}_env
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
        maniskill_libero|libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_${ENV_NAME}_env
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
        maniskill_libero|libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_${ENV_NAME}_env
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

install_aupi_model() {
    PLATFORM_FLASH_ATTN_PREBUILT=0
    # Installs RLinf (this workspace, per its own instructions) together
    # with a local openpi (au / R1 Pro) checkout (per ITS OWN instructions,
    # see aupi05/install.sh) into a single shared venv. Unlike
    # install_openpi_model (which pulls the RLinf-maintained openpi fork via
    # `uv pip install git+...`), this targets an arbitrary local checkout in
    # editable mode so local edits under AUPI_PATH take effect immediately.
    #
    # There is no simulator ENV_NAME for this model: R1 Pro is a real robot
    # served over WebSocket, so --env is not required/used here.

    if [ -z "$AUPI_PATH" ]; then
        echo "--aupi-path is required for --model aupi (local openpi (au / R1 Pro) checkout)." >&2
        exit 1
    fi
    # Check existence before canonicalizing: under `set -eo pipefail`,
    # `readlink -f` exits non-zero (and silently, since it's unguarded) when
    # any component but the last is missing, which would abort the script
    # without printing a helpful error.
    if [ ! -d "$AUPI_PATH" ]; then
        echo "AUPI_PATH '$AUPI_PATH' does not exist or is not a directory." >&2
        exit 1
    fi
    AUPI_PATH="$(readlink -f "$AUPI_PATH")"
    if [ ! -f "$AUPI_PATH/pyproject.toml" ]; then
        echo "AUPI_PATH '$AUPI_PATH' does not look like an openpi checkout (no pyproject.toml found there)." >&2
        exit 1
    fi

    # openpi (au) requires Python >= 3.11 (see aupi05/pyproject.toml); RLinf's
    # default PYTHON_VERSION (3.11.14) already satisfies this, but warn if the
    # user explicitly requested something older via --python.
    local _aupi_mm
    _aupi_mm="$(echo "$PYTHON_VERSION" | awk -F. '{print $1"."$2}')"
    if [ "$(printf '%s\n3.11\n' "$_aupi_mm" | sort -V | head -n1)" != "3.11" ]; then
        echo "[au_install.sh] WARNING: openpi (au) requires Python >=3.11, but PYTHON_VERSION=${PYTHON_VERSION}. The install may fail." >&2
    fi

    # This model is specifically about installing RLinf itself (not just its
    # deps) alongside a local openpi checkout, so force the RLinf package to
    # be installed regardless of --install-rlinf.
    NO_INSTALL_RLINF_CMD=""

    # Convenience default so `--model aupi` works with just --aupi-path (or
    # no flags at all) on this host; users who pass --venv explicitly are
    # respected as usual.
    if [ "$VENV_DIR" = ".venv" ]; then
        VENV_DIR="/mnt/r/VENV/rlinf"
        echo "[au_install.sh] --venv not specified; defaulting to '$VENV_DIR' for --model aupi."
    fi

    # 1) Make sure the target venv exists, creating it from scratch when
    #    $VENV_DIR is missing (or has an incompatible Python version) —
    #    create_and_sync_venv() handles both cases — then install RLinf per
    #    its own installation instructions (same routine every other
    #    embodied model in this script uses).
    if [ -d "$VENV_DIR" ] && [ -f "$VENV_DIR/bin/activate" ]; then
        echo "[au_install.sh] Found existing venv at '$VENV_DIR'; it will be reused."
    else
        echo "[au_install.sh] Venv '$VENV_DIR' does not exist yet; creating it..."
        mkdir -p "$(dirname "$VENV_DIR")"
    fi
    create_and_sync_venv
    install_common_embodied_deps

    # 1b) Verify RLinf itself actually ended up installed in the venv, and
    #     install it in editable/development mode if not. Clearing
    #     NO_INSTALL_RLINF_CMD above already makes `uv sync` (called from
    #     create_and_sync_venv/install_common_embodied_deps) install RLinf in
    #     editable mode as part of the regular sync, so this is normally a
    #     no-op — it only kicks in for a venv that was created/reused from a
    #     state where RLinf wasn't installed (e.g. previously set up with
    #     --no-install-project, or with RLinf later uninstalled by hand).
    local repo_root
    repo_root="$(dirname "$SCRIPT_DIR")"
    if ! uv pip show rlinf >/dev/null 2>&1; then
        echo "[au_install.sh] RLinf is not installed in '$VENV_DIR'; installing it from '$repo_root' in editable mode..."
        uv pip install -e "$repo_root" --no-deps
    else
        echo "[au_install.sh] RLinf is already installed in '$VENV_DIR'."
    fi

    # 2) Install the local openpi (au) checkout per its own install.sh
    #    recipe: `uv sync` followed by an editable install.
    #    - `--active` points the sync at the venv just set up above instead
    #      of creating openpi's own separate .venv.
    #    - `--inexact` keeps `uv sync` from removing packages that aren't in
    #      openpi's own uv.lock (its default "exact" mode would otherwise
    #      uninstall everything install_common_embodied_deps just installed
    #      for RLinf). Openpi's own pins still win for any overlapping
    #      package (e.g. transformers/torch), since this sync runs last.
    #    - `GIT_LFS_SKIP_SMUDGE=1` skips LeRobot's LFS large-file pulls, same
    #      as aupi05/install.sh.
    echo "[au_install.sh] Installing local openpi (au / R1 Pro) checkout from '$AUPI_PATH' into venv '$VENV_DIR'..."
    pushd "$AUPI_PATH" >/dev/null
    GIT_LFS_SKIP_SMUDGE=1 uv sync --active --inexact
    GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .
    popd >/dev/null

    # 2b) Pin the whole torch stack (torch / torchvision / torchaudio 2.6.0 +
    #     torchcodec 0.2.1) to RLinf's torch 2.6 set from a SINGLE cu126 index so
    #     every compiled extension (torch, torchvision, torchaudio, torchcodec,
    #     and the flash-attn wheel picked in step 3) shares one ABI.
    #
    #     Why this is needed: openpi (aupi05/pyproject.toml) pins torch==2.7.1, so
    #     the `uv sync --active --inexact` above drags parts of the stack toward
    #     the 2.7 line (torchvision==0.22.1 built against torch 2.7.1, and
    #     torchcodec==0.4.0 which ONLY supports torch 2.7) while RLinf's
    #     torch==2.6.0+cu126 (a local-version-tagged build kept in place by
    #     --inexact) is NOT upgraded. The mismatched pairs break at runtime in two
    #     ways: (a) torch 2.6 + torchvision 0.22.1 makes `import torchvision` raise
    #     `RuntimeError: operator torchvision::nms does not exist`, crashing
    #     `import openpi.models_pytorch.pi0_pytorch` (via `from transformers import
    #     GemmaForCausalLM` -> torchvision) at actor-init; (b) torch 2.6 +
    #     torchcodec 0.4.0 fails to register the CPU video-decode custom ops, so
    #     LeRobot dataset loading dies with `Could not run
    #     'torchcodec_ns::add_video_stream' with arguments from the 'CPU' backend`.
    #     torch 2.6 is self-consistent for both RLinf and openpi's PyTorch path
    #     (SFT trains fine on it), so we align the whole stack DOWN to 2.6 (torch
    #     2.6 <-> torchcodec 0.2 per the official compat table) rather than up to
    #     2.7.1.
    #     Must run BEFORE install_flash_attn so the flash-attn wheel matches torch 2.6.
    #
    #     --no-config is REQUIRED here: this runs with cwd inside the RLinf repo,
    #     whose pyproject.toml `[tool.uv] override-dependencies` pins
    #     `torchcodec==0.2`. uv overrides win over command-line requirements, so
    #     without --no-config our explicit `torchcodec==0.2.1` is silently forced
    #     down to 0.2.0 — and the 0.2.0+cu126 wheel ships ONLY the ffmpeg 5/6/7
    #     decoder libs (libtorchcodec5/6/7.so, needing libavutil.so.57/58/59),
    #     which fail to load on hosts that only have FFmpeg 4 (libavutil.so.56).
    #     0.2.1+cu126 additionally ships libtorchcodec4.so, so it loads against
    #     the system FFmpeg 4. --no-config makes uv ignore pyproject/uv.toml so
    #     the exact 0.2.1 pin is honored (the cu126 index is still forced via the
    #     explicit --index-url below).
    echo "[au_install.sh] Aligning torch/torchvision/torchaudio/torchcodec to torch 2.6 (cu126) for openpi..."
    uv pip install --reinstall --no-config \
        torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 torchcodec==0.2.1 \
        --index-url https://download.pytorch.org/whl/cu126
    # torchcodec 0.2.1+cu126 is a CUDA build whose decoder .so files dlopen
    # libnppicc.so.12 (NVIDIA NPP). uv does not pull NPP in automatically, so
    # install it explicitly here; it is exposed on LD_LIBRARY_PATH in step 2c.
    uv pip install nvidia-npp-cu12==12.4.1.87
    python -c "import torch, torchvision, torchaudio; from torchvision.transforms import InterpolationMode; print('[au_install.sh] torch', torch.__version__, '| torchvision', torchvision.__version__, '| torchaudio', torchaudio.__version__, '| torchvision ops OK')"

    # 2c) Expose the NPP shared libs (libnppicc.so.12 et al.) that torchcodec's
    #     cu126 decoder needs at runtime. Without this, `from torchcodec.decoders
    #     import VideoDecoder` fails with `libnppicc.so.12: cannot open shared
    #     object file`, which breaks LeRobot video decoding during SFT. Append the
    #     dir to the venv's activate so every future `source .../activate` (and
    #     Ray worker that inherits it) picks it up; guard against duplicate lines
    #     when the venv is reused. Also export it in the current shell so the
    #     verification import below can load the decoder.
    local _aupi_py_mm npp_lib
    _aupi_py_mm=$(python - <<'EOF'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
EOF
)
    npp_lib="$VENV_DIR/lib/python${_aupi_py_mm}/site-packages/nvidia/npp/lib"
    if [ -d "$npp_lib" ]; then
        export LD_LIBRARY_PATH="${npp_lib}:${LD_LIBRARY_PATH:-}"
        if ! grep -qsF "$npp_lib" "$VENV_DIR/bin/activate"; then
            echo "export LD_LIBRARY_PATH=\"${npp_lib}:\$LD_LIBRARY_PATH\"" >> "$VENV_DIR/bin/activate"
            echo "[au_install.sh] Added NPP lib dir to venv activate LD_LIBRARY_PATH: ${npp_lib}"
        fi
        python -c "from torchcodec.decoders import VideoDecoder; import torchcodec; print('[au_install.sh] torchcodec', torchcodec.__version__, 'decode ops load OK')"
    else
        echo "[au_install.sh] WARNING: NPP lib dir not found at '${npp_lib}'; torchcodec video decode may fail at runtime (libnppicc.so.12)." >&2
    fi
    uv pip install --reinstall hydra-core==1.4.0.dev1 omegaconf==2.4.0.dev4
    # 3) Install flash-attn matching the torch 2.6 aligned in step 2b above.
    #    Mirrors every other OpenPI-based embodied model in this script, which
    #    installs flash-attn right after the openpi package itself.
    install_flash_attn

    # 4) Apply openpi's transformers_replace patch, exactly like
    #    install_openpi_model() does. openpi.models_pytorch.pi0_pytorch's
    #    PI0Pytorch.__init__ hard-requires it (`from transformers.models.siglip
    #    import check` + check_whether_transformers_replace_is_installed_correctly(),
    #    which asserts transformers==4.53.2 AND that the patched siglip/gemma/
    #    paligemma modules are present). Without this, training crashes at
    #    actor-init even though `import openpi` alone succeeds.
    #
    #    Unlike install_openpi_model (openpi in site-packages), here openpi is an
    #    EDITABLE install of the local checkout, so the patch source lives in
    #    $AUPI_PATH/src, not under site-packages/openpi. Copy from there into the
    #    venv's transformers package.
    local py_major_minor transformers_dir replace_src
    py_major_minor=$(python - <<'EOF'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
EOF
)
    transformers_dir="$VENV_DIR/lib/python${py_major_minor}/site-packages/transformers"
    replace_src="$AUPI_PATH/src/openpi/models_pytorch/transformers_replace"
    if [ -d "$replace_src" ] && [ -d "$transformers_dir" ]; then
        echo "[au_install.sh] Applying openpi transformers_replace patch: '$replace_src' -> '$transformers_dir'"
        cp -r "$replace_src/"* "$transformers_dir/"
        python -c "from transformers.models.siglip import check; assert check.check_whether_transformers_replace_is_installed_correctly(), 'transformers_replace patch verification failed'; import transformers; print('[au_install.sh] transformers_replace OK, transformers', transformers.__version__)"
    else
        echo "[au_install.sh] WARNING: could not apply transformers_replace patch (missing '$replace_src' or '$transformers_dir'); openpi pi0.5 init will fail until it is applied." >&2
    fi

    uv pip uninstall pynvml || true
}

install_starvla_model() {
    case "$ENV_NAME" in
        maniskill_libero|libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_${ENV_NAME}_env
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
        maniskill_libero|libero)
            install_${ENV_NAME}_env
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
        maniskill_libero|libero)
            create_and_sync_venv
            install_common_embodied_deps

            local dexbotic_path
            dexbotic_path=$(clone_or_reuse_repo DEXBOTIC_PATH "$VENV_DIR/dexbotic" https://github.com/dexmal/dexbotic.git -b 0.2.0)
            uv pip install -e "$dexbotic_path"

            install_${ENV_NAME}_env
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
        maniskill_libero|libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_${ENV_NAME}_env
            uv pip install -r $SCRIPT_DIR/embodied/models/dreamzero.txt
            install_flash_attn
            ;;
        "")
            create_and_sync_venv
            install_common_embodied_deps
            uv pip install -r $SCRIPT_DIR/embodied/models/dreamzero.txt
            install_flash_attn
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for DreamZero model." >&2
            exit 1
            ;;
    esac
}

# Pin torch/torchvision/torchcodec to FastWAM's CUDA 12.8 wheels (outside the RLinf repo so
# uv does not read pyproject.toml override-dependencies during resolution).
install_fastwam_torch_cu128() {
    local torch_base="${TORCH_VERSION:-2.7.1}"
    local torch_major torch_minor torch_patch tv_minor torchvision_base
    IFS='.' read -r torch_major torch_minor torch_patch <<< "$torch_base"
    if [ "$torch_major" != "2" ] || [ -z "$torch_minor" ] || [ -z "$torch_patch" ]; then
        echo "[install.sh] install_fastwam_torch_cu128: --torch must be 2.Y.Z (got '${torch_base}')." >&2
        exit 1
    fi
    tv_minor=$((torch_minor + 15))
    torchvision_base="0.${tv_minor}.${torch_patch}"

    local cuda_index="https://download.pytorch.org/whl/cu128"
    if [ "$USE_MIRRORS" -eq 1 ]; then
        cuda_index="https://mirrors.nju.edu.cn/pytorch/whl/cu128"
    fi

    echo "[install.sh] Pinning FastWAM torch stack: torch==${torch_base}+cu128, torchvision==${torchvision_base}+cu128, torchcodec==0.5+cu128"
    (
        cd /tmp || exit 1
        uv pip install \
            "torch==${torch_base}+cu128" \
            "torchvision==${torchvision_base}+cu128" \
            "torchcodec==0.5+cu128" \
            --index-strategy unsafe-best-match \
            --extra-index-url "${cuda_index}"
    )
}

# Resolve FASTWAM_ROOT (env, or ../../Robot/FastWAM relative to the RLinf repo) and install editable.
install_fastwam_editable_package() {
    local fastwam_root="${FASTWAM_ROOT:-}"
    local rlinf_root
    rlinf_root="$(dirname "$SCRIPT_DIR")"

    if [ -z "$fastwam_root" ]; then
        local candidate="${rlinf_root}/../../Robot/FastWAM"
        if [ -f "${candidate}/pyproject.toml" ]; then
            fastwam_root="$(realpath "${candidate}")"
        fi
    fi

    if [ -z "$fastwam_root" ] || [ ! -f "${fastwam_root}/pyproject.toml" ]; then
        echo "[install.sh] FASTWAM_ROOT is unset and FastWAM was not found at ${rlinf_root}/../../Robot/FastWAM." >&2
        echo "[install.sh] Export FASTWAM_ROOT=/path/to/FastWAM and re-run, or: uv pip install -e /path/to/FastWAM --no-deps" >&2
        return 0
    fi

    fastwam_root="$(realpath "$fastwam_root")"
    echo "[install.sh] Installing fastwam editable from ${fastwam_root}"
    uv pip install -e "${fastwam_root}" --no-deps

    if ! grep -q 'FASTWAM_ROOT=' "$VENV_DIR/bin/activate" 2>/dev/null; then
        {
            echo "export FASTWAM_ROOT=\"${fastwam_root}\""
            echo "export FASTWAM_PATH=\"${fastwam_root}/src\""
        } >> "$VENV_DIR/bin/activate"
    fi
}

_install_fastwam_model_finish() {
    uv pip install -r "$SCRIPT_DIR/embodied/models/fastwam.txt"
    install_flash_attn
    install_fastwam_torch_cu128
    install_fastwam_editable_package
}

install_fastwam_model() {
    case "$ENV_NAME" in
        libero)
            create_and_sync_venv
            install_common_embodied_deps
            install_libero_env
            _install_fastwam_model_finish
            ;;
        robotwin)
            create_and_sync_venv
            install_common_embodied_deps
            install_robotwin_env
            _install_fastwam_model_finish
            ;;
        "")
            create_and_sync_venv
            install_common_embodied_deps
            _install_fastwam_model_finish
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for FastWAM model." >&2
            exit 1
            ;;
    esac
}

install_qwen3_vl_model() {
    create_and_sync_venv
    install_common_embodied_deps

    case "$ENV_NAME" in
        maniskill_libero|libero)
            install_${ENV_NAME}_env
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for Qwen3-VL model." >&2
            exit 1
            ;;
    esac

    uv pip install --upgrade "transformers>=4.57.1,<=4.57.6" "tokenizers>=0.22,<0.23"

    install_flash_attn
}

install_franka_realworld_env() {
    uv sync --extra franka --active $NO_INSTALL_RLINF_CMD
    if [ "$SKIP_ROS" -ne 1 ]; then
        if [ "$NO_ROOT" -eq 0 ]; then
            bash $SCRIPT_DIR/embodied/ros_install.sh
        fi
        install_franka_env
    fi
}

install_env_only() {
    if [ "$ENV_NAME" = "d4rl" ]; then
        PYTHON_VERSION="3.10"
    fi
    create_and_sync_venv
    SKIP_ROS=${SKIP_ROS:-0}
    case "$ENV_NAME" in
        d4rl)
            install_d4rl_env
            ;;
        dummy)
            install_dummy_env
            ;;
        franka)
            install_franka_realworld_env
            ;;
        franka-dexhand)
            install_franka_realworld_env
            install_franka_dexhand_deps
            ;;
        xsquare_turtle2)
            uv sync --extra xsquare_turtle2 --active $NO_INSTALL_RLINF_CMD
            install_xsquare_turtle2_env
            ;;
        habitat)
            install_common_embodied_deps
            install_habitat_env
            ;;
        embodichain)
            install_common_embodied_deps
            install_embodichain_env
            ;;
        gim_arm)
            uv sync --extra gim_arm --active $NO_INSTALL_RLINF_CMD
            ;;
        dosw1)
            install_dosw1_env
            ;;
        *)
            echo "Environment '$ENV_NAME' is not supported for env-only installation." >&2
            exit 1
            ;;
    esac
}

#=======================ENV INSTALLERS=======================

install_dummy_env() {
    uv sync --extra embodied --active $NO_INSTALL_RLINF_CMD
}

install_libero_env() {
    # Prefer an existing checkout if LIBERO_PATH is provided; otherwise clone into the venv.
    local libero_dir
    libero_dir=$(clone_or_reuse_repo LIBERO_PATH "$VENV_DIR/libero" https://github.com/RLinf/LIBERO.git)

    uv pip install -e "$libero_dir"
    echo "export PYTHONPATH=$(realpath "$libero_dir"):\$PYTHONPATH" >> "$VENV_DIR/bin/activate"
}

install_maniskill_libero_env() {
    install_libero_env
    uv pip install git+${GITHUB_PREFIX}https://github.com/haosulab/ManiSkill.git@v3.0.0b22

    # Maniskill assets
    bash $SCRIPT_DIR/embodied/download_assets.sh --assets maniskill
}

install_d4rl_env() {
    # Install base embodied dependencies first (gym/gymnasium/transformers stack).
    uv sync --extra embodied --active $NO_INSTALL_RLINF_CMD

    uv pip install "cython<3.0"
    uv pip install "gym==0.23.1"
    uv pip install "d4rl @ git+${GITHUB_PREFIX}https://github.com/Dps799/D4RL@master"

    # Install MuJoCo 2.1.0 native library (mujoco-py only provides Python bindings).
    local mujoco_root="${MUJOCO_PATH:-$HOME/.mujoco}"
    local mujoco_dir="$mujoco_root/mujoco210"
    if [ -f "$mujoco_dir/bin/libmujoco210.so" ]; then
        echo "[install_d4rl_env] MuJoCo 2.1.0 already installed at $mujoco_dir, skipping download."
    else
        echo "[install_d4rl_env] Downloading and extracting MuJoCo 2.1.0..."
        mkdir -p "$mujoco_root"
        local tmpdir archive url extracted
        tmpdir=$(mktemp -d)
        archive="$tmpdir/mujoco210.tar.gz"
        if [ -n "$GITHUB_PREFIX" ]; then
            url="${GITHUB_PREFIX}github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz"
        else
            url="https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz"
        fi
        echo "[install_d4rl_env] URL: $url"
        download_ok=0
        if command -v wget &>/dev/null; then
            wget --progress=bar:force --timeout=120 --tries=3 -O "$archive" "$url" && download_ok=1
        elif command -v curl &>/dev/null; then
            curl -fSL --connect-timeout 120 --max-time 600 --retry 3 -o "$archive" "$url" && download_ok=1
        else
            echo "Neither wget nor curl found. Please install one to download MuJoCo." >&2
            rm -rf "$tmpdir"
            exit 1
        fi
        if [ "$download_ok" -ne 1 ]; then
            echo "[install_d4rl_env] Download failed. Try without --use-mirror, or download manually:" >&2
            echo "  $url" >&2
            rm -rf "$tmpdir"
            exit 1
        fi
        tar -xzf "$archive" -C "$tmpdir"
        extracted=$(find "$tmpdir" -mindepth 1 -maxdepth 1 -type d | head -1)
        if [ -n "$extracted" ] && [ -d "$extracted" ]; then
            mv "$extracted" "$mujoco_dir"
        else
            echo "[install_d4rl_env] Unexpected tarball layout. Expected a single top-level directory." >&2
            ls -la "$tmpdir" >&2
            rm -rf "$tmpdir"
            exit 1
        fi
        rm -rf "$tmpdir"
        echo "[install_d4rl_env] MuJoCo 2.1.0 installed at $mujoco_dir"
    fi
    if ! grep -q "mujoco210/bin" "$VENV_DIR/bin/activate" 2>/dev/null; then
        echo "export LD_LIBRARY_PATH=\"${mujoco_dir}/bin:\$LD_LIBRARY_PATH\"" >> "$VENV_DIR/bin/activate"
    fi

    uv pip install "mujoco-py==2.1.2.14"
    uv pip install "tqdm"
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

install_franka_dexhand_deps() {
    uv pip install "RLinf-dexterous-hands[glove]"
}

install_xsquare_turtle2_env() {
    uv pip install git+${GITHUB_PREFIX}https://github.com/RLinf/xsquare_turtle_basics.git
}

install_robotwin_env() {
    # Set TORCH_CUDA_ARCH_LIST based on the CUDA version
    local cuda_mm cuda_major cuda_minor
    cuda_mm=$(detect_cuda_major_minor) || {
        echo "Could not detect CUDA version. Cannot build robotwin environment." >&2
        exit 1
    }
    cuda_major="${cuda_mm%% *}"
    cuda_minor="${cuda_mm##* }"
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

install_embodichain_env() {
    uv pip install embodichain --extra-index-url http://pyp.open3dv.site:2345/simple/ --trusted-host pyp.open3dv.site
}

install_dosw1_env() {
    # Reuse the standard embodied extra so dosw1 picks up the same
    # transformers/imageio/gymnasium dependency set as other embodied envs.
    uv sync --extra embodied --active $NO_INSTALL_RLINF_CMD
    # The default patch_syncer uses nvcomp_lz4. Keep DOSW1 lightweight by
    # installing only this shared compression runtime instead of the full
    # common simulator dependency set.
    uv pip install nvidia-nvcomp-cu12
    uv pip install evdev opencv-python

    # Install DOSW1 SDK. The wheel / airbot_api source are pre-deployed on the
    # DOS-W1 robot under ~/dos_w1/airbot by default; on a generic server they
    # are usually absent. Users may override the paths via env vars:
    #   DOSW1_SDK_WHEEL  - path to airbot_py-*.whl
    #   DOSW1_API_PATH   - path to the airbot_api source tree
    # If the paths are missing, we skip the SDK install with a warning so the
    # rest of the env still gets set up (e.g. for server-side training runs
    # that talk to the robot over gRPC and do not need the local SDK).
    local dosw1_sdk_wheel="${DOSW1_SDK_WHEEL:-$HOME/dos_w1/airbot/5.1.6/airbot_py-5.1.6-py3-none-any.whl}"
    local dosw1_api_path="${DOSW1_API_PATH:-$HOME/dos_w1/airbot/airbot_api}"

    if [ -f "$dosw1_sdk_wheel" ]; then
        uv pip install "$dosw1_sdk_wheel"
    else
        echo "[dosw1] WARNING: DOSW1 SDK wheel not found at '$dosw1_sdk_wheel'." >&2
        echo "[dosw1] WARNING: Skipping 'airbot_py' install. Set DOSW1_SDK_WHEEL to the wheel path if you need the local SDK." >&2
    fi

    if [ -d "$dosw1_api_path" ]; then
        uv pip install -e "$dosw1_api_path"
    else
        echo "[dosw1] WARNING: DOSW1 airbot_api source not found at '$dosw1_api_path'." >&2
        echo "[dosw1] WARNING: Skipping 'airbot_api' install. Set DOSW1_API_PATH to the source directory if you need the local SDK." >&2
    fi

    local repo_root
    repo_root="$(dirname "$SCRIPT_DIR")"
    uv pip install -e "$repo_root" --no-deps
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

    # xformers 0.0.29.post2 only has wheels for torch<=2.5, but we pin
    # torch==2.6.0. UV_TORCH_BACKEND=auto rejects mismatched torch-version
    # labels, so unset UV_TORCH_BACKEND entirely for this install so uv
    # picks the non-CUDA wheel without torch-version filtering.
    env -u UV_TORCH_BACKEND uv pip install "xformers==0.0.29.post2"

    # Install remaining opensora dependencies (xformers handled above).
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
    uv pip install "mujoco==3.3.7" "dm-control==1.0.34" --force-reinstall
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
    validate_python_version
    configure_platform
    setup_mirror
    apply_torch_override

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
            elif [ "$MODEL" != "dreamzero" ] && [ "$MODEL" != "fastwam" ] && [ "$MODEL" != "aupi" ]; then
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
                fastwam)
                    install_fastwam_model
                    ;;
                qwen3_vl)
                    install_qwen3_vl_model
                    ;;
                aupi)
                    install_aupi_model
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

    install_platform_extras
    unset_mirror
}

main "$@"
