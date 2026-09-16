#!/bin/bash
# 在 GPU 容器 (maniskill_libero) 内创建 4dwvla venv.
# 前置: 容器已启动, /workspace/4WVLA 已挂载.
set -euo pipefail

VENV_DIR="/opt/venv/4dwvla"
PYTHON_VERSION="3.11"
UV="/opt/venv/.cache/uv/uv"  # uv binary 的实际位置

# 如果 uv 不在上述位置, 尝试 PATH
if [[ ! -x "${UV}" ]]; then
    UV=$(which uv 2>/dev/null || echo "")
    if [[ -z "${UV}" ]]; then
        echo "ERROR: uv not found. Expected at /opt/venv/.cache/uv/uv" >&2
        exit 1
    fi
fi

echo "=== Using uv: ${UV} ($(${UV} --version)) ==="

# Step 1: 创建 venv
if [[ -d "${VENV_DIR}" ]]; then
    echo "WARNING: ${VENV_DIR} already exists. Skipping creation."
else
    echo "=== Creating venv at ${VENV_DIR} ==="
    ${UV} venv --python "${PYTHON_VERSION}" "${VENV_DIR}"
fi

# Step 2: 激活 venv
export VIRTUAL_ENV="${VENV_DIR}"
export PATH="${VENV_DIR}/bin:${PATH}"

PYTHON="${VENV_DIR}/bin/python"
PIP="${UV} pip"

echo "=== Python: $(${PYTHON} --version) ==="

# Step 3: 安装 PyTorch (CUDA 12.8)
echo "=== Installing PyTorch ==="
${PIP} install torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu128

# Step 4: 安装 transformers (4DWVLA 需要 5.2.0)
echo "=== Installing transformers ==="
${PIP} install transformers==5.2.0

# Step 5: 安装其他依赖
echo "=== Installing other dependencies ==="
${PIP} install \
    'accelerate>=1.5.0' \
    'pillow>=10.0' \
    numpy==1.26.4 \
    'scipy>=1.10' \
    'draccus>=0.10' \
    einops \
    timm \
    'peft>=0.11' \
    datasets \
    safetensors

# Step 6: 安装 flash-attn (编译安装, 可能需要几分钟)
echo "=== Installing flash-attn ==="
${PIP} install flash-attn==2.8.3 --no-build-isolation 2>/dev/null || \
    echo "WARNING: flash-attn build failed; model will use eager attention (slower)"

# Step 7: 安装 4DWVLA 包 (editable mode)
echo "=== Installing 4DWVLA (lerobot) ==="
if [[ -d "/workspace/4WVLA" ]]; then
    ${PIP} install -e /workspace/4WVLA
else
    echo "ERROR: /workspace/4WVLA not mounted" >&2
    exit 1
fi

# Step 8: Patch transformers with Qwen3.5 model code
echo "=== Patching transformers ==="
TRANSFORMERS_DIR=$(${PYTHON} -c "import transformers, pathlib; print(pathlib.Path(transformers.__file__).parent)")
for subdir in \
    src/lerobot/policies/pi0/transformers_replace/models \
    src/lerobot/policies/pi05/transformers_replace/models \
    src/lerobot/policies/internvla_a1_5/transformers_replace/models; do
    src="/workspace/4WVLA/${subdir}"
    if [[ -d "${src}" ]]; then
        cp -r "${src}"/* "${TRANSFORMERS_DIR}/models/" 2>/dev/null || true
        echo "  Patched from ${subdir}"
    fi
done

# Step 9: 验证
echo "=== Verification ==="
${PYTHON} -c "
import torch
print(f'torch {torch.__version__}, CUDA available: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'  GPU: {torch.cuda.get_device_name(0)}, {torch.cuda.get_device_properties(0).total_mem // 1024**2} MiB')
import transformers
print(f'transformers {transformers.__version__}')
from lerobot.transforms.core import compose, NormalizeTransformFn, UnNormalizeTransformFn
print('lerobot transforms: OK')
from lerobot.policies.internvla_a1_5.configuration_internvla_a1_5 import InternVLAA15Config
print('InternVLA-A1.5 config: OK')
"

echo ""
echo "=== Setup complete ==="
echo "Activate with: source ${VENV_DIR}/bin/activate"
echo "Or use: ${VENV_DIR}/bin/python"
