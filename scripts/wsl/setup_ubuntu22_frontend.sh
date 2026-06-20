#!/usr/bin/env bash
set -euo pipefail

# Setup a WSL Ubuntu 22.04 Python environment for full EuRoC frontend tests.
# Run from the project root inside WSL:
#   bash scripts/wsl/setup_ubuntu22_frontend.sh

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_ROOT"

sudo apt-get update
sudo apt-get install -y \
  python3-venv \
  python3-pip \
  git \
  wget \
  unzip \
  libgl1 \
  libglib2.0-0 \
  libsm6 \
  libxext6 \
  libxrender1

python3 -m venv .venv-wsl
source .venv-wsl/bin/activate
python -m pip install -U pip setuptools wheel

# CUDA wheel. WSL sees the Windows NVIDIA driver; no Linux CUDA toolkit is required for PyTorch runtime.
python -m pip install --index-url https://download.pytorch.org/whl/cu128 torch torchvision
python -m pip install -e ".[dev]"
python -m pip install git+https://github.com/cvg/LightGlue.git

python - <<'PY'
import torch
print("torch:", torch.__version__)
print("cuda:", torch.cuda.is_available())
print("device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "none")
PY

echo "WSL frontend environment ready. Activate with: source .venv-wsl/bin/activate"
