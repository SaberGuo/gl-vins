#!/usr/bin/env bash
set -euo pipefail

python3 -m pip install --user -U pip
python3 -m pip install --user evo

echo "If evo is not on PATH, add this to ~/.bashrc:"
echo 'export PATH="$HOME/.local/bin:$PATH"'
