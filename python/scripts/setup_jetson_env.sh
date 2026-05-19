#!/usr/bin/env bash
# Minimal Jetson env setup. Assumes:
#   - apt python3-opencv is installed (provides cv2.CAP_V4L2)
#   - .venv exists and was created with --system-site-packages
#   - onnxruntime-gpu is either already present, or will be installed
#     manually from Jetson AI Lab when the user wants GPU execution
#
# Run from anywhere:
#   chmod +x python/scripts/setup_jetson_env.sh
#   ./python/scripts/setup_jetson_env.sh
#
# The .venv is created at python/.venv (alongside requirements-jetson.txt),
# not at the top of the repo, because the C++ migration owns the repo root.

set -euo pipefail

PY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PY_ROOT"

if [[ ! -d ".venv" ]]; then
  echo "[setup] creating .venv with --system-site-packages"
  python3 -m venv --system-site-packages .venv
fi

if [[ ! -f ".venv/bin/activate" ]]; then
  echo "[setup] python/.venv exists but is incomplete."
  echo "[setup] Install python3.10-venv, then remove the partial venv and rerun:"
  echo "        rm -rf ${PY_ROOT}/.venv"
  exit 1
fi

# shellcheck disable=SC1091
source .venv/bin/activate

export PYTHONNOUSERSITE=1
if ! python -m pip --version >/dev/null 2>&1; then
  echo "[setup] pip is unavailable in python/.venv."
  echo "[setup] Install python3.10-venv/python3-pip, remove python/.venv, and rerun."
  exit 1
fi

python -m pip install --upgrade pip
python -m pip install -r requirements-jetson.txt

echo "[setup] verifying runtime..."
python - <<'PY'
import cv2
import numpy as np

print(f"cv2:    {cv2.__version__}  ({cv2.__file__})")
print(f"numpy:  {np.__version__}")

try:
    import onnxruntime as ort
except ModuleNotFoundError:
    print("ort:    not installed")
    print("providers: install the Jetson AI Lab onnxruntime-gpu wheel when GPU/ORT execution is needed")
else:
    print(f"ort:    {ort.__version__}")
    print(f"providers: {ort.get_available_providers()}")
PY

echo "[setup] done."
echo
echo "If you need CUDA / TensorRT execution providers, install the Jetson AI Lab"
echo "wheel after this script (see README)."
