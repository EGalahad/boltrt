#!/usr/bin/env bash
# Build the XRoboToolkit SDK Python binding (xrobotoolkit_sdk) into this project's
# environment. Run `uv sync --project src/tasks/track/teleop` first.
set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DEPS="$HERE/third_party"
SERVICE_REPO=https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
SERVICE_REV=85bac4dbc1fd5cef42c74a160d9c30aa3491f122
PYBIND_REPO=https://github.com/YanjieZe/XRoboToolkit-PC-Service-Pybind.git
PYBIND_REV=75cb1130ac63e76d8f7e7788049be415e8be44f2

case "$(uname -m)" in
  x86_64|amd64) ARCH=x86_64 ;;
  aarch64|arm64) ARCH=aarch64 ;;
  *) echo "Unsupported architecture: $(uname -m)" >&2; exit 1 ;;
esac

fetch() {  # fetch <url> <rev> <dir>
  if [[ ! -d "$3/.git" ]]; then
    git clone --quiet "$1" "$3"
  fi
  git -C "$3" fetch --quiet origin "$2" 2>/dev/null || true
  git -C "$3" checkout --quiet --force "$2"
}

mkdir -p "$DEPS"
fetch "$SERVICE_REPO" "$SERVICE_REV" "$DEPS/XRoboToolkit-PC-Service"
fetch "$PYBIND_REPO" "$PYBIND_REV" "$DEPS/XRoboToolkit-PC-Service-Pybind"
SERVICE="$DEPS/XRoboToolkit-PC-Service"
PYBIND="$DEPS/XRoboToolkit-PC-Service-Pybind"
SDK="$SERVICE/RoboticsService/PXREARobotSDK"

if [[ "$ARCH" == aarch64 ]]; then
  GRPC="$SERVICE/RoboticsService/Redistributable/linux_aarch64"
  if [[ ! -d "$GRPC/grpc/include/google/protobuf/stubs" && -d "$GRPC/grpc.upstream/include/google/protobuf/stubs" ]]; then
    cp -r "$GRPC/grpc.upstream/include/google/protobuf/stubs" "$GRPC/grpc/include/google/protobuf/stubs"
  fi
fi

echo "[xrobot] building PXREARobotSDK ($ARCH)"
(cd "$SDK" && bash build.sh)

if [[ "$ARCH" == x86_64 ]]; then INC="$PYBIND/include"; LIB="$PYBIND/lib"; else INC="$PYBIND/include/aarch64"; LIB="$PYBIND/lib/aarch64"; fi
mkdir -p "$INC" "$LIB"
cp "$SDK/PXREARobotSDK.h" "$INC/"
rm -rf "$INC/nlohmann" && cp -r "$SDK/nlohmann" "$INC/nlohmann"
cp "$SDK/build/libPXREARobotSDK.so" "$LIB/"

# Upstream setup.py needs pybind11's CMake dir and accepts Python 3.8 on JetPack 5.
if ! grep -q "pybind11.get_cmake_dir" "$PYBIND/setup.py"; then
  sed -i 's/^from setuptools import Command/import pybind11\nfrom setuptools import Command/' "$PYBIND/setup.py"
  sed -i 's|"-DPYTHON_EXECUTABLE=" + sys.executable,|"-DPYTHON_EXECUTABLE=" + sys.executable,\n            "-Dpybind11_DIR=" + pybind11.get_cmake_dir(),|' "$PYBIND/setup.py"
  sed -i 's/python_requires=">=3.10"/python_requires=">=3.8"/' "$PYBIND/setup.py"
fi

PY="$HERE/.venv/bin/python"
[[ -x "$PY" ]] || { echo "Run 'uv sync --project src/tasks/track/teleop' first." >&2; exit 1; }
uv pip install --python "$PY" --no-build-isolation -e "$PYBIND"
"$PY" -c "import xrobotoolkit_sdk; print('[xrobot] xrobotoolkit_sdk installed')"
