#!/usr/bin/env bash
set -euo pipefail
want_g1=0
want_rp1=0
for arg in "$@"; do
  case "$arg" in
    --sim-only) ;;
    --g1) want_g1=1 ;;
    --rp1) want_rp1=1 ;;
    --help) echo 'Usage: tools/fetch-deps.sh [--sim-only] [--g1] [--rp1]'; exit 0 ;;
    *) echo "Unknown option: $arg" >&2; exit 2 ;;
  esac
done
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
deps_dir="${project_dir}/third_party"
cache_dir="${BOLTRT_DOWNLOAD_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/boltrt/downloads}"
mkdir -p "$deps_dir/include/nlohmann" "$deps_dir/include/GLFW" "$deps_dir/lodepng" "$cache_dir"
# A checksum-verified cache is shared across clean checkouts; no SDK is fetched by default.
download() {
  local url="$1" sha="$2" target="$3" cached="$cache_dir/$(basename "$3")"
  if ! [[ -f "$cached" ]] || ! printf '%s  %s\n' "$sha" "$cached" | sha256sum --status -c -; then
    curl -fL --retry 2 -o "$cached.part.$$" "$url"
    printf '%s  %s\n' "$sha" "$cached.part.$$" | sha256sum -c -
    mv "$cached.part.$$" "$cached"
  fi
  cp "$cached" "$target"
}
checkout() {
  local url="$1" commit="$2" directory="$3"
  if [[ ! -d "${directory}/.git" ]]; then
    git init "${directory}"
    git -C "${directory}" remote add origin "${url}"
  fi
  if [[ "$(git -C "${directory}" remote get-url origin)" != "${url}" ]]; then
    echo "Unexpected SDK origin: ${directory}" >&2
    exit 1
  fi
  if [[ -n "$(git -C "${directory}" status --porcelain)" ]]; then
    echo "SDK has local changes; preserve them before dependency setup: ${directory}" >&2
    exit 1
  fi
  if ! git -C "${directory}" cat-file -e "${commit}^{commit}" 2>/dev/null; then
    git -C "${directory}" fetch --depth 1 origin "${commit}"
  fi
  git -C "${directory}" checkout --detach "${commit}"
}

if ((want_g1)); then
  checkout https://github.com/unitreerobotics/unitree_sdk2.git \
    8e1b927586712edd6a4c29d50dd52dff1e163439 "$deps_dir/unitree_sdk2-official"
fi
if ((want_rp1)); then
  checkout https://github.com/Roboparty/roboparty_deploy.git \
    e3d05946cd17c5f886c345f6cf502ee2b6ab7bde "$deps_dir/roboparty_deploy-official"
  git -C "$deps_dir/roboparty_deploy-official" submodule update --init --recursive src/inference src/motors src/imu
fi
case "$(uname -m)" in
  x86_64) ort_arch=x64; ort_sha=1fa4dcaef22f6f7d5cd81b28c2800414350c10116f5fdd46a2160082551c5f9b; mj_sha=4b9ba10f1b43fe77768c2354a9789e863972715ceed009e508c55195b35a83c6 ;;
  aarch64) ort_arch=aarch64; ort_sha=7c63c73560ed76b1fac6cff8204ffe34fe180e70d6582b5332ec094810241e5c; mj_sha=35141ba3e5d4cd3af14b9dc74cdb43c18128bb8379483c8da5fc3d2f630ff1ce ;;
  *) echo 'Supported platforms: Linux x86_64 / aarch64' >&2; exit 1 ;;
esac
ort_name="onnxruntime-linux-${ort_arch}-1.23.2"
download "https://github.com/microsoft/onnxruntime/releases/download/v1.23.2/${ort_name}.tgz" "$ort_sha" "$deps_dir/${ort_name}.tgz"
tar -xzf "$deps_dir/${ort_name}.tgz" -C "$deps_dir"
mj_name="mujoco-3.10.0-linux-$(uname -m)"
download "https://github.com/google-deepmind/mujoco/releases/download/3.10.0/${mj_name}.tar.gz" "$mj_sha" "$deps_dir/${mj_name}.tar.gz"
tar -xzf "$deps_dir/${mj_name}.tar.gz" -C "$deps_dir"
download "https://raw.githubusercontent.com/nlohmann/json/v3.12.0/single_include/nlohmann/json.hpp" "aaf127c04cb31c406e5b04a63f1ae89369fccde6d8fa7cdda1ed4f32dfc5de63" "$deps_dir/include/nlohmann/json.hpp"
download "https://raw.githubusercontent.com/nlohmann/json/v3.12.0/LICENSE.MIT" "46a65cffd1ea955132d95a8dd921640714a8d6b537d2e4e482d31145ae95b603" "$deps_dir/include/nlohmann/LICENSE.MIT"
download "https://raw.githubusercontent.com/zeromq/libzmq/v4.3.5/include/zmq.h" "af28f65648ab8fbf3c000bdf6395776249fc5d98d28c720f4350c61fb2d91b1a" "$deps_dir/include/zmq.h"
download "https://raw.githubusercontent.com/zeromq/libzmq/v4.3.5/LICENSE" "1f256ecad192880510e84ad60474eab7589218784b9a50bc7ceee34c2b91f1d5" "$deps_dir/include/ZMQ-LICENSE"
download "https://raw.githubusercontent.com/glfw/glfw/3.4/include/GLFW/glfw3.h" "aa370985f6b493bbe0358a36ab49f5780a6397c0209c6d98a62143dea595b73c" "$deps_dir/include/GLFW/glfw3.h"
download "https://raw.githubusercontent.com/glfw/glfw/3.4/LICENSE.md" "149704059b5d0bf551637e50042dd4de9c2cae921021f6636298911e3a5f9462" "$deps_dir/include/GLFW/LICENSE.md"
download "https://raw.githubusercontent.com/lvandeve/lodepng/7735153f542e54e94285dc5c1e7f196b02e15385/lodepng.h" "b7588f2a55033b362e7f1b244f84f9441c292a393824730f5a85ec7f78ddbe30" "$deps_dir/lodepng/lodepng.h"
download "https://raw.githubusercontent.com/lvandeve/lodepng/7735153f542e54e94285dc5c1e7f196b02e15385/lodepng.cpp" "0a2542830190ebf3eee53cc7b098b9442b439f5a6e7534410764bc7635d602b5" "$deps_dir/lodepng/lodepng.cpp"
# Only portable Boost headers are extracted; no package binaries are installed.
if ((want_rp1)) && ! [[ -f /usr/include/boost/lockfree/queue.hpp ]]; then
  mkdir -p "$deps_dir/boost-headers/extracted"
  download "https://archive.ubuntu.com/ubuntu/pool/main/b/boost1.83/libboost1.83-dev_1.83.0-2.1ubuntu3.2_amd64.deb" \
    519ecf2c64308527e15b6582955681d192a832250baa4bc424967aaf7d02d68f \
    "$deps_dir/boost-headers/boost-headers.deb"
  dpkg-deb --fsys-tarfile "$deps_dir/boost-headers/boost-headers.deb" | \
    tar -xf - -C "$deps_dir/boost-headers/extracted" ./usr/include ./usr/share/doc/libboost1.83-dev/copyright
fi
echo "Native dependencies ready (G1=$want_g1, RP1=$want_rp1). Runtime and CMake do not require Python."
