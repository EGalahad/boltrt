# Installation

Bolt Runtime builds on Linux x86_64 and aarch64 with CMake 3.22+ and a C++20
compiler. Python is not needed to build or run it.

## Simulation build

```bash
sudo apt-get install cmake g++ curl libeigen3-dev libyaml-cpp-dev \
  zlib1g-dev libzmq3-dev libglfw3-dev libgl1-mesa-dev
./tools/fetch-deps.sh --sim-only
cmake --preset sim && cmake --build --preset sim --parallel
ctest --preset sim
export PATH="$PWD/build/sim:$PATH"
```

`fetch-deps.sh` downloads pinned MuJoCo and ONNX Runtime releases (and the robot
SDKs when requested) into `third_party/`, verified against
`dependencies.lock.json`.

## CMake presets and options

| Preset | Contents |
|---|---|
| `sim` | MuJoCo backend with native viewer |
| `ci` | `sim` without the viewer (headless machines) |
| `robots` | `sim` plus the G1 and RP1 hardware backends |

| Option | Default | Effect |
|---|---|---|
| `BOLTRT_VIEWER` | ON | Native MuJoCo viewer (needs OpenGL and GLFW) |
| `BOLTRT_G1` | OFF | Unitree G1 backend ([real robots](../real-robots.md)) |
| `BOLTRT_RP1` | OFF | RoboParty RP1 backend; links GPL-3.0 code ([licensing](../reference/licensing.md)) |

## Robot assets

Robot configs reference models such as `hf://elijahgalahad/g1_xmls@main/g1-mode_13_15.xml`.
On first use Bolt Runtime downloads the whole repository snapshot into the
standard Hugging Face cache (`~/.cache/huggingface/hub`), so later runs and the
Python tools share it. It calls the system `curl`; no Python is involved.

| Variable | Effect |
|---|---|
| `HF_HUB_OFFLINE=1` | Never download; fail if the asset is not cached |
| `HF_ENDPOINT` | Use a mirror instead of `https://huggingface.co` |
| `HF_TOKEN` | Token for private repositories (also read from the HF token file) |
| `HF_HUB_CACHE` / `HF_HOME` | Cache location |

On a robot without internet access, run once on a connected machine and copy
the cache, then set `HF_HUB_OFFLINE=1`.
