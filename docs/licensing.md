# Licensing

Boltrt's project-owned core, adapters and examples are **Apache-2.0**. See
[LICENSE](../LICENSE) and [NOTICE](../NOTICE). Dependencies and downloaded robot
assets keep their own licenses; this project does not relicense them.

## Optional RP1 integration

`BOLTRT_RP1=OFF` is the default. Enabling it links RoboParty's
[roboparty_deploy](https://github.com/Roboparty/roboparty_deploy) and its
inference, motors and imu submodules, treated here as **GPL-3.0-only**.
Distribute the resulting linked executable under GPL-3.0, with its corresponding
source, build instructions and upstream notices. The Apache-2.0 license of
independent core source files does not change. The GPL text is retained in
[licenses/GPL-3.0.txt](licenses/GPL-3.0.txt).

No RP1-enabled prebuilt executable is committed, uploaded or published here.
CI builds and tests simulation only and does not download hardware SDKs.

## Dependencies

| Component | Upstream license | Use |
|---|---|---|
| MuJoCo | Apache-2.0 | Physics, kinematics, optional native viewer |
| ONNX Runtime | MIT | CPU inference |
| Unitree SDK2 | BSD-3-Clause | Optional G1 adapter |
| RoboParty deploy / inference / motors / imu | GPL-3.0-only | Optional RP1 adapter |
| Eigen | MPL-2.0 | Linear algebra |
| yaml-cpp | MIT | Configuration |
| nlohmann/json | MIT | Messages and recording |
| libzmq | MPL-2.0 | Teleop reference input |
| GLFW | zlib/libpng | Optional viewer |
| LodePNG | zlib | Optional rendering utility |
| zlib | zlib | NPZ decompression |
| Boost headers | BSL-1.0 | Optional RP1 dependency |
| fmt / spdlog | MIT | Optional RP1 dependencies |

`tools/fetch-deps.sh` preserves upstream license files and source headers.
SDK repositories are checked out unchanged at revisions in
[dependencies.lock.json](../dependencies.lock.json). Their license files,
including submodule notices, remain in the downloaded trees. Unitree's notice
is also retained in [licenses/unitree_sdk2.txt](licenses/unitree_sdk2.txt).
Robot meshes and checkpoints are downloaded separately and remain subject to
their upstream terms. See those distributions before redistributing them.
