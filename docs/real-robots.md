# Real robots

:::note
G1 and RP1 (serial ankles) run on hardware. RP1 parallel-ankle output
(`rp1-parallel`) has not been run on hardware yet. As with any new policy, run
`--validate` first and keep an emergency stop ready.
:::

## Build

```bash
sudo apt-get install libfmt-dev libspdlog-dev libboost-dev
./tools/fetch-deps.sh --g1 --rp1      # or only the one you need
cmake --preset robots && cmake --build --preset robots --parallel
```

The SDKs are cloned from their official repositories at the commits pinned in
`dependencies.lock.json` and are not modified. The G1 adapter compiles as C++17
for the SDK's DDS headers; the rest of the runtime is C++20. Enabling RP1 links
GPL-3.0 code ([licensing](reference/licensing.md)).

## Same command, `--backend real`

Simulation and hardware differ only in `--backend`:

```bash
boltrt --task tracking --robot rp1 --backend mujoco --policy policy.yaml --motion walk.npz
boltrt --task tracking --robot rp1 --backend real   --policy policy.yaml --motion walk.npz
```

`--backend real` takes the SDK driver and its parameters from the `hardware`
section of the robot config, so the robot name selects everything:

| Robot | `--robot` | Driver |
|---|---|---|
| Unitree G1 (29 DoF) | `g1` | Unitree SDK2 |
| RoboParty RP1, 28 DoF, serial ankles | `rp1` | RoboParty deploy SDK |
| RoboParty RP1, 28 DoF, parallel ankles | `rp1-parallel` | RoboParty deploy SDK, ankle motors commanded directly |
| RoboParty RP1, 24 DoF | `rp1-24dof` | RoboParty deploy SDK |

`--validate` loads configs, assets, calibration and the ONNX model and checks every
binding without opening a device session.

## Hardware section

```yaml
# configs/robots/rp1.yaml
hardware:
  driver: rp1
  calibration: ../calibration/rp1.yaml
  control_rate_hz: 200     # command writer rate
  command_timeout_s: 0.1   # watchdog: policy commands must arrive within this time
```

```yaml
# configs/robots/g1.yaml
hardware:
  driver: g1
  interface: eth0          # network interface connected to the robot
  domain_id: 0
  control_rate_hz: 500
  command_timeout_s: 0.25
  state_timeout_s: 0.1
  startup_timeout_s: 5
  release_motion_mode: true  # release Unitree's high-level motion service at startup
```

## RP1 calibration

The RP1 driver reads this robot's SDK calibration (motor zero offsets, signs,
joint-to-motor mapping, IMU extrinsics, ankle closed-chain indices) from
`configs/calibration/`. One file is shared by every config of the same machine:
`rp1` and `rp1-parallel` both use `configs/calibration/rp1.yaml`. Calibration files
are machine-specific and ignored by git. Start from the vendor template:

```bash
cp third_party/roboparty_deploy-official/src/inference/robots/rp1/robot.yaml configs/calibration/rp1.yaml
```

then set the zero offsets of your robot. The runtime refuses to start hardware with
the unmodified template. The closed-chain `robot.type` must be one the pinned SDK
knows: `rp1` (older calibrations may say `rp1.3`, which is the same solver; rename it).

### Parallel ankles

`rp1-parallel` models the physical ankle linkage: 40 model joints, of which 28 are
motors. The extra linkage joints are listed under `hardware.passive_joint_names`;
they are never measured or commanded and read as zero. With
`direct_ankle_motors: true` the policy commands the ankle motors directly and the
SDK closed-chain solver is disabled, using the same calibration file.

## Onboard computers

Bolt Runtime builds natively on aarch64 (e.g. Jetson Orin, RK3588). Robots are
often offline: populate the Hugging Face cache once and set `HF_HUB_OFFLINE=1`
([assets](getting-started/installation.md#robot-assets)). On the RP1's RK3588
(Cortex-A76 core), the full MimicLite policy step takes 0.79 ms against 2.04 ms in
the Python stack ([performance](reference/performance.md)).
