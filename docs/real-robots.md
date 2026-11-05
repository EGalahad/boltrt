# Real robots

:::warning
The G1 and RP1 backends build and pass `--validate`, but **have not yet been run
on physical robots**. Run `--validate` first, keep the robot suspended for the
first runs, and have an emergency stop ready.
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

## Check, then run

```bash
boltrt --task tracking --robot g1 --policy policy.yaml --backend g1 --validate
boltrt --task tracking --robot g1 --policy policy.yaml --backend g1
```

`--validate` loads configs, assets and the ONNX model and checks every binding
without opening an SDK session. The run itself is the same command as in
simulation with a different `--backend`.

## Robot and backend combinations

| Robot | `--robot` | `--backend` |
|---|---|---|
| Unitree G1 (29 DoF) | `g1` | `g1` |
| RoboParty RP1, 24 DoF | `rp1-24dof` | `rp1` |
| RoboParty RP1, 28 DoF | `rp1` | `rp1-28dof` |

`rp1-parallel` and `rp1-24dof-parallel` model the parallel ankle and are
simulation-only.

## Backend settings

```yaml
# configs/backends/g1.yaml
type: g1
interface: eth0            # network interface connected to the robot
domain_id: 0
control_rate_hz: 500       # command writer rate
command_timeout_s: 0.25    # watchdog: policy commands must arrive within this time
state_timeout_s: 0.1
startup_timeout_s: 5
release_motion_mode: true  # release Unitree's high-level motion service at startup
```

RP1 backends reference the SDK's robot calibration (`sdk_config`). The bundled
file is RoboParty's example calibration, not a measurement of your robot.
Copy the backend YAML, set your interface or CAN devices and calibration, and
pass the copy with `--backend path/to/backend.yaml`.

## Onboard computers

Bolt Runtime builds natively on aarch64 (e.g. Jetson Orin). Robots are often
offline: populate the Hugging Face cache once and set `HF_HUB_OFFLINE=1`
([assets](getting-started/installation.md#robot-assets)).
