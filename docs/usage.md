# Usage

Boltrt runs on Linux x86_64 and aarch64. Native dependencies are pinned in
`dependencies.lock.json`; Python is not part of the executable or build.

## Assets

Download deploy artifacts from the shared
[Google Drive checkpoint folder](https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e).
Keep each export's ONNX, YAML and any original JSON sidecar together. New exports
can embed `onnx.in_keys/out_keys` in YAML and need only ONNX + YAML.

Robot configs resolve `hf://` assets from the local Hugging Face cache. Cache
those assets once before offline deployment; the runtime does not download them.
See each `configs/robots/*.yaml` for its pinned repository and revision.
For G1, one optional asset setup command is:

```bash
uvx --from huggingface-hub hf download elijahgalahad/g1_xmls --revision main
```

This populates the shared HF cache; Python is not needed by the native runtime.
The base policy example uses a tiny zero-action ONNX test fixture, not a deploy
checkpoint. Actual policy checkpoints remain external downloads.
Download a robot-matched motion dataset for file tracking. Robot joint layouts
must match; G1 motions cannot be reinterpreted as RP1 motions.

## Task selection

Without `--task`, the application runs `PolicyLoop` with common state/action
observations and no motion. `--list-tasks` lists registered task names;
`--task tracking` selects `TrackingLoop` and registers tracking observations.
`--motion` belongs to that task and is rejected for the base loop.

## Tracking

```bash
boltrt --task tracking --robot g1 --policy policy.yaml --backend mujoco
boltrt --task tracking --robot g1 --policy policy.yaml --backend mujoco \
  --motion motions/walk1_subject1.npz --start --run-once
```

Without `--motion`, a robot tracking policy receives a static reference at its
default joint posture. SMPL policies require an explicit SMPL source. File motion
paths point to an NPZ below an any4hdmi `manifest.json`.

| Terminal key | Action |
|---|---|
| `I` | Initialize toward policy default pose |
| `O` | Hold current pose |
| `]` | Enter policy; reset reference and policy history; reference starts paused |
| `Space` | Play/pause reference only |
| `Q` / `Ctrl-C` | Exit |

MuJoCo shortcuts apply when its window has focus: `Space` pauses physics,
Backspace resets physics, and F1 shows native help. The native sidebars are
hidden initially. Neither viewer pause nor reset changes policy/motion state.
`--start` explicitly enters policy and plays the reference at startup.

`--headless` removes the window and real-time pacing. For unattended checks:

```bash
boltrt --task tracking --robot g1 --policy policy.yaml --backend mujoco \
  --motion motions/walk1_subject1.npz --headless --start \
  --steps 500 --no-keyboard --timings outputs/latency.csv
```

`policy_ms` includes reference sampling, observations, inference and command
construction/submission. `inference_ms` times the inference adapter. Both exclude
physics stepping, feedback reads, rendering, recording and deliberate sleeps.

`--validate` checks configuration, assets, observations and model bindings without
opening a hardware session. `--print-config`, `--list-robots`, `--list-backends`
and `--list-observations` expose available configuration.

## Experimental hardware

G1: `--robot g1 --backend g1`. RP1 24-DoF:
`--robot rp1-24dof --backend rp1`. RP1 28-DoF:
`--robot rp1 --backend rp1-28dof`. Parallel RP1 models are available for simulation
as `rp1-parallel` and `rp1-24dof-parallel`; they are not hardware driver layouts.

The adapters have compiled and passed `--validate`, but have **not been run on
physical robots**. Set the actual network/CAN interface and SDK calibration in
backend configuration before any physical use. RP1 example calibration is not a
measurement of your robot. See [licensing](licensing.md) before distributing an
RP1-enabled binary.

## Teleop

ZMQ is used only for reference input, never for robot state or commands:

```bash
boltrt --task tracking --robot g1 --policy policy.yaml --backend mujoco \
  --motion tcp://127.0.0.1:5555
```

Canonical JSON fields are `joint_names`, `body_names`, `joint_pos`, `body_pos_w`,
and `body_quat_w` (wxyz). Optional fields include `joint_vel`, `seq`, timestamps
and `motion_first_frame`. The source waits for valid input, keeps the final valid
reference after disconnect, and aligns segment XY/yaw continuity.
SONIC SMPL accepts its JSON or packed reference protocol; it does not solve or
retarget human pose inside the runtime.

## Rendering and numerical scope

The studio scene is private to `backend/mujoco/scene.cpp`; model loading/FK is in
`utils/mujoco/`. Rendering changes do not alter shared robot dynamics. The optional
`render_scene` developer target renders a recorded pose with the same scene.

Motion backing defaults to float16 for compatibility with existing exports;
observation arithmetic uses float32 or higher precision. Set
`motion.storage_dtype: float32` when required. Same-state observation parity is
separate from motion-loader parity, closed-loop tracking quality and hardware
validation. CUDA/TensorRT and the complete tracking benchmark are not supported.

Boltrt is a standalone C++ migration from Python sim2real. The original Python
repository and its documentation remain unchanged.
