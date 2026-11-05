# Tracking task

`--task tracking` runs `TrackingLoop`, which extends the base loop with a
reference motion that the policy tracks.

```bash
boltrt --task tracking --robot g1 --backend mujoco \
  --policy policy.yaml --motion motions/walk1_subject1.npz
```

## What it adds

- **Motion source**: an NPZ file ([Motion](motion.md)), a ZMQ teleop stream
  ([Teleop](teleop.md)), or, without `--motion`, a static pose at the policy's
  default joint positions.
- **Motion playback**: play, pause and replay are owned by the tracking task.
  File references are sampled on the policy clock, so frames stay aligned even
  when physics is paused.
- **Tracking observations** for every supported family
  ([Supported policies](policies.md)).
- Reference frames in `--record` output.

## Controls

Three independent controllers, each with its own keys:

| Controls | Where | Keys |
|---|---|---|
| Motion | terminal | `Space` play / pause reference |
| Policy | terminal | `I` init pose · `O` hold · `]` enter policy (resets history, motion paused at frame 0) · `Q` quit |
| Physics | MuJoCo window | `Space` pause · `Backspace` reset · `F1` help |

Typical sequence: `I` → `]` → `Space`. `--start` enters the policy and plays the
motion immediately, which is convenient for headless runs.

Pausing the reference keeps the policy running: it holds the current reference
frame. Pausing or resetting physics leaves motion and policy state untouched;
press `]` to re-enter the policy after a physics reset.
