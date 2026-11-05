# Quick start

This page runs everything in simulation (sim2sim). For hardware see
[Real robots](../real-robots.md).

## 1. Smoke test the base loop

The repository ships a zero-action test policy. It only checks that the build,
robot model and viewer work; the robot will not balance.

```bash
boltrt --robot g1 --policy examples/base-policy.yaml --backend mujoco
```

## 2. Track a motion with a trained policy

Download a checkpoint, for example `mimic-lite/v1_1`, from the
[checkpoint folder](https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e).
A deploy checkpoint is just `policy.onnx` + `policy.yaml`.

```bash
boltrt --task tracking --robot g1 --backend mujoco \
  --policy mimic-lite/v1_1/policy.yaml \
  --motion motions/walk1_subject1.npz
```

The motion must be retargeted for the same robot. Without `--motion`, the policy
tracks a static reference at its default pose.

## 3. Drive it

With the **terminal** focused:

| Key | Action |
|---|---|
| `I` | Move toward the policy's default pose |
| `]` | Enter the policy; the reference waits at frame 0 |
| `Space` | Play / pause the reference motion |
| `O` | Hold the current pose |
| `Q` | Quit |

With the **MuJoCo window** focused, its native shortcuts apply: `Space` pauses
physics, `Backspace` resets physics, `F1` shows help. Motion, policy and physics
are controlled independently; see [Tracking task](../tasks/track/overview.md).

## 4. Batch and measure

```bash
boltrt --task tracking --robot g1 --backend mujoco \
  --policy mimic-lite/v1_1/policy.yaml --motion motions/walk1_subject1.npz \
  --headless --start --run-once --no-keyboard \
  --record outputs/run.jsonl --timings outputs/latency.csv
```

`--headless` drops the window and real-time pacing; `--record` writes state,
observations, actions and commands per tick; `--timings` writes per-tick latency.
`--validate` checks configs, assets and ONNX bindings without running.
