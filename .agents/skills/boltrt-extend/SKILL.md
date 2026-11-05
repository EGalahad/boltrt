---
name: boltrt-extend
description: Use when adding or changing a robot, policy checkpoint, observation term, backend, task or utility in Bolt Runtime (boltrt), e.g. "deploy my checkpoint", "add my robot", "my observation doesn't match training", "write a backend for my SDK", or when a policy behaves differently in boltrt than in training. Covers where code goes, ownership rules, known pitfalls and the verification required before a change is done.
---

# Extending Bolt Runtime

Bolt Runtime runs one `PolicyLoop`; MuJoCo and hardware differ only by backend.
A run composes `--robot`, `--policy`, `--backend` and an optional `--task`
(`tracking` adds reference motion). Docs: `docs/` (published at
https://egalahad.github.io/boltrt/).

## Ownership rules (never violate)

- **Robot** (`configs/robots/*.yaml` + MJCF) owns joint/body names, structure,
  mass/inertia, armature and torque limits.
- **Policy** (`policy.onnx` + `policy.yaml`) owns ONNX bindings, observations,
  PD gains, default pose, action scale/clip. It never overrides robot physics and
  never ships its own copy of the robot model or motions.
- **Backend** implements only `start / read_state / submit / stop`. No key
  events, pause or reset cross between a backend/viewer and the loop.
- **Tasks** subclass `PolicyLoop`; the base loop never depends on motion.
  Tracking code stays in `src/tasks/track/`.
- Joints and bodies are matched **by name**, never by index.
- Training-only fields (observation noise, domain randomization, curricula,
  physics overrides) are accepted so exports load, and **never applied**.

## Route the request

| Request | Read |
|---|---|
| Deploy a trained checkpoint / new policy family | `references/add-policy.md` |
| New observation term | `references/add-observation.md` |
| New robot or model revision | `references/add-robot.md` |
| New backend or SDK | `references/add-backend.md` |
| Numbers differ from training, robot falls, wrong asset, build errors | `references/pitfalls.md` |
| How to prove a change works | `references/verify.md` |

## Workflow

1. Find the closest existing implementation and copy its pattern:
   `src/tasks/track/observation/mimic_lite.cpp`, `configs/robots/g1.yaml`,
   `src/backend/mujoco/`, `examples/custom_observation.cpp`.
2. Make the smallest change that satisfies the request. Do not change key
   bindings, viewer behavior, scene appearance, CLI layers or other policies
   unless asked. When changing interaction, match the existing semantics first.
3. Verify with `references/verify.md`. Report what was verified and what was not
   (for hardware: compile and `--validate` only, unless run on a robot).
4. Update the matching page under `docs/` when behavior or configuration changes.
