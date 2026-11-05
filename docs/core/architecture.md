# Architecture

Bolt Runtime is one process with one policy loop. Simulation and hardware differ
only in the backend object the loop talks to.

import {ArchitectureFigure} from '@site/src/components/Figures';

<ArchitectureFigure />

## The policy loop

`PolicyLoop` owns everything that is the same for every robot and task:

- its own clock at `--rate` (default 50 Hz) and deadline accounting;
- the control modes: **hold** (startup default), **init** (move to the policy's
  default pose over `--init-steps`) and **policy**;
- the observation graph, inference, action mapping and the joint command;
- terminal input for policy keys, recording and timings.

It knows nothing about motion, MuJoCo or SDKs. Task-specific behavior lives in
a subclass selected with `--task`; see [Tasks](../tasks/overview.md).

## Process and threads

| Thread | Owner | Rate |
|---|---|---|
| Policy loop | `PolicyLoop` | policy rate |
| Physics | MuJoCo backend (online mode) | `physics_dt`, independent of the policy |
| Command writer + watchdog | hardware backends | `control_rate_hz` (e.g. 500 Hz) |
| Viewer | MuJoCo backend | display rate |

There is no simulator process and no robot bridge. Teleop is the only optional
second process, and it only sends reference motion over ZMQ.

## Decoupling rules

These boundaries keep sim and real interchangeable:

- Backends expose only `start`, `read_state`, `submit` and `stop`. No key events,
  pause or reset flow between a backend and the loop.
- Pausing or resetting physics in the viewer never touches policy history or
  motion playback; terminal `]` re-enters the policy explicitly.
- The robot config owns dynamics and torque limits; the policy config owns PD
  gains, default pose and action scale. A policy never overrides robot physics.
- The base loop never depends on motion; tracking code stays in `tasks/track/`.

## Source layout

```text
apps/boltrt.cpp            CLI and assembly
include/boltrt/            public headers, mirroring src/
src/
  config/                  robot / policy / backend YAML, hf:// asset resolution
  observation/             Observation interface, registry, graph, generic terms
  runtime/                 PolicyLoop, task registry, terminal UI, recording
  backend/                 RobotBackend registry and shared hardware scheduler
    mujoco/  g1/  rp1/  replay/
  tasks/track/             tracking task: loop, motion sources, policy-family observations
  utils/                   ONNX inference, MuJoCo model loading and FK
configs/robots/            builtin robots
configs/backends/          simulation backend (mujoco)
configs/calibration/       machine-specific SDK calibration (git-ignored)
```
