# Architecture

`TaskRegistry` selects a task with `--task tracking`; without it the executable
runs the generic base loop. Each task registers its factories, observations and
CLI options with the application. One process, interchangeable robot backends,
and task-specific loops built on a
motion-independent `PolicyLoop`.

```text
apps/boltrt.cpp                  Configuration and dependency assembly
include/boltrt/                  Public interfaces, mirroring component names
src/
  backend/                      Shared registry and hardware scheduler
    mujoco/                     Physics worker, viewer, studio scene and rendering
    g1/                         Unitree adapter and CMake rules
    rp1/                        RoboParty adapter and CMake rules
    replay/                     Recorded robot state input
  observation/                  Generic registry, graph and reusable history
  runtime/                      Motion-independent PolicyLoop, terminal and recording
  tasks/
    track/                      Self-contained tracking target and dependencies
      loop.cpp                  TrackingLoop and task-specific recording/panel
      motion/                   File, static pose, ZMQ and SMPL sources
      observation/              Shared tracking helpers; one file per policy family
  utils/
    onnx_inference.cpp          Typed CPU ONNX binding and recurrent state
    mujoco/                     MJCF loading/FK shared by tracking and simulation
  config/                       Generic YAML loading and named robot layouts
```

`PolicyLoop` owns its clock, mode transitions, observation graph, inference and
joint commands. It has no motion source, playback or MuJoCo control dependency.
`TrackingLoop` inherits it and supplies task hooks for reference sampling,
playback keys and tracking recording. Generic observation contexts carry an optional type-erased task extension;
only the tracking module knows `TrackingContext` and `MotionWindow`.

All backends implement `start`, `read_state`, `submit` and `stop`. A closed
feedback session returns no state; a device fault raises an error. No backend
exposes policy keyboard events, simulation pause or reset methods.

Online MuJoCo physics runs independently at `physics_dt`, using the latest
command. Hardware adapters keep their own high-frequency writer and watchdog.
The policy clock does not depend on either. Non-real-time headless evaluation
uses deterministic command/state stepping inside the MuJoCo backend.

The viewer handles its own shortcuts. Pausing or resetting physics does not
pause reference playback or reset policy history. Terminal `]` explicitly
re-enters policy mode and resets tracking playback/history. File references are
sampled on the consumer clock for frame alignment; external teleop publishers
supply their own reference stream.

Robot dynamics and fixed torque limits belong to the robot/MJCF configuration.
Policy configuration owns PD gains, default pose, action scaling and observations.
Historical policy dynamics overrides are ignored with a message.

## Extending the runtime

Implement `Observation::compute` and optional `reset/update`, then register a
factory with `ObservationRegistry::add`. YAML selects it with `_target_`:

```yaml
observation:
  actor:
    joint_position:
      _target_: my_plugin.joint_position
```

Tracking-specific factories live in `tasks/track/observation/`; shared tracking
math stays within that module and generic history stays in `observation/`. A non-tracking plugin needs no motion.

To add a backend, implement `RobotBackend` and register its factory. Keep SDK
includes, lifecycle and build rules inside its own `backend/<name>/` directory.
To add another task domain, derive from `PolicyLoop`, bind an optional task
context, and override the task hooks; do not add its behavior to the base loop.

Tasks register a factory, an optional observation-registration callback and their
own CLI option names with `TaskRegistry::add` in the application. The base task
is built in; `register_tracking_task(tasks)` is the only tracking composition
entry point in `apps/boltrt.cpp`. Task-owned options cannot be used with another
task. Additional domains can use their own `src/tasks/<domain>/` target without
adding dependencies to `boltrt_core`.
