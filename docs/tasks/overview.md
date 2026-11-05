# Tasks

A task decides what the loop does around the policy. The core
[`PolicyLoop`](../core/architecture.md#the-policy-loop) runs without any task;
a task subclasses it and adds its own inputs, keys and recording.

| Task | Flag | Adds |
|---|---|---|
| base | *(default)* | nothing: proprioception in, joint commands out |
| tracking | `--task tracking` | reference motion (file or teleop), playback keys, tracking observations |

`boltrt --list-tasks` lists registered tasks. Task options such as `--motion`
are only accepted with their task.

Tasks are registered in `apps/boltrt.cpp` through `TaskRegistry`, and each one
lives in its own directory under `src/tasks/`. A task may hook into the loop at
well-defined points (prepare, start, policy entry, per-tick update, key events,
status line, recording) and may expose a task context to its observations. See
[Add a task](../extending/add-task.md).
