# Add a backend

1. Create `src/backend/<name>/` with `backend.cpp` and `CMakeLists.txt`. Keep SDK
   includes, compile flags and linking inside this directory.
2. Implement `RobotBackend`: `start`, `read_state`, `submit`, `stop`. Map joints
   by name between the robot config and the SDK's motor order.
3. Register a factory with `BackendRegistry::add` and add a build option if the
   backend needs an external SDK.
4. Add `configs/backends/<name>.yaml` with `type: <name>`.

Hardware backends should reuse the shared inline hardware scheduler, which runs
the high-rate command writer and the command/state watchdog.

Do not expose key events, pause or reset through the backend: the policy loop
must not depend on which backend it runs on.
