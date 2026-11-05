# Add a backend

1. `src/backend/<name>/backend.cpp` + `CMakeLists.txt`; keep SDK includes,
   flags (e.g. an older C++ standard for SDK headers) and linking local.
2. Implement `RobotBackend` and register with `BackendRegistry::add`; add a
   `BOLTRT_<NAME>` CMake option if it needs an external SDK, default OFF.
3. Hardware: reuse the inline hardware scheduler (high-rate writer, command and
   state watchdog). Map motors by joint name.
4. Hardware: a `hardware` section in the robot config (`driver: <name>` plus device settings), selected with `--backend real`. Machine calibration goes in git-ignored `configs/calibration/`. Simulation backends: `configs/backends/<name>.yaml` with `type: <name>`.
5. Fetch SDKs from the vendor's official repository at a pinned commit; never
   copy an SDK from another project. Check its license before linking (GPL code
   makes the linked binary GPL).
6. Verify the sim-only build still passes; hardware is compile + `--validate`
   until run on a robot.
