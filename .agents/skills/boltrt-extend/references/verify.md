# Verification

Run in order and report results with numbers.

1. **Validate**: `boltrt ... --validate` (configs, assets, observation graph vs
   ONNX bindings; opens no device).
2. **Rollout**: `boltrt ... --backend mujoco --headless --start --steps 200
   --no-keyboard --record out.jsonl`. The robot must not fall in a standing or
   walking reference.
3. **Same-state parity** (new family or changed term): feed identical robot
   states and reference windows to the training/original implementation and to
   boltrt; compare each observation component, model input and action. Report
   max abs difference per component. Expect ~1e-5; larger means a convention bug.
4. **Builds and tests**: `cmake --build --preset sim && ctest --preset sim`;
   if hardware code changed, also the `robots` preset.
5. **Regression**: for refactors, record a fixed rollout before and after and
   require identical state/action traces.
