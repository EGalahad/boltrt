# Add an observation term

- Robot-state-only terms: `src/observation/`, registered in
  `register_common_observations`.
- Terms needing reference motion: `src/tasks/track/observation/<family>.cpp`,
  registered via the family's `register_*_observations` called from
  `registry.cpp`, and added to `src/tasks/track/CMakeLists.txt`.

Implement `Observation`: `compute` (required), `update` for per-tick state such as
history, `reset` to clear on policy entry. Read state only through
`ObservationContext`; tracking terms read the motion window from the task context.

Register with `require_keys` listing every accepted key. Include training-only
keys the export carries (`noise_std`, `bias_noise_std`, randomization ranges) so
loading succeeds, and ignore them. Unknown keys are an error by design.

Groups concatenate terms along the last dimension in YAML order; keep shapes and
dtypes exactly as the ONNX input expects.
