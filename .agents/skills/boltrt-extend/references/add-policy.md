# Deploy a policy

## Same family already supported

1. Export `policy.onnx` + `policy.yaml`. Put ONNX bindings in the YAML:
   `onnx: {in_keys: [...], out_keys: [...]}`.
2. Use builtin robot and backend names; do not bundle robot XML, meshes or motions.
3. Set `compatible_robots` if the policy only fits one robot layout.
4. `boltrt --task tracking --robot <r> --policy policy.yaml --backend mujoco --validate`.

## New policy family

1. List every observation term of the training environment with its exact
   semantics: frame, units, history order and spacing, normalization, clipping.
2. Reuse generic terms (`joint_pos`, `joint_vel`, `prev_action`,
   `projected_gravity`, `root_ang_vel`) where they match exactly; otherwise add
   `<family>.<term>` terms (`add-observation.md`).
3. Check the model I/O: dtypes (`float32`/`int64`/`bool`), recurrent or KV-cache
   state, extra outputs, what must reset on policy entry.
4. Prove parity from the same state (`verify.md`), component by component.
5. Add the family to `docs/tasks/track/policies.md`.
