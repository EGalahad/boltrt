# Add an observation

Adding observation terms is how a new policy family is supported.

## 1. Decide where it lives

| Term needs | Location | Registration |
|---|---|---|
| Robot state only | `src/observation/` | `register_common_observations` |
| Reference motion | `src/tasks/track/observation/<family>.cpp` | the family's `register_*_observations`, called from `registry.cpp` |

Name tracking terms `<family>.<term>` so families never collide.

## 2. Implement it

```cpp
class JointPosScaled final : public boltrt::Observation {
public:
  explicit JointPosScaled(float scale) : scale_(scale) {}
  boltrt::Tensor compute(const boltrt::ObservationContext &c) const override {
    boltrt::Vec out(c.state.q.size());
    for (size_t i = 0; i < out.size(); ++i)
      out[i] = scale_ * c.state.q[i];
    return boltrt::Tensor::floats(out);
  }
private:
  float scale_;
};
```

Use `update` for per-tick state such as history buffers and `reset` to clear it
on policy entry. A complete standalone example is in
`examples/custom_observation.cpp`.

## 3. Register it and validate its keys

```cpp
r.add("my_family.joint_pos_scaled", [](const YAML::Node &cfg, const ObservationContext &) {
  require_keys(cfg, {"_target_", "scale", "noise_std"}, "my_family.joint_pos_scaled");
  return std::make_unique<JointPosScaled>(cfg["scale"].as<float>());
});
```

Allow training-only keys (`noise_std`, randomization ranges, ...) in the key list
so exports load, but never apply them at deployment.

New source files must be added to the matching `CMakeLists.txt`.

## 4. Use and verify it

```yaml
observation:
  policy:
    joint_pos:
      _target_: my_family.joint_pos_scaled
      scale: 0.5
```

1. `boltrt ... --validate` checks the graph against the ONNX inputs.
2. `boltrt ... --headless --start --steps 100 --record out.jsonl` records every
   observation component per tick.
3. Compare components against your training environment from the **same state**,
   term by term, not just the final action. Conventions to check: joint order,
   quaternion order (`wxyz`) and normalization, body vs world frame, yaw
   alignment, history order and spacing, units, and precision.
