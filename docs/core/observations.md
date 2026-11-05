# Observations

Observations are what make Bolt Runtime policy-agnostic. Each observation term
is a small C++ class registered under a name; a policy's YAML composes terms by
name into the tensors its ONNX model expects. Supporting a new policy family
means adding its terms, not changing the loop.

## Composition in YAML

```yaml
observation:
  policy:                       # group → one ONNX input
    root_ang_vel_history:
      _target_: mimic_lite.root_ang_vel_history
      history_steps: [0, 1, 2, 3, 4, 8, 16]
    joint_pos_history:
      _target_: mimic_lite.joint_pos_history
      history_steps: [0, 1, 2, 3, 4, 8, 16]
    prev_actions:
      _target_: mimic_lite.prev_actions
      steps: 3
  command:                      # another group → another input
    ref_joint_pos_future:
      _target_: mimic_lite.ref_joint_pos_future
      future_steps: [-8, -4, -2, 0, 1, 2, 3, 4]
```

- Each **group** becomes one tensor; its terms are concatenated along the last
  dimension in YAML order. Groups are bound to ONNX inputs by `onnx.in_keys`.
- `_target_` names a registered term; the remaining keys are its parameters.
- Tensors keep their shape and dtype (`float32`, `int64`, `bool`).
- Each term validates its keys. Unknown keys are an error, so typos fail fast;
  training-only keys such as `noise_std` are accepted and ignored
  ([details](configuration.md#training-only-fields)).

## Available terms

`boltrt --list-observations` prints the registered names. The base loop has
generic proprioception: `joint_pos`, `joint_vel`, `prev_action`,
`projected_gravity`, `root_ang_vel`. `--task tracking` adds the reference-motion
terms of every supported family, namespaced by family (`mimic_lite.*`,
`sonic.*`, `holomotion.*`, ...).

## The interface

```cpp
class Observation {
public:
  virtual void reset(const ObservationContext &) {}   // on policy entry
  virtual void update(const ObservationContext &) {}  // once per tick, e.g. history
  virtual Tensor compute(const ObservationContext &) const = 0;
};
```

`ObservationContext` gives read access to the robot spec, policy config, current
robot state, previous action, previous model outputs, and an optional task
context (the tracking task uses it to expose the reference motion window).
Generic terms never see task types.

Registration:

```cpp
registry.add("my_policy.joint_pos_scaled", [](const YAML::Node &cfg, const ObservationContext &) {
  return std::make_unique<JointPosScaled>(cfg["scale"].as<float>());
});
```

Step-by-step: [Add an observation](../extending/add-observation.md).
