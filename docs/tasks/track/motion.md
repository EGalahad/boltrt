# Motion

`--motion path/to/clip.npz` plays a retargeted motion clip.

## Format

Clips follow the any4hdmi layout: an NPZ per clip below a dataset directory with
a `manifest.json`. Clips store `qpos` (and optionally velocities); the loader
resamples to the policy rate and computes body poses and velocities with MuJoCo
forward kinematics on the robot model. Velocities are interpolated from
source-frame finite differences, matching the training pipeline.

- The motion must be retargeted for the **same robot**: G1 motions cannot drive
  an RP1 policy, and serial and parallel-ankle models are distinct robots.
- Joints are matched by name. The floating base is found by the model's
  freejoint name or canonical `root_*` fields.
- Motion data is stored as `float16` by default for compatibility with existing
  datasets; all observation math runs in `float32` or higher. Set
  `motion.storage_dtype: float32` in the policy YAML to keep full precision.

## Without a file

Without `--motion`, the reference is a static pose at the policy's default joint
positions, which is useful for checking that a policy stands.
