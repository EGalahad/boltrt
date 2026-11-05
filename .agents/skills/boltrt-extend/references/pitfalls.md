# Pitfalls, by symptom

## Action differs from training by ~1e-3 or more

- Joint or body order: match by name, never assume order.
- Quaternion order is `wxyz`; check whether the training code normalizes before
  use (some families must NOT normalize, some must).
- History order (newest first or last) and frame spacing (`history_steps`).
- Future/past reference windows (some families use a backward window).
- Yaw alignment special cases, e.g. a projection fallback near vertical.
- Reference velocities: training interpolates source-frame finite differences and
  uses MuJoCo FK body velocities; naive differencing at the policy rate differs.
- Precision: motion data may be stored as float16; compute in float32. Decide
  whether parity targets the float32 math or a legacy float16 path.

## Observation adds noise or behaves stochastically

- A training-only field (`noise_std`, `bias_noise_std`, randomization) is being
  applied. Deployment must accept and ignore them.

## Real robot behaves badly although sim2sim is fine

- First run a checkpoint known to work on that robot with the same backend and
  calibration. If it works, the runtime path is fine and the policy is the suspect.
- Record with `--record`, then compare observations against sim from the same
  state. Training-only domain randomization (action delay, filters, noise) is
  never applied at deployment, so a policy trained without delay randomization can
  be fragile on hardware.

## Load error: unknown key / missing name

- Unknown observation key: add training-only keys to the term's `require_keys`
  list (ignored), or fix the typo.
- Missing body or joint: the checkpoint's model revision has different names
  (added palm links, removed toe links, renamed wrist joints). Update the robot
  config to the model revision the policy was trained on.
- Root not found in motion: the loader accepts the model's freejoint name or
  canonical `root_*` fields; check the motion was made for this robot.

## Model loads but outputs are wrong or crash

- Extra ONNX outputs (diagnostics) are fine; bind only the needed `out_keys`.
- Recurrent / KV-cache / rotary-position state must reset on policy entry.
- `int64` or `bool` inputs must keep their dtype.
- YAML merge keys (`<<`) and anchors must be expanded.

## Wrong model or appearance loads

- A checkpoint directory with its own `robot.xml` shadows the builtin robot;
  use builtin robots and keep checkpoints to `policy.onnx` + `policy.yaml`.
- `hf://` with a branch revision follows the cached ref; pin a commit.

## Policy behaves differently in sim than expected

- Physics must come from the robot config; policy physics fields are ignored.
- Passive joints cannot be policy actions.
- Viewer pause/reset affects physics only; press `]` to re-enter the policy.

## Build problems

- Vendor SDK headers may not compile as C++20; compile that adapter separately
  with the SDK's standard.
- Always build both the sim-only and the hardware presets after touching shared code.

## Hardware

- Use the robot model matching the actual hardware version (e.g. G1
  `mode_machine`); serial and parallel ankles are different robots.
- Parallel-ankle conversions may happen on the robot side; do not invent a
  kinematic solution the SDK does not document.
- Set the real network interface / CAN devices and calibration; bundled RP1
  calibration is an example and the runtime refuses to start with it.
- RP1 calibration `robot.type` must be `rp1` for the pinned SDK; older files say
  `rp1.3` (same solver) and fail at device start with "Unknown close_chain type".
- Parallel-ankle robots have more model joints than motors: list the linkage
  joints as `passive_joint_names` and set `direct_ankle_motors: true`.
