# Configuration

Every run combines a **robot**, a **policy** and a **backend**. Robots and
backends can be given by builtin name (`--robot g1`) or by YAML path; policies
are always a path. `--print-config` shows the fully resolved result.

## Robot

`configs/robots/<name>.yaml` describes the physical robot and is shared by every
policy that runs on it.

| Key | Meaning |
|---|---|
| `name` | Builtin name |
| `joint_names`, `body_names` | Canonical joint and body order used across the runtime |
| `model` | MJCF path or `hf://owner/repo@revision/path.xml` |
| `root_body` | Floating base body |
| `initial_q`, `initial_position` | Simulation start state |
| `effort_limit`, `joint_armature`, ... | Fixed dynamics and torque limits |

Builtin robots: `g1`, `rp1` (28 DoF), `rp1-24dof`, `rp1-parallel`,
`rp1-24dof-parallel`. List them with `--list-robots`.

## Policy

A deploy checkpoint is `policy.onnx` + `policy.yaml`:

| Key | Meaning |
|---|---|
| `model_path` | ONNX file; defaults to the YAML's sibling `policy.onnx` |
| `onnx.in_keys` / `onnx.out_keys` | Input/output tensor bindings |
| `joint_names_simulation`, `body_names_simulation` | Names the policy was trained with |
| `policy_joint_names` | Joints driven by the action, in action order |
| `default_joint_pos`, `joint_kp`, `joint_kd` | Per joint; maps accept regex keys such as `.*_knee_joint` |
| `action_scale`, `action_clip` | Action mapping |
| `compatible_robots` | Optional guard against loading on the wrong robot |
| `observation` | Observation groups; see [Observations](observations.md) |

Joints are always matched **by name**, never by index, so a policy trained with
a different joint order works unchanged.

### Training-only fields

Exports often carry fields that only matter during training: observation noise
(`noise_std`, `bias_noise_std`), domain-randomization ranges, curriculum settings,
and historical physics overrides (`joint_armature`, `joint_effort_limit`, ...).
Bolt Runtime accepts these so that original exports load as-is, but **never
applies them**: deployment adds no noise and no randomization, and physics comes
from the robot config. Physics overrides print a notice when ignored.

### Legacy exports

Older exports keep their ONNX bindings in a JSON sidecar next to the YAML; this
is still read. New exports should embed `onnx.in_keys/out_keys` in the YAML.

## Backend

`--backend mujoco` simulates; `--backend real` drives the hardware described by the
robot's `hardware` section (driver, calibration, rates; see [Real robots](../real-robots.md)).
A backend YAML path can also be given, e.g. for the replay backend; see [Backends](backends.md).

## Paths and assets

- Relative paths inside a YAML resolve against that YAML's directory; paths on
  the command line resolve against the current directory.
- `hf://owner/repo@revision/path` resolves through the Hugging Face cache and is
  downloaded on first use; see [Installation](../getting-started/installation.md#robot-assets).
- `--robot-model` overrides the robot's model for a single run.
