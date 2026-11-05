# Supported policies

Original deploy exports from the
[checkpoint folder](https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e)
load as-is: no per-policy C++ configuration is needed.

| Family | Robot | Checkpoint path |
|---|---|---|
| MimicLite | G1, RP1 | `mimic-lite/v1_1/`, `mimic-lite/rp1/` |
| BFM-Zero | G1 | `bfm-zero/exp_lafan40-100style_update_z10/` |
| GRIT | G1 | `grit/v0_0_1/` |
| HEFT PMG / Compliance | G1 | `heft/{pmg,compliance}/` |
| HoloMotion v1.4.0 | G1 | `holomotion/v1_4_0/` |
| Humanoid-GPT | G1 | `humanoid-gpt/` |
| ScaleBFM M / XL | G1 | `scalebfm/humanoid_transformer_{m,xl}/` |
| SONIC G1 | G1 | `sonic/{release,low_latency,v1_1}/g1/` |
| SONIC SMPL | G1 | `sonic/{release,low_latency}/smpl/` ([SMPL input](teleop.md#smpl-input)) |
| TeleopIT | G1 | `teleopit/` |
| TWIST2 | G1 | `twist2/` |

## Parity with the Python deployment

Each of the 15 external configurations was run from identical robot states and
reference windows in both Bolt Runtime and its original Python deployment, for
32 ticks, comparing every observation component and the action. The maximum
absolute action difference was **4.0e-5** (TWIST2). This verifies the adapters,
not tracking quality. Per-configuration results:
[parity-results.json](../../reference/data/parity-results.json).

## Family details that matter

Adapters reproduce each family's exact conventions, which differ in ways that
are easy to get wrong:

- History order and frame spacing differ per family (SONIC, HEFT, TWIST2).
- BFM-Zero uses a backward future window.
- HEFT computes reference gravity from an unnormalized quaternion; HoloMotion
  normalizes before computing the initial heading.
- MimicLite yaw alignment falls back to a projection when the body is near vertical.
- HoloMotion carries a KV cache and resets its rotary positions on policy entry,
  and exports two extra diagnostic outputs.
- TeleopIT YAML uses merge keys and anchors.

When adding a family, check each of these kinds of convention against the
training code, component by component.
