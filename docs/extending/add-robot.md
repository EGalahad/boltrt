# Add a robot

1. Make an MJCF model whose joint and body names are the names your policies use.
   Publishing it on Hugging Face lets every machine fetch it with an
   `hf://owner/repo@revision/path.xml` reference; pin a revision for reproducibility.
2. Copy `configs/robots/g1.yaml` to `configs/robots/<name>.yaml` and fill in
   `name`, `joint_names`, `body_names`, `model`, `root_body`, the initial state,
   and dynamics (`effort_limit`, `joint_armature`, ...).
3. Check it: `boltrt --robot <name> --policy <policy.yaml> --backend mujoco --validate`.

Rules that keep robots reusable:

- The robot owns structure and dynamics. Policies must not carry their own copy
  of the model or override physics.
- Visual-only model updates must leave joints, inertia and collision unchanged;
  verify by comparing a rollout before and after.
- A different kinematic structure (for example a parallel ankle instead of a
  serial one) is a different robot with its own name.
- For hardware, add a matching backend config; see [Real robots](../real-robots.md).
