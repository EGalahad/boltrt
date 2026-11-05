# Add a robot or model revision

1. MJCF names must equal the names policies were trained with.
2. Publish to Hugging Face and reference it as `hf://owner/repo@<commit>/path.xml`
   (pin a commit). The runtime downloads snapshots on first use; with
   `HF_HUB_OFFLINE=1` it only reads the cache.
3. Copy `configs/robots/g1.yaml`; fill names, model, root body, initial state,
   dynamics and limits.
4. `--validate`, then a headless rollout.

For a revision of an existing model, compare old vs new: joint/body arrays,
inertia, collision, and a fixed-seed rollout must be identical unless the change
is intentionally physical. A different structure (serial vs parallel ankle,
different DoF) is a new robot name, not an edit.
