"""Normalize an existing export to policy.onnx + policy.yaml.

Offline developer utility. It does not alter original exports or robot assets.
External auxiliary assets remain shared references; no model/mesh bundle is made.
"""
import argparse
import json
import shutil
from pathlib import Path
import yaml

p = argparse.ArgumentParser()
p.add_argument('--policy', type=Path, required=True)
p.add_argument('--robot', required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
source = a.policy.resolve()
cfg = yaml.safe_load(source.read_text())
model = (source.parent / cfg.get('model_path', source.with_suffix('.onnx').name)).resolve()
if not model.is_file():
    p.error(f'ONNX file missing: {model}')
if a.output.exists():
    p.error('Use a fresh output directory to preserve existing checkpoints')
meta = model.with_suffix('.json')
if 'onnx' not in cfg and meta.is_file():
    bindings = json.loads(meta.read_text())
    cfg['onnx'] = {k: bindings[k] for k in ('in_keys', 'out_keys') if k in bindings}
cfg['model_path'] = 'policy.onnx'
cfg['compatible_robots'] = [a.robot]
removed = []
for key in ('joint_effort_limit', 'joint_armature', 'joint_frictionloss',
            'motor_velocity_limit', 'motor_keep_velocity'):
    if key in cfg:
        removed.append(key)
        del cfg[key]
# These known auxiliary model paths must not become relative to the new folder.
for node, key in [(cfg.get('motion', {}), 'mjcf_path')]:
    value = node.get(key)
    if value and '://' not in value and not Path(value).is_absolute():
        candidates = [parent / value for parent in [source.parent, *source.parents]]
        found = next((f for f in candidates if f.is_file()), None)
        if found is None:
            p.error(f'Unresolved shared reference asset: {value}')
        node[key] = str(found.resolve())
a.output.mkdir(parents=True)
shutil.copy2(model, a.output / 'policy.onnx')
(a.output / 'policy.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
print(json.dumps({'output': str(a.output.resolve()), 'robot': a.robot,
                  'removed_policy_physics_fields': removed, 'original_export_untouched': True}))
