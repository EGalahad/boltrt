# Robot calibration

`--backend real` reads the SDK calibration of the physical robot from this
directory: motor zero offsets, motor signs, joint-to-motor mapping, IMU
extrinsics and ankle closed-chain indices. One file per machine type is shared
by every robot config of that machine (for example `rp1` and `rp1-parallel`).

Calibration files are machine-specific and not tracked by git (`*.yaml` here is
ignored). Create yours from the vendor template and edit the values for your robot:

```bash
cp third_party/roboparty_deploy-official/src/inference/robots/rp1/robot.yaml configs/calibration/rp1.yaml
cp third_party/roboparty_deploy-official/src/inference/robots/rp1_24dof/robot.yaml configs/calibration/rp1-24dof.yaml
```

The runtime refuses to start hardware with the unmodified vendor template, whose
zero offsets belong to a different robot. `robot.type` must be a closed-chain type
the pinned SDK knows (`rp1` or `rpo`).
