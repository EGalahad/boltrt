# Teleop

Teleop runs as a separate publisher process that streams reference motion over
ZMQ. Robot state and commands never leave the runtime process.

```bash
boltrt --task tracking --robot g1 --policy policy.yaml --backend mujoco \
  --motion tcp://127.0.0.1:5555
```

## Robot reference messages

JSON messages carry a robot-space reference:

| Field | Required | Meaning |
|---|---|---|
| `joint_names`, `joint_pos` | yes | Joint positions by name |
| `body_names`, `body_pos_w`, `body_quat_w` | yes | Body poses in world frame, quaternions `wxyz` |
| `joint_vel` | no | Joint velocities |
| `seq`, `publish_t_ns` | no | Sequencing and latency diagnostics |
| `motion_first_frame` | no | Marks the start of a new segment |

## Behavior

- The runtime waits for the first valid message before tracking.
- A new segment is aligned in XY position and yaw to stay continuous with the
  current reference.
- If the stream stops, the last valid frame is held.
- Invalid messages are logged and never overwrite a valid reference.

## SMPL input

SONIC SMPL policies consume a human reference instead of robot joints. The source
accepts SONIC's JSON or packed (1280-byte header) protocol with
`smpl_joint_pos_root` (N×24×3), `smpl_root_quat_w` (wxyz) and robot `joint_pos`.
The publisher must provide solved SMPL joints; the runtime does not solve or
retarget human pose.
