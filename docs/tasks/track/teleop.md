# Teleop

Teleop is two processes: a **publisher** that turns a live source into a robot
reference, and the runtime, which subscribes to it over ZMQ. Robot state and
commands never leave the runtime process.

```text
Pico / XR  ──▶  pico_pub.py (retarget to the robot)  ──ZMQ :28701──▶  boltrt --task tracking
```

The publishers live in [`src/tasks/track/teleop/`](https://github.com/EGalahad/boltrt/tree/main/src/tasks/track/teleop),
a self-contained Python project with its own environment. They read robot names,
joints, bodies and models from the same `configs/robots/*.yaml` as the runtime.

| Publisher | Source | Use |
|---|---|---|
| `pico_pub.py` | Pico / XR whole-body tracking, retargeted with GMR | live teleoperation |
| `npz_pub.py` | a retargeted NPZ clip | testing the teleop path without a headset |

## Set up the publisher

```bash
uv sync --project src/tasks/track/teleop
```

For Pico, also install the XRoboToolkit PC Service and build its Python binding:

1. Install the PC Service `.deb` from
   [XRoboToolkit-PC-Service releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases)
   (`sudo apt install ./XRoboToolkit_PC_Service_*.deb`) and start **XRoboToolkit** from the app launcher.
2. Build the SDK binding into the teleop environment. The script clones the SDK
   repositories at pinned commits into `src/tasks/track/teleop/third_party/`:

   ```bash
   src/tasks/track/teleop/setup_xrobot_sdk.sh
   ```

3. On the headset: wear the trackers, finish whole-body calibration, connect
   XRoboToolkit to this machine and enable whole-body streaming.

## Run

Start the publisher, then the runtime:

```bash
# Terminal 1: Pico publisher (prints an mjviser URL showing the retargeted robot)
uv run --project src/tasks/track/teleop src/tasks/track/teleop/pico_pub.py --robot g1

# Terminal 2: runtime
boltrt --task tracking --robot g1 --policy policy.yaml --backend mujoco \
  --motion tcp://127.0.0.1:28701
```

Without a headset, use a clip instead:

```bash
uv run --project src/tasks/track/teleop src/tasks/track/teleop/npz_pub.py \
  --robot g1 --motion-path motions/walk1_subject1.npz --loop
```

If the publisher runs on another machine, point `--motion` at it:
`--motion tcp://<publisher-ip>:28701`. Use `--backend real` for the real robot.

### Controls

| Control | Where | Effect |
|---|---|---|
| `X` | Pico controller | Pause / resume the published motion (handled by the publisher) |
| `I`, `]` | runtime terminal | Init pose, enter policy |
| `Q` | runtime terminal | Quit |

The publisher also sends Pico button states on port `5592` and analog trigger
grips on port `5593` for other consumers; Bolt Runtime does not read them, so
mode changes are made from the runtime terminal.

### Ports

| Port | Content |
|---|---|
| `28701` | robot reference JSON (`pico_pub.py`, `npz_pub.py`) |
| `28702` | SONIC SMPL reference (`pico_pub.py --publish-smpl`) |
| `5592` | Pico button states (binary) |
| `5593` | hand-grip triggers (binary) |

Supported robots for Pico retargeting: `g1`, `rp1` and `rp1-24dof`. Retargeting
uses [GMR](https://github.com/YanjieZe/GMR) onto the robot model from
`configs/robots/`. G1 uses GMR's own IK config. The RP1 IK configs in
`src/tasks/track/teleop/boltrt_teleop/configs/` are adapted to the current RP1
models and have not yet been validated with a headset.

## Robot reference messages

JSON messages carry a robot-space reference:

| Field | Required | Meaning |
|---|---|---|
| `joint_names`, `joint_pos` | yes | Joint positions by name |
| `body_names`, `body_pos_w`, `body_quat_w` | yes | Body poses in world frame, quaternions `wxyz` |
| `joint_vel` | no | Joint velocities |
| `seq`, `publish_t_ns`, `pico_recv_time_ns` | no | Sequencing and latency diagnostics |
| `motion_first_frame` | no | Marks the start of a new segment |

Any publisher that sends these fields works with the runtime.

## Behavior

- The runtime waits for the first valid message before tracking.
- A new segment is aligned in XY position and yaw to stay continuous with the
  current reference.
- If the stream stops, the last valid frame is held.
- Invalid messages are logged and never overwrite a valid reference.

## SMPL input

SONIC SMPL policies consume a human reference instead of robot joints. Start the
Pico publisher with `--publish-smpl --smpl-human-joints-info-path <sonic-smpl-checkpoint>/human_joints_info.pkl`
and connect the runtime to port `28702`.
The source accepts SONIC's JSON or packed (1280-byte header) protocol with
`smpl_joint_pos_root` (N×24×3), `smpl_root_quat_w` (wxyz) and robot `joint_pos`.
The runtime does not solve or retarget human pose itself.
