"""Publish an any4hdmi NPZ clip as a live reference stream for `boltrt --task tracking`.

Usage:
  uv run --project src/tasks/track/teleop src/tasks/track/teleop/npz_pub.py \
    --motion-path motions/walk1_subject1.npz --robot g1 --loop

Useful for testing the teleop path without a Pico: boltrt subscribes with
`--motion tcp://<host>:28701` and receives the same canonical JSON as from pico_pub.py.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import tyro
import zmq

from boltrt_teleop.protocol import (
    BODY_NAMES_KEY,
    BODY_POS_W_KEY,
    BODY_QUAT_W_KEY,
    JOINT_NAMES_KEY,
    JOINT_POS_KEY,
    MOTION_FIRST_FRAME_KEY,
    MOTION_PORT,
    PUBLISH_T_NS_KEY,
    SEQ_KEY,
)
from boltrt_teleop.robot import get_robot_cfg


@dataclass
class Args:
    motion_path: Path
    """any4hdmi NPZ clip; its dataset manifest.json is found in a parent directory."""
    robot: str = "g1"
    """Builtin boltrt robot name or robot YAML path; must match the clip's retarget."""
    bind: str = f"tcp://*:{MOTION_PORT}"
    publish_hz: float = 50.0
    loop: bool = False
    """Restart from the first frame at the end; otherwise hold the last frame."""
    startup_sleep_s: float = 0.5


def find_manifest(motion_path: Path) -> dict:
    for parent in motion_path.resolve().parents:
        manifest = parent / "manifest.json"
        if manifest.is_file():
            return json.loads(manifest.read_text())
    raise FileNotFoundError(f"no manifest.json above {motion_path}")


class Clip:
    """qpos frames mapped onto the robot model, with forward kinematics per frame."""

    def __init__(self, args: Args):
        self.robot = get_robot_cfg(args.robot)
        manifest = find_manifest(args.motion_path)
        self.qpos = np.load(args.motion_path)["qpos"].astype(np.float64)
        self.dt = float(manifest["timestep"])
        names = list(manifest["qpos_names"])
        self.model = mujoco.MjModel.from_xml_path(str(self.robot.resolve_mjcf_path()))
        self.data = mujoco.MjData(self.model)
        self.root = names.index("root_tx")
        self.joint_cols, self.joint_adr = [], []
        for name in self.robot.joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0 or name not in names:
                raise ValueError(f"joint {name!r} missing from the model or the clip")
            self.joint_cols.append(names.index(name))
            self.joint_adr.append(self.model.jnt_qposadr[jid])
        self.body_ids = []
        for name in self.robot.body_names:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                raise ValueError(f"body {name!r} missing from the model")
            self.body_ids.append(bid)

    def __len__(self) -> int:
        return len(self.qpos)

    def frame(self, i: int) -> dict:
        row = self.qpos[i]
        self.data.qpos[:7] = row[self.root : self.root + 7]  # xyz + wxyz
        self.data.qpos[self.joint_adr] = row[self.joint_cols]
        mujoco.mj_kinematics(self.model, self.data)
        return {
            JOINT_NAMES_KEY: list(self.robot.joint_names),
            BODY_NAMES_KEY: list(self.robot.body_names),
            JOINT_POS_KEY: row[self.joint_cols].astype(np.float32).tolist(),
            BODY_POS_W_KEY: self.data.xpos[self.body_ids].astype(np.float32).tolist(),
            BODY_QUAT_W_KEY: self.data.xquat[self.body_ids].astype(np.float32).tolist(),
        }


def main(args: Args) -> None:
    clip = Clip(args)
    sock = zmq.Context.instance().socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.bind(args.bind)
    time.sleep(args.startup_sleep_s)
    print(f"[npz_pub] {args.motion_path.name}: {len(clip)} frames at {1 / clip.dt:.1f} fps -> {args.bind}")

    period, start, seq, last = 1.0 / args.publish_hz, time.monotonic(), 0, -1
    while True:
        index = int((time.monotonic() - start) / clip.dt)
        if index >= len(clip):
            if args.loop:
                start, index, last = time.monotonic(), 0, -1
            else:
                index = len(clip) - 1
        payload = clip.frame(index)
        payload.update({
            SEQ_KEY: seq,
            PUBLISH_T_NS_KEY: time.time_ns(),
            MOTION_FIRST_FRAME_KEY: index == 0 and last != 0,
        })
        sock.send_json(payload, flags=zmq.NOBLOCK)
        seq, last = seq + 1, index
        time.sleep(max(0.0, period - (time.monotonic() - start) % period))


if __name__ == "__main__":
    main(tyro.cli(Args))
