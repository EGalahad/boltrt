"""Development-only canonical ZMQ interoperability/continuity smoke test."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import mujoco
import numpy as np
import yaml
import zmq
from mjhub import resolve_asset_reference


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--binary", type=Path, required=True)
    p.add_argument("--robot", type=Path, required=True)
    p.add_argument("--policy", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    os.environ["HF_HUB_OFFLINE"] = "1"
    robot = yaml.safe_load(args.robot.read_text())
    model = mujoco.MjModel.from_xml_path(str(resolve_asset_reference(robot["model"])))
    data = mujoco.MjData(model)
    data.qpos[:3] = robot["initial_position"]
    data.qpos[3] = 1
    for name, q in zip(robot["joint_names"], robot["initial_q"]):
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[j]] = q
    mujoco.mj_forward(model, data)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in robot["body_names"]]
    positions, orientations = data.xpos[ids].copy(), data.xquat[ids].copy()
    args.output.mkdir(parents=True, exist_ok=True)
    backend = args.output / "realtime.yaml"
    backend.write_text("type: mujoco\nphysics_dt: 0.005\nrealtime: true\nfloor: true\nviewer: false\n")
    trace = args.output / "teleop.jsonl"
    ready, stop = threading.Event(), threading.Event()
    shared = {}

    def publish():
        context = zmq.Context()
        sock = context.socket(zmq.PUB)
        sock.setsockopt(zmq.LINGER, 0)
        port = sock.bind_to_random_port("tcp://127.0.0.1")
        shared["endpoint"] = f"tcp://127.0.0.1:{port}"
        ready.set()
        for seq in range(55):
            if stop.is_set():
                break
            pos, quat = positions.copy(), orientations.copy()
            # A fresh segment begins far away; buffer must preserve motion continuity.
            if seq >= 30:
                pos[:, 0] += 10
            if seq == 40:
                sock.send_json({"seq": seq, "joint_pos": [0]})
                time.sleep(0.02)
                continue
            sock.send_json({
                "seq": seq, "publish_t_ns": time.time_ns(),
                "motion_first_frame": seq == 30,
                "joint_names": robot["joint_names"], "body_names": robot["body_names"],
                "joint_pos": robot["initial_q"],
                "body_pos_w": pos.tolist(), "body_quat_w": quat.tolist(),
            })
            time.sleep(0.02)
        sock.close()
        context.term()

    thread = threading.Thread(target=publish)
    thread.start()
    assert ready.wait(5)
    try:
        result = subprocess.run([
            str(args.binary.resolve()), "--task", "tracking", "--robot", str(args.robot.resolve()),
            "--policy", str(args.policy.resolve()), "--backend", str(backend.resolve()),
            "--motion", shared["endpoint"], "--steps", "80", "--start", "--no-keyboard",
            "--record", str(trace.resolve()),
        ], text=True, capture_output=True, timeout=20)
        assert result.returncode == 0, result.stderr
    finally:
        stop.set()
        thread.join(5)
    rows = [json.loads(line) for line in trace.read_text().splitlines()]
    root = robot["body_names"].index(robot["root_body"])
    positions = np.asarray([row["reference"]["body_position"][root] for row in rows])
    assert len(rows) == 80
    assert np.max(np.linalg.norm(positions - positions[0], axis=-1)) < 1e-4, "segment position discontinuity"
    assert np.array_equal(positions[-10:], np.broadcast_to(positions[-1], positions[-10:].shape)), "reference lost after disconnect"
    report = {"result": "passed", "ticks": len(rows), "checks": ["JSON/ZMQ input", "segment continuity", "hold final reference after publisher disconnect", "invalid payload leaves latest valid reference intact"], "runtime": result.stdout.strip()}
    (args.output / "teleop-result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
