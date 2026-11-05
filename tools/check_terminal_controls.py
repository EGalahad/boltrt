"""Check terminal policy/motion controls through a PTY, without desktop input."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import pty
import re
import select
import struct
import subprocess
import tempfile
import termios
import time

p = argparse.ArgumentParser()
p.add_argument('--binary', type=Path, required=True)
p.add_argument('--robot', required=True)
p.add_argument('--policy', type=Path, required=True)
p.add_argument('--motion', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
master, slave = pty.openpty()
fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 28, 130, 0, 0))
before = termios.tcgetattr(slave)
raw = b''
ansi = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
checks = {}


def drain(seconds=.2):
    global raw
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if select.select([master], [], [], .03)[0]:
            try:
                raw += os.read(master, 65536)
            except OSError:
                break
    return ansi.sub('', raw.decode(errors='replace'))


def wait(text, timeout=20):
    mark = len(raw)
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        drain(.05)
        if text in ansi.sub('', raw[mark:].decode(errors='replace')):
            return
        if proc.poll() is not None:
            raise RuntimeError(f'CLI exited: {raw[-3000:]!r}')
    raise RuntimeError(f'Timed out waiting for {text!r}: {raw[-3000:]!r}')


def key(value, expected):
    os.write(master, value)
    wait(expected)


def snapshot():
    text = ansi.sub('', raw.decode(errors='replace'))
    frames = re.findall(r'Frame (\d+)/(\d+)', text)
    times = re.findall(r'Backend\s*:\s*([0-9.]+)', text)
    return int(frames[-1][0]), float(times[-1])


with tempfile.TemporaryDirectory(prefix='boltrt-controls-') as tmp:
    backend = Path(tmp) / 'mujoco.yaml'
    # Real-time pacing makes the control sequence reproducible without a viewer.
    backend.write_text('type: mujoco\nphysics_dt: 0.005\nviewer: false\nrealtime: true\nfloor: true\n')
    cmd = [str(a.binary.resolve()), '--task', 'tracking', '--robot', a.robot, '--policy',
           str(a.policy.resolve()), '--backend', str(backend), '--motion',
           str(a.motion.resolve())]
    proc = subprocess.Popen(cmd, stdin=slave, stdout=slave, stderr=slave,
                            start_new_session=True)
    try:
        wait('Status  : Ready')
        drain(.2)
        f0, t0 = snapshot()
        checks['default_hold_motion_paused'] = f0 == 0
        key(b'i', 'Initializing')
        key(b'o', 'Holding pose')
        checks['policy_modes_acknowledged'] = True
        key(b']', 'Policy active; motion reset and paused')
        drain(.3)
        f1, t1 = snapshot()
        checks['enter_policy_resets_and_pauses_motion'] = f1 == 0
        checks['paused_motion_keeps_simulation_running'] = t1 > t0
        key(b' ', 'Reference resumed')
        drain(.3)
        checks['space_plays_motion'] = snapshot()[0] > 0
        key(b' ', 'Reference paused')
        drain(.2)
        f2, t2 = snapshot()
        drain(.3)
        f3, t3 = snapshot()
        checks['space_pauses_only_motion'] = f3 == f2 and t3 > t2
        key(b']', 'Policy active; motion reset and paused')
        drain(.2)
        checks['policy_reentry_resets_motion'] = snapshot()[0] == 0
        # Simulation controls must not be captured by the terminal.
        old_time = snapshot()[1]
        os.write(master, b'\x7f')
        drain(.2)
        checks['terminal_backspace_does_not_reset_simulation'] = snapshot()[1] > old_time
        os.write(master, b'q')
        proc.wait(timeout=15)
        drain(.1)
        checks['exit_code_zero'] = proc.returncode == 0
        checks['terminal_restored'] = termios.tcgetattr(slave) == before
        assert all(checks.values()), checks
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.with_suffix('.txt').write_text(ansi.sub('', raw.decode(errors='replace')))
        a.output.write_text(json.dumps({'result': 'passed', 'checks': checks,
                                       'command': cmd}, indent=2) + '\n')
        print(json.dumps(checks, indent=2))
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=15)
        os.close(master)
        os.close(slave)
