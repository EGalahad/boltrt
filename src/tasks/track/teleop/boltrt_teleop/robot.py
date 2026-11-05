"""Robot description read from Bolt Runtime's own `configs/robots/<name>.yaml`."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml
from mjhub import resolve_asset_reference

CONFIG_ROOT = Path(__file__).resolve().parents[5] / "configs" / "robots"  # <repo>/configs/robots


@dataclass(frozen=True)
class RobotCfg:
    name: str
    joint_names: tuple[str, ...]
    body_names: tuple[str, ...]
    mjcf_path: str
    default_qpos: tuple[float, ...]
    root_body: str = "pelvis"

    @property
    def qpos_size(self) -> int:
        return 7 + len(self.joint_names)

    @property
    def viewer_track_body_names(self) -> tuple[str, ...]:
        return (self.root_body,)

    def resolve_mjcf_path(self) -> Path:
        if "://" in self.mjcf_path:
            return Path(resolve_asset_reference(self.mjcf_path))
        return (CONFIG_ROOT / self.mjcf_path).resolve()


def get_robot_cfg(name_or_path: str) -> RobotCfg:
    """Load a builtin robot by name (`g1`, `rp1`, ...) or a robot YAML by path."""
    path = Path(name_or_path)
    if not path.suffix:
        path = CONFIG_ROOT / f"{name_or_path}.yaml"
    data = yaml.safe_load(path.read_text())
    root_pos = list(data.get("initial_position", [0.0, 0.0, 0.8]))
    joint_q = list(data.get("initial_q", [0.0] * len(data["joint_names"])))
    return RobotCfg(
        name=str(data["name"]),
        joint_names=tuple(data["joint_names"]),
        body_names=tuple(data["body_names"]),
        mjcf_path=str(data["model"]),
        default_qpos=tuple(root_pos + [1.0, 0.0, 0.0, 0.0] + joint_q),
        root_body=str(data.get("root_body", "pelvis")),
    )
