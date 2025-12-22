from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence


ADAM_U_19_JOINT_NAMES: tuple[str, ...] = (
    "dof_pos/waistRoll",
    "dof_pos/waistPitch",
    "dof_pos/waistYaw",
    "dof_pos/neckYaw",
    "dof_pos/neckPitch",
    "dof_pos/shoulderPitch_Left",
    "dof_pos/shoulderRoll_Left",
    "dof_pos/shoulderYaw_Left",
    "dof_pos/elbow_Left",
    "dof_pos/wristYaw_Left",
    "dof_pos/wristPitch_Left",
    "dof_pos/wristRoll_Left",
    "dof_pos/shoulderPitch_Right",
    "dof_pos/shoulderRoll_Right",
    "dof_pos/shoulderYaw_Right",
    "dof_pos/elbow_Right",
    "dof_pos/wristYaw_Right",
    "dof_pos/wristPitch_Right",
    "dof_pos/wristRoll_Right",
)


# Preferred 12-DoF hand inputs directly matching your VR JointState list.
VR_HAND_12_NAMES: tuple[str, ...] = (
    "dof_pos/hand_pinky_Left",
    "dof_pos/hand_ring_Left",
    "dof_pos/hand_middle_Left",
    "dof_pos/hand_index_Left",
    "dof_pos/hand_thumb_1_Left",
    "dof_pos/hand_thumb_2_Left",
    "dof_pos/hand_pinky_Right",
    "dof_pos/hand_ring_Right",
    "dof_pos/hand_middle_Right",
    "dof_pos/hand_index_Right",
    "dof_pos/hand_thumb_1_Right",
    "dof_pos/hand_thumb_2_Right",
)


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass(frozen=True)
class VrToCmdConfig:
    """Pure conversion config (no ROS dependency).

    - arm joints are interpreted as radians.
    - hand joints are mapped to int[0..1000] (adam_u HandCmd).

    The defaults assume:
      - hand open corresponds to 1000
      - hand closed corresponds to 0
      - VR hand DOF value increases as you close.

    If your VR values are opposite, swap open/close angles.
    """

    hand_open_angle: float = 0.0
    hand_close_angle: float = 1.0
    hand_open_value: int = 1000
    hand_close_value: int = 0


def name_position_to_map(names: Sequence[str], positions: Sequence[float]) -> dict[str, float]:
    """Build a name->position map from JointState fields.

    This is intentionally ROS-agnostic so it can be used on the VR sender side.
    """

    out: dict[str, float] = {}
    n = min(len(names), len(positions))
    for i in range(n):
        out[str(names[i])] = float(positions[i])
    return out


def extract_adam_u_targets(
    name_to_pos: Mapping[str, float],
    *,
    cfg: VrToCmdConfig = VrToCmdConfig(),
) -> tuple[list[float], list[int] | None]:
    """Extract targets for adam_u LowCmd (19) and HandCmd (12).

    Returns:
      - q_targets: length 19 (missing joints default to 0.0)
      - hand_targets: length 12 int[0..1000] if available, else None

    This function is the reusable part you can move to the VR sender.
    """

    q_targets: list[float] = [0.0] * len(ADAM_U_19_JOINT_NAMES)
    for i, joint_name in enumerate(ADAM_U_19_JOINT_NAMES):
        if joint_name in name_to_pos:
            q_targets[i] = float(name_to_pos[joint_name])

    hand_targets: list[int] | None = None
    if all(n in name_to_pos for n in VR_HAND_12_NAMES):
        denom = (cfg.hand_close_angle - cfg.hand_open_angle)
        # Avoid div-by-zero; in that case, just keep everything at open.
        if abs(denom) < 1e-9:
            hand_targets = [int(cfg.hand_open_value)] * len(VR_HAND_12_NAMES)
        else:
            hand_targets = []
            for n in VR_HAND_12_NAMES:
                angle = float(name_to_pos[n])
                t = (angle - cfg.hand_open_angle) / denom
                t = _clamp(t, 0.0, 1.0)
                # t=0 -> open_value, t=1 -> close_value
                value = (1.0 - t) * cfg.hand_open_value + t * cfg.hand_close_value
                hand_targets.append(int(round(_clamp(value, 0.0, 1000.0))))

    return q_targets, hand_targets
