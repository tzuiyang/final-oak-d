"""
Convert a Detection into (x_vel, y_vel, ang_vel) commands for the Pupper.

This is a deliberately simple reactive P-controller. It does not plan a path,
build a map, or avoid obstacles — it just turns toward the closest target and
walks forward until close enough to stop.

Matches the Pupper walking-controller interface: three scalar velocities.
    x_vel   forward   m/s   (Pupper body frame; positive = forward)
    y_vel   lateral   m/s   (positive = left; we leave this at 0)
    ang_vel yaw       rad/s (positive = turn left)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from detector import Detection


@dataclass
class VelocityCommand:
    x_vel: float
    y_vel: float
    ang_vel: float

    @classmethod
    def zero(cls) -> "VelocityCommand":
        return cls(0.0, 0.0, 0.0)


@dataclass
class FollowerConfig:
    # Target behavior
    target_distance: float = 0.4     # meters; stop when object is within this distance
    timeout_frames: int = 10         # stop if no detection for this many frames in a row

    # Velocity gains (tuned conservatively so the default walking policy stays stable)
    k_forward: float = 0.8           # m/s per meter of distance error
    k_yaw: float = 1.2               # rad/s per radian of bearing error

    # Velocity clamps
    max_forward_vel: float = 0.5     # m/s
    max_yaw_vel: float = 1.5         # rad/s

    # Class preference — if multiple targets detected, go for this class first.
    # None means "go for whichever is closest."
    preferred_class: Optional[str] = None


class ObjectFollower:
    """Stateful controller that tracks the closest target across frames."""

    def __init__(self, config: FollowerConfig):
        self._cfg = config
        self._frames_since_detection = 0

    def step(self, detections: List[Detection]) -> VelocityCommand:
        target = self._pick_target(detections)

        if target is None:
            self._frames_since_detection += 1
            if self._frames_since_detection >= self._cfg.timeout_frames:
                return VelocityCommand.zero()
            # Brief dropouts: hold still rather than drift on stale commands
            return VelocityCommand.zero()

        self._frames_since_detection = 0

        # Forward velocity: positive when farther than target distance, zero otherwise.
        distance_error = target.distance - self._cfg.target_distance
        x_vel = _clamp(
            self._cfg.k_forward * max(0.0, distance_error),
            0.0,
            self._cfg.max_forward_vel,
        )

        # Yaw: turn toward the object. DepthAI bearing is positive when object
        # is to the right, so we negate (positive ang_vel in Pupper = turn left).
        ang_vel = _clamp(
            -self._cfg.k_yaw * target.bearing,
            -self._cfg.max_yaw_vel,
            self._cfg.max_yaw_vel,
        )

        return VelocityCommand(x_vel=x_vel, y_vel=0.0, ang_vel=ang_vel)

    def _pick_target(self, detections: List[Detection]) -> Optional[Detection]:
        if not detections:
            return None

        if self._cfg.preferred_class is not None:
            preferred = [d for d in detections if d.class_name == self._cfg.preferred_class]
            if preferred:
                return min(preferred, key=lambda d: d.distance)

        return min(detections, key=lambda d: d.distance)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
