# Copyright 2026 The RPent Authors.
# Licensed under the Apache License, Version 2.0.

"""Bounded base-frame EE increments over the existing Seer WBC transport."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

from robots.seer_dual_arm.homing import (
    DualPose,
    MotionConfig,
    _finite_positive,
    _normalize_quaternion,
    _qmul,
    execute_pose_motion,
)


@dataclass(frozen=True)
class CartesianConfig(MotionConfig):
    """Per-request increment limits and arrival thresholds."""

    max_translation_m: float
    max_rotation_rad: float


def coerce_arm(value: Any) -> Literal["left", "right", "both"]:
    """Normalize the selected arm and reject unsupported names."""
    arm = str(value).strip().lower()
    if arm not in {"left", "right", "both"}:
        raise ValueError("arm must be 'left', 'right', or 'both'")
    return arm  # type: ignore[return-value]


def load_cartesian_config(path: Path | str) -> CartesianConfig:
    """Read optional ``teleop.cartesian`` settings, using small-motion defaults."""
    raw = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or not isinstance(raw.get("teleop"), dict):
        raise ValueError("robot config must define a teleop object")
    settings = raw["teleop"].get("cartesian", {})
    if not isinstance(settings, dict):
        raise ValueError("teleop.cartesian must be an object")
    cycles = settings.get("stable_cycles", 3)
    if isinstance(cycles, bool) or not isinstance(cycles, int) or cycles < 1:
        raise ValueError("cartesian stable_cycles must be a positive integer")
    return CartesianConfig(
        position_threshold_m=_finite_positive(
            settings.get("position_threshold_m", 0.005), "position_threshold_m"
        ),
        rotation_threshold_deg=_finite_positive(
            settings.get("rotation_threshold_deg", 1.0), "rotation_threshold_deg"
        ),
        stable_cycles=cycles,
        max_translation_m=_finite_positive(
            settings.get("max_translation_m", 0.10), "max_translation_m"
        ),
        max_rotation_rad=_finite_positive(
            settings.get("max_rotation_rad", 0.5), "max_rotation_rad"
        ),
    )


def _vec3(value: Any) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("delta must contain exactly three finite numbers")
    if any(isinstance(item, bool) for item in value):
        raise ValueError("delta must contain exactly three finite numbers")
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("delta must contain exactly three finite numbers") from exc
    if not all(math.isfinite(item) for item in result):
        raise ValueError("delta must contain exactly three finite numbers")
    return result  # type: ignore[return-value]


def execute_cartesian_delta(
    *,
    arm: str,
    delta: list[float],
    action: Literal["move_delta", "rotate_delta"],
    rbk_config: Path | str,
    **runtime: Any,
) -> dict[str, Any]:
    """Translate one or both EEs, or rotate one EE from measured feedback.

    Translation uses base_link metres. Rotation uses extrinsic XYZ Euler
    radians, composed as ``q_delta * q_current`` in base_link. Vector norms
    exceeding the configured per-request limits are rejected before SDK init.
    Preview reports only the increment, since no measured start is available.
    """
    arm = coerce_arm(arm)
    requested = _vec3(delta)
    config = load_cartesian_config(rbk_config)
    if action == "move_delta":
        limit, unit = config.max_translation_m, "m"
        request_key = "requested_delta_xyz"
    elif action == "rotate_delta":
        if arm == "both":
            raise ValueError("rotate_delta arm must be 'left' or 'right'")
        limit, unit = config.max_rotation_rad, "rad"
        request_key = "requested_delta_rpy"
    else:
        raise ValueError("action must be move_delta or rotate_delta")
    if math.hypot(*requested) > limit:
        raise ValueError(f"{action} delta norm exceeds configured limit {limit} {unit}")

    offsets = (0, 7) if arm == "both" else (0,) if arm == "left" else (7,)

    def target_from_start(start: DualPose) -> DualPose:
        target = list(start)
        for offset in offsets:
            if action == "move_delta":
                for index, value in enumerate(requested):
                    target[offset + index] += value
            else:
                roll, pitch, yaw = (value / 2.0 for value in requested)
                qx = (math.sin(roll), 0.0, 0.0, math.cos(roll))
                qy = (0.0, math.sin(pitch), 0.0, math.cos(pitch))
                qz = (0.0, 0.0, math.sin(yaw), math.cos(yaw))
                delta_q = _qmul(qz, _qmul(qy, qx))
                target[offset + 3 : offset + 7] = _normalize_quaternion(
                    _qmul(delta_q, start[offset + 3 : offset + 7]),
                    name="target quaternion",
                )
        return tuple(target)

    result = execute_pose_motion(
        config=config,
        target=target_from_start,
        action=action,
        **runtime,
    )
    return {
        **result,
        "arm": arm,
        request_key: list(requested),
        "held_arm": None if arm == "both" else "right" if arm == "left" else "left",
    }
