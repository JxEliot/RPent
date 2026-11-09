# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Seer dual-arm homing using only the standard library and lazy SDK imports."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from rpent.utils.logging import get_logger

logger = get_logger("seer_dual_arm")

Pose = tuple[float, float, float, float, float, float, float]
DualPose = tuple[float, ...]


@dataclass(frozen=True)
class HomingConfig:
    """Validated homing target and arrival thresholds."""

    target: DualPose
    position_threshold_m: float
    rotation_threshold_deg: float
    stable_cycles: int


def _finite_positive(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite and positive")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be finite and positive") from exc
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _normalize_quaternion(
    values: Any, *, name: str
) -> tuple[float, float, float, float]:
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        raise ValueError(f"{name} must contain xyzw values")
    try:
        quaternion = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain finite values") from exc
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError(f"{name} must contain finite values")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-9:
        raise ValueError(f"{name} must not be a zero quaternion")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def load_homing_config(path: Path | str) -> HomingConfig:
    """Load both initial EE poses from ``teleop.homing`` in Seer config."""
    source = Path(path).expanduser()
    raw = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("rbk config must be a JSON object")
    teleop = raw.get("teleop")
    if not isinstance(teleop, dict):
        raise ValueError("rbk config must define teleop")
    homing = teleop.get("homing")
    if not isinstance(homing, dict):
        raise ValueError("rbk config must define teleop.homing")
    if homing.get("enabled", True) is not True:
        raise ValueError("teleop.homing.enabled must be true")
    poses = homing.get("initial_poses")
    if not isinstance(poses, dict):
        raise ValueError("teleop.homing.initial_poses must define left and right")
    target: list[float] = []
    for side in ("left", "right"):
        pose = poses.get(side)
        if not isinstance(pose, dict):
            raise ValueError(f"teleop.homing.initial_poses.{side} is required")
        position = pose.get("pos")
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            raise ValueError(f"{side} initial pose pos must contain xyz values")
        try:
            values = tuple(float(value) for value in position)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"{side} initial pose pos must contain finite values"
            ) from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{side} initial pose pos must contain finite values")
        target.extend(values)
        target.extend(
            _normalize_quaternion(pose.get("quat"), name=f"{side} initial pose quat")
        )
    position_threshold = _finite_positive(
        homing.get("position_threshold_m", 0.02), "position_threshold_m"
    )
    rotation_threshold = _finite_positive(
        homing.get("rotation_threshold_deg", 5.0), "rotation_threshold_deg"
    )
    stable_cycles = homing.get("stable_cycles", 15)
    if (
        isinstance(stable_cycles, bool)
        or not isinstance(stable_cycles, int)
        or stable_cycles < 1
    ):
        raise ValueError("stable_cycles must be a positive integer")
    return HomingConfig(
        target=tuple(target),
        position_threshold_m=position_threshold,
        rotation_threshold_deg=rotation_threshold,
        stable_cycles=stable_cycles,
    )


def _qmul(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _qrotate(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotated = _qmul(
        _qmul(quaternion, (*vector, 0.0)),
        (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]),
    )
    return rotated[:3]


def _pose_compose(parent: Pose, child: Pose) -> Pose:
    translated = _qrotate(parent[3:], child[:3])
    return (
        parent[0] + translated[0],
        parent[1] + translated[1],
        parent[2] + translated[2],
        *_normalize_quaternion(
            _qmul(parent[3:], child[3:]), name="composed quaternion"
        ),
    )


def _pose_inverse(pose: Pose) -> Pose:
    rotation = (-pose[3], -pose[4], -pose[5], pose[6])
    translated = _qrotate(rotation, (-pose[0], -pose[1], -pose[2]))
    return (*translated, *rotation)


def _pose_graph_path(
    edges: list[tuple[str, str, Pose]], start_frames: list[str], target: str
) -> Pose | None:
    adjacency: dict[str, list[tuple[str, Pose]]] = {}
    for parent, child, pose in edges:
        adjacency.setdefault(parent, []).append((child, pose))
        adjacency.setdefault(child, []).append((parent, _pose_inverse(pose)))
    for start in start_frames:
        queue = deque([(start, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))])
        visited = {start}
        while queue:
            frame, start_to_frame = queue.popleft()
            if frame == target:
                return start_to_frame
            for next_frame, frame_to_next in adjacency.get(frame, []):
                if next_frame in visited:
                    continue
                visited.add(next_frame)
                queue.append((next_frame, _pose_compose(start_to_frame, frame_to_next)))
    return None


def _read_measured_poses(message: Any) -> DualPose | None:
    edges: list[tuple[str, str, Pose]] = []
    targets: dict[str, str] = {}
    for transform in getattr(message, "transform", ()):
        parent = str(getattr(transform, "frameid", ""))
        child = str(getattr(transform, "childframeid", ""))
        translation = getattr(transform, "translation", None)
        rotation = getattr(transform, "rotation", None)
        if not parent or not child or translation is None or rotation is None:
            continue
        pose: Pose = (
            float(translation.x),
            float(translation.y),
            float(translation.z),
            *_normalize_quaternion(
                (rotation.x, rotation.y, rotation.z, rotation.w),
                name="feedback quaternion",
            ),
        )
        edges.append((parent, child, pose))
        leaf = child.strip("/").split("/")[-1].lower()
        if leaf.endswith("_ee_link"):
            if "left_ee" in leaf:
                targets["left"] = child
            elif "right_ee" in leaf:
                targets["right"] = child
    if set(targets) != {"left", "right"}:
        return None
    frames = {frame for parent, child, _ in edges for frame in (parent, child)}
    bases = [
        frame
        for frame in frames
        if frame.strip("/").split("/")[-1].lower() in {"base", "base_link"}
    ]
    resolved = [
        _pose_graph_path(edges, bases, targets[side]) for side in ("left", "right")
    ]
    if any(pose is None for pose in resolved):
        return None
    return tuple(value for pose in resolved for value in pose or ())


def interpolate_poses(start: DualPose, target: DualPose, fraction: float) -> DualPose:
    """Interpolate two base-frame poses with smoothstep and quaternion SLERP."""
    amount = min(1.0, max(0.0, float(fraction)))
    weight = amount * amount * (3.0 - 2.0 * amount)
    values: list[float] = []
    for offset in (0, 7):
        values.extend(
            start[offset + index] * (1.0 - weight) + target[offset + index] * weight
            for index in range(3)
        )
        first = _normalize_quaternion(
            start[offset + 3 : offset + 7], name="start quaternion"
        )
        second = _normalize_quaternion(
            target[offset + 3 : offset + 7], name="target quaternion"
        )
        dot = sum(a * b for a, b in zip(first, second, strict=True))
        if dot < 0.0:
            second = tuple(-value for value in second)
            dot = -dot
        if dot > 0.9995:
            quaternion = _normalize_quaternion(
                tuple(a + weight * (b - a) for a, b in zip(first, second, strict=True)),
                name="interpolated quaternion",
            )
        else:
            angle = math.acos(min(1.0, max(-1.0, dot)))
            sine = math.sin(angle)
            left = math.sin((1.0 - weight) * angle) / sine
            right = math.sin(weight * angle) / sine
            quaternion = tuple(
                left * a + right * b for a, b in zip(first, second, strict=True)
            )
        values.extend(quaternion)
    return tuple(values)


def pose_errors(
    current: DualPose, target: DualPose
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return position metres and orientation degrees for both arms."""
    errors = []
    for offset in (0, 7):
        distance = math.sqrt(
            sum(
                (current[offset + index] - target[offset + index]) ** 2
                for index in range(3)
            )
        )
        first = _normalize_quaternion(
            current[offset + 3 : offset + 7], name="current quaternion"
        )
        second = _normalize_quaternion(
            target[offset + 3 : offset + 7], name="target quaternion"
        )
        dot = abs(sum(a * b for a, b in zip(first, second, strict=True)))
        angle_deg = math.degrees(2.0 * math.acos(min(1.0, max(0.0, dot))))
        errors.append((distance, angle_deg))
    return errors[0], errors[1]


def _send_pose(
    datapool: Any, message_type: type, topic: str, values: DualPose, sequence: int
) -> None:
    message = message_type()
    message.header.dataNsec = message.header.pubNsec = time.time_ns()
    message.header.frameID = "base_link"
    message.header.seq = sequence
    for index, side in enumerate(("left", "right")):
        pose = getattr(message, f"{side}EEPose")
        for field, value in zip(
            ("x", "y", "z", "qx", "qy", "qz", "qw"),
            values[index * 7 : (index + 1) * 7],
            strict=True,
        ):
            setattr(pose, field, float(value))
        velocity = getattr(message, f"{side}EEVel")
        for field in (
            "linear_x",
            "linear_y",
            "linear_z",
            "angular_x",
            "angular_y",
            "angular_z",
        ):
            setattr(velocity, field, 0.0)
    datapool.put(topic, message)


def _message_signature(message: Any) -> tuple[int, int, int, bytes]:
    header = getattr(message, "header", None)
    sequence = int(getattr(header, "seq", 0)) if header is not None else 0
    timestamp = int(getattr(header, "dataNsec", 0)) if header is not None else 0
    published = int(getattr(header, "pubNsec", 0)) if header is not None else 0
    try:
        payload = message.SerializeToString()
    except Exception:
        payload = repr(message).encode("utf-8", errors="replace")
    return sequence, timestamp, published, payload


def execute_return_to_initial(
    *,
    rbk_config: Path | str,
    robokit_config: Path | str,
    live: bool,
    duration: float,
    timeout: float,
    feedback_timeout: float,
    hz: float,
    node_name: str,
    check_cancelled: Callable[[], None],
) -> dict[str, Any]:
    """Preview or execute the configured dual-arm homing action."""
    config = load_homing_config(rbk_config)
    duration = _finite_positive(duration, "duration")
    timeout = _finite_positive(timeout, "timeout")
    feedback_timeout = _finite_positive(feedback_timeout, "feedback_timeout")
    hz = _finite_positive(hz, "hz")
    if timeout <= duration:
        raise ValueError("timeout must exceed duration")
    target = list(config.target)
    if not live:
        return {
            "success": True,
            "moved": False,
            "summary": "Preview only; the robot was not moved.",
            "target_frame": "base_link",
            "target_xyzw": target,
            "grippers_changed": False,
        }

    from robokit.core import datapool, system
    from robokit.protocol.messageV4_stampedtransform_pb2 import (
        MessageV4_StampedTransforms,
    )
    from robokit.protocol.messageV4_wbccmd_pb2 import WBCCommand

    action_topic = "/RobotControl/WbcEECommand"
    feedback_topic = "/RobotControl/WbcDebugTF"
    system.init(node_name, config_path=Path(robokit_config))
    feedback_reader: Any | None = None
    motion_started = False
    completed = False
    sequence = 0
    latest: DualPose | None = None
    latest_at = 0.0
    previous_signature: tuple[int, int, int, bytes] | None = None

    def current_pose() -> DualPose | None:
        nonlocal latest, latest_at, previous_signature
        message = datapool.get(feedback_topic, MessageV4_StampedTransforms)
        if message is not None:
            signature = _message_signature(message)
            if signature != previous_signature:
                previous_signature = signature
                pose = _read_measured_poses(message)
                if pose is not None:
                    latest = pose
                    latest_at = time.monotonic()
        if latest is None or time.monotonic() - latest_at > 0.5:
            return None
        return latest

    try:
        feedback_reader = datapool.subscribe(
            feedback_topic, MessageV4_StampedTransforms
        )
        datapool.publish(action_topic, WBCCommand)
        logger.info("waiting for measured left/right EE feedback on %s", feedback_topic)
        deadline = time.monotonic() + feedback_timeout
        start: DualPose | None = None
        while time.monotonic() < deadline:
            check_cancelled()
            start = current_pose()
            if start is not None:
                break
            time.sleep(0.02)
        if start is None:
            raise RuntimeError("no fresh measured EE feedback; homing not started")

        started = time.monotonic()
        stable = 0
        next_log = started
        while True:
            check_cancelled()
            current = current_pose()
            if current is None:
                raise RuntimeError("EE feedback stale; homing stopped")
            elapsed = time.monotonic() - started
            if elapsed > timeout:
                raise RuntimeError("homing timed out")
            command = interpolate_poses(start, config.target, elapsed / duration)
            _send_pose(datapool, WBCCommand, action_topic, command, sequence)
            sequence += 1
            motion_started = True
            errors = pose_errors(current, config.target)
            arrived = elapsed >= duration and all(
                distance <= config.position_threshold_m
                and angle <= config.rotation_threshold_deg
                for distance, angle in errors
            )
            stable = stable + 1 if arrived else 0
            if time.monotonic() >= next_log:
                logger.info(
                    "left/right homing error (m, deg): %s; stable=%d/%d",
                    errors,
                    stable,
                    config.stable_cycles,
                )
                next_log = time.monotonic() + 1.0
            if stable >= config.stable_cycles:
                completed = True
                logger.info("both arms reached the configured initial poses")
                return {
                    "success": True,
                    "moved": True,
                    "summary": "Both arms reached the configured initial poses.",
                    "target_frame": "base_link",
                    "target_xyzw": target,
                    "position_error_m": [errors[0][0], errors[1][0]],
                    "rotation_error_deg": [errors[0][1], errors[1][1]],
                    "stable_cycles": stable,
                    "grippers_changed": False,
                }
            time.sleep(1.0 / hz)
    finally:
        try:
            if feedback_reader is not None and motion_started and not completed:
                current = current_pose()
                if current is not None:
                    _send_pose(datapool, WBCCommand, action_topic, current, sequence)
                    time.sleep(0.1)
                    logger.warning("homing stopped; sent a hold at the current EE pose")
        finally:
            if feedback_reader is not None:
                unsubscribe = getattr(datapool, "unsubscribe", None)
                if callable(unsubscribe):
                    unsubscribe(feedback_reader)
            system.shutdown()
