# Copyright 2026 The RPent Authors.
# Licensed under the Apache License, Version 2.0.

"""Gripper topic client for the independently managed Seer gripper service."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from robots.seer_dual_arm.cartesian import coerce_arm
from robots.seer_dual_arm.homing import HomingSession, _finite_positive


@dataclass(frozen=True)
class GripperConfig:
    """Validated command parameters and feedback limits."""

    open_position: float
    close_position: float
    velocity: float
    acceleration: float
    deceleration: float
    force: float
    position_tolerance: float
    status_timeout: float
    action_timeout: float
    max_status_age: float


def _percent(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number in [0, 100]")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number in [0, 100]") from exc
    if not math.isfinite(number) or not 0 <= number <= 100:
        raise ValueError(f"{name} must be a finite number in [0, 100]")
    return number


def load_gripper_config(path: Path | str) -> GripperConfig:
    """Load ``teleop.gripper``; old robot configs use service-compatible defaults."""
    raw = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or not isinstance(raw.get("teleop"), dict):
        raise ValueError("robot config must define a teleop object")
    settings = raw["teleop"].get("gripper", {})
    if not isinstance(settings, dict):
        raise ValueError("teleop.gripper must be an object")
    config = GripperConfig(
        open_position=_percent(settings.get("open_position", 100), "open_position"),
        close_position=_percent(settings.get("close_position", 0), "close_position"),
        velocity=_percent(settings.get("velocity", 100), "velocity"),
        acceleration=_percent(settings.get("acceleration", 100), "acceleration"),
        deceleration=_percent(settings.get("deceleration", 100), "deceleration"),
        force=_percent(settings.get("force", 100), "force"),
        position_tolerance=_finite_positive(
            settings.get("position_tolerance", 2), "position_tolerance"
        ),
        status_timeout=_finite_positive(
            settings.get("status_timeout_s", 3), "status_timeout_s"
        ),
        action_timeout=_finite_positive(
            settings.get("action_timeout_s", 10), "action_timeout_s"
        ),
        max_status_age=_finite_positive(
            settings.get("max_status_age_s", 0.5), "max_status_age_s"
        ),
    )
    if config.open_position <= config.close_position:
        raise ValueError("open_position must exceed close_position")
    if config.position_tolerance > 100:
        raise ValueError("position_tolerance must not exceed 100")
    return config


def _parse_status(message: Any, max_age: float) -> dict[str, Any] | None:
    if message is None:
        return None
    try:
        item = next(item for item in message.data if item.key == "gripper_status")
        status = json.loads(item.string_value)
        timestamp = status["emitted_unix_ns"]
        sequence = status["sequence"]
        if (
            status.get("schema") != "seer.gripper.status.v1"
            or isinstance(timestamp, bool)
            or not isinstance(timestamp, int)
            or timestamp <= 0
            or isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or sequence <= 0
            or not isinstance(status.get("feedback"), dict)
            or not isinstance(status.get("applied"), dict)
            or not isinstance(status.get("errors"), dict)
        ):
            return None
        age = (time.time_ns() - timestamp) / 1e9
        if abs(age) > max_age:
            return None
        return status
    except (
        StopIteration,
        AttributeError,
        TypeError,
        ValueError,
        KeyError,
        OverflowError,
    ):
        return None


def _wait_status(
    datapool: Any,
    message_type: type,
    config: GripperConfig,
    check_cancelled: Callable[[], None],
    *,
    after: tuple[int, int] | None = None,
) -> dict[str, Any]:
    deadline = time.monotonic() + config.status_timeout
    while time.monotonic() < deadline:
        check_cancelled()
        status = _parse_status(
            datapool.get("/GripperService/Status", message_type), config.max_status_age
        )
        if status is not None:
            identity = (status["emitted_unix_ns"], status["sequence"])
            if after is None or identity > after:
                return status
        time.sleep(0.02)
    raise RuntimeError(
        "no fresh gripper service status; check that the service is running"
    )


def _identity(status: dict[str, Any]) -> tuple[int, int]:
    return status["emitted_unix_ns"], status["sequence"]


def _feedback_sequence(status: dict[str, Any], side: str) -> int:
    feedback = status["feedback"].get(side)
    value = feedback.get("feedback_sequence") if isinstance(feedback, dict) else None
    return (
        value
        if isinstance(value, int) and not isinstance(value, bool) and value >= 0
        else 0
    )


def _observe(
    status: dict[str, Any],
    side: str,
    target: float,
    config: GripperConfig,
    baseline: int,
    published_ns: int,
) -> str:
    error = status["errors"].get(side)
    if error:
        return f"service error: {error}"
    applied = status["applied"].get(side)
    if not isinstance(applied, dict):
        return "waiting for applied command"
    desired = {
        "position": target,
        "velocity": config.velocity,
        "acceleration": config.acceleration,
        "deceleration": config.deceleration,
        "force": config.force,
    }
    try:
        if any(
            not math.isfinite(float(applied[k])) or abs(float(applied[k]) - v) > 1e-4
            for k, v in desired.items()
        ):
            return "waiting for matching applied command"
    except (KeyError, TypeError, ValueError, OverflowError):
        return "waiting for matching applied command"
    feedback = status["feedback"].get(side)
    if not isinstance(feedback, dict) or feedback.get("feedback_fresh") is not True:
        return "waiting for fresh measured feedback"
    timestamp = feedback.get("timestamp_ns")
    if (
        _feedback_sequence(status, side) <= baseline
        or isinstance(timestamp, bool)
        or not isinstance(timestamp, int)
        or timestamp < published_ns
    ):
        return "waiting for feedback after command"
    fault, state = feedback.get("fault_code"), feedback.get("state_code")
    if isinstance(fault, bool) or not isinstance(fault, int):
        return "waiting for fault status"
    if fault != 0:
        return f"hardware fault: {feedback.get('fault', fault)}"
    if state == 2:
        return "gripper stalled; grasp is not confirmed"
    if state == 3:
        return "gripper reports object dropped"
    try:
        measured = float(feedback["measured_position"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return "waiting for measured position"
    if not math.isfinite(measured) or not 0 <= measured <= 100:
        return "invalid measured position"
    if (
        isinstance(state, bool)
        or state != 0
        or abs(measured - target) > config.position_tolerance
    ):
        return "waiting for target position"
    return "target observed"


def execute_gripper(
    *,
    action: str,
    rbk_config: Path | str,
    robokit_config: Path | str,
    live: bool,
    node_name: str,
    check_cancelled: Callable[[], None],
    session: HomingSession | None = None,
    arm: str = "both",
    position: float | None = None,
    **_runtime: Any,
) -> dict[str, Any]:
    """Preview or execute a gripper action using service status and GripCmd topics."""
    config = load_gripper_config(rbk_config)
    arm = coerce_arm(arm)
    if action not in {"open", "close", "set", "state"}:
        raise ValueError("unknown gripper action")
    target = (
        config.open_position
        if action == "open"
        else config.close_position
        if action == "close"
        else _percent(position, "position")
        if action == "set"
        else None
    )
    sides = ("left", "right") if arm == "both" else (arm,)
    result: dict[str, Any] = {
        "success": False,
        "action": action,
        "arm": arm,
        "target_position": target,
        "command_attempted_sides": [],
        "target_observed": False,
        "grippers_changed": False,
        "moved": False,
    }
    check_cancelled()
    if not live:
        return {
            **result,
            "success": True,
            "summary": "Preview only; no gripper feedback read or command sent.",
        }

    from robokit.core import datapool
    from robokit.protocol.messageV4_grip_pb2 import THUMB, MessageV4_grip
    from robokit.protocol.messageV4_universal_pb2 import MessageV4_UniversalDatas

    owns_session = session is None
    session = session or HomingSession(
        node_name=node_name, config_path=Path(robokit_config)
    )
    try:
        session.start()
        datapool.subscribe("/GripperService/Status", MessageV4_UniversalDatas)
        # A fresh-looking retained sample alone does not prove the service is alive.
        cached = _wait_status(
            datapool, MessageV4_UniversalDatas, config, check_cancelled
        )
        status = _wait_status(
            datapool,
            MessageV4_UniversalDatas,
            config,
            check_cancelled,
            after=_identity(cached),
        )
        if status["sequence"] <= cached["sequence"]:
            raise RuntimeError(
                "gripper service restarted; retry after checking its state"
            )
        if action == "state":
            return {
                **result,
                "success": True,
                "status": status,
                "summary": "Fresh service status; inspect feedback freshness separately.",
            }
        for side in sides:
            if status["errors"].get(side):
                raise RuntimeError(
                    f"{side} gripper service error: {status['errors'][side]}"
                )
        baseline = {side: _feedback_sequence(status, side) for side in sides}
        published_ns: dict[str, int] = {}
        for side in sides:
            check_cancelled()
            topic = f"/GripCmd/Gripper-{'000' if side == 'left' else '001'}"
            message = MessageV4_grip()
            message.GripFinger.type = THUMB
            control = message.GripFinger.fingerinfos.control
            control.Flexion, control.vel, control.Acc, control.Dec, control.force = (
                target,
                config.velocity,
                config.acceleration,
                config.deceleration,
                config.force,
            )
            datapool.publish(topic, MessageV4_grip)
            # A transport failure can happen after a message has left the process.
            result["command_attempted_sides"].append(side)
            result["grippers_changed"] = True
            published_ns[side] = time.time_ns()
            datapool.put(topic, message)
        deadline = time.monotonic() + config.action_timeout
        identity = _identity(status)
        observations: dict[str, str] = {}
        while time.monotonic() < deadline:
            check_cancelled()
            status = _parse_status(
                datapool.get("/GripperService/Status", MessageV4_UniversalDatas),
                config.max_status_age,
            )
            if status is not None and _identity(status) > identity:
                if status["sequence"] <= identity[1]:
                    return {
                        **result,
                        "error": "Gripper service restarted; completion is unknown.",
                    }
                identity = _identity(status)
                observations = {
                    side: _observe(
                        status, side, target, config, baseline[side], published_ns[side]
                    )
                    for side in sides
                }
                result["observations"] = observations
                result["status"] = status
                if all(value == "target observed" for value in observations.values()):
                    return {
                        **result,
                        "success": True,
                        "target_observed": True,
                        "summary": "Target observed in fresh feedback; topic protocol has no request ID.",
                    }
                if any(
                    value.startswith(
                        (
                            "service error:",
                            "hardware fault:",
                            "gripper stalled",
                            "gripper reports",
                        )
                    )
                    for value in observations.values()
                ):
                    return {
                        **result,
                        "error": "Gripper service or hardware reported an error.",
                    }
            time.sleep(0.02)
        return {
            **result,
            "error": "Timed out waiting for fresh target feedback; completion is unknown.",
        }
    except Exception as exc:
        return {**result, "error": f"{type(exc).__name__}: {exc}"}
    finally:
        if owns_session:
            session.close()
