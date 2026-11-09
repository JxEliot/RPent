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
    status_timeout: float
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
        status_timeout=_finite_positive(
            settings.get("status_timeout_s", 3), "status_timeout_s"
        ),
        max_status_age=_finite_positive(
            settings.get("max_status_age_s", 0.5), "max_status_age_s"
        ),
    )
    if config.open_position <= config.close_position:
        raise ValueError("open_position must exceed close_position")
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
        "command_published_sides": [],
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

    owns_session = session is None
    session = session or HomingSession(
        node_name=node_name, config_path=Path(robokit_config)
    )
    try:
        session.start()
        if action == "state":
            from robokit.protocol.messageV4_universal_pb2 import (
                MessageV4_UniversalDatas,
            )

            datapool.subscribe("/GripperService/Status", MessageV4_UniversalDatas)
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
            return {
                **result,
                "success": True,
                "status": status,
                "summary": "Fresh service status; inspect feedback freshness separately.",
            }
        from robokit.protocol.messageV4_grip_pb2 import THUMB, MessageV4_grip

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
            datapool.put(topic, message)
            result["command_published_sides"].append(side)
        return {
            **result,
            "success": True,
            "summary": "Commands published; service receipt and physical motion were not checked.",
        }
    except Exception as exc:
        return {**result, "error": f"{type(exc).__name__}: {exc}"}
    finally:
        if owns_session:
            session.close()
