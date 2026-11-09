# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Homing and Cartesian tools for the Seer dual-arm setup."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Any, Callable, Literal

from pydantic import BeforeValidator, Field

from robots.seer_dual_arm.cartesian import coerce_arm, execute_cartesian_delta
from robots.seer_dual_arm.gripper import _percent, execute_gripper
from robots.seer_dual_arm.homing import HomingSession, execute_return_to_initial
from rpent.tools import ToolResult, tool
from rpent.utils.logging import get_logger

logger = get_logger("seer_dual_arm")


class SeerDualArmPrimitives:
    """EE actions with one lazily initialized SDK session."""

    def __init__(self, *, runtime: dict[str, Any], check_cancelled: Callable[[], None]):
        self._runtime = runtime
        self._check_cancelled = check_cancelled
        self._session: HomingSession | None = None

    def close(self) -> None:
        """Close the SDK after the toolkit has drained active actions."""
        if self._session is not None:
            self._session.close()

    def _execute(
        self, operation: Callable[..., dict[str, Any]], **kwargs
    ) -> ToolResult:
        try:
            if self._runtime["live"] and self._session is None:
                self._session = HomingSession(
                    node_name=str(self._runtime["node_name"]),
                    config_path=Path(self._runtime["robokit_config"]),
                )
            result = operation(
                **kwargs,
                rbk_config=Path(self._runtime["rbk_config"]),
                robokit_config=Path(self._runtime["robokit_config"]),
                live=bool(self._runtime["live"]),
                duration=float(self._runtime["duration"]),
                timeout=float(self._runtime["timeout"]),
                feedback_timeout=float(self._runtime["feedback_timeout"]),
                hz=float(self._runtime["hz"]),
                node_name=str(self._runtime["node_name"]),
                check_cancelled=self._check_cancelled,
                session=self._session,
            )
            if not self._runtime["live"]:
                logger.info(
                    "Seer action preview: %s",
                    result,
                )
            return ToolResult(data=result)
        except Exception as exc:
            logger.exception("Seer dual-arm action failed")
            return ToolResult(
                data={"success": False, "error": f"{type(exc).__name__}: {exc}"}
            )

    @tool
    def return_to_initial(self) -> ToolResult:
        """Move both arm end effectors to teleop.homing.initial_poses.

        Preview reports the target without connecting to the robot. Live mode
        publishes WBC end-effector commands. Both grippers remain unchanged.
        """
        return self._execute(execute_return_to_initial)

    @tool
    def move_delta(
        self,
        arm: Annotated[Literal["left", "right", "both"], BeforeValidator(coerce_arm)],
        delta_xyz: Annotated[list[float], Field(min_length=3, max_length=3)],
    ) -> ToolResult:
        """Translate selected EEs by [dx, dy, dz] metres in base_link.

        arm='both' moves both arms by the same delta in one synchronized WBC
        trajectory. Preserves each orientation; a single-arm move holds the
        other arm at its measured start. Both grippers stay unchanged.
        The vector norm must fit teleop.cartesian.max_translation_m
        (default 0.10 m). Preview reports the increment without reading hardware.
        """
        return self._execute(
            execute_cartesian_delta, action="move_delta", arm=arm, delta=delta_xyz
        )

    @tool
    def rotate_delta(
        self,
        arm: Annotated[Literal["left", "right"], BeforeValidator(coerce_arm)],
        delta_rpy: Annotated[list[float], Field(min_length=3, max_length=3)],
    ) -> ToolResult:
        """Rotate one end effector by extrinsic [roll, pitch, yaw] radians in base_link.

        Applies q_delta * q_current, holding its position and the other arm's
        measured start pose. The RPY vector norm must fit
        teleop.cartesian.max_rotation_rad (default 0.5 rad). Both grippers stay
        unchanged. Preview reports the increment without reading hardware.
        """
        return self._execute(
            execute_cartesian_delta, action="rotate_delta", arm=arm, delta=delta_rpy
        )

    @tool
    def open_gripper(
        self,
        arm: Annotated[Literal["left", "right", "both"], BeforeValidator(coerce_arm)],
    ) -> ToolResult:
        """Open selected grippers. Both sides use separate topic commands."""
        return self._execute(execute_gripper, action="open", arm=arm)

    @tool
    def close_gripper(
        self,
        arm: Annotated[Literal["left", "right", "both"], BeforeValidator(coerce_arm)],
    ) -> ToolResult:
        """Close selected grippers. A target position does not establish a grasp."""
        return self._execute(execute_gripper, action="close", arm=arm)

    @tool
    def set_gripper_position(
        self,
        arm: Annotated[Literal["left", "right", "both"], BeforeValidator(coerce_arm)],
        position: Annotated[
            float, BeforeValidator(lambda value: _percent(value, "position"))
        ],
    ) -> ToolResult:
        """Set selected grippers to 0-100, where 0 is closed and 100 open."""
        return self._execute(execute_gripper, action="set", arm=arm, position=position)

    @tool
    def get_gripper_state(self) -> ToolResult:
        """Read both grippers' service status and measured feedback in live mode."""
        return self._execute(execute_gripper, action="state")
