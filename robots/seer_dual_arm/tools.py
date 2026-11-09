# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Fixed robot actions for the Seer dual-arm setup."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

from robots.seer_dual_arm.homing import HomingSession, execute_return_to_initial
from rpent.tools import ToolResult, tool
from rpent.utils.logging import get_logger

logger = get_logger("seer_dual_arm")


class SeerDualArmPrimitives:
    """One fixed homing action, with the SDK loaded only for live execution."""

    def __init__(self, *, runtime: dict[str, Any], check_cancelled: Callable[[], None]):
        self._runtime = runtime
        self._check_cancelled = check_cancelled
        self._session: HomingSession | None = None

    def close(self) -> None:
        """Close the SDK after the toolkit has drained active actions."""
        if self._session is not None:
            self._session.close()

    @tool
    def return_to_initial(self) -> ToolResult:
        """Move both arm end effectors to teleop.homing.initial_poses.

        Preview mode reports the target without connecting to the robot. Live
        mode publishes WBC end-effector commands and leaves both grippers alone.
        """
        try:
            if self._runtime["live"] and self._session is None:
                self._session = HomingSession(
                    node_name=str(self._runtime["node_name"]),
                    config_path=Path(self._runtime["robokit_config"]),
                )
            result = execute_return_to_initial(
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
            if not result["moved"]:
                logger.info(
                    "homing preview target in %s, xyz+xyzw: %s",
                    result["target_frame"],
                    result["target_xyzw"],
                )
            return ToolResult(data=result)
        except Exception as exc:
            logger.exception("Seer dual-arm homing failed")
            return ToolResult(
                data={"success": False, "error": f"{type(exc).__name__}: {exc}"}
            )
