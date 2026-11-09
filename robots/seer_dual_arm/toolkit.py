# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""RPent toolkit for the minimal Seer fixed-action test."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

from robots.seer_dual_arm.tools import SeerDualArmPrimitives
from rpent.dashboard.events import DashboardEventSink
from rpent.session import EnvState
from rpent.tools import ToolResult, iter_tools
from rpent.tools.toolkit import Toolkit

if TYPE_CHECKING:
    from rpent.memory.manager import MemoryManager


class SeerDualArmToolkit(Toolkit):
    """Common RPent tools plus one Seer dual-arm homing action."""

    def __init__(
        self,
        *,
        runtime_kwargs: dict[str, Any],
        dashboard_events: DashboardEventSink,
        memory: MemoryManager,
        state_output_dir: Path | str,
    ) -> None:
        state = EnvState(Path(state_output_dir))
        super().__init__(dashboard_events=dashboard_events, state=state, memory=memory)
        self._primitives = SeerDualArmPrimitives(
            runtime=runtime_kwargs,
            check_cancelled=self.raise_if_cancelled,
        )
        self.add_tools(iter_tools(self._primitives))

    def get_env_state(
        self,
        *,
        command: dict[str, Any],
        result: dict[str, Any],
        elapsed_s: float,
    ) -> ToolResult:
        """Record the fixed-action result as the robot observation."""
        snapshot = {
            "robot": "seer_dual_arm",
            "target_frame": result.get("target_frame", "base_link"),
            "moved": result.get("moved", False),
            "grippers_changed": result.get("grippers_changed", False),
        }
        with self.state.record_step(
            state=snapshot,
            command=command,
            result=result,
            elapsed_s=elapsed_s,
        ):
            pass
        return ToolResult(data={**result, "step_recorded": True})
