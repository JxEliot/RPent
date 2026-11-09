# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Planner that executes one explicitly selected toolkit action."""

from __future__ import annotations

import queue
import time
from typing import Any

from rpent.dashboard.interaction import DashboardInteractionPort
from rpent.planner.base import Planner, PlannerResult
from rpent.tools.toolkit import Toolkit


class DirectToolPlanner(Planner):
    """Execute one registered tool call without contacting a model service."""

    def __init__(self, *, tool_name: str, tool_args: dict[str, Any] | None = None):
        if not tool_name:
            raise ValueError("tool_name must be non-empty")
        if tool_args is not None and not isinstance(tool_args, dict):
            raise TypeError("tool_args must be a dictionary")
        self._tool_name = tool_name
        self._tool_args = dict(tool_args or {})

    def solve(
        self,
        *,
        system_prompt: str,
        user_message: str,
        toolkit: Toolkit,
        max_turns: int,
        input_queue: queue.Queue[str | None] | None = None,
        dashboard_interaction: DashboardInteractionPort | None = None,
    ) -> PlannerResult:
        """Run the selected tool once and return its result as the session result."""
        del system_prompt, user_message, max_turns, input_queue, dashboard_interaction
        started = time.perf_counter()
        result = toolkit.execute_tool(self._tool_name, self._tool_args)
        elapsed_s = round(time.perf_counter() - started, 3)
        payload = result.to_dict()
        error = payload.get("error")
        status = "failure" if result.is_error else "success"
        summary = payload.get("summary")
        if not isinstance(summary, str) or not summary:
            summary = (
                f"Tool {self._tool_name} failed."
                if result.is_error
                else f"Executed tool {self._tool_name}."
            )
        messages = [
            {
                "role": "assistant",
                "tool_call": {"name": self._tool_name, "arguments": self._tool_args},
            },
            {"role": "tool", "name": self._tool_name, "content": result.to_text()},
        ]
        return PlannerResult(
            finish_result={"_finish": True, "status": status, "summary": summary},
            messages=messages,
            stats={"turns_used": 0, "tool_calls": 1, "elapsed_s": elapsed_s},
            error=str(error) if result.is_error else None,
        )
