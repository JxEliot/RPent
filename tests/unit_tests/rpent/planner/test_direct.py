# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

from __future__ import annotations

from unittest.mock import Mock

from rpent.planner.direct import DirectToolPlanner
from rpent.tools import ToolResult


def test_direct_planner_executes_selected_tool_once_without_model() -> None:
    toolkit = Mock()
    toolkit.execute_tool.return_value = ToolResult(
        data={"success": True, "summary": "Preview complete."}
    )
    planner = DirectToolPlanner(
        tool_name="return_to_initial", tool_args={"live": False}
    )

    result = planner.solve(
        system_prompt="unused",
        user_message="unused",
        toolkit=toolkit,
        max_turns=100,
    )

    toolkit.execute_tool.assert_called_once_with("return_to_initial", {"live": False})
    assert result.error is None
    assert result.finish_result == {
        "_finish": True,
        "status": "success",
        "summary": "Preview complete.",
    }
    assert result.stats["tool_calls"] == 1
    assert result.stats["turns_used"] == 0


def test_direct_planner_surfaces_tool_error() -> None:
    toolkit = Mock()
    toolkit.execute_tool.return_value = ToolResult(data={"error": "feedback stale"})

    result = DirectToolPlanner(tool_name="return_to_initial").solve(
        system_prompt="",
        user_message="",
        toolkit=toolkit,
        max_turns=1,
    )

    assert result.finish_result["status"] == "failure"
    assert result.error == "feedback stale"
