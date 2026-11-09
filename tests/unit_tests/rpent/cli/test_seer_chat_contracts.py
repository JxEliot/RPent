# Copyright 2026 The RPent Authors.
# Licensed under the Apache License, Version 2.0.

"""Seer terminal chat uses the registered action and waits for operator input."""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from pydantic_ai.models.function import DeltaToolCall, FunctionModel


@pytest.mark.parametrize("live", [False, True])
def test_api_chat_waits_for_input_executes_homing_and_keeps_conversation(
    tmp_path: Path, monkeypatch, live: bool
) -> None:
    import pydantic_ai._cli as native_cli

    from rpent.cli import main as cli
    from rpent.utils.logging import init_output_dir

    config = tmp_path / "robot.json"
    config.write_text(
        json.dumps(
            {
                "teleop": {
                    "homing": {
                        "initial_poses": {
                            "left": {"pos": [0.6, 0.2, 1.0], "quat": [0, 0, 0, 1]},
                            "right": {"pos": [0.6, -0.2, 1.0], "quat": [0, 0, 0, 1]},
                        }
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    dds = tmp_path / "robokit.json"
    dds.write_text("{}", encoding="utf-8")
    api_config = tmp_path / "api.toml"
    api_config.write_text(
        'model = "gpt-6-astra"\nbase_url = "https://relay.invalid/openai"\n'
        'wire_api = "responses"\napi_key = "offline-test-key"\n',
        encoding="utf-8",
    )
    output = tmp_path / "output"
    init_output_dir(output)
    requests = []
    actions = []
    inputs = iter(["回到初始位置", "刚刚做了什么？", "/exit"])

    async def read_prompt(*args, **kwargs):
        if not actions:
            assert not requests, "The model must wait for the first operator message"
        return next(inputs)

    async def stream(messages, info):
        requests.append(copy.deepcopy(messages))
        if len(requests) == 1:
            yield {0: DeltaToolCall(name="return_to_initial", json_args="{}")}
        else:
            yield "回初始位动作已完成。" if live else "预览完成，机器人未移动。"

    def execute(**kwargs):
        actions.append(kwargs)
        assert kwargs["live"] is live
        assert kwargs["rbk_config"] == config
        assert kwargs["robokit_config"] == dds
        return {"success": True, "moved": live, "grippers_changed": False}

    def build_model(model, base_url, *, api_config):
        assert model == "openai:gpt-6-astra"
        assert api_config == str(tmp_path / "api.toml")
        return FunctionModel(stream_function=stream)

    monkeypatch.setattr(native_cli, "PYDANTIC_AI_HOME", tmp_path / "cli")
    monkeypatch.setattr(
        native_cli,
        "PromptSession",
        lambda **kwargs: SimpleNamespace(prompt_async=read_prompt),
    )
    monkeypatch.setenv("PYDANTIC_AI_NO_BANNER", "1")
    monkeypatch.setattr(sys.stdin, "isatty", lambda: True)
    monkeypatch.setattr("robots.seer_dual_arm.robot_spec.CONFIG_DIR", tmp_path)
    monkeypatch.setattr(
        "rpent.planner.base.build_api_model",
        build_model,
    )
    monkeypatch.setattr("robots.seer_dual_arm.tools.execute_return_to_initial", execute)
    monkeypatch.setattr(
        "rpent.memory.MemoryManager.sync",
        lambda *a, **kw: pytest.fail("local chat must not sync memory"),
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "rpent",
            "--robot",
            "seer_dual_arm",
            "--planner",
            "api",
            "--tui",
            "--output-dir",
            str(output),
            "--memory-dir",
            str(tmp_path / "memory"),
            *(["--live"] if live else []),
        ],
    )

    assert cli.main() == 0
    assert len(actions) == 1
    assert len(requests) == 3
    transcript = json.loads(next(output.glob("transcript_*.json")).read_text())
    assert transcript["finish"] is None
    assert transcript["error"] is None
    assert [m["content"] for m in transcript["messages"] if m["role"] == "user"] == [
        "回到初始位置",
        "刚刚做了什么？",
    ]
    steps = json.loads((output / "states.json").read_text())["steps"]
    assert len(steps) == 1
    assert steps[0]["state"]["moved"] is live


def test_claude_chat_exposes_homing_without_builtin_or_file_tools(
    tmp_path: Path,
) -> None:
    import claude_agent_sdk

    from robots.seer_dual_arm.toolkit import SeerDualArmToolkit
    from rpent.dashboard.events import NullDashboardEventSink
    from rpent.memory import MemoryManager
    from rpent.planner.base import build_planner

    sink = NullDashboardEventSink()
    toolkit = SeerDualArmToolkit(
        runtime_kwargs={},
        dashboard_events=sink,
        memory=MemoryManager(tmp_path / "memory"),
        state_output_dir=tmp_path,
        interactive=True,
    )
    planner = build_planner(
        "claude_code",
        output_dir=tmp_path,
        recipe_tag="test",
        robot_name="seer_dual_arm",
        dashboard_events=sink,
        claude_code_tools="",
    )
    options = planner._build_options(claude_agent_sdk, toolkit=toolkit, max_turns=10)
    assert options.tools == []
    assert set(options.allowed_tools) == {
        "mcp__rpent__return_to_initial",
        "mcp__rpent__finish",
    }
    toolkit.close()
