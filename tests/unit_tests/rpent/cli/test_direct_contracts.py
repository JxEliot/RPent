# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

from __future__ import annotations

import json
import sys
from pathlib import Path

from robots.seer_dual_arm.robot_spec import get_robot_spec


def test_seer_direct_preview_runs_without_tty_memory_sync_or_sdk(
    tmp_path: Path, monkeypatch
) -> None:
    from rpent.cli import main as cli
    from rpent.memory import loading

    service = tmp_path / "service"
    (service / "config").mkdir(parents=True)
    (service / "config" / "rbk_config.json").write_text(
        json.dumps(
            {
                "teleop": {
                    "homing": {
                        "enabled": True,
                        "initial_poses": {
                            "left": {"pos": [0.1, 0.2, 0.3], "quat": [0, 0, 0, 1]},
                            "right": {"pos": [0.1, -0.2, 0.3], "quat": [0, 0, 0, 1]},
                        },
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    output_dir = tmp_path / "output"
    robot_spec = get_robot_spec()
    monkeypatch.setattr(cli, "enumerate_robots", lambda: ("seer_dual_arm",))
    monkeypatch.setattr(cli, "get_robot_spec", lambda name: robot_spec)
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
    monkeypatch.setattr(
        loading,
        "prepare_run_memory",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError("direct preview must skip memory preparation")
        ),
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "rpent",
            "--robot",
            "seer_dual_arm",
            "--planner",
            "direct",
            "--direct-tool",
            "return_to_initial",
            "--service-root",
            str(service),
            "--output-dir",
            str(output_dir),
            "--memory-dir",
            str(tmp_path / "memory"),
        ],
    )

    assert cli.main() == 0

    transcript_path = next(output_dir.glob("transcript_*.json"))
    transcript = json.loads(transcript_path.read_text(encoding="utf-8"))
    assert transcript["finish"]["summary"] == "Preview only; the robot was not moved."
    assert transcript["error"] is None
    assert transcript["stats"]["tool_calls"] == 1
    assert transcript["messages"][1]["name"] == "return_to_initial"
    state = json.loads((output_dir / "states.json").read_text(encoding="utf-8"))
    assert len(state["steps"]) == 1
    assert state["steps"][0]["state"]["moved"] is False
