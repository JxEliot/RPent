# Copyright 2026 The RPent Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Runner hooks for the minimal Seer dual-arm fixed-action integration."""

from __future__ import annotations

import argparse
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any

from robots.seer_dual_arm.prompt_bundle import system_prompt, user_prompt
from rpent.dashboard.events import DashboardEventSink
from rpent.memory import MemoryManager
from rpent.robots.prompt_bundle import PromptBundle
from rpent.robots.robot_spec import RobotSpec, RunConfig
from rpent.utils.config import get_memory_dir, get_repo_root

if TYPE_CHECKING:
    from rpent.utils.daemon import ProcessDaemon


CONFIG_DIR = Path(__file__).resolve().parent / "config"


def get_robot_spec() -> RobotSpec:
    """Return the Seer dual-arm RobotSpec."""
    return RobotSpec(
        name="seer_dual_arm",
        prompts=PromptBundle(system=system_prompt, user=user_prompt),
        add_cli_args=_add_cli_args,
        parse_config=_parse_config,
        init_runtime=_init_runtime,
        is_real_robot=True,
        # The homing tool never reads stdin; the TUI owns operator input.
        requires_operator_confirmation=lambda args: bool(args.live and not args.tui),
        validate_args=_validate_args,
    )


def get_toolkit(
    *,
    runtime_kwargs: dict[str, Any],
    dashboard_events: DashboardEventSink,
    config: RunConfig,
):
    """Build the toolkit without importing the optional Seer SDK."""
    from robots.seer_dual_arm.toolkit import SeerDualArmToolkit

    memory = MemoryManager(
        root=config.prompt_vars.get("memory_dir") or get_memory_dir("seer_dual_arm")
    )
    return SeerDualArmToolkit(
        runtime_kwargs=runtime_kwargs,
        dashboard_events=dashboard_events,
        memory=memory,
        state_output_dir=config.output_dir,
        interactive=bool(config.prompt_vars.get("interactive")),
    )


def _add_cli_args(parser: argparse.ArgumentParser, use_dashboard: bool) -> None:
    """Register only the configuration needed by the fixed-action test."""
    del use_dashboard
    parser.set_defaults(memory_profile="local", claude_code_tools="")
    parser.add_argument(
        "--service-root",
        default=None,
        help="Read robot configs from an external SeerTeleoperationService checkout instead.",
    )
    parser.add_argument(
        "--robot-config",
        "--rbk-config",
        dest="rbk_config",
        default=None,
        help="Homing JSON (default: robots/seer_dual_arm/config/robot.json).",
    )
    parser.add_argument(
        "--robokit-config",
        default=None,
        help="DDS JSON (default: robots/seer_dual_arm/config/robokit.json).",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Publish WBC commands after reading fresh EE feedback. Omit for preview.",
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--feedback-timeout", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--node-name", default="rpent_seer_homing")


def _validate_args(args: argparse.Namespace) -> None:
    if args.tui and (sys.stdin is None or not sys.stdin.isatty()):
        raise ValueError("--tui requires an interactive terminal (TTY)")
    if args.dashboard:
        raise ValueError("Seer supports terminal chat with --tui; no Dashboard yet")
    if args.memory_profile != "local":
        raise ValueError("Seer fixed actions require --memory-profile local")
    if args.planner == "api" and args.api_config is None:
        local_api_config = CONFIG_DIR / "api.toml"
        if local_api_config.is_file():
            args.api_config = str(local_api_config)


def _config_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    """Resolve local defaults or explicitly selected external controller files."""
    if args.service_root:
        service_config = Path(args.service_root).expanduser() / "config"
        default_robot = service_config / "rbk_config.json"
        default_robokit = service_config / "controller" / "robokit.json"
    else:
        default_robot = CONFIG_DIR / "robot.json"
        default_robokit = CONFIG_DIR / "robokit.json"
    return (
        Path(args.rbk_config or default_robot).expanduser().resolve(),
        Path(args.robokit_config or default_robokit).expanduser().resolve(),
    )


def _parse_config(args: argparse.Namespace) -> RunConfig:
    """Resolve paths and validate scalar motion settings without SDK imports."""
    rbk_config, robokit_config = _config_paths(args)
    if not rbk_config.is_file():
        raise ValueError(f"Seer rbk config does not exist: {rbk_config}")
    if args.live and not robokit_config.is_file():
        raise ValueError(f"robokit DDS config does not exist: {robokit_config}")
    from robots.seer_dual_arm.homing import load_homing_config

    load_homing_config(rbk_config)
    numeric = (args.duration, args.timeout, args.feedback_timeout, args.hz)
    if any(not math.isfinite(value) or value <= 0 for value in numeric):
        raise ValueError(
            "--duration, --timeout, --feedback-timeout, and --hz must be positive"
        )
    if args.timeout <= args.duration:
        raise ValueError("--timeout must exceed a positive --duration")
    if not str(args.node_name).strip():
        raise ValueError("--node-name must be non-empty")
    output_dir = Path(
        args.output_dir
        or get_repo_root() / "logs" / f"{datetime.now():%Y%m%d-%H:%M:%S}_seer_dual_arm"
    )
    recipe_tag = f"seer_dual_arm_{datetime.now():%Y%m%d-%H%M%S}"
    memory_dir = (
        Path(args.memory_dir or get_memory_dir("seer_dual_arm")).expanduser().resolve()
    )
    return RunConfig(
        recipe_tag=recipe_tag,
        output_dir=output_dir,
        prompt_vars={
            "memory_dir": str(memory_dir),
            "rbk_config": str(rbk_config),
            "robokit_config": str(robokit_config),
            "live": bool(args.live),
            "interactive": bool(args.tui),
        },
        task_desc={
            "robot": "seer_dual_arm",
            "action": "return_to_initial",
            "live": bool(args.live),
            "rbk_config": str(rbk_config),
        },
    )


def _init_runtime(
    args: argparse.Namespace,
    output_dir: Path,
    dashboard_events: DashboardEventSink,
    components: set[str] | None,
) -> tuple[list["ProcessDaemon"], dict[str, Any]]:
    """Return lazy hardware settings; the toolkit owns the optional SDK import."""
    del output_dir, dashboard_events, components
    rbk_config, robokit_config = _config_paths(args)
    return [], {
        "rbk_config": rbk_config,
        "robokit_config": robokit_config,
        "live": bool(args.live),
        "duration": float(args.duration),
        "timeout": float(args.timeout),
        "feedback_timeout": float(args.feedback_timeout),
        "hz": float(args.hz),
        "node_name": str(args.node_name),
    }
