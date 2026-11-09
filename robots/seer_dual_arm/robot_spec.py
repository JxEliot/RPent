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


DEFAULT_SERVICE_ROOT = Path("/home/seer/RBK/SeerTeleoperationService")


def get_robot_spec() -> RobotSpec:
    """Return the Seer dual-arm RobotSpec."""
    return RobotSpec(
        name="seer_dual_arm",
        prompts=PromptBundle(system=system_prompt, user=user_prompt),
        add_cli_args=_add_cli_args,
        parse_config=_parse_config,
        init_runtime=_init_runtime,
        is_real_robot=True,
        requires_operator_confirmation=lambda args: bool(args.live),
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
    )


def _add_cli_args(parser: argparse.ArgumentParser, use_dashboard: bool) -> None:
    """Register only the configuration needed by the fixed-action test."""
    del use_dashboard
    parser.set_defaults(memory_profile="local")
    parser.add_argument(
        "--service-root",
        default=None,
        help="SeerTeleoperationService checkout (default: /home/seer/RBK/SeerTeleoperationService).",
    )
    parser.add_argument(
        "--rbk-config",
        default=None,
        help="Seer rbk_config.json containing teleop.homing.initial_poses.",
    )
    parser.add_argument(
        "--robokit-config",
        default=None,
        help="robokit DDS config; defaults to <service-root>/config/controller/robokit.json.",
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


def _parse_config(args: argparse.Namespace) -> RunConfig:
    """Resolve paths and validate scalar motion settings without SDK imports."""
    service_root = (
        Path(args.service_root or DEFAULT_SERVICE_ROOT).expanduser().resolve()
    )
    rbk_config = (
        Path(args.rbk_config or service_root / "config" / "rbk_config.json")
        .expanduser()
        .resolve()
    )
    robokit_config = (
        Path(
            args.robokit_config
            or service_root / "config" / "controller" / "robokit.json"
        )
        .expanduser()
        .resolve()
    )
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
            "service_root": str(service_root),
            "rbk_config": str(rbk_config),
            "robokit_config": str(robokit_config),
            "live": bool(args.live),
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
    service_root = (
        Path(args.service_root or DEFAULT_SERVICE_ROOT).expanduser().resolve()
    )
    return [], {
        "rbk_config": Path(
            args.rbk_config or service_root / "config" / "rbk_config.json"
        )
        .expanduser()
        .resolve(),
        "robokit_config": Path(
            args.robokit_config
            or service_root / "config" / "controller" / "robokit.json"
        )
        .expanduser()
        .resolve(),
        "live": bool(args.live),
        "duration": float(args.duration),
        "timeout": float(args.timeout),
        "feedback_timeout": float(args.feedback_timeout),
        "hz": float(args.hz),
        "node_name": str(args.node_name),
    }
