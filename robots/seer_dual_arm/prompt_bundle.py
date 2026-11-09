# Copyright 2026 The RPent Authors.
# Licensed under the Apache License, Version 2.0.

"""Prompt placeholders for the fixed-action Seer integration."""

from __future__ import annotations

from collections.abc import Mapping

from rpent.prompt.utils import PromptNode


def system_prompt(variables: Mapping[str, object] | None = None) -> PromptNode:
    """Return a small prompt bundle for compatibility with the RPent runner."""
    del variables
    return {
        "ROLE": "The Seer dual-arm runner executes one explicitly selected robot tool.",
        "SAFETY": "Motion is opt-in. Preview first and use --live only when the robot is clear.",
    }


def user_prompt(variables: Mapping[str, object] | None = None) -> PromptNode:
    """Return the fixed-action task prompt."""
    del variables
    return {"TASK": "Execute the selected fixed robot action once."}
