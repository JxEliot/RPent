# Copyright 2026 The RPent Authors.
# Licensed under the Apache License, Version 2.0.

"""Conversation instructions for the Seer fixed-action integration."""

from __future__ import annotations

from collections.abc import Mapping

from rpent.prompt.utils import PromptNode


def system_prompt(variables: Mapping[str, object] | None = None) -> PromptNode:
    """Describe the available action, execution mode, and conversation lifecycle."""
    variables = variables or {}
    return {
        "ROLE": "You control a Seer robot with two arms and two grippers. "
        "Reply in the user's language (Chinese by default).",
        "ACTION": "The only robot action is return_to_initial(), which returns both "
        "arm end effectors to the configured initial poses. It leaves both grippers "
        "unchanged. Requests such as 回到初始位置、回初始位、双臂归位 refer to this action. "
        "Call it once when the user asks to perform it. Questions about capabilities "
        "or how homing works are not requests to move. Clarify ambiguous requests; "
        "explain unsupported actions without inventing tools or motion targets. "
        "Claude exposes these tools as mcp__rpent__<name>.",
        "MODE": (
            "LIVE: return_to_initial sends real motion commands. The operator enabled "
            "this mode at launch; wait for their explicit action request."
            if variables.get("live")
            else "PREVIEW: return_to_initial only reads the configured targets. "
            "It cannot move the robot. Report this as a preview, never as actual motion. "
            "Real motion requires the operator to restart with --live."
        ),
        "RESULT": "Report the tool result faithfully. Claim arrival only when the tool "
        "reports success=true and moved=true. On error, explain it and wait for the "
        "operator; do not retry motion automatically. Never run shell commands or "
        "modify configuration to perform robot actions or change the execution mode.",
        "SESSION": (
            "This is an ongoing conversation. After answering or completing an action, "
            "wait for the next user message. Call finish only if the user explicitly "
            "asks to end the conversation, not after each action."
            if variables.get("interactive")
            else "After completing the requested action, call finish with its outcome."
        ),
    }


def user_prompt(variables: Mapping[str, object] | None = None) -> PromptNode:
    """Start chat with an empty input and provide a concrete task for batch runs."""
    if (variables or {}).get("interactive"):
        return ""
    return "请调用 return_to_initial 一次，并汇报结果。"
