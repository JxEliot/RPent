# Copyright 2026 The RPent Authors.
# Licensed under the Apache License, Version 2.0.

"""Local relay configuration reaches the API request without publishing secrets."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest
from pydantic_ai import Agent

from rpent.planner.api_config import load_api_config
from rpent.planner.base import build_api_model
from rpent.planner.check import LlmCheckRequest, check_llm


def write_config(path: Path, *, wire_api="responses", style="codex") -> None:
    if style == "local":
        path.write_text(
            'model = "gpt-6-astra"\nbase_url = "https://relay.invalid/openai"\n'
            f'wire_api = "{wire_api}"\napi_key = "private-bearer-value"\n'
            '[http_headers]\nx-test-key = "private-test-value"\n',
            encoding="utf-8",
        )
        return
    path.write_text(
        'model = "gpt-6-astra"\nmodel_provider = "relay"\n'
        '[model_providers.relay]\nbase_url = "https://relay.invalid/openai"\n'
        f'wire_api = "{wire_api}"\n'
        '[model_providers.relay.http_headers]\nx-test-key = "private-test-value"\n',
        encoding="utf-8",
    )


@pytest.mark.parametrize(
    ("wire_api", "model_override", "url_override", "style"),
    [
        ("responses", None, None, "codex"),
        ("chat", None, None, "codex"),
        ("responses", None, None, "local"),
        ("chat", "openai-chat:override-model", "https://override.invalid/v1", "local"),
    ],
)
def test_config_auth_headers_reach_api_request(
    tmp_path, monkeypatch, wire_api, model_override, url_override, style
) -> None:
    import httpx
    from openai import AsyncOpenAI

    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    path = tmp_path / "config.toml"
    write_config(path, wire_api=wire_api, style=style)
    requests = []
    expected_url = url_override or "https://relay.invalid/openai"
    expected_model = (
        model_override.split(":", 1)[1] if model_override else "gpt-6-astra"
    )

    def handle(request):
        requests.append(json.loads(request.content))
        assert request.headers["x-test-key"] == "private-test-value"
        if style == "local":
            assert request.headers["Authorization"] == "Bearer private-bearer-value"
        endpoint = "responses" if wire_api == "responses" else "chat/completions"
        assert str(request.url) == f"{expected_url}/{endpoint}"
        if wire_api == "responses":
            payload = {
                "id": "resp_test",
                "object": "response",
                "created_at": 0,
                "model": expected_model,
                "status": "completed",
                "parallel_tool_calls": True,
                "tools": [],
                "tool_choice": "auto",
                "output": [
                    {
                        "id": "msg_test",
                        "type": "message",
                        "role": "assistant",
                        "status": "completed",
                        "content": [
                            {"type": "output_text", "text": "ok", "annotations": []}
                        ],
                    }
                ],
            }
        else:
            payload = {
                "id": "chat_test",
                "object": "chat.completion",
                "created": 0,
                "model": expected_model,
                "choices": [
                    {
                        "index": 0,
                        "finish_reason": "stop",
                        "message": {"role": "assistant", "content": "ok"},
                    }
                ],
            }
        return httpx.Response(200, json=payload)

    original_init = AsyncOpenAI.__init__

    def client_init(self, **kwargs):
        original_init(
            self,
            **kwargs,
            http_client=httpx.AsyncClient(transport=httpx.MockTransport(handle)),
        )

    monkeypatch.setattr(AsyncOpenAI, "__init__", client_init)
    model = build_api_model(model_override, url_override, api_config=path)
    result = asyncio.run(Agent(model).run("hello"))
    assert result.output == "ok"
    assert requests[0]["model"] == expected_model
    assert isinstance(model.client, AsyncOpenAI)
    settings = load_api_config(path)
    assert "private-test-value" not in repr(settings)
    assert "private-bearer-value" not in repr(settings)
    asyncio.run(model.client.close())


def test_check_uses_local_config_without_key_and_redacts_provider_errors(
    tmp_path, monkeypatch
) -> None:
    from pydantic_ai.exceptions import ModelHTTPError
    from pydantic_ai.models.function import FunctionModel

    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    path = tmp_path / "config.toml"
    write_config(path)

    def reply(messages, info):
        raise ModelHTTPError(401, "gpt-6-astra", {"error": "private-test-value"})

    def build(model, base_url, *, api_config):
        assert api_config == str(path)
        assert model == "openai:gpt-6-astra"
        return FunctionModel(reply)

    monkeypatch.setattr("rpent.planner.base.build_api_model", build)
    result = check_llm(LlmCheckRequest(api_config=str(path)))
    assert result.status == "auth_failed"
    assert result.credential_present is True
    assert "private-test-value" not in json.dumps(result.as_dict())
    assert result.base_url == "https://relay.invalid/openai"


def test_invalid_config_error_does_not_echo_secret_line(tmp_path) -> None:
    path = tmp_path / "config.toml"
    path.write_text('token = "private-test-value" broken', encoding="utf-8")
    with pytest.raises(ValueError) as exc:
        load_api_config(path)
    assert "private-test-value" not in str(exc.value)
