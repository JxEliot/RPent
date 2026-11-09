# Copyright 2026 The RPent Authors.
# Licensed under the Apache License, Version 2.0.

"""Load local API settings, including selected custom Codex providers."""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


@dataclass(frozen=True)
class ApiConfig:
    """Resolved relay settings; credentials are excluded from object reprs."""

    model: str
    base_url: str
    headers: dict[str, str] = field(repr=False)
    api_key: str = field(repr=False)

    def redact(self, text: str) -> str:
        """Remove configured credential values from a provider error."""
        for value in (self.api_key, *self.headers.values()):
            if value:
                text = text.replace(value, "<redacted>")
        return text


def load_api_config(path: str | Path) -> ApiConfig:
    """Load API settings without copying or changing the configuration file.

    Accepts flat TOML settings or a selected Codex custom provider. Responses
    and Chat Completions support literal headers and a local or environment
    API key. CLI OAuth credentials are not read.
    """
    try:
        raw = tomllib.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    except tomllib.TOMLDecodeError:
        # TOML parser messages can include the offending line, including secrets.
        raise ValueError("API configuration is not valid TOML") from None
    provider = raw
    if "model_provider" in raw:
        selected = raw["model_provider"]
        providers = raw.get("model_providers", {})
        if not isinstance(selected, str) or not isinstance(providers, dict):
            raise ValueError("Codex configuration must select a custom model_provider")
        provider = providers.get(selected, {})
        if not isinstance(provider, dict):
            raise ValueError("Codex model_provider must refer to a provider table")
    base_url = provider.get("base_url")
    model = raw.get("model")
    if not isinstance(base_url, str) or not base_url.startswith(
        ("https://", "http://")
    ):
        raise ValueError("API configuration requires an HTTP(S) base_url")
    if not isinstance(model, str) or not model.strip():
        raise ValueError("API configuration requires a model")
    wire_api = provider.get("wire_api", "responses")
    if wire_api not in ("responses", "chat"):
        raise ValueError("API wire_api must be responses or chat")
    headers = provider.get("http_headers", {})
    if not isinstance(headers, dict) or not all(
        isinstance(key, str) and isinstance(value, str)
        for key, value in headers.items()
    ):
        raise ValueError("API http_headers must map strings to strings")
    key_env = provider.get("env_key", "OPENAI_API_KEY")
    if not isinstance(key_env, str):
        raise ValueError("API env_key must be a string")
    api_key = provider.get("api_key", "")
    if not isinstance(api_key, str):
        raise ValueError("API api_key must be a string")
    api_key = api_key or os.environ.get(key_env, "")
    if provider.get("env_key") and not api_key:
        raise ValueError(f"API credential environment variable {key_env} is unset")
    if not api_key and not headers:
        raise ValueError(
            "API configuration needs api_key, http_headers, or an API key environment variable"
        )
    prefix = "openai" if wire_api == "responses" else "openai-chat"
    return ApiConfig(
        model=f"{prefix}:{model}",
        base_url=base_url,
        headers=headers,
        api_key=api_key,
    )
