"""Fixtures shared by the annotator test modules."""

from pathlib import Path

import pytest
import requests
from pydantic import SecretStr

from configs.annotator import AnnotationConfig


@pytest.fixture(autouse=True)
def no_live_requests(monkeypatch: pytest.MonkeyPatch) -> None:
    def reject(*_args: object, **_kwargs: object) -> None:
        pytest.fail("Tests must never call a live service")

    monkeypatch.setattr(requests.sessions.Session, "request", reject)


@pytest.fixture
def config(tmp_path: Path) -> AnnotationConfig:
    return AnnotationConfig(
        enabled=True,
        api_key=SecretStr("test-key-never-send"),
        state_path=tmp_path / "annotations",
        processed_episodes_path=tmp_path / "processed",
        spec_path=tmp_path / "spec.json",
        timeout_seconds=1,
    )
