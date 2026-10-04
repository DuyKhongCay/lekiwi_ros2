# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Test fakes and doubles for orchestrator unit testing."""

from __future__ import annotations

from lekiwi_orchestrator.motion_client import (
    FakeMotionClient,
    _MockGoalHandle,
)

__all__ = [
    "FakeMotionClient",
    "_MockGoalHandle",
]
