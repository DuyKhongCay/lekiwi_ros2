"""Verify navigation startup heartbeat, retry and exactly-once success contracts."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from lekiwi_orchestrator.navigation_startup import NavigationStartup
from nav2_msgs.srv import ManageLifecycleNodes
from std_msgs.msg import Bool


@pytest.fixture
def startup(monkeypatch):
    """Construct the production helper with controlled transport and monotonic time."""
    clock = [10.0]
    monkeypatch.setattr(
        "lekiwi_orchestrator.navigation_startup.time.monotonic", lambda: clock[0]
    )
    node = Mock()
    node.declare_parameter.side_effect = lambda name, default: SimpleNamespace(
        value=default
    )
    client = node.create_client.return_value
    client.service_is_ready.return_value = True
    client.call_async.side_effect = lambda request: Future()
    return NavigationStartup(node), client, clock


def test_requires_live_readiness_and_starts_once(startup):
    """A stale or false heartbeat cannot start Nav2; success prevents duplicate startup."""
    helper, client, clock = startup
    helper.tick()
    client.call_async.assert_not_called()
    helper.on_ready(Bool(data=True))
    clock[0] += 1.1
    helper.tick()
    client.call_async.assert_not_called()
    helper.on_ready(Bool(data=False))
    helper.tick()
    client.call_async.assert_not_called()
    helper.on_ready(Bool(data=True))
    helper.tick()
    assert (
        client.call_async.call_args.args[0].command
        == ManageLifecycleNodes.Request.STARTUP
    )
    helper.pending.set_result(SimpleNamespace(success=True))
    helper.tick()
    clock[0] += 2.0
    helper.on_ready(Bool(data=True))
    helper.tick()
    assert helper.started
    assert client.call_async.call_count == 1


def test_timeout_removes_pending_and_retries_only_after_backoff(startup):
    """Abandon an expired request and require fresh readiness before retry."""
    helper, client, clock = startup
    helper.on_ready(Bool(data=True))
    helper.tick()
    old = helper.pending
    clock[0] += 5.0
    helper.tick()
    client.remove_pending_request.assert_called_once_with(old)
    assert old.cancelled() and helper.pending is None
    helper.on_ready(Bool(data=True))
    helper.tick()
    assert client.call_async.call_count == 1
    clock[0] += 1.01
    helper.tick()
    assert client.call_async.call_count == 1
    helper.on_ready(Bool(data=True))
    helper.tick()
    assert client.call_async.call_count == 2


@pytest.mark.parametrize("exception", [False, True])
def test_failed_response_can_retry(startup, exception):
    """Service rejection and transport failure both leave startup retryable."""
    helper, client, clock = startup
    helper.on_ready(Bool(data=True))
    helper.tick()
    if exception:
        helper.pending.set_exception(RuntimeError("service unavailable"))
    else:
        helper.pending.set_result(SimpleNamespace(success=False))
    helper.tick()
    assert not helper.started
    clock[0] += 1.1
    helper.on_ready(Bool(data=True))
    helper.tick()
    assert client.call_async.call_count == 2
