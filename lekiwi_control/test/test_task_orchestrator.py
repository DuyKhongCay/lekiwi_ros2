# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import asyncio
import threading

from lekiwi_control.task_orchestrator import TaskOrchestratorNode
from lekiwi_interfaces.msg import CameraMode, HailoInferenceStatus
from lekiwi_interfaces.srv import SetCamMode


class _Logger:
    def error(self, _message):
        pass

    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class _FailingCameraHubClient:
    def __init__(self, response):
        self._response = response

    def service_is_ready(self):
        return True

    async def call_async(self, _request):
        return self._response


def _node_for_mode_tests():
    node = TaskOrchestratorNode.__new__(TaskOrchestratorNode)
    node._state_lock = threading.Lock()
    node._current_mode = CameraMode.CHESS_THINKING
    node._camera_hub_ready = True
    node.get_logger = lambda: _Logger()
    return node


def test_failed_camera_service_response_updates_authoritative_mode():
    node = _node_for_mode_tests()
    hub_response = SetCamMode.Response()
    hub_response.success = False
    hub_response.applied_mode.value = CameraMode.STANDBY
    hub_response.message = "pipeline failed"
    node._camera_mode_client = _FailingCameraHubClient(hub_response)

    request = SetCamMode.Request()
    request.requested_mode.value = CameraMode.STANDBY
    response = asyncio.run(node._handle_mode_request(request, SetCamMode.Response()))

    assert not response.success
    assert response.applied_mode.value == CameraMode.STANDBY
    assert node._current_mode == CameraMode.STANDBY


def test_camera_hub_error_status_logs_error():
    node = _node_for_mode_tests()
    status = HailoInferenceStatus()
    status.pipeline_state = HailoInferenceStatus.PIPELINE_ERROR
    status.last_error = "mock error"

    node._handle_camera_hub_status(status)
    assert node._current_mode == CameraMode.CHESS_THINKING


def test_mode_switch_publishes_camera_mode():
    node = _node_for_mode_tests()
    node._camera_mode_client = None
    published_modes = []

    class _MockPublisher:
        def publish(self, msg):
            published_modes.append(msg.value)

    node._mode_publisher = _MockPublisher()

    request = SetCamMode.Request()
    request.requested_mode.value = CameraMode.STANDBY
    response = asyncio.run(node._handle_mode_request(request, SetCamMode.Response()))

    assert response.success
    assert response.applied_mode.value == CameraMode.STANDBY
    assert node._current_mode == CameraMode.STANDBY
    assert published_modes == [CameraMode.STANDBY]
