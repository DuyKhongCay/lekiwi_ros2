# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

import threading

from lekiwi_control.fsm import is_transition_allowed, MODE_NAMES
from lekiwi_interfaces.msg import CamHubStatus, CameraMode
from lekiwi_interfaces.srv import SetCamMode
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class TaskOrchestratorNode(Node):
    """Coordinate lifecycle startup and the four-mode camera FSM."""

    def __init__(self):
        super().__init__('task_orchestrator')
        self.declare_parameter('cam_hub_node', '/multi_camera_hub')
        self.declare_parameter('cam_hub_srv', '/camera_hub/set_mode')
        self.declare_parameter('camera_hub_node', '')
        self.declare_parameter('camera_hub_service', '')
        self.declare_parameter('orchestrator_service', '/orchestrator/set_mode')
        self.declare_parameter('camera_hub_status_topic', '/camera_hub/status')
        self.declare_parameter('lifecycle_poll_period_sec', 0.5)

        cam_hub_node = self._param_with_legacy('cam_hub_node', 'camera_hub_node').rstrip('/')
        cam_hub_srv = self._param_with_legacy('cam_hub_srv', 'camera_hub_service')
        orchestrator_service = self.get_parameter('orchestrator_service').value
        status_topic = self.get_parameter('camera_hub_status_topic').value
        poll_period = float(self.get_parameter('lifecycle_poll_period_sec').value)

        self._state_lock = threading.Lock()
        self._current_mode = CameraMode.STANDBY
        self._camera_hub_ready = False
        self._bootstrap_pending = False

        self._lifecycle_group = MutuallyExclusiveCallbackGroup()
        self._mode_group = ReentrantCallbackGroup()
        self._get_state_client = self.create_client(
            GetState,
            f'{cam_hub_node}/get_state',
            callback_group=self._lifecycle_group,
        )
        self._change_state_client = self.create_client(
            ChangeState,
            f'{cam_hub_node}/change_state',
            callback_group=self._lifecycle_group,
        )
        self._camera_mode_client = self.create_client(
            SetCamMode,
            cam_hub_srv,
            callback_group=self._mode_group,
        )
        self._mode_service = self.create_service(
            SetCamMode,
            orchestrator_service,
            self._handle_mode_request,
            callback_group=self._mode_group,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_subscription = self.create_subscription(
            CamHubStatus,
            status_topic,
            self._handle_camera_hub_status,
            status_qos,
            callback_group=self._mode_group,
        )
        self._bootstrap_timer = self.create_timer(
            poll_period,
            self._bootstrap_camera_hub,
            callback_group=self._lifecycle_group,
        )

    def _param_with_legacy(self, canonical, legacy):
        """Read a canonical parameter or its temporary legacy alias."""
        legacy_value = self.get_parameter(legacy).value
        if legacy_value:
            self.get_logger().warning(
                f"Parameter {legacy} is deprecated; use {canonical} instead"
            )
            return legacy_value
        return self.get_parameter(canonical).value

    def _handle_camera_hub_status(self, status):
        """Mirror the camera hub effective mode as the FSM authority."""
        mode = status.effective_mode.value
        if mode not in MODE_NAMES:
            self.get_logger().error(f'Camera hub published invalid mode: {mode}')
            return
        with self._state_lock:
            previous = self._current_mode
            self._current_mode = mode
        if previous != mode:
            self.get_logger().warning(
                f'Camera hub status changed orchestrator mode to {MODE_NAMES[mode]}'
            )

    def _bootstrap_camera_hub(self):
        """Advance the camera hub lifecycle without blocking the executor."""
        if self._camera_hub_ready or self._bootstrap_pending:
            return
        if not self._get_state_client.service_is_ready():
            return
        self._bootstrap_pending = True
        future = self._get_state_client.call_async(GetState.Request())
        future.add_done_callback(self._handle_lifecycle_state)

    def _handle_lifecycle_state(self, future):
        """Request the next lifecycle transition from a state response."""
        self._bootstrap_pending = False
        try:
            state = future.result().current_state.id
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to read camera hub lifecycle state: {exc}')
            return

        if state == State.PRIMARY_STATE_ACTIVE:
            if not self._camera_hub_ready:
                self._camera_hub_ready = True
                self.get_logger().info('Camera hub lifecycle is ACTIVE')
            return
        if not self._change_state_client.service_is_ready():
            return

        trans_id = None
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            trans_id = Transition.TRANSITION_CONFIGURE
        elif state == State.PRIMARY_STATE_INACTIVE:
            trans_id = Transition.TRANSITION_ACTIVATE
        elif state == State.PRIMARY_STATE_FINALIZED:
            self.get_logger().error('Camera hub is FINALIZED and cannot be restarted')
            return
        else:
            return

        request = ChangeState.Request()
        request.transition.id = trans_id
        self._bootstrap_pending = True
        trans_future = self._change_state_client.call_async(request)
        trans_future.add_done_callback(self._handle_lifecycle_trans)

    def _handle_lifecycle_trans(self, future):
        """Log lifecycle transition failures and allow the next poll."""
        self._bootstrap_pending = False
        try:
            if not future.result().success:
                self.get_logger().error('Camera hub rejected a lifecycle transition')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Camera hub lifecycle transition failed: {exc}')

    async def _handle_mode_request(self, request, response):
        """Validate the FSM then forward a mode request asynchronously."""
        with self._state_lock:
            current_mode = self._current_mode
        requested_mode = request.requested_mode.value
        response.applied_mode.value = current_mode

        if not self._camera_hub_ready:
            response.success = False
            response.message = 'Camera hub lifecycle is not active'
            return response
        if requested_mode not in MODE_NAMES:
            response.success = False
            response.message = f'Invalid mode value: {requested_mode}'
            return response
        if not is_transition_allowed(current_mode, requested_mode):
            response.success = False
            response.message = (
                f'FSM rejects {MODE_NAMES[current_mode]} -> {MODE_NAMES[requested_mode]}'
            )
            return response
        if not self._camera_mode_client.service_is_ready():
            response.success = False
            response.message = 'Camera hub mode service is unavailable'
            return response

        forwarded = SetCamMode.Request()
        forwarded.requested_mode.value = requested_mode
        try:
            hub_response = await self._camera_mode_client.call_async(forwarded)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'Camera hub mode request failed: {exc}'
            return response

        response.success = hub_response.success
        response.applied_mode = hub_response.applied_mode
        response.message = hub_response.message
        with self._state_lock:
            self._current_mode = hub_response.applied_mode.value
        if hub_response.success:
            self.get_logger().info(
                f'Orchestrator entered {MODE_NAMES[self._current_mode]}'
            )
        return response


def main(args=None):
    """Run the task orchestrator on a multi-threaded executor."""
    rclpy.init(args=args)
    node = TaskOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
