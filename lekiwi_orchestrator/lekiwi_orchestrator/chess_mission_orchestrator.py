# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Central Mission Orchestrator for LeKiwi Mobile Manipulation Chess Robot.

Acts as a Mediator and Facade coordinating:
1. TF readiness & EKF convergence (/system/tf_ready) via TfReadinessGatekeeper
2. Game status, FIDE legality & Stockfish best move (/chess/game_status) from lekiwi_chess_master
3. Reachability & base standoff queries (/workspace/check_move_feasibility) from lekiwi_motion
4. Navigation goals to chessboard perimeter standoff via Nav2 (NavigateToPose)
5. Physical manipulation pick & place via /manipulation/execute_chess_move
6. Dynamic operational camera modes (/camera_mode)
"""

from __future__ import annotations

import math
import re
import time
from typing import Any, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped
from lekiwi_interfaces.msg import CameraMode, ChessGameStatus
from lekiwi_interfaces.srv import CheckMoveFeasibility, SetCamMode
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from lekiwi_orchestrator.chessboard_coordinate_mapper import (
    ChessboardCoordinateMapper,
    UciMoveDetails,
)
from lekiwi_orchestrator.fsm import (
    CAMERA_MODE_NAMES,
    MISSION_STATE_NAMES,
    MissionState,
    is_camera_transition_allowed,
    is_mission_transition_allowed,
)

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None

try:
    from lekiwi_interfaces.action import ExecuteChessMove
except ImportError:
    ExecuteChessMove = None


DEFAULT_ROBOT_COLOR = "b"  # Default robot plays Black (waiting for White's move)
DEFAULT_BOARD_FRAME = "chessboard_frame"
DEFAULT_MAP_FRAME = "map"
DEFAULT_FEASIBILITY_TIMEOUT_SEC = 5.0
DEFAULT_ACTION_TIMEOUT_SEC = 60.0


class ChessMissionOrchestrator(Node):
    """
    High-level mission coordinator implementing Mediator and Facade design patterns.
    """

    def __init__(
        self,
        node_name: str = "chess_mission_orchestrator",
        coordinate_mapper: Optional[ChessboardCoordinateMapper] = None,
    ) -> None:
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("robot_color", DEFAULT_ROBOT_COLOR)
        self.declare_parameter("board_frame", DEFAULT_BOARD_FRAME)
        self.declare_parameter("map_frame", DEFAULT_MAP_FRAME)
        self.declare_parameter("board_width", 0.20)
        self.declare_parameter("board_height", 0.20)
        self.declare_parameter(
            "feasibility_timeout_sec", DEFAULT_FEASIBILITY_TIMEOUT_SEC
        )
        self.declare_parameter("action_timeout_sec", DEFAULT_ACTION_TIMEOUT_SEC)
        self.declare_parameter(
            "skip_navigation", False
        )  # Useful for tabletop sim/tests

        self._robot_color = str(self.get_parameter("robot_color").value).lower()
        self._board_frame = str(self.get_parameter("board_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        board_w = float(self.get_parameter("board_width").value)
        board_h = float(self.get_parameter("board_height").value)
        self._feasibility_timeout = float(
            self.get_parameter("feasibility_timeout_sec").value
        )
        self._action_timeout = float(self.get_parameter("action_timeout_sec").value)
        self._skip_nav = bool(self.get_parameter("skip_navigation").value)

        # Spatial chess square mapping is now handled centrally in lekiwi_motion
        # (via /workspace/check_move_feasibility). coordinate_mapper is maintained
        # for backwards compatibility.
        self._mapper = coordinate_mapper

        # Internal State Machine tracking
        self._mission_state = MissionState.BOOT_INITIALIZING
        self._camera_mode = CameraMode.STANDBY
        self._tf_ready = False
        self._last_readiness_heartbeat = None
        self._readiness_timeout = self.declare_parameter(
            "readiness_timeout_sec", 1.0
        ).value
        if not math.isfinite(self._readiness_timeout) or self._readiness_timeout <= 0:
            raise ValueError("readiness_timeout_sec must be finite and positive")
        self._current_goal_move: Optional[str] = None
        self._current_move_details: Optional[UciMoveDetails] = None
        self._last_processed_fen: Optional[str] = None

        # Callback Groups
        self._cb_group_sub = MutuallyExclusiveCallbackGroup()
        self._cb_group_client = ReentrantCallbackGroup()

        # Subscriptions
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._tf_ready_sub = self.create_subscription(
            Bool,
            "/system/tf_ready",
            self._on_tf_ready,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self._cb_group_sub,
        )
        self._game_status_sub = self.create_subscription(
            ChessGameStatus,
            "/chess/game_status",
            self._on_game_status,
            10,
            callback_group=self._cb_group_sub,
        )

        # Service Clients
        self._feasibility_client = self.create_client(
            CheckMoveFeasibility,
            "/workspace/check_move_feasibility",
            callback_group=self._cb_group_client,
        )
        self._cam_mode_client = self.create_client(
            SetCamMode,
            "/orchestrator/set_mode",
            callback_group=self._cb_group_client,
        )

        # Action Clients
        self._nav2_client = (
            ActionClient(
                self,
                NavigateToPose,
                "/navigate_to_pose",
                callback_group=self._cb_group_client,
            )
            if NavigateToPose is not None
            else None
        )
        self._manipulation_client = (
            ActionClient(
                self,
                ExecuteChessMove,
                "/manipulation/execute_chess_move",
                callback_group=self._cb_group_client,
            )
            if ExecuteChessMove is not None
            else None
        )

        # Publishers
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._cam_mode_pub = self.create_publisher(
            CameraMode, "/camera_mode", latched_qos
        )

        # Periodic Diagnostic Timer (1 Hz)
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._readiness_timer = self.create_timer(
            min(0.2, self._readiness_timeout / 2),
            self._expire_readiness,
            callback_group=self._cb_group_sub,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

        # Initial State transition to WAITING_FOR_TF_READY
        self.transition_to(MissionState.WAITING_FOR_TF_READY)
        self.get_logger().info(
            f"ChessMissionOrchestrator initialized. Robot Color: '{self._robot_color}'. "
            f"Waiting for TF Tree Readiness on /system/tf_ready..."
        )

    @property
    def mission_state(self) -> MissionState:
        return self._mission_state

    @property
    def camera_mode(self) -> int:
        return self._camera_mode

    @property
    def is_tf_ready(self) -> bool:
        return (
            self._tf_ready
            and self._last_readiness_heartbeat is not None
            and time.monotonic() - self._last_readiness_heartbeat
            <= self._readiness_timeout
        )

    # ================= State Machine Management =================

    def transition_to(self, target_state: MissionState) -> bool:
        """
        Safely transition to target_state validating against legal transition matrix.
        Fail fast and loud if an illegal state transition is requested.
        """
        if target_state == self._mission_state:
            return True

        if not is_mission_transition_allowed(self._mission_state, target_state):
            err_msg = (
                f"ILLEGAL FSM TRANSITION: Cannot move from "
                f"{MISSION_STATE_NAMES.get(self._mission_state, 'UNKNOWN')} to "
                f"{MISSION_STATE_NAMES.get(target_state, 'UNKNOWN')}"
            )
            self.get_logger().error(err_msg)
            return False

        old_state_name = MISSION_STATE_NAMES.get(self._mission_state, "UNKNOWN")
        new_state_name = MISSION_STATE_NAMES.get(target_state, "UNKNOWN")
        self._mission_state = target_state
        self.get_logger().info(
            f"[MISSION FSM] Transitioned: {old_state_name} -> {new_state_name}"
        )
        return True

    def set_camera_mode(self, requested_mode: int) -> bool:
        """Update system camera mode and notify /camera_mode topic."""
        if requested_mode == self._camera_mode:
            return True

        if not is_camera_transition_allowed(self._camera_mode, requested_mode):
            self.get_logger().warn(
                f"[CAMERA FSM] Rejected mode switch: "
                f"{CAMERA_MODE_NAMES.get(self._camera_mode, 'UNKNOWN')} -> "
                f"{CAMERA_MODE_NAMES.get(requested_mode, 'UNKNOWN')}"
            )
            return False

        self._camera_mode = requested_mode
        msg = CameraMode()
        msg.value = self._camera_mode
        self._cam_mode_pub.publish(msg)
        self.get_logger().info(
            f"[CAMERA FSM] Switched CameraMode to {CAMERA_MODE_NAMES.get(requested_mode, 'UNKNOWN')}"
        )
        return True

    # ================= Subscription Callbacks =================

    def _expire_readiness(self):
        """Withdraw cached readiness when its heartbeat lease expires."""
        if self._tf_ready and not self.is_tf_ready:
            self._tf_ready = False
            self.get_logger().warning("TF readiness heartbeat expired")

    def _on_tf_ready(self, msg: Bool) -> None:
        """Handle readiness notifications from TfReadinessGatekeeper."""
        previous = self.is_tf_ready
        self._last_readiness_heartbeat = time.monotonic()
        self._tf_ready = bool(msg.data)

        if self._tf_ready and not previous:
            self.get_logger().info(
                ">>> [ORCHESTRATOR] TF Ready confirmed! System is localized & stationary."
            )
            if self._mission_state == MissionState.WAITING_FOR_TF_READY:
                # If robot is White, it plays first; otherwise waits for Black/White player move
                initial_state = (
                    MissionState.EVALUATING_BEST_MOVE
                    if self._robot_color == "w"
                    else MissionState.WAITING_FOR_PLAYER_MOVE
                )
                self.transition_to(initial_state)
                self.set_camera_mode(CameraMode.CHESS_THINKING)
        elif not self._tf_ready and previous:
            self.get_logger().warn(
                "<<< [ORCHESTRATOR] TF Unready! Robot moved or EKF lost convergence."
            )

    def _on_game_status(self, msg: ChessGameStatus) -> None:
        """Process game state updates from lekiwi_chess_master."""
        if not self.is_tf_ready:
            return

        # Game over condition
        if msg.is_checkmate or msg.is_draw:
            if self._mission_state != MissionState.GAME_OVER:
                self.transition_to(MissionState.GAME_OVER)
                reason = "CHECKMATE" if msg.is_checkmate else "DRAW"
                self.get_logger().info(
                    f"*** GAME OVER: {reason}! FEN: {msg.full_fen} ***"
                )
                self.set_camera_mode(CameraMode.STANDBY)
            return

        # Robot Turn Trigger
        is_robot_turn = msg.active_color == self._robot_color
        is_ready_to_act = (
            msg.is_board_stable
            and is_robot_turn
            and bool(msg.best_move)
            and msg.game_phase
            in (
                ChessGameStatus.PHASE_ROBOT_READY,
                ChessGameStatus.PHASE_WAITING_PLAYER,
            )
        )

        if is_ready_to_act and self._mission_state in (
            MissionState.WAITING_FOR_PLAYER_MOVE,
            MissionState.EVALUATING_BEST_MOVE,
            MissionState.TURN_COMPLETED,
        ):
            # Avoid re-triggering the same FEN position
            if msg.full_fen == self._last_processed_fen:
                return

            self._last_processed_fen = msg.full_fen
            self._current_goal_move = msg.best_move
            self.get_logger().info(
                f">>> [ORCHESTRATOR] New Best Move Received: '{self._current_goal_move}' "
                f"(Eval: {msg.eval_centipawns} cp, Color: {msg.active_color})"
            )
            self._dispatch_move_workflow(self._current_goal_move)

    # ================= Workflow Orchestration =================

    def _dispatch_move_workflow(self, uci_move: str) -> None:
        """Begin autonomous execution of a single chess move."""
        if not self.is_tf_ready:
            return

        cleaned_move = uci_move.strip().lower()
        match = re.match(r"^([a-h][1-8])([a-h][1-8])([qrbn])?$", cleaned_move)
        if not match:
            self.get_logger().error(
                f"Failed to parse UCI move '{uci_move}': Expected format like 'e2e4' or 'e7e8q'."
            )
            self.transition_to(MissionState.ERROR_FALLBACK)
            return

        from_square = match.group(1)
        to_square = match.group(2)
        promo = match.group(3)

        details = UciMoveDetails(
            uci=cleaned_move,
            from_square=from_square,
            to_square=to_square,
            promotion=promo,
            pick_point=Point(),
            place_point=Point(),
            is_capture=False,
        )
        self._current_move_details = details

        if not self.transition_to(MissionState.CHECKING_REACHABILITY):
            return

        if not self._feasibility_client.service_is_ready():
            self.get_logger().error(
                "Workspace Checker service '/workspace/check_move_feasibility' is unavailable!"
            )
            self.transition_to(MissionState.ERROR_FALLBACK)
            return

        req = CheckMoveFeasibility.Request()
        req.uci_move = cleaned_move
        req.is_capture = False

        self.get_logger().info(
            f"Querying reachability feasibility for {from_square} -> {to_square} ({cleaned_move})..."
        )
        future = self._feasibility_client.call_async(req)
        future.add_done_callback(lambda f: self._on_feasibility_response(f, details))

    def _on_feasibility_response(self, future, details: UciMoveDetails) -> None:
        """Handle feasibility calculation result from lekiwi_motion."""
        try:
            resp: CheckMoveFeasibility.Response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Workspace feasibility query failed: {exc}")
            self.transition_to(MissionState.ERROR_FALLBACK)
            return

        if not resp.feasible:
            self.get_logger().error(
                f"Move {details.uci} is declared NOT FEASIBLE: {resp.message}"
            )
            self.transition_to(MissionState.ERROR_FALLBACK)
            return

        self.get_logger().info(
            f"Feasibility confirmed! Plan Type: {resp.plan_type} ({resp.message})"
        )

        # Decide navigation vs direct manipulation
        if (
            resp.plan_type == CheckMoveFeasibility.Response.PLAN_ZERO_NAV
            or self._skip_nav
        ):
            self._execute_manipulation_step(details, resp)
        else:
            # Need base navigation to standoff pose
            self._execute_navigation_step(resp.pick_base_pose, details, resp)

    def _execute_navigation_step(
        self,
        target_pose: PoseStamped,
        details: UciMoveDetails,
        feasibility_resp: CheckMoveFeasibility.Response,
    ) -> None:
        """Dispatch Nav2 goal to standoff base position."""
        if self._nav2_client is None or not self._nav2_client.server_is_ready():
            self.get_logger().warn(
                "Nav2 action server not ready; proceeding in tabletop test mode."
            )
            self._execute_manipulation_step(details, feasibility_resp)
            return

        if not self.transition_to(MissionState.NAVIGATING_TO_STANDOFF):
            return

        self.set_camera_mode(CameraMode.NAVIGATING)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.get_logger().info(
            f"Dispatching Nav2 NavigateToPose to ({target_pose.pose.position.x:.3f}, "
            f"{target_pose.pose.position.y:.3f})..."
        )
        send_future = self._nav2_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda f: self._on_nav2_goal_submitted(f, details, feasibility_resp)
        )

    def _on_nav2_goal_submitted(
        self,
        future,
        details: UciMoveDetails,
        feasibility_resp: CheckMoveFeasibility.Response,
    ) -> None:
        """Handle Nav2 goal acceptance response."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Nav2 rejected navigation goal!")
                self.transition_to(MissionState.ERROR_FALLBACK)
                return

            res_future = goal_handle.get_result_async()
            res_future.add_done_callback(
                lambda f: self._on_nav2_completed(f, details, feasibility_resp)
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to submit Nav2 goal: {exc}")
            self.transition_to(MissionState.ERROR_FALLBACK)

    def _on_nav2_completed(
        self,
        future,
        details: UciMoveDetails,
        feasibility_resp: CheckMoveFeasibility.Response,
    ) -> None:
        """Handle Nav2 navigation completion."""
        self.get_logger().info(
            "Nav2 navigation completed successfully! Commencing manipulation."
        )
        self._execute_manipulation_step(details, feasibility_resp)

    def _execute_manipulation_step(
        self, details: UciMoveDetails, feasibility_resp: CheckMoveFeasibility.Response
    ) -> None:
        """Dispatch manipulation pick and place action."""
        if not self.transition_to(MissionState.EXECUTING_MANIPULATION):
            return

        self.set_camera_mode(CameraMode.MANIPULATION_LEROBOT)

        if (
            self._manipulation_client is None
            or not self._manipulation_client.server_is_ready()
        ):
            self.get_logger().info(
                f"[SIMULATION COMPLETE] Simulated manipulation of move {details.uci} "
                f"from {details.from_square} to {details.to_square}."
            )
            self._finalize_turn()
            return

        # Resolve pick and place 3D coordinates (from lekiwi_motion response or details fallback)
        has_resp_pick = hasattr(feasibility_resp, "pick_point") and (
            feasibility_resp.pick_point.x != 0.0
            or feasibility_resp.pick_point.y != 0.0
            or feasibility_resp.pick_point.z != 0.0
        )
        pick_point = (
            feasibility_resp.pick_point if has_resp_pick else details.pick_point
        )

        has_resp_place = hasattr(feasibility_resp, "place_point") and (
            feasibility_resp.place_point.x != 0.0
            or feasibility_resp.place_point.y != 0.0
            or feasibility_resp.place_point.z != 0.0
        )
        place_point = (
            feasibility_resp.place_point if has_resp_place else details.place_point
        )

        goal = ExecuteChessMove.Goal()
        goal.instruction = f"Pick {details.from_square}, place {details.to_square}"
        goal.from_square = details.from_square
        goal.to_square = details.to_square
        goal.pick_point = pick_point
        goal.place_point = place_point
        goal.is_capture = details.is_capture
        goal.target_frame = self._board_frame
        goal.pick_ik_hint = feasibility_resp.pick_ik_solution
        goal.place_ik_hint = feasibility_resp.place_ik_solution

        self.get_logger().info(
            f"Sending ExecuteChessMove goal: '{goal.instruction}'..."
        )
        send_future = self._manipulation_client.send_goal_async(
            goal, feedback_callback=self._on_manipulation_feedback
        )
        send_future.add_done_callback(self._on_manipulation_goal_submitted)

    def _on_manipulation_feedback(self, feedback_msg) -> None:
        """Log progress feedback during physical manipulation."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[MANIPULATION FEEDBACK] Phase: {fb.current_phase} ({fb.progress_percent:.1f}%)"
        )

    def _on_manipulation_goal_submitted(self, future) -> None:
        """Handle manipulation goal acceptance response."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Manipulation server rejected chess move goal!")
                self.transition_to(MissionState.ERROR_FALLBACK)
                return

            res_future = goal_handle.get_result_async()
            res_future.add_done_callback(self._on_manipulation_completed)
        except Exception as exc:
            self.get_logger().error(f"Failed to submit manipulation goal: {exc}")
            self.transition_to(MissionState.ERROR_FALLBACK)

    def _on_manipulation_completed(self, future) -> None:
        """Handle manipulation execution completion."""
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info(
                    f"Manipulation execution successful in {result.execution_time_sec:.2f}s: {result.message}"
                )
                self._finalize_turn()
            else:
                self.get_logger().error(
                    f"Manipulation execution failed: {result.message}"
                )
                self.transition_to(MissionState.ERROR_FALLBACK)
        except Exception as exc:
            self.get_logger().error(f"Error reading manipulation result: {exc}")
            self.transition_to(MissionState.ERROR_FALLBACK)

    def _finalize_turn(self) -> None:
        """Conclude robot turn and return to waiting for opponent."""
        self.transition_to(MissionState.TURN_COMPLETED)
        self.set_camera_mode(CameraMode.CHESS_THINKING)
        self.transition_to(MissionState.WAITING_FOR_PLAYER_MOVE)
        self.get_logger().info(
            ">>> Turn finalized successfully. Waiting for opponent move."
        )

    # ================= Diagnostics =================

    def _publish_diagnostics(self) -> None:
        """Publish mission state telemetry on /diagnostics."""
        diag = DiagnosticStatus()
        diag.name = "Chess Mission Orchestrator"
        diag.hardware_id = "LeKiwi_Brain"

        if self._mission_state == MissionState.ERROR_FALLBACK:
            diag.level = DiagnosticStatus.ERROR
            diag.message = "System in Error Fallback state"
        elif not self.is_tf_ready:
            diag.level = DiagnosticStatus.WARN
            diag.message = "Waiting for TF readiness"
        else:
            diag.level = DiagnosticStatus.OK
            diag.message = (
                f"Active ({MISSION_STATE_NAMES.get(self._mission_state, 'UNKNOWN')})"
            )

        diag.values = [
            KeyValue(
                key="mission_state",
                value=MISSION_STATE_NAMES.get(self._mission_state, "UNKNOWN"),
            ),
            KeyValue(
                key="camera_mode",
                value=CAMERA_MODE_NAMES.get(self._camera_mode, "UNKNOWN"),
            ),
            KeyValue(key="tf_ready", value=str(self._tf_ready)),
            KeyValue(key="robot_color", value=self._robot_color),
            KeyValue(key="last_goal_move", value=str(self._current_goal_move)),
        ]

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        diag_array.status.append(diag)
        self._diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    node = ChessMissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
