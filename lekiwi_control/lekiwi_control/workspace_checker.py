# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
URDF-Based Workspace Checker & Base Standoff Generator Node for LeKiwi.

Provides the /workspace/check_reachability service:
- Transforms query coordinates via TF2.
- Solves closed-form analytical IK with physical joint limits.
- Computes optimal Base Standoff Pose on the chessboard perimeter if out of reach.
- Strictly failsafes if dynamic parameters cannot be synchronized from remote nodes.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from lekiwi_interfaces.srv import CheckMoveFeasibility
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs

from lekiwi_control.kinematics_engine import (
    DEFAULT_CLEARANCE_PADDING,
    DEFAULT_PITCH_ANGLE,
    compute_standoff_pose,
    find_common_standoff_pose,
    is_single_base_geometrically_possible,
    solve_analytical_ik,
)

DEFAULT_SYNC_TIMEOUT_SEC = 3.0
DEFAULT_DIAGNOSTIC_RATE_HZ = 1.0


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a planar yaw angle (rad) into a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def transform_point_by_tf(
    point: Point,
    transform: tf2_geometry_msgs.TransformStamped,
) -> Point:
    """Transform geometry_msgs/Point using a TransformStamped."""
    p_stamped = PointStamped()
    p_stamped.header = transform.header
    p_stamped.point = point
    res = tf2_geometry_msgs.do_transform_point(p_stamped, transform)
    return res.point


class WorkspaceCheckerNode(Node):
    """
    ROS 2 Service Node for URDF-based reachability and base standoff calculation.
    Enforces strict failsafe if perception/controller parameters are unavailable.
    """

    def __init__(self):
        super().__init__("workspace_checker")

        # Static frame parameters
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("sync_timeout_sec", DEFAULT_SYNC_TIMEOUT_SEC)
        self.declare_parameter("clearance_padding", DEFAULT_CLEARANCE_PADDING)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._sync_timeout = float(self.get_parameter("sync_timeout_sec").value)
        self._clearance_padding = float(self.get_parameter("clearance_padding").value)

        # Dynamic physical parameters (must be synchronized from external nodes)
        self._is_configured = False
        self._board_width: Optional[float] = None
        self._board_height: Optional[float] = None
        self._board_frame: Optional[str] = None
        self._robot_radius: Optional[float] = None
        self._failsafe_reason = "FAILSAFE: Initializing parameter synchronization"

        # TF2 Buffer and Listener
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Callback Groups
        self._srv_cbg = MutuallyExclusiveCallbackGroup()
        self._param_cbg = ReentrantCallbackGroup()

        # Service Server
        self._service = self.create_service(
            CheckMoveFeasibility,
            "/workspace/check_move_feasibility",
            self.handle_check_move_feasibility,
            callback_group=self._srv_cbg,
        )

        # Diagnostic Publisher
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._diag_timer = self.create_timer(
            1.0 / DEFAULT_DIAGNOSTIC_RATE_HZ, self.publish_diagnostics
        )

        # Persistent parameter clients & futures
        from rcl_interfaces.srv import GetParameters

        self._param_client_board = self.create_client(
            GetParameters,
            "/chessboard_pose_estimator/get_parameters",
            callback_group=self._param_cbg,
        )
        self._param_client_omni = self.create_client(
            GetParameters,
            "/omni_base_controller/get_parameters",
            callback_group=self._param_cbg,
        )
        self._param_client_cm = self.create_client(
            GetParameters,
            "/controller_manager/get_parameters",
            callback_group=self._param_cbg,
        )

        self._board_param_future = None
        self._robot_param_future = None

        # Asynchronous parameter synchronization timer
        self._sync_start_time = self.get_clock().now()
        self._sync_timer = self.create_timer(
            0.5, self._attempt_parameter_synchronization, callback_group=self._param_cbg
        )

        self.get_logger().info(
            "WorkspaceCheckerNode started. Querying external nodes for dynamic parameters..."
        )

    # ================= Parameter Synchronization & Strict Failsafe =================

    def _attempt_parameter_synchronization(self):
        """Poll parameters from /chessboard_pose_estimator and /omni_base_controller."""
        if self._is_configured:
            return

        from rcl_interfaces.srv import GetParameters

        # 1. Query /chessboard_pose_estimator if not yet received
        if self._board_width is None or self._board_height is None:
            if (
                self._param_client_board.service_is_ready()
                and self._board_param_future is None
            ):
                req = GetParameters.Request()
                req.names = ["tags.positions_x", "tags.positions_y", "chessboard_frame"]
                self._board_param_future = self._param_client_board.call_async(req)
                self._board_param_future.add_done_callback(
                    self._on_board_params_received
                )

        # 2. Query robot_radius (try /omni_base_controller first, then /controller_manager)
        if self._robot_radius is None and self._robot_param_future is None:
            if self._param_client_omni.service_is_ready():
                req = GetParameters.Request()
                req.names = ["robot_radius"]
                self._robot_param_future = self._param_client_omni.call_async(req)
                self._robot_param_future.add_done_callback(
                    self._on_omni_params_received
                )
            elif self._param_client_cm.service_is_ready():
                req = GetParameters.Request()
                req.names = ["omni_base_controller.robot_radius"]
                self._robot_param_future = self._param_client_cm.call_async(req)
                self._robot_param_future.add_done_callback(self._on_cm_params_received)

        # Check timeout
        elapsed_sec = (self.get_clock().now() - self._sync_start_time).nanoseconds / 1e9
        if elapsed_sec > self._sync_timeout and not self._is_configured:
            missing = []
            if self._board_width is None or self._board_height is None:
                status = (
                    "service not ready"
                    if not self._param_client_board.service_is_ready()
                    else "pending response"
                )
                missing.append(f"/chessboard_pose_estimator ({status})")
            if self._robot_radius is None:
                omni_ready = self._param_client_omni.service_is_ready()
                cm_ready = self._param_client_cm.service_is_ready()
                status = (
                    "controllers not ready"
                    if not (omni_ready or cm_ready)
                    else "pending response"
                )
                missing.append(f"/omni_base_controller ({status})")

            self._failsafe_reason = (
                f"FAILSAFE: Unable to synchronize parameters from: {', '.join(missing)}"
            )
            self.get_logger().error(self._failsafe_reason)

    def _on_board_params_received(self, future):
        """Callback when /chessboard_pose_estimator responds."""
        self._board_param_future = None
        try:
            result = future.result()
            if result and len(result.values) == 3:
                xs = result.values[0].double_array_value
                ys = result.values[1].double_array_value
                frame = result.values[2].string_value

                if len(xs) >= 2 and len(ys) >= 2:
                    self._board_width = float(max(xs) - min(xs))
                    self._board_height = float(max(ys) - min(ys))
                    self._board_frame = frame if frame else "chessboard_frame"
                    self.get_logger().info(
                        f"Synchronized board params: W={self._board_width:.4f}m, "
                        f"H={self._board_height:.4f}m, frame='{self._board_frame}'"
                    )
                    self._check_configuration_complete()
        except Exception as e:
            self.get_logger().warn(f"Failed to read board parameters: {e}")

    def _on_omni_params_received(self, future):
        """Callback when /omni_base_controller responds."""
        self._robot_param_future = None
        try:
            result = future.result()
            if result and len(result.values) >= 1:
                r = result.values[0].double_value
                if r > 0.01:
                    self._robot_radius = float(r)
                    self.get_logger().info(
                        f"Synchronized robot radius: {self._robot_radius:.4f}m"
                    )
                    self._check_configuration_complete()
        except Exception as e:
            self.get_logger().warn(f"Failed to read /omni_base_controller radius: {e}")

    def _on_cm_params_received(self, future):
        """Callback when /controller_manager responds with omni_base_controller.robot_radius."""
        self._robot_param_future = None
        try:
            result = future.result()
            if result and len(result.values) >= 1:
                r = result.values[0].double_value
                if r > 0.01:
                    self._robot_radius = float(r)
                    self.get_logger().info(
                        f"Synchronized robot radius from controller_manager: {self._robot_radius:.4f}m"
                    )
                    self._check_configuration_complete()
        except Exception as e:
            self.get_logger().warn(f"Failed to read controller_manager radius: {e}")

    def _check_configuration_complete(self):
        """Check if both board and robot radius are available."""
        if (
            self._board_width is not None
            and self._board_height is not None
            and self._robot_radius is not None
        ):
            self._is_configured = True
            self._failsafe_reason = ""
            if self._sync_timer:
                self._sync_timer.cancel()
            self.get_logger().info(
                f"Dynamic parameters synchronized successfully! "
                f"Board: W={self._board_width:.3f}m, H={self._board_height:.3f}m, frame='{self._board_frame}'. "
                f"Robot radius: {self._robot_radius:.4f}m."
            )

    # ================= Service Handler =================

    # ================= Service Handler =================
    def _create_base_pose_stamped(
        self, x_board: float, y_board: float, yaw_board: float
    ) -> Optional[PoseStamped]:
        """Convert a (x, y, yaw) standoff pose in chessboard_frame to PoseStamped in map_frame."""
        st_pose_board = PoseStamped()
        st_pose_board.header.stamp = self.get_clock().now().to_msg()
        st_pose_board.header.frame_id = self._board_frame
        st_pose_board.pose.position.x = x_board
        st_pose_board.pose.position.y = y_board
        st_pose_board.pose.position.z = 0.0
        st_pose_board.pose.orientation = yaw_to_quaternion(yaw_board)

        try:
            tf_board_to_map = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._board_frame,
                Time(),
                timeout=Duration(seconds=0.15),
            )
            return tf2_geometry_msgs.do_transform_pose_stamped(
                st_pose_board, tf_board_to_map
            )
        except TransformException as ex:
            self.get_logger().error(f"Failed to transform base pose to map frame: {ex}")
            return None

    def _dict_to_joint_state(self, joint_dict: Dict[str, float]) -> JointState:
        """Convert joint dictionary into sensor_msgs/JointState."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = self._base_frame
        js.name = list(joint_dict.keys())
        js.position = list(joint_dict.values())
        return js

    def handle_check_move_feasibility(
        self,
        request: CheckMoveFeasibility.Request,
        response: CheckMoveFeasibility.Response,
    ) -> CheckMoveFeasibility.Response:
        """Handle move feasibility queries using Tiered Cost-Based Decision Strategy."""
        # 1. Strict Failsafe check
        if not self._is_configured:
            response.feasible = False
            response.plan_type = CheckMoveFeasibility.Response.PLAN_DUAL_BASE
            response.message = f"{self._failsafe_reason}. Service rejected for safety."
            self.get_logger().warn(
                f"Move feasibility check rejected: {response.message}"
            )
            return response

        target_frame = (
            request.target_frame if request.target_frame else self._board_frame
        )
        required_pitch = (
            request.required_pitch_angle
            if request.required_pitch_angle != 0.0
            else DEFAULT_PITCH_ANGLE
        )

        # 2. Transform target points to chessboard_frame
        pick_pt_board = request.pick_point
        place_pt_board = request.place_point
        if target_frame != self._board_frame:
            try:
                tf_to_board = self._tf_buffer.lookup_transform(
                    self._board_frame,
                    target_frame,
                    Time(),
                    timeout=Duration(seconds=0.15),
                )
                pick_pt_board = transform_point_by_tf(request.pick_point, tf_to_board)
                if not request.is_capture:
                    place_pt_board = transform_point_by_tf(
                        request.place_point, tf_to_board
                    )
            except TransformException as ex:
                response.feasible = False
                response.message = f"TF transform to '{self._board_frame}' failed: {ex}"
                self.get_logger().error(response.message)
                return response

        # =========================================================================
        # Case A: Piece Capture Move (is_capture == True)
        # Dedicated PLAN_SINGLE_BASE: Only pick victim piece, arm drops it into
        # the onboard chassis bin (zero base navigation for drop).
        # =========================================================================
        if request.is_capture:
            # Compute optimal standoff for pick_point
            x_st, y_st, th_st, edge = compute_standoff_pose(
                target_x=pick_pt_board.x,
                target_y=pick_pt_board.y,
                board_w=self._board_width,
                board_h=self._board_height,
                robot_radius=self._robot_radius,
                d_clearance=self._clearance_padding,
            )

            # Solve IK at this standoff
            b_pick_x, b_pick_y, b_pick_z = transform_point_to_base_frame(
                pick_pt_board.x, pick_pt_board.y, pick_pt_board.z, x_st, y_st, th_st
            )
            ok_pick, sol_pick, _, ik_msg = solve_analytical_ik(
                b_pick_x, b_pick_y, b_pick_z, required_pitch=required_pitch
            )

            base_pose_map = self._create_base_pose_stamped(x_st, y_st, th_st)
            if not base_pose_map:
                response.feasible = False
                response.message = "Failed to transform victim base pose to map frame."
                return response

            response.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
            response.feasible = ok_pick
            response.pick_base_pose = base_pose_map
            response.place_base_pose = base_pose_map
            if ok_pick and sol_pick:
                response.pick_ik_solution = self._dict_to_joint_state(sol_pick)
            response.message = (
                f"CAPTURE MOVE: PLAN_SINGLE_BASE to edge {edge}. "
                f"Pick victim piece and deposit into onboard bin. IK: {ik_msg}"
            )
            return response

        # =========================================================================
        # Case B: Standard Chess Move (Pick & Place)
        # 4-Tier Cost-Based Execution Strategy
        # =========================================================================

        # ---------------- Tier 0: Zero-Nav Check (Current robot pose) ----------------
        try:
            tf_to_base = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._board_frame,
                Time(),
                timeout=Duration(seconds=0.10),
            )
            curr_pick_base = transform_point_by_tf(pick_pt_board, tf_to_base)
            curr_place_base = transform_point_by_tf(place_pt_board, tf_to_base)

            ok_p0, sol_p0, _, _ = solve_analytical_ik(
                curr_pick_base.x,
                curr_pick_base.y,
                curr_pick_base.z,
                required_pitch=required_pitch,
            )
            ok_d0, sol_d0, _, _ = solve_analytical_ik(
                curr_place_base.x,
                curr_place_base.y,
                curr_place_base.z,
                required_pitch=required_pitch,
            )

            if ok_p0 and ok_d0 and sol_p0 and sol_d0:
                # Get current robot pose in map
                tf_base_in_map = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    self._base_frame,
                    Time(),
                    timeout=Duration(seconds=0.10),
                )
                curr_map_pose = PoseStamped()
                curr_map_pose.header.stamp = self.get_clock().now().to_msg()
                curr_map_pose.header.frame_id = self._map_frame
                curr_map_pose.pose.position.x = tf_base_in_map.transform.translation.x
                curr_map_pose.pose.position.y = tf_base_in_map.transform.translation.y
                curr_map_pose.pose.position.z = tf_base_in_map.transform.translation.z
                curr_map_pose.pose.orientation = tf_base_in_map.transform.rotation

                response.plan_type = CheckMoveFeasibility.Response.PLAN_ZERO_NAV
                response.feasible = True
                response.pick_base_pose = curr_map_pose
                response.place_base_pose = curr_map_pose
                response.pick_ik_solution = self._dict_to_joint_state(sol_p0)
                response.place_ik_solution = self._dict_to_joint_state(sol_d0)
                response.message = "PLAN_ZERO_NAV: Current base pose satisfies both pick and place! (0s Nav2)"
                return response
        except TransformException:
            # Non-fatal: if current base pose lookup fails, proceed to Tier 1
            pass

        # ---------- Tier 1: Early Geometric Pruning (O(1) chord check) -----------
        possible_single = is_single_base_geometrically_possible(
            pick_pt_board.x,
            pick_pt_board.y,
            place_pt_board.x,
            place_pt_board.y,
        )

        # --------- Tier 2: 1D Perimeter Fast Sampling (Single-Base Search) --------
        if possible_single:
            common_sol = find_common_standoff_pose(
                pick_x=pick_pt_board.x,
                pick_y=pick_pt_board.y,
                pick_z=pick_pt_board.z,
                place_x=place_pt_board.x,
                place_y=place_pt_board.y,
                place_z=place_pt_board.z,
                board_w=self._board_width,
                board_h=self._board_height,
                robot_radius=self._robot_radius,
                d_clearance=self._clearance_padding,
                required_pitch=required_pitch,
            )

            if common_sol is not None:
                c_x, c_y, c_th, sol_pick, sol_place, edge = common_sol
                c_pose_map = self._create_base_pose_stamped(c_x, c_y, c_th)
                if c_pose_map:
                    response.plan_type = CheckMoveFeasibility.Response.PLAN_SINGLE_BASE
                    response.feasible = True
                    response.pick_base_pose = c_pose_map
                    response.place_base_pose = c_pose_map
                    response.pick_ik_solution = self._dict_to_joint_state(sol_pick)
                    response.place_ik_solution = self._dict_to_joint_state(sol_place)
                    response.message = f"PLAN_SINGLE_BASE: Common standoff found on edge {edge} (1x Nav2)"
                    return response

        # -------- Tier 3: Dual-Base Generation (Safe Nav2-Interleaved Fallback) --------
        x_st1, y_st1, th_st1, edge1 = compute_standoff_pose(
            target_x=pick_pt_board.x,
            target_y=pick_pt_board.y,
            board_w=self._board_width,
            board_h=self._board_height,
            robot_radius=self._robot_radius,
            d_clearance=self._clearance_padding,
        )
        x_st2, y_st2, th_st2, edge2 = compute_standoff_pose(
            target_x=place_pt_board.x,
            target_y=place_pt_board.y,
            board_w=self._board_width,
            board_h=self._board_height,
            robot_radius=self._robot_radius,
            d_clearance=self._clearance_padding,
        )

        b_pick_x, b_pick_y, b_pick_z = transform_point_to_base_frame(
            pick_pt_board.x, pick_pt_board.y, pick_pt_board.z, x_st1, y_st1, th_st1
        )
        b_place_x, b_place_y, b_place_z = transform_point_to_base_frame(
            place_pt_board.x, place_pt_board.y, place_pt_board.z, x_st2, y_st2, th_st2
        )

        ok_pick, sol_pick, _, msg_p = solve_analytical_ik(
            b_pick_x, b_pick_y, b_pick_z, required_pitch=required_pitch
        )
        ok_place, sol_place, _, msg_d = solve_analytical_ik(
            b_place_x, b_place_y, b_place_z, required_pitch=required_pitch
        )

        pose1_map = self._create_base_pose_stamped(x_st1, y_st1, th_st1)
        pose2_map = self._create_base_pose_stamped(x_st2, y_st2, th_st2)

        if not pose1_map or not pose2_map:
            response.feasible = False
            response.message = "Failed to transform dual base poses to map frame."
            return response

        response.plan_type = CheckMoveFeasibility.Response.PLAN_DUAL_BASE
        response.feasible = ok_pick and ok_place
        response.pick_base_pose = pose1_map
        response.place_base_pose = pose2_map
        if ok_pick and sol_pick:
            response.pick_ik_solution = self._dict_to_joint_state(sol_pick)
        if ok_place and sol_place:
            response.place_ik_solution = self._dict_to_joint_state(sol_place)

        reason = (
            "Distance exceeded reach"
            if not possible_single
            else "No perimeter overlap found"
        )
        response.message = (
            f"PLAN_DUAL_BASE ({reason}): Move to {edge1} for Pick -> Stow Arm -> Move to {edge2} for Place. "
            f"IK: pick={'OK' if ok_pick else msg_p}, place={'OK' if ok_place else msg_d}"
        )
        return response

    # ================= Diagnostics =================

    def publish_diagnostics(self):
        """Publish node status and parameter synchronization state."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "lekiwi_control: Workspace Checker"
        status.hardware_id = "lekiwi_workspace_checker"

        if self._is_configured:
            status.level = DiagnosticStatus.OK
            status.message = "Configured & Active (Closed-Form IK)"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = self._failsafe_reason

        status.values = [
            KeyValue(key="is_configured", value=str(self._is_configured)),
            KeyValue(key="board_frame", value=str(self._board_frame)),
            KeyValue(
                key="board_width",
                value=f"{self._board_width:.4f} m" if self._board_width else "N/A",
            ),
            KeyValue(
                key="board_height",
                value=f"{self._board_height:.4f} m" if self._board_height else "N/A",
            ),
            KeyValue(
                key="robot_radius",
                value=f"{self._robot_radius:.4f} m" if self._robot_radius else "N/A",
            ),
        ]

        msg.status.append(status)
        self._diag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
