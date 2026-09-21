"""Standoff & Edge Clearance Calibrator Node for LeKiwi."""

import math
import os
import sys
import time
from typing import Optional, Tuple
import yaml

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger


class StandoffCalibratorNode(Node):
    """Calibrates and evaluates edge clearance distance between LeKiwi base and chessboard."""

    def __init__(self) -> None:
        super().__init__("standoff_calibrator")

        self.declare_parameter("board_frame", "chessboard_frame")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("board_w", 0.390)
        self.declare_parameter("board_h", 0.390)
        self.declare_parameter("front_wheel_offset", 0.065)
        self.declare_parameter("nominal_reach", 0.245)
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter(
            "output_yaml",
            "/root/docker_ws/lekiwi_ros2/lekiwi_calibration/config/calib_result.yaml",
        )

        self.board_frame: str = str(self.get_parameter("board_frame").value)
        self.base_frame: str = str(self.get_parameter("base_frame").value)
        self.half_w: float = float(self.get_parameter("board_w").value) / 2.0
        self.half_h: float = float(self.get_parameter("board_h").value) / 2.0
        self.front_wheel_offset: float = float(
            self.get_parameter("front_wheel_offset").value
        )
        self.nominal_reach: float = float(self.get_parameter("nominal_reach").value)
        self.output_yaml: str = str(self.get_parameter("output_yaml").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.save_srv = self.create_service(
            Trigger, "/calibration/save_standoff", self.handle_save_standoff
        )

        self.last_measured_clearance: Optional[float] = None
        self.last_edge: Optional[str] = None
        self.last_pose: Optional[Tuple[float, float, float]] = None
        self.last_gripper_pose: Optional[Tuple[float, float, float]] = None

        rate_hz: float = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / rate_hz, self.update_callback)

        self.get_logger().info(
            f"StandoffCalibratorNode initialized. Monitoring TF '{self.board_frame}' -> '{self.base_frame}'"
        )

    def update_callback(self) -> None:
        """Looks up TF and prints formatted diagnostic assessment."""
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                self.board_frame, self.base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().info(
                f"Waiting for TF between '{self.board_frame}' and '{self.base_frame}'... ({ex})",
                throttle_duration_sec=3.0,
            )
            return

        rx = t.transform.translation.x
        ry = t.transform.translation.y
        rz = t.transform.translation.z

        # Orientation yaw
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.last_pose = (rx, ry, yaw)

        # Distances to each of the 4 outer edges of the board
        south_dist = ry + self.half_h
        north_dist = self.half_h - ry
        west_dist = rx + self.half_w
        east_dist = self.half_w - rx

        # Determine closest edge
        min_dist = south_dist
        edge = "SOUTH"
        edge_clearance = -ry - self.half_h

        if north_dist < min_dist:
            min_dist = north_dist
            edge = "NORTH"
            edge_clearance = ry - self.half_h
        if west_dist < min_dist:
            min_dist = west_dist
            edge = "WEST"
            edge_clearance = -rx - self.half_w
        if east_dist < min_dist:
            min_dist = east_dist
            edge = "EAST"
            edge_clearance = rx - self.half_w

        self.last_edge = edge
        self.last_measured_clearance = edge_clearance

        # Real-time Gripper Tracking directly from TF
        gripper_info = "Chưa phát hiện frame gripper"
        gripper_pos = None
        for g_frame in ["gripperframe", "gripper", "moving_jaw_so101_v1"]:
            try:
                tg = self.tf_buffer.lookup_transform(self.board_frame, g_frame, rclpy.time.Time())
                gx = tg.transform.translation.x
                gy = tg.transform.translation.y
                gz = tg.transform.translation.z
                gripper_pos = (gx, gy, gz)
                self.last_gripper_pose = gripper_pos
                dist_to_center = math.sqrt(gx * gx + gy * gy)

                tb = self.tf_buffer.lookup_transform(self.base_frame, g_frame, rclpy.time.Time())
                arm_reach_forward = tb.transform.translation.x
                arm_reach_dist = math.sqrt(tb.transform.translation.x**2 + tb.transform.translation.y**2)

                # Depth into chessboard from approaching edge
                if edge == "NORTH":
                    gripper_depth = self.half_h - gy
                elif edge == "SOUTH":
                    gripper_depth = gy + self.half_h
                elif edge == "EAST":
                    gripper_depth = self.half_w - gx
                else:  # WEST
                    gripper_depth = gx + self.half_w

                if dist_to_center <= 0.08:
                    reach_eval = f"ĐANG Ở VÙNG TÂM BÀN CỜ (cách tâm {dist_to_center * 100:.1f} cm, cao {gz * 100:.1f} cm)"
                else:
                    reach_eval = f"Vươn sâu {gripper_depth * 100:.1f} cm vào bàn (cách tâm {dist_to_center * 100:.1f} cm)"

                gripper_info = (
                    f"pos=[{gx:+.3f}, {gy:+.3f}, {gz:+.3f}] | "
                    f"Tầm với Base->Gripper: {arm_reach_dist * 100:.1f} cm | {reach_eval}"
                )
                break
            except tf2_ros.TransformException:
                pass

        # Wheel safety buffer
        wheel_buffer = edge_clearance - self.front_wheel_offset
        if wheel_buffer >= 0.015:
            wheel_status = f"SAFE (+{wheel_buffer * 100:.1f} cm clearance)"
        elif wheel_buffer >= 0.0:
            wheel_status = f"CLOSE (+{wheel_buffer * 100:.1f} cm clearance)"
        else:
            wheel_status = f"COLLISION RISK ({wheel_buffer * 100:.1f} cm into edge)"

        # Nav2 costmap impact
        nav2_robot_radius = 0.160
        nav2_inflation = 0.220
        nav2_overlap = nav2_robot_radius - edge_clearance

        # Format terminal report
        report = (
            f"\n┌── [LeKiwi Standoff Calibration] ────────────────────────────────────────┐\n"
            f"│ Vị trí Base:    x={rx:+.3f}m, y={ry:+.3f}m, yaw={math.degrees(yaw):+.1f}°\n"
            f"│ Cạnh đối diện:  {edge:<5s} | Edge Clearance đo được: {edge_clearance:.4f} m ({edge_clearance * 100:.1f} cm)\n"
            f"│ 1. Gripper TF:  {gripper_info}\n"
            f"│ 2. An toàn xe:  {wheel_status}\n"
            f"│ 3. Nav2 Costmap:Chênh lệch so với robot_radius (16cm): {nav2_overlap * 100:+.1f} cm\n"
            f"│ Gợi ý control:  planning.edge_clearance: {max(0.065, round(edge_clearance, 3)):.3f}\n"
            f"└─────────────────────────────────────────────────────────────────────────┘"
        )
        self.get_logger().info(report, throttle_duration_sec=1.5)

    def handle_save_standoff(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Saves calibrated edge clearance to YAML file."""
        if self.last_measured_clearance is None:
            response.success = False
            response.message = "Chưa nhận được dữ liệu TF giữa robot và bàn cờ."
            return response

        dist = max(0.065, min(0.120, self.last_measured_clearance))
        data = {
            "standoff_calibration": {
                "timestamp": time.time(),
                "edge": self.last_edge,
                "measured_edge_clearance": float(self.last_measured_clearance),
                "recommended_edge_clearance": float(dist),
                "robot_pose": {
                    "x": float(self.last_pose[0]) if self.last_pose else 0.0,
                    "y": float(self.last_pose[1]) if self.last_pose else 0.0,
                    "yaw_deg": (
                        float(math.degrees(self.last_pose[2]))
                        if self.last_pose
                        else 0.0
                    ),
                },
                "gripper_pose": {
                    "x": float(self.last_gripper_pose[0]) if self.last_gripper_pose else 0.0,
                    "y": float(self.last_gripper_pose[1]) if self.last_gripper_pose else 0.0,
                    "z": float(self.last_gripper_pose[2]) if self.last_gripper_pose else 0.0,
                },
            }
        }

        try:
            os.makedirs(os.path.dirname(self.output_yaml), exist_ok=True)
            with open(self.output_yaml, "w") as f:
                yaml.dump(data, f, default_flow_style=False)
            response.success = True
            response.message = (
                f"Đã lưu edge_clearance={dist:.4f}m vào {self.output_yaml} "
                f"(Khuyến nghị: planning.edge_clearance: {dist:.3f})"
            )
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Lỗi ghi file YAML: {str(e)}"

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StandoffCalibratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
