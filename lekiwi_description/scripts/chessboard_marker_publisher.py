#!/usr/bin/env python3
# Copyright 2026 LeKiwi Labs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class ChessboardMarkerPublisher(Node):
    """Publishes static latched RViz MarkerArray for AprilTag corners and 8x8 chessboard gridlines.

    Adheres strictly to REP-105 & RViz2 conventions:
    - Pure parameter-driven configuration without hardcoded values.
    - Transient Local (latched) QoS: published once at startup to eliminate 1Hz scene-graph rebuilding & flickering.
    - Valid normalized unit quaternions (w=1.0) on all poses to prevent degraded text matrix projection.
    - frame_locked enabled to rigidly couple labels and tags to chessboard_frame.
    """

    def __init__(self):
        super().__init__('chessboard_marker_publisher')

        # -----------------------------------------------------------------
        # 1. Parameter Declarations
        # -----------------------------------------------------------------
        self.declare_parameter(
            'chessboard_frame',
            'chessboard_frame',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Target reference frame for published markers.'
            )
        )
        self.declare_parameter(
            'topic_name',
            'chessboard_tag_markers',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Output MarkerArray topic name.'
            )
        )
        self.declare_parameter(
            'tag_size',
            0.029,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Physical AprilTag side length in meters.'
            )
        )

        # Dynamic array parameters from tags namespace (loaded via YAML)
        self.declare_parameter(
            'tags.names',
            rclpy.Parameter.Type.STRING_ARRAY,
            ParameterDescriptor(description='List of AprilTag corner names (e.g. [A1, H1, H8, A8])')
        )
        self.declare_parameter(
            'tags.positions_x',
            rclpy.Parameter.Type.DOUBLE_ARRAY,
            ParameterDescriptor(description='X coordinates of tags in chessboard_frame (meters)')
        )
        self.declare_parameter(
            'tags.positions_y',
            rclpy.Parameter.Type.DOUBLE_ARRAY,
            ParameterDescriptor(description='Y coordinates of tags in chessboard_frame (meters)')
        )
        self.declare_parameter(
            'tags.positions_z',
            rclpy.Parameter.Type.DOUBLE_ARRAY,
            ParameterDescriptor(description='Z coordinates of tags in chessboard_frame (meters)')
        )

        # -----------------------------------------------------------------
        # 2. Extract & Validate Parameters (Fail-Fast)
        # -----------------------------------------------------------------
        self.frame_id = self.get_parameter('chessboard_frame').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        tag_size = self.get_parameter('tag_size').get_parameter_value().double_value

        names = self.get_parameter('tags.names').get_parameter_value().string_array_value
        xs = self.get_parameter('tags.positions_x').get_parameter_value().double_array_value
        ys = self.get_parameter('tags.positions_y').get_parameter_value().double_array_value
        zs = self.get_parameter('tags.positions_z').get_parameter_value().double_array_value

        self._validate_parameters(names, xs, ys, zs)

        self.get_logger().info(
            f'Initialized chessboard visualizer for tags {names}. '
            f'Publishing latched static markers to "{topic_name}" on "{self.frame_id}".'
        )

        # -----------------------------------------------------------------
        # 3. Setup Latched Publisher & Publish Once
        # -----------------------------------------------------------------
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(MarkerArray, topic_name, qos)

        # Build markers with valid quaternions, scale and frame_locked
        self.marker_array = self._build_markers(names, xs, ys, zs, tag_size)

        # Publish initial latched message
        self.pub.publish(self.marker_array)

        # Infrequent heartbeat (every 10s) without re-stamping to allow late DDS discovery without flickering
        self.timer = self.create_timer(10.0, self._heartbeat_callback)

    def _validate_parameters(self, names, xs, ys, zs):
        if not names:
            error_msg = (
                "Parameter 'tags.names' is empty or not provided. "
                "Ensure chessboard_tags.yaml is loaded into node parameters."
            )
            self.get_logger().fatal(error_msg)
            raise ValueError(error_msg)

        n = len(names)
        if not (len(xs) == n and len(ys) == n and len(zs) == n):
            error_msg = (
                f"Tag parameters length mismatch! names={n}, "
                f"positions_x={len(xs)}, positions_y={len(ys)}, positions_z={len(zs)}. "
                "All arrays must have identical length."
            )
            self.get_logger().fatal(error_msg)
            raise ValueError(error_msg)

    def _build_markers(self, names, xs, ys, zs, tag_size: float) -> MarkerArray:
        markers = MarkerArray()

        # -----------------------------------------------------------------
        # 0. CLEANUP: Clear all previous stale markers from RViz scene graph
        # -----------------------------------------------------------------
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        # -----------------------------------------------------------------
        # 1. MARKER: AprilTag Corner Cubes & Text Labels
        # -----------------------------------------------------------------
        for idx, (name, x, y, z) in enumerate(zip(names, xs, ys, zs)):
            # Cube Tag
            box = Marker()
            box.header.frame_id = self.frame_id
            box.ns = 'chessboard/tags'
            box.id = idx
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(x)
            box.pose.position.y = float(y)
            box.pose.position.z = float(z)
            # Valid normalized unit quaternion
            box.pose.orientation.x = 0.0
            box.pose.orientation.y = 0.0
            box.pose.orientation.z = 0.0
            box.pose.orientation.w = 1.0
            box.scale.x = float(tag_size)
            box.scale.y = float(tag_size)
            box.scale.z = 0.002
            box.color = ColorRGBA(r=0.0, g=0.9, b=0.2, a=0.85)  # Bright green
            box.frame_locked = True
            markers.markers.append(box)

            # Text Label with corner name only (A1, H1, H8, A8)
            text = Marker()
            text.header.frame_id = self.frame_id
            text.ns = 'chessboard/labels'
            text.id = 100 + idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(x)
            text.pose.position.y = float(y)
            text.pose.position.z = float(z) + 0.005  # Position 5mm close to tag surface
            # Essential: valid unit quaternion avoids degenerate rotation matrix
            text.pose.orientation.x = 0.0
            text.pose.orientation.y = 0.0
            text.pose.orientation.z = 0.0
            text.pose.orientation.w = 1.0
            text.scale.x = 0.0
            text.scale.y = 0.0
            text.scale.z = 0.015  # Font character height (15mm)
            text.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Bright yellow
            text.text = str(name)
            text.frame_locked = True
            markers.markers.append(text)

        # -----------------------------------------------------------------
        # 2. MARKER: 8x8 Gridlines (LINE_LIST)
        # -----------------------------------------------------------------
        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)
        base_z = sum(zs) / len(zs)

        grid_lines = Marker()
        grid_lines.header.frame_id = self.frame_id
        grid_lines.ns = 'chessboard/gridlines'
        grid_lines.id = 200
        grid_lines.type = Marker.LINE_LIST
        grid_lines.action = Marker.ADD
        grid_lines.pose.orientation.x = 0.0
        grid_lines.pose.orientation.y = 0.0
        grid_lines.pose.orientation.z = 0.0
        grid_lines.pose.orientation.w = 1.0
        grid_lines.scale.x = 0.0015  # 1.5mm line width
        grid_lines.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)  # Light blue
        grid_lines.frame_locked = True

        step_x = (max_x - min_x) / 8.0
        step_y = (max_y - min_y) / 8.0

        for col in range(9):
            x_line = min_x + col * step_x
            p_start = Point(x=float(x_line), y=float(min_y), z=float(base_z))
            p_end = Point(x=float(x_line), y=float(max_y), z=float(base_z))
            grid_lines.points.extend([p_start, p_end])

        for row in range(9):
            y_line = min_y + row * step_y
            p_start = Point(x=float(min_x), y=float(y_line), z=float(base_z))
            p_end = Point(x=float(max_x), y=float(y_line), z=float(base_z))
            grid_lines.points.extend([p_start, p_end])

        markers.markers.append(grid_lines)

        # -----------------------------------------------------------------
        # 3. MARKER: Dark Squares (CUBE_LIST for maximum performance)
        # -----------------------------------------------------------------
        black_squares = Marker()
        black_squares.header.frame_id = self.frame_id
        black_squares.ns = 'chessboard/squares'
        black_squares.id = 300
        black_squares.type = Marker.CUBE_LIST
        black_squares.action = Marker.ADD
        black_squares.pose.orientation.x = 0.0
        black_squares.pose.orientation.y = 0.0
        black_squares.pose.orientation.z = 0.0
        black_squares.pose.orientation.w = 1.0
        black_squares.scale.x = step_x * 0.96
        black_squares.scale.y = step_y * 0.96
        black_squares.scale.z = 0.0005
        black_squares.color = ColorRGBA(r=0.2, g=0.2, b=0.2, a=0.6)  # Dark translucent
        black_squares.frame_locked = True

        for row in range(8):
            for col in range(8):
                if (row + col) % 2 == 0:
                    cx = min_x + (col + 0.5) * step_x
                    cy = min_y + (row + 0.5) * step_y
                    black_squares.points.append(
                        Point(x=float(cx), y=float(cy), z=float(base_z - 0.0005))
                    )

        markers.markers.append(black_squares)
        return markers

    def _heartbeat_callback(self):
        # Re-publish same static markers without re-stamping to avoid scene destruction
        self.pub.publish(self.marker_array)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ChessboardMarkerPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
