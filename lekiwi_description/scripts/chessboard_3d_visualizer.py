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

"""3D chessboard state and move trajectory visualizer for LeKiwi in RViz2.

Subscribes to /chess/game_status and renders 3D STL chess pieces, board highlight
overlays (origin, target, captures, king check), and 3D trajectory arrows for
executed and engine-recommended moves.
"""

from __future__ import annotations

from typing import Any

import rclpy
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from lekiwi_interfaces.msg import ChessGameStatus, ChessMoveDetails

PIECE_TYPE_MAP = {
    "p": "pawn",
    "r": "rook",
    "n": "knight",
    "b": "bishop",
    "q": "queen",
    "k": "king",
}

CASTLING_ROOK_MAP: dict[str, tuple[str, str]] = {
    "e1g1": ("h1", "f1"),
    "e1c1": ("a1", "d1"),
    "e8g8": ("h8", "f8"),
    "e8c8": ("a8", "d8"),
}


def move_from_square(move: ChessMoveDetails | None) -> str:
    """Extract origin square from UCI string."""
    return move.uci[:2] if (move and len(move.uci) >= 4) else ""


def move_to_square(move: ChessMoveDetails | None) -> str:
    """Extract target square from UCI string."""
    return move.uci[2:4] if (move and len(move.uci) >= 4) else ""


def move_captured_square(move: ChessMoveDetails | None) -> str:
    """Extract square of captured piece, handling en-passant offset."""
    if not move or not move.is_capture:
        return ""
    from_sq = move_from_square(move)
    to_sq = move_to_square(move)
    if move.is_en_passant and len(to_sq) == 2 and len(from_sq) == 2:
        return f"{to_sq[0]}{from_sq[1]}"
    return to_sq


def move_castling_rook(move: ChessMoveDetails | None) -> tuple[str, str]:
    """Return (from_square, to_square) for castling rook or ('', '')."""
    if not move or not move.is_castling:
        return ("", "")
    return CASTLING_ROOK_MAP.get(move.uci, ("", ""))


class Chessboard3DVisualizer(Node):
    """Visualizes live 3D chess pieces, highlights, and move trajectories in RViz2."""

    def __init__(self) -> None:
        """Initialize parameters, subscribers, and publishers for 3D chess visualization."""
        super().__init__("chessboard_3d_visualizer")

        # -----------------------------------------------------------------
        # 1. Parameter Declarations
        # -----------------------------------------------------------------
        self.declare_parameter(
            "chessboard_frame",
            "chessboard_frame",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Target frame ID for chessboard visualization.",
            ),
        )
        self.declare_parameter(
            "game_status_topic",
            "/chess/game_status",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Input ChessGameStatus topic name.",
            ),
        )
        self.declare_parameter(
            "candidate_move_topic",
            "/chess/candidate_move",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Input ChessMoveDetails candidate move topic name.",
            ),
        )
        self.declare_parameter(
            "marker_topic",
            "/chess/game_markers",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Output MarkerArray topic name.",
            ),
        )
        self.declare_parameter(
            "square_size",
            0.0475,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Side length of each chessboard square in meters.",
            ),
        )
        self.declare_parameter(
            "board_z",
            0.006,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Top surface Z coordinate of chessboard relative to frame (meters).",
            ),
        )
        self.declare_parameter("enable_pieces", True)
        self.declare_parameter("enable_highlights", True)
        self.declare_parameter("enable_arrows", True)

        self.frame_id = self.get_parameter("chessboard_frame").value
        game_status_topic = self.get_parameter("game_status_topic").value
        candidate_move_topic = self.get_parameter("candidate_move_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        self.square_size = float(self.get_parameter("square_size").value)
        self.board_z = float(self.get_parameter("board_z").value)

        self.enable_pieces = bool(self.get_parameter("enable_pieces").value)
        self.enable_highlights = bool(self.get_parameter("enable_highlights").value)
        self.enable_arrows = bool(self.get_parameter("enable_arrows").value)

        # -----------------------------------------------------------------
        # 2. Setup QoS, Publisher & Subscriber
        # -----------------------------------------------------------------
        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_sub = self.create_subscription(
            ChessGameStatus, game_status_topic, self.game_status_callback, sub_qos
        )

        candidate_sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.candidate_sub = self.create_subscription(
            ChessMoveDetails,
            candidate_move_topic,
            self.candidate_move_callback,
            candidate_sub_qos,
        )

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, pub_qos)

        # Track latest status and candidate move
        self.candidate_move: ChessMoveDetails | None = None
        self.latest_game_status: ChessGameStatus | None = None

        # Track previously occupied squares to dispatch Marker.DELETE for vacated squares
        self.previously_occupied_squares: set[int] = set()

        self.get_logger().info(
            f"ChessRvizVisualizer started. Listening to '{game_status_topic}' and '{candidate_move_topic}', "
            f"publishing 3D STL pieces & moves to '{marker_topic}' on '{self.frame_id}'."
        )

    # ---------------------------------------------------------------------
    # Coordinate Helpers
    # ---------------------------------------------------------------------
    def square_to_metric(self, square: str) -> tuple[float, float] | None:
        """Convert a standard FIDE square name (e.g., 'e4') to metric (x, y) coordinates."""
        if not square or len(square) < 2:
            return None
        cleaned = square.strip().lower()
        file_char = cleaned[0]
        rank_char = cleaned[1]

        if not ("a" <= file_char <= "h" and "1" <= rank_char <= "8"):
            return None

        file_idx = ord(file_char) - ord("a")  # 0..7
        rank_idx = ord(rank_char) - ord("1")  # 0..7

        # Center offset relative to chessboard origin (0, 0) at the board center
        x = (file_idx - 3.5) * self.square_size
        y = (rank_idx - 3.5) * self.square_size
        return x, y

    def parse_fen_board(self, full_fen: str) -> list[str | None]:
        """Decode a FEN string into a 64-element list from a1 (index 0) to h8 (index 63).

        Args:
            full_fen: Full FEN string representing current game state.

        Returns:
            List of 64 elements containing piece characters (e.g., 'P', 'k') or None for empty squares.
        """
        board: list[str | None] = [None] * 64
        if not full_fen:
            return board

        fen_board_part = full_fen.split()[0]
        ranks = fen_board_part.split("/")
        if len(ranks) != 8:
            return board

        # FEN ranks are serialized from rank 8 (index 7) down to rank 1 (index 0)
        for rank_offset, rank_str in enumerate(ranks):
            rank_idx = 7 - rank_offset
            file_idx = 0
            for ch in rank_str:
                if ch.isdigit():
                    file_idx += int(ch)
                else:
                    if file_idx < 8:
                        sq_idx = rank_idx * 8 + file_idx
                        board[sq_idx] = ch
                        file_idx += 1
        return board

    # ---------------------------------------------------------------------
    # Main Callbacks
    # ---------------------------------------------------------------------
    def game_status_callback(self, msg: ChessGameStatus) -> None:
        """Process game status updates and publish aggregate visual markers."""
        self.latest_game_status = msg
        self.publish_visualization(msg)

    def candidate_move_callback(self, msg: ChessMoveDetails) -> None:
        """Process candidate move updates and publish aggregate visual markers."""
        self.candidate_move = msg
        if self.latest_game_status is not None:
            self.publish_visualization(self.latest_game_status)

    def publish_visualization(self, msg: ChessGameStatus) -> None:
        """Publish aggregate visual markers for current game status and candidate move."""
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        # 1. 3D mesh STL pieces
        if self.enable_pieces:
            piece_markers = self._build_piece_markers(msg.full_fen, now)
            markers.markers.extend(piece_markers)

        # 2. Square highlights (from, to, capture, check)
        if self.enable_highlights:
            highlight_markers: list[Any] = self._build_highlight_markers(msg, now)
            markers.markers.extend(highlight_markers)

        # 3. 3D trajectory arrows
        if self.enable_arrows:
            arrow_markers = self._build_arrow_markers(msg, now)
            markers.markers.extend(arrow_markers)

        self.marker_pub.publish(markers)

    # ---------------------------------------------------------------------
    # Marker Builders
    # ---------------------------------------------------------------------
    def _build_piece_markers(self, full_fen: str, stamp: Any) -> list[Marker]:
        """Build 3D mesh markers for all active chess pieces from FEN state.

        Args:
            full_fen: Full FEN board representation string.
            stamp: ROS timestamp attached to generated markers.

        Returns:
            List of ADD and DELETE Marker objects representing board pieces.
        """
        markers: list[Marker] = []
        board = self.parse_fen_board(full_fen)
        current_occupied_squares: set[int] = set()

        for sq_idx, piece_char in enumerate(board):
            file_idx = sq_idx % 8
            rank_idx = sq_idx // 8
            x = (file_idx - 3.5) * self.square_size
            y = (rank_idx - 3.5) * self.square_size

            if piece_char is not None:
                current_occupied_squares.add(sq_idx)
                is_white = piece_char.isupper()
                color_name = "white" if is_white else "black"
                piece_type = PIECE_TYPE_MAP.get(piece_char.lower(), "pawn")

                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = stamp
                marker.ns = "chess/pieces"
                marker.id = sq_idx
                marker.type = Marker.MESH_RESOURCE
                marker.action = Marker.ADD
                marker.mesh_resource = f"package://lekiwi_description/assets/chess_pieces/{color_name}_{piece_type}.stl"
                # STL model units are in millimeters; convert to ROS standard meters (0.001)
                marker.scale.x = 0.001
                marker.scale.y = 0.001
                marker.scale.z = 0.001

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = self.board_z

                # Rotate black knight 180 degrees to face white knight across the board
                if piece_char == "n":
                    marker.pose.orientation.z = 1.0
                    marker.pose.orientation.w = 0.0
                else:
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0

                # Material colors
                if is_white:
                    marker.color = ColorRGBA(r=0.94, g=0.92, b=0.84, a=1.0)  # Ivory white
                else:
                    marker.color = ColorRGBA(r=0.18, g=0.18, b=0.20, a=1.0)  # Charcoal black

                marker.mesh_use_embedded_materials = False
                marker.frame_locked = True
                markers.append(marker)
            else:
                # Issue Marker.DELETE if square was previously occupied but is now empty
                if sq_idx in self.previously_occupied_squares:
                    del_marker = Marker()
                    del_marker.header.frame_id = self.frame_id
                    del_marker.header.stamp = stamp
                    del_marker.ns = "chess/pieces"
                    del_marker.id = sq_idx
                    del_marker.action = Marker.DELETE
                    markers.append(del_marker)

        self.previously_occupied_squares = current_occupied_squares
        return markers

    def _build_highlight_markers(
        self, msg: ChessGameStatus, stamp: Any
    ) -> list[Marker]:
        """Build square overlay markers highlighting previous moves, captures, and checks.

        Args:
            msg: Current ChessGameStatus message.
            stamp: ROS timestamp attached to generated markers.

        Returns:
            List of highlight Marker objects for square status display.
        """
        markers: list[Marker] = []
        last_move = msg.last_move_details
        tile_size = self.square_size * 0.94
        tile_thick = 0.001
        z_pos = self.board_z + 0.0005

        from_sq = move_from_square(last_move)
        to_sq = move_to_square(last_move)
        cap_sq = move_captured_square(last_move)

        # 1. From-square highlight (semi-transparent amber)
        if from_sq:
            pt = self.square_to_metric(from_sq)
            if pt:
                m = self._create_cube_marker(
                    "chess/highlights",
                    101,
                    pt[0],
                    pt[1],
                    z_pos,
                    tile_size,
                    tile_size,
                    tile_thick,
                    ColorRGBA(r=1.0, g=0.75, b=0.0, a=0.45),
                    stamp,
                )
                markers.append(m)
        else:
            markers.append(self._create_delete_marker("chess/highlights", 101, stamp))

        # 2. To-square highlight (semi-transparent green)
        if to_sq:
            pt = self.square_to_metric(to_sq)
            if pt:
                m = self._create_cube_marker(
                    "chess/highlights",
                    102,
                    pt[0],
                    pt[1],
                    z_pos,
                    tile_size,
                    tile_size,
                    tile_thick,
                    ColorRGBA(r=0.0, g=0.9, b=0.3, a=0.55),
                    stamp,
                )
                markers.append(m)
        else:
            markers.append(self._create_delete_marker("chess/highlights", 102, stamp))

        # 3. Capture square alert (semi-transparent red cylinder)
        if last_move.is_capture and cap_sq:
            cap_pt = self.square_to_metric(cap_sq)
            if cap_pt:
                cap_marker = Marker()
                cap_marker.header.frame_id = self.frame_id
                cap_marker.header.stamp = stamp
                cap_marker.ns = "chess/highlights"
                cap_marker.id = 103
                cap_marker.type = Marker.CYLINDER
                cap_marker.action = Marker.ADD
                cap_marker.pose.position.x = cap_pt[0]
                cap_marker.pose.position.y = cap_pt[1]
                cap_marker.pose.position.z = z_pos + 0.0005
                cap_marker.pose.orientation.w = 1.0
                cap_marker.scale.x = self.square_size * 0.8
                cap_marker.scale.y = self.square_size * 0.8
                cap_marker.scale.z = 0.0015
                cap_marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.7)  # Bright red
                cap_marker.frame_locked = True
                markers.append(cap_marker)
        else:
            markers.append(self._create_delete_marker("chess/highlights", 103, stamp))

        # 4. Check / checkmate alert (semi-transparent bright red cube over king)
        if (msg.is_check or msg.is_checkmate) and msg.full_fen:
            king_char = "K" if msg.active_color == "w" else "k"
            board = self.parse_fen_board(msg.full_fen)
            try:
                k_idx = board.index(king_char)
                kx = (k_idx % 8 - 3.5) * self.square_size
                ky = (k_idx // 8 - 3.5) * self.square_size
                m_king = self._create_cube_marker(
                    "chess/highlights",
                    104,
                    kx,
                    ky,
                    z_pos + 0.001,
                    tile_size,
                    tile_size,
                    0.002,
                    ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.75),
                    stamp,
                )
                markers.append(m_king)
            except ValueError:
                markers.append(
                    self._create_delete_marker("chess/highlights", 104, stamp)
                )
        else:
            markers.append(self._create_delete_marker("chess/highlights", 104, stamp))

        return markers

    def _build_arrow_markers(self, msg: ChessGameStatus, stamp: Any) -> list[Marker]:
        """Build 3D trajectory arrow markers for executed moves and candidate moves.

        Args:
            msg: Current ChessGameStatus message containing move details.
            stamp: ROS timestamp attached to generated markers.

        Returns:
            List of arrow Marker objects for trajectory visualization.
        """
        markers: list[Marker] = []
        last_move = msg.last_move_details
        candidate_move = self.candidate_move

        last_from = move_from_square(last_move)
        last_to = move_to_square(last_move)

        # 1. Last executed move arrow (green)
        if last_from and last_to:
            p_from = self.square_to_metric(last_from)
            p_to = self.square_to_metric(last_to)
            if p_from and p_to:
                arrow = self._create_trajectory_arrow(
                    "chess/move_arrows",
                    201,
                    p_from,
                    p_to,
                    ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.85),
                    stamp,
                )
                markers.append(arrow)
        else:
            markers.append(self._create_delete_marker("chess/move_arrows", 201, stamp))

        # 2. Candidate move arrow (orange)
        cand_from = move_from_square(candidate_move)
        cand_to = move_to_square(candidate_move)
        if cand_from and cand_to:
            bp_from = self.square_to_metric(cand_from)
            bp_to = self.square_to_metric(cand_to)
            if bp_from and bp_to:
                arrow_cand = self._create_trajectory_arrow(
                    "chess/move_arrows",
                    202,
                    bp_from,
                    bp_to,
                    ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.85),
                    stamp,
                )
                markers.append(arrow_cand)
        else:
            markers.append(self._create_delete_marker("chess/move_arrows", 202, stamp))

        # 3. Castling secondary rook move arrow (cyan)
        rook_from, rook_to = move_castling_rook(last_move)
        if rook_from and rook_to:
            rp_from = self.square_to_metric(rook_from)
            rp_to = self.square_to_metric(rook_to)
            if rp_from and rp_to:
                arrow_rook = self._create_trajectory_arrow(
                    "chess/move_arrows",
                    203,
                    rp_from,
                    rp_to,
                    ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.85),
                    stamp,
                )
                markers.append(arrow_rook)
        else:
            markers.append(self._create_delete_marker("chess/move_arrows", 203, stamp))

        return markers

    # ---------------------------------------------------------------------
    # Utility Factory Methods
    # ---------------------------------------------------------------------
    def _create_cube_marker(
        self,
        ns: str,
        m_id: int,
        x: float,
        y: float,
        z: float,
        sx: float,
        sy: float,
        sz: float,
        color: ColorRGBA,
        stamp: Any,
    ) -> Marker:
        """Create a cube marker with specified pose, dimensions, and color."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = m_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = sx
        m.scale.y = sy
        m.scale.z = sz
        m.color = color
        m.frame_locked = True
        return m

    def _create_trajectory_arrow(
        self,
        ns: str,
        m_id: int,
        p_from: tuple[float, float],
        p_to: tuple[float, float],
        color: ColorRGBA,
        stamp: Any,
    ) -> Marker:
        """Create an elevated trajectory arrow marker connecting start and end points."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = m_id
        m.type = Marker.ARROW
        m.action = Marker.ADD

        # Arrow shaft diameter (3.5 mm), head diameter (7.5 mm), head length (12 mm)
        m.scale.x = 0.0035
        m.scale.y = 0.0075
        m.scale.z = 0.0120

        # Elevated trajectory mimicking robotic pick-and-place arc
        z_lift = self.board_z + 0.025
        m.points = [
            Point(x=p_from[0], y=p_from[1], z=z_lift),
            Point(x=p_to[0], y=p_to[1], z=z_lift),
        ]
        m.color = color
        m.frame_locked = True
        return m

    def _create_delete_marker(self, ns: str, m_id: int, stamp: Any) -> Marker:
        """Create a deletion marker for a specific namespace and ID."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = m_id
        m.action = Marker.DELETE
        return m


def main(args: list[str] | None = None) -> None:
    """Initialize ROS 2 client library and spin Chessboard3DVisualizer node."""
    rclpy.init(args=args)
    try:
        node = Chessboard3DVisualizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
