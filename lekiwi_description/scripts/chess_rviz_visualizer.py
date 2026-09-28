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

"""
Node trực quan hóa trạng thái bàn cờ 3D và nước cờ trên RViz2 cho robot LeKiwi.

Tính năng:
- Đọc thông điệp /chess/game_status (lekiwi_interfaces/msg/ChessGameStatus).
- Dựng 32 quân cờ 3D Mesh STL thực tế (đã chuẩn hóa gốc tọa độ tâm đáy).
- Tô màu ô xuất phát, ô đích, ô ăn quân (En Passant) và ô Vua bị chiếu.
- Vẽ mũi tên 3D biểu diễn nước đi vừa thực hiện (last_move) và nước tối ưu (best_move).
- Bảng thông tin 3D HUD lơ lửng hiển thị FEN, Eval Stockfish và Game Phase.
"""

from typing import Any
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from lekiwi_interfaces.msg import ChessGameStatus, ChessMoveDetails


PIECE_TYPE_MAP = {
    'p': 'pawn',
    'r': 'rook',
    'n': 'knight',
    'b': 'bishop',
    'q': 'queen',
    'k': 'king'
}

PHASE_LABELS = {
    ChessGameStatus.PHASE_WAITING_PLAYER: "WAITING FOR PLAYER",
    ChessGameStatus.PHASE_ROBOT_THINKING: "ROBOT THINKING...",
    ChessGameStatus.PHASE_ROBOT_READY: "ROBOT READY",
    ChessGameStatus.PHASE_ROBOT_EXECUTING: "ROBOT EXECUTING MOVE",
    ChessGameStatus.PHASE_GAME_OVER: "GAME OVER"
}


class ChessRvizVisualizer(Node):
    """Visualizes live 3D chess pieces, highlights, move trajectories, and HUD in RViz2."""

    def __init__(self):
        super().__init__('chess_rviz_visualizer')

        # -----------------------------------------------------------------
        # 1. Parameter Declarations
        # -----------------------------------------------------------------
        self.declare_parameter(
            'chessboard_frame',
            'chessboard_frame',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Target frame ID for chessboard visualization.'
            )
        )
        self.declare_parameter(
            'game_status_topic',
            '/chess/game_status',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Input ChessGameStatus topic name.'
            )
        )
        self.declare_parameter(
            'marker_topic',
            '/chess/game_markers',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Output MarkerArray topic name.'
            )
        )
        self.declare_parameter(
            'square_size',
            0.0475,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Side length of each chessboard square in meters.'
            )
        )
        self.declare_parameter(
            'board_z',
            0.006,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Top surface Z coordinate of chessboard relative to frame (meters).'
            )
        )
        self.declare_parameter('enable_pieces', True)
        self.declare_parameter('enable_highlights', True)
        self.declare_parameter('enable_arrows', True)
        self.declare_parameter('enable_hud', True)

        self.frame_id = self.get_parameter('chessboard_frame').value
        game_status_topic = self.get_parameter('game_status_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.square_size = float(self.get_parameter('square_size').value)
        self.board_z = float(self.get_parameter('board_z').value)

        self.enable_pieces = bool(self.get_parameter('enable_pieces').value)
        self.enable_highlights = bool(self.get_parameter('enable_highlights').value)
        self.enable_arrows = bool(self.get_parameter('enable_arrows').value)
        self.enable_hud = bool(self.get_parameter('enable_hud').value)

        # -----------------------------------------------------------------
        # 2. Setup QoS, Publisher & Subscriber
        # -----------------------------------------------------------------
        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.status_sub = self.create_subscription(
            ChessGameStatus,
            game_status_topic,
            self.game_status_callback,
            sub_qos
        )

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, pub_qos)

        # Trạng thái ô cờ đã được dựng trước đó (để gửi Marker.DELETE đúng ID)
        self.previously_occupied_squares: set[int] = set()

        self.get_logger().info(
            f"ChessRvizVisualizer started. Listening to '{game_status_topic}', "
            f"publishing 3D STL pieces & moves to '{marker_topic}' on '{self.frame_id}'."
        )

    # ---------------------------------------------------------------------
    # Coordinate Helpers
    # ---------------------------------------------------------------------
    def square_to_metric(self, square: str) -> Optional[Tuple[float, float]]:
        """Chuyển đổi chuỗi ô chuẩn FIDE (e.g. 'e4') sang tọa độ (x, y) trên chessboard_frame."""
        if not square or len(square) < 2:
            return None
        cleaned = square.strip().lower()
        file_char = cleaned[0]
        rank_char = cleaned[1]

        if not ('a' <= file_char <= 'h' and '1' <= rank_char <= '8'):
            return None

        file_idx = ord(file_char) - ord('a')  # 0..7
        rank_idx = ord(rank_char) - ord('1')  # 0..7

        # Tâm ô cờ đối với hệ tọa độ bàn cờ có tâm tại (0,0)
        x = (file_idx - 3.5) * self.square_size
        y = (rank_idx - 3.5) * self.square_size
        return x, y

    def parse_fen_board(self, full_fen: str) -> List[Optional[str]]:
        """
        Giải mã FEN thành mảng 64 phần tử từ a1 (index 0) đến h8 (index 63).
        Trả về ký tự đại diện quân cờ (e.g. 'P', 'k') hoặc None nếu ô trống.
        """
        board: List[Optional[str]] = [None] * 64
        if not full_fen:
            return board

        fen_board_part = full_fen.split()[0]
        ranks = fen_board_part.split('/')
        if len(ranks) != 8:
            return board

        # FEN bắt đầu từ hàng 8 (rank 7) xuống hàng 1 (rank 0)
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
    # Main Callback
    # ---------------------------------------------------------------------
    def game_status_callback(self, msg: ChessGameStatus):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        # 1. 3D Mesh STL Pieces
        if self.enable_pieces:
            piece_markers = self._build_piece_markers(msg.full_fen, now)
            markers.markers.extend(piece_markers)

        # 2. Square Highlights (From, To, Capture, Check)
        if self.enable_highlights:
            highlight_markers: list[Any] = self._build_highlight_markers(msg, now)
            markers.markers.extend(highlight_markers)

        # 3. 3D Trajectory Arrows
        if self.enable_arrows:
            arrow_markers = self._build_arrow_markers(msg, now)
            markers.markers.extend(arrow_markers)

        # 4. 3D Floating HUD Billboard
        if self.enable_hud:
            hud_marker = self._build_hud_marker(msg, now)
            markers.markers.append(hud_marker)

        self.marker_pub.publish(markers)

    # ---------------------------------------------------------------------
    # Marker Builders
    # ---------------------------------------------------------------------
    def _build_piece_markers(self, full_fen: str, stamp) -> List[Marker]:
        markers: List[Marker] = []
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
                color_name = 'white' if is_white else 'black'
                piece_type = PIECE_TYPE_MAP.get(piece_char.lower(), 'pawn')

                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = stamp
                marker.ns = 'chess/pieces'
                marker.id = sq_idx
                marker.type = Marker.MESH_RESOURCE
                marker.action = Marker.ADD
                marker.mesh_resource = (
                    f"package://lekiwi_description/assets/chess_pieces/{color_name}_{piece_type}.stl"
                )
                # Tỷ lệ: đơn vị file STL là mm -> đổi sang mét chuẩn ROS (0.001)
                marker.scale.x = 0.001
                marker.scale.y = 0.001
                marker.scale.z = 0.001

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = self.board_z

                # Hướng quay: Quân Mã Đen quay 180 độ đối diện với Mã Trắng
                if piece_char == 'n':
                    marker.pose.orientation.z = 1.0
                    marker.pose.orientation.w = 0.0
                else:
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0

                # Màu sắc quân cờ
                if is_white:
                    marker.color = ColorRGBA(r=0.94, g=0.92, b=0.84, a=1.0)  # Trắng ngà
                else:
                    marker.color = ColorRGBA(r=0.18, g=0.18, b=0.20, a=1.0)  # Đen than

                marker.mesh_use_embedded_materials = False
                marker.frame_locked = True
                markers.append(marker)
            else:
                # Nếu ô này từng có quân nhưng nay đã đi/bị ăn -> gửi Marker.DELETE
                if sq_idx in self.previously_occupied_squares:
                    del_marker = Marker()
                    del_marker.header.frame_id = self.frame_id
                    del_marker.header.stamp = stamp
                    del_marker.ns = 'chess/pieces'
                    del_marker.id = sq_idx
                    del_marker.action = Marker.DELETE
                    markers.append(del_marker)

        self.previously_occupied_squares = current_occupied_squares
        return markers

    def _build_highlight_markers(self, msg: ChessGameStatus, stamp) -> List[Marker]:
        markers: List[Marker] = []
        last_move = msg.last_move_details
        tile_size = self.square_size * 0.94
        tile_thick = 0.001
        z_pos = self.board_z + 0.0005

        # 1. From Square Highlight (Vàng cam bán trong suốt)
        if last_move.from_square:
            pt = self.square_to_metric(last_move.from_square)
            if pt:
                m = self._create_cube_marker(
                    'chess/highlights', 101, pt[0], pt[1], z_pos,
                    tile_size, tile_size, tile_thick,
                    ColorRGBA(r=1.0, g=0.75, b=0.0, a=0.45), stamp
                )
                markers.append(m)
        else:
            markers.append(self._create_delete_marker('chess/highlights', 101, stamp))

        # 2. To Square Highlight (Xanh lục dạ quang bán trong suốt)
        if last_move.to_square:
            pt = self.square_to_metric(last_move.to_square)
            if pt:
                m = self._create_cube_marker(
                    'chess/highlights', 102, pt[0], pt[1], z_pos,
                    tile_size, tile_size, tile_thick,
                    ColorRGBA(r=0.0, g=0.9, b=0.3, a=0.55), stamp
                )
                markers.append(m)
        else:
            markers.append(self._create_delete_marker('chess/highlights', 102, stamp))

        # 3. Capture Square Alert (Bia ngắm Đỏ rực tại ô bị ăn)
        if last_move.is_capture and last_move.captured_square:
            cap_pt = self.square_to_metric(last_move.captured_square)
            if cap_pt:
                cap_marker = Marker()
                cap_marker.header.frame_id = self.frame_id
                cap_marker.header.stamp = stamp
                cap_marker.ns = 'chess/highlights'
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
                cap_marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.7)  # Đỏ rực
                cap_marker.frame_locked = True
                markers.append(cap_marker)
        else:
            markers.append(self._create_delete_marker('chess/highlights', 103, stamp))

        # 4. Check / Checkmate King Alert (Tô đỏ ô quân Vua đang bị chiếu)
        if (msg.is_check or msg.is_checkmate) and msg.full_fen:
            king_char = 'K' if msg.active_color == 'w' else 'k'
            board = self.parse_fen_board(msg.full_fen)
            try:
                k_idx = board.index(king_char)
                kx = (k_idx % 8 - 3.5) * self.square_size
                ky = (k_idx // 8 - 3.5) * self.square_size
                m_king = self._create_cube_marker(
                    'chess/highlights', 104, kx, ky, z_pos + 0.001,
                    tile_size, tile_size, 0.002,
                    ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.75), stamp
                )
                markers.append(m_king)
            except ValueError:
                markers.append(self._create_delete_marker('chess/highlights', 104, stamp))
        else:
            markers.append(self._create_delete_marker('chess/highlights', 104, stamp))

        return markers

    def _build_arrow_markers(self, msg: ChessGameStatus, stamp) -> List[Marker]:
        markers: List[Marker] = []
        last_move = msg.last_move_details
        best_move = msg.best_move_details

        # 1. Last Move Arrow (Màu Cyan uốn nhẹ trong không gian)
        if last_move.from_square and last_move.to_square:
            p_from = self.square_to_metric(last_move.from_square)
            p_to = self.square_to_metric(last_move.to_square)
            if p_from and p_to:
                arrow = self._create_trajectory_arrow(
                    'chess/move_arrows', 201, p_from, p_to,
                    ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.9), stamp
                )
                markers.append(arrow)
        else:
            markers.append(self._create_delete_marker('chess/move_arrows', 201, stamp))

        # 2. Engine Best Move Arrow (Màu Xanh lá dạ quang đề xuất)
        if best_move.from_square and best_move.to_square:
            bp_from = self.square_to_metric(best_move.from_square)
            bp_to = self.square_to_metric(best_move.to_square)
            if bp_from and bp_to:
                arrow_best = self._create_trajectory_arrow(
                    'chess/move_arrows', 202, bp_from, bp_to,
                    ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.85), stamp
                )
                markers.append(arrow_best)
        else:
            markers.append(self._create_delete_marker('chess/move_arrows', 202, stamp))

        # 3. Castling Secondary Rook Arrow (Màu Cam cho quân Xe nhập thành)
        if last_move.is_castling and last_move.castling_rook_from and last_move.castling_rook_to:
            rp_from = self.square_to_metric(last_move.castling_rook_from)
            rp_to = self.square_to_metric(last_move.castling_rook_to)
            if rp_from and rp_to:
                arrow_rook = self._create_trajectory_arrow(
                    'chess/move_arrows', 203, rp_from, rp_to,
                    ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.85), stamp
                )
                markers.append(arrow_rook)
        else:
            markers.append(self._create_delete_marker('chess/move_arrows', 203, stamp))

        return markers

    def _build_hud_marker(self, msg: ChessGameStatus, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'chess/status_hud'
        marker.id = 300
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Đặt bảng lơ lửng phía mép trên của bàn cờ
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.23
        marker.pose.position.z = self.board_z + 0.12
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.016  # Kích thước font 16mm

        phase_str = PHASE_LABELS.get(msg.game_phase, "UNKNOWN")
        turn_str = "WHITE" if msg.active_color == 'w' else "BLACK"
        eval_score = msg.eval_centipawns / 100.0

        last_str = f"{msg.last_move_details.uci} ({msg.last_move_details.san})" if msg.last_move_details.uci else "None"
        best_str = f"{msg.best_move_details.uci} ({msg.best_move_details.san})" if msg.best_move_details.uci else "None"

        alert_str = ""
        if msg.is_checkmate:
            alert_str = " | [CHECKMATE!]"
        elif msg.is_check:
            alert_str = " | [CHECK!]"
        elif msg.is_draw:
            alert_str = " | [DRAW]"

        marker.text = (
            f"● [{phase_str}]{alert_str}\n"
            f"Turn: {turn_str} | Eval: {eval_score:+.2f} (Stockfish)\n"
            f"Last Move: {last_str}\n"
            f"Engine Best: {best_str}"
        )

        # Đổi màu text theo Game Phase
        if msg.game_phase == ChessGameStatus.PHASE_WAITING_PLAYER:
            marker.color = ColorRGBA(r=1.0, g=0.9, b=0.3, a=1.0)  # Vàng
        elif msg.game_phase == ChessGameStatus.PHASE_ROBOT_THINKING:
            marker.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=1.0)  # Xanh lam
        elif msg.game_phase == ChessGameStatus.PHASE_ROBOT_EXECUTING:
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Cam
        elif msg.is_checkmate or msg.game_phase == ChessGameStatus.PHASE_GAME_OVER:
            marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)  # Đỏ
        else:
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # Trắng

        marker.frame_locked = True
        return marker

    # ---------------------------------------------------------------------
    # Utility Factory Methods
    # ---------------------------------------------------------------------
    def _create_cube_marker(self, ns: str, m_id: int, x: float, y: float, z: float,
                            sx: float, sy: float, sz: float, color: ColorRGBA, stamp) -> Marker:
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

    def _create_trajectory_arrow(self, ns: str, m_id: int, p_from: Tuple[float, float],
                                 p_to: Tuple[float, float], color: ColorRGBA, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = m_id
        m.type = Marker.ARROW
        m.action = Marker.ADD

        # Đường kính thân và đầu mũi tên
        m.scale.x = 0.0035  # Đường kính thân (3.5mm)
        m.scale.y = 0.0075  # Đường kính đầu mũi tên (7.5mm)
        m.scale.z = 0.0120  # Chiều dài đầu mũi tên (12mm)

        # Mũi tên nâng cao trên không mô phỏng đường đi gắp nhả
        z_lift = self.board_z + 0.025
        m.points = [
            Point(x=p_from[0], y=p_from[1], z=z_lift),
            Point(x=p_to[0], y=p_to[1], z=z_lift)
        ]
        m.color = color
        m.frame_locked = True
        return m

    def _create_delete_marker(self, ns: str, m_id: int, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = m_id
        m.action = Marker.DELETE
        return m


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ChessRvizVisualizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
