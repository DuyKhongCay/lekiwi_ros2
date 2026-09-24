# ==============================================================================
# Micro Move Entity: Đóng gói toàn bộ thuộc tính của một nước cờ FIDE
# ==============================================================================

# Định danh nước đi
string uci                   # Ký hiệu UCI (e.g., "e2e4", "e7e8q", "e1g1")
string san                   # Ký hiệu chuẩn FIDE (e.g., "e4", "Nf3", "O-O", "exd6 e.p.", "e8=Q#")

# Tọa độ hình học trên bàn cờ
string from_square           # Ô xuất phát (e.g., "e2")
string to_square             # Ô đích đến của quân cờ chính (e.g., "e4")

# Ngữ nghĩa quân cờ
string piece_type            # Loại quân di chuyển: "pawn", "knight", "bishop", "rook", "queen", "king"
string piece_color           # Màu quân di chuyển: "w" hoặc "b"
string promotion_piece       # Quân phong cấp: "queen", "knight", "rook", "bishop" hoặc ""

# Đặc tính tương tác vật lý (Phục vụ trực tiếp cho Manipulation)
bool is_capture              # Có ăn quân hay không
string captured_piece_type   # Loại quân bị ăn (e.g., "pawn", "bishop"...) hoặc ""
string captured_square       # Ô chứa quân bị ăn (Bình thường trùng to_square; Khi En Passant là ô cạnh bên!)

# Cờ nước đi đặc biệt
bool is_en_passant           # Nước bắt tốt qua đường
bool is_castling             # Nước nhập thành

# Dữ liệu động học phụ trợ cho nước Nhập thành (Castling Kinematics)
string castling_rook_from    # Ô ban đầu của xe (e.g., "h1" hoặc "a1")
string castling_rook_to      # Ô đích của xe (e.g., "f1" hoặc "d1")
