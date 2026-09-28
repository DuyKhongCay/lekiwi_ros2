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
Script chuẩn hóa gốc tọa độ cho các file 3D Mesh (.stl) quân cờ.

Chức năng:
1. Đọc từng file binary STL trong thư mục assets/chess_pieces.
2. Tính toán Bounding Box (AABB) và trọng tâm (Center X, Y) và mặt đáy (Min Z).
3. Tịnh tiến toàn bộ đỉnh (vertices) sao cho:
   - Tâm hình học X, Y nằm chính xác tại (0.0, 0.0).
   - Đáy tiếp xúc của quân cờ nằm tại Z = 0.0.
4. Tự động sao lưu (backup) các file STL gốc vào thư mục raw_backup/.
"""

import argparse
import glob
import os
import shutil
import struct
import sys
import numpy as np


# Kiểu cấu trúc binary STL theo chuẩn 50 byte / tam giác
STL_RECORD_DTYPE = np.dtype([
    ('normal', '<f4', (3,)),
    ('v1', '<f4', (3,)),
    ('v2', '<f4', (3,)),
    ('v3', '<f4', (3,)),
    ('attr', '<u2')
])


def get_default_assets_dir() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # script nằm tại lekiwi_description/scripts -> assets tại lekiwi_description/assets/chess_pieces
    candidate = os.path.normpath(os.path.join(script_dir, '..', 'assets', 'chess_pieces'))
    if os.path.isdir(candidate):
        return candidate
    return os.path.abspath('lekiwi_description/assets/chess_pieces')


def recenter_stl(file_path: str, dry_run: bool = False) -> dict:
    with open(file_path, 'rb') as fp:
        header = fp.read(80)
        n_tris_bytes = fp.read(4)
        if len(n_tris_bytes) < 4:
            raise ValueError(f"File {file_path} không phải định dạng binary STL hợp lệ.")
        n_tris = int(np.frombuffer(n_tris_bytes, dtype='<u4')[0])
        raw_body = fp.read()

    expected_len = n_tris * 50
    if len(raw_body) < expected_len:
        raise ValueError(
            f"File {file_path} bị cắt cụt (expected {expected_len} bytes, got {len(raw_body)})."
        )

    data = np.frombuffer(raw_body[:expected_len], dtype=STL_RECORD_DTYPE, count=n_tris).copy()

    all_verts = np.vstack([data['v1'], data['v2'], data['v3']])
    min_pt = all_verts.min(axis=0)
    max_pt = all_verts.max(axis=0)

    center_x = float((min_pt[0] + max_pt[0]) / 2.0)
    center_y = float((min_pt[1] + max_pt[1]) / 2.0)
    min_z = float(min_pt[2])

    dim_x = float(max_pt[0] - min_pt[0])
    dim_y = float(max_pt[1] - min_pt[1])
    dim_z = float(max_pt[2] - min_pt[2])

    # Offset vector đưa tâm XY về 0 và đáy Z về 0
    offset = np.array([center_x, center_y, min_z], dtype=np.float32)

    data['v1'] -= offset
    data['v2'] -= offset
    data['v3'] -= offset

    new_verts = np.vstack([data['v1'], data['v2'], data['v3']])
    new_min = new_verts.min(axis=0)
    new_max = new_verts.max(axis=0)
    new_cx = float((new_min[0] + new_max[0]) / 2.0)
    new_cy = float((new_min[1] + new_max[1]) / 2.0)

    if not dry_run:
        with open(file_path, 'wb') as fp:
            fp.write(header)
            fp.write(struct.pack('<I', n_tris))
            fp.write(data.tobytes())

    return {
        'n_tris': n_tris,
        'orig_center': (center_x, center_y, min_z),
        'dims': (dim_x, dim_y, dim_z),
        'new_center': (new_cx, new_cy, float(new_min[2])),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Chuẩn hóa gốc tọa độ các file 3D Mesh quân cờ (.stl) về tâm đáy (0, 0, 0)."
    )
    parser.add_argument(
        '--assets-dir',
        type=str,
        default=get_default_assets_dir(),
        help='Đường dẫn thư mục chứa các file .stl của quân cờ.'
    )
    parser.add_argument(
        '--no-backup',
        action='store_true',
        help='Không tạo thư mục sao lưu raw_backup/ trước khi ghi đè.'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Chỉ kiểm tra và in thông số, không thay đổi file trên đĩa.'
    )

    args = parser.parse_args()
    assets_dir = os.path.abspath(args.assets_dir)

    if not os.path.isdir(assets_dir):
        print(f"[ERROR] Thư mục không tồn tại: {assets_dir}", file=sys.stderr)
        sys.exit(1)

    stl_files = sorted(glob.glob(os.path.join(assets_dir, '*.stl')))
    if not stl_files:
        print(f"[WARNING] Không tìm thấy file .stl nào trong: {assets_dir}")
        sys.exit(0)

    print("=" * 95)
    print(f" CÔNG CỤ CHUẨN HÓA GỐC TỌA ĐỘ MESH STL QUÂN CỜ (LeKiwi Chess)")
    print(f" Thư mục đích: {assets_dir}")
    print(f" Số lượng file: {len(stl_files)}")
    print(f" Chế độ: {'DRY RUN (Chỉ đọc)' if args.dry_run else 'GHI ĐÈ (Có sao lưu)'}")
    print("=" * 95)

    # Thực hiện sao lưu
    if not args.dry_run and not args.no_backup:
        backup_dir = os.path.join(assets_dir, 'raw_backup')
        os.makedirs(backup_dir, exist_ok=True)
        print(f"[INFO] Đang sao lưu file gốc sang: {backup_dir}")
        for f in stl_files:
            dest = os.path.join(backup_dir, os.path.basename(f))
            if not os.path.exists(dest):
                shutil.copy2(f, dest)

    print("-" * 95)
    print(f"{'Tên file':<20} | {'Tam giác':<8} | {'Kích thước (Dx, Dy, Dz mm)':<26} | {'Tâm cũ (X, Y)':<18} | {'Tâm mới (X, Y)'}")
    print("-" * 95)

    for f in stl_files:
        filename = os.path.basename(f)
        try:
            res = recenter_stl(f, dry_run=args.dry_run)
            dim_str = f"{res['dims'][0]:.1f} x {res['dims'][1]:.1f} x {res['dims'][2]:.1f}"
            orig_c = f"({res['orig_center'][0]:6.2f}, {res['orig_center'][1]:6.2f})"
            new_c = f"({res['new_center'][0]:6.2f}, {res['new_center'][1]:6.2f})"
            print(f"{filename:<20} | {res['n_tris']:<8} | {dim_str:<26} | {orig_c:<18} | {new_c}")
        except Exception as e:
            print(f"{filename:<20} | ERROR: {e}", file=sys.stderr)

    print("-" * 95)
    status_msg = "Kiểm tra hoàn tất (không sửa đổi file)." if args.dry_run else "Chuẩn hóa thành công toàn bộ file STL! Tâm đáy các quân cờ đã ở (0.0, 0.0, 0.0)."
    print(f"[SUCCESS] {status_msg}")
    print("=" * 95)


if __name__ == '__main__':
    main()
