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

"""Normalizes origin coordinates for 3D binary STL mesh chess piece assets.

Reads binary STL files in assets/chess_pieces, computes axis-aligned bounding boxes (AABB),
and translates all vertex coordinates such that:
- Planar geometric center (X, Y) is centered at (0.0, 0.0).
- Contact base of the piece rests exactly at Z = 0.0.
Supports automated backup to raw_backup/ and non-destructive dry-run inspection.
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import struct
import sys
from typing import Any

import numpy as np


# Standard 50-byte record format per triangle for binary STL files
STL_RECORD_DTYPE = np.dtype([
    ('normal', '<f4', (3,)),
    ('v1', '<f4', (3,)),
    ('v2', '<f4', (3,)),
    ('v3', '<f4', (3,)),
    ('attr', '<u2')
])


def get_default_assets_dir() -> str:
    """Resolve default directory path containing chess piece STL assets."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Script resides at lekiwi_description/scripts -> assets at lekiwi_description/assets/chess_pieces
    candidate = os.path.normpath(os.path.join(script_dir, '..', 'assets', 'chess_pieces'))
    if os.path.isdir(candidate):
        return candidate
    return os.path.abspath('lekiwi_description/assets/chess_pieces')


def recenter_stl(file_path: str, dry_run: bool = False) -> dict[str, Any]:
    """Recenter binary STL vertices so XY center is (0, 0) and bottom base is at Z=0.

    Args:
        file_path: Absolute or relative path to the binary STL file.
        dry_run: When True, computes metrics without writing changes to disk.

    Returns:
        Dictionary containing triangle count, original center, dimensions, and new center.

    Raises:
        ValueError: If file is not a valid or complete binary STL.
    """
    with open(file_path, 'rb') as fp:
        header = fp.read(80)
        n_tris_bytes = fp.read(4)
        if len(n_tris_bytes) < 4:
            raise ValueError(f"File {file_path} is not a valid binary STL format.")
        n_tris = int(np.frombuffer(n_tris_bytes, dtype='<u4')[0])
        raw_body = fp.read()

    expected_len = n_tris * 50
    if len(raw_body) < expected_len:
        raise ValueError(
            f"File {file_path} is truncated (expected {expected_len} bytes, got {len(raw_body)})."
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

    # Offset vector to align planar center to (0, 0) and base to Z = 0
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


def main() -> None:
    """Parse command-line arguments and batch-process chess piece STL recentering.

    Scans the specified assets directory for .stl files, creates safety backups if requested,
    executes vertex recentering, and prints a tabular summary of geometric transformations.

    Raises:
        SystemExit: If assets directory does not exist or contains no STL files.
    """
    parser = argparse.ArgumentParser(
        description="Recenter 3D chess piece binary STL meshes to base center (0, 0, 0)."
    )
    parser.add_argument(
        '--assets-dir',
        type=str,
        default=get_default_assets_dir(),
        help='Path to directory containing chess piece .stl files.'
    )
    parser.add_argument(
        '--no-backup',
        action='store_true',
        help='Do not create raw_backup/ directory before modifying files.'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Perform dry-run inspection without writing files to disk.'
    )

    args = parser.parse_args()
    assets_dir = os.path.abspath(args.assets_dir)

    if not os.path.isdir(assets_dir):
        print(f"[ERROR] Directory does not exist: {assets_dir}", file=sys.stderr)
        sys.exit(1)

    stl_files = sorted(glob.glob(os.path.join(assets_dir, '*.stl')))
    if not stl_files:
        print(f"[WARNING] No .stl files found in: {assets_dir}")
        sys.exit(0)

    print("=" * 95)
    print(" LEKIWI CHESS PIECE STL MESH RECENTERING UTILITY")
    print(f" Target directory: {assets_dir}")
    print(f" Total files:     {len(stl_files)}")
    print(f" Mode:            {'DRY RUN (Read-only)' if args.dry_run else 'OVERWRITE (With backup)'}")
    print("=" * 95)

    # Backup original files before modification
    if not args.dry_run and not args.no_backup:
        backup_dir = os.path.join(assets_dir, 'raw_backup')
        os.makedirs(backup_dir, exist_ok=True)
        print(f"[INFO] Backing up original STL files to: {backup_dir}")
        for f in stl_files:
            dest = os.path.join(backup_dir, os.path.basename(f))
            if not os.path.exists(dest):
                shutil.copy2(f, dest)

    print("-" * 95)
    print(f"{'File Name':<20} | {'Triangles':<9} | {'Dimensions (Dx, Dy, Dz mm)':<26} | {'Old Center (X, Y)':<18} | {'New Center (X, Y)'}")
    print("-" * 95)

    for f in stl_files:
        filename = os.path.basename(f)
        try:
            res = recenter_stl(f, dry_run=args.dry_run)
            dim_str = f"{res['dims'][0]:.1f} x {res['dims'][1]:.1f} x {res['dims'][2]:.1f}"
            orig_c = f"({res['orig_center'][0]:6.2f}, {res['orig_center'][1]:6.2f})"
            new_c = f"({res['new_center'][0]:6.2f}, {res['new_center'][1]:6.2f})"
            print(f"{filename:<20} | {res['n_tris']:<9} | {dim_str:<26} | {orig_c:<18} | {new_c}")
        except Exception as e:
            print(f"{filename:<20} | ERROR: {e}", file=sys.stderr)

    print("-" * 95)
    status_msg = (
        "Inspection complete (no files modified)."
        if args.dry_run
        else "All STL meshes successfully recentered to base (0.0, 0.0, 0.0)."
    )
    print(f"[SUCCESS] {status_msg}")
    print("=" * 95)


if __name__ == '__main__':
    main()
