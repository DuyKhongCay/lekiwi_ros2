#!/usr/bin/env python3
# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unified CLI tool to generate ChArUco calibration boards (single or multi-board A4/A3)."""

import argparse
import os
import cv2
import cv2.aruco as aruco
import numpy as np
from PIL import Image

DICT_MAP = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_6X6_250": aruco.DICT_6X6_250,
}


def create_charuco_board(squares_x, squares_y, square_len_m, marker_len_m, dict_type):
    aruco_dict = aruco.getPredefinedDictionary(dict_type)
    if hasattr(aruco, "CharucoBoard_create"):
        board = aruco.CharucoBoard_create(
            squares_x, squares_y, square_len_m, marker_len_m, aruco_dict
        )
    else:
        board = aruco.CharucoBoard(
            (squares_x, squares_y), square_len_m, marker_len_m, aruco_dict
        )
    return aruco_dict, board


def draw_charuco_image(board, width_px, height_px):
    if hasattr(board, "draw"):
        return board.draw((width_px, height_px), 0, 1)
    return board.generateImage((width_px, height_px), marginSize=0, borderBits=1)


def generate_single_board(args):
    dpi = args.dpi
    mm2px = dpi / 25.4
    dict_type = DICT_MAP.get(args.dict, aruco.DICT_4X4_50)

    board_w_px = int(args.squares_x * args.square_mm * mm2px)
    board_h_px = int(args.squares_y * args.square_mm * mm2px)

    _, board = create_charuco_board(
        args.squares_x, args.squares_y, args.square_mm / 1000.0, args.marker_mm / 1000.0, dict_type
    )
    board_img = draw_charuco_image(board, board_w_px, board_h_px)

    # Page sizing
    if args.paper.upper() == "A4":
        page_w_mm, page_h_mm = 210.0, 297.0
    elif args.paper.upper() == "A3":
        page_w_mm, page_h_mm = 297.0, 420.0
    else:
        page_w_mm, page_h_mm = 210.0, 297.0

    page_w_px = int(page_w_mm * mm2px)
    page_h_px = int(page_h_mm * mm2px)
    page = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)

    off_x = (page_w_px - board_w_px) // 2
    off_y = (page_h_px - board_h_px) // 2
    page[off_y : off_y + board_h_px, off_x : off_x + board_w_px] = board_img

    # Crop marks
    m = int(args.margin_mm * mm2px / 3)
    for x, y in [
        (off_x, off_y),
        (off_x + board_w_px, off_y),
        (off_x, off_y + board_h_px),
        (off_x + board_w_px, off_y + board_h_px),
    ]:
        cv2.line(page, (x - m, y), (x + m, y), 0, 2)
        cv2.line(page, (x, y - m), (x, y + m), 0, 2)

    # Title
    font = cv2.FONT_HERSHEY_SIMPLEX
    title = f"ChArUco {args.squares_x}x{args.squares_y} [sq={args.square_mm}mm, mk={args.marker_mm}mm] - {args.dict}"
    scale = 0.6 * (dpi / 300.0)
    thickness = max(1, int(1.5 * (dpi / 300.0)))
    t_size = cv2.getTextSize(title, font, scale, thickness)[0]
    t_x = (page_w_px - t_size[0]) // 2
    t_y = off_y - int(8 * mm2px)
    if t_y > 20:
        cv2.putText(page, title, (t_x, t_y), font, scale, (0,), thickness, cv2.LINE_AA)

    out_prefix = os.path.expanduser(args.output)
    out_png = f"{out_prefix}.png" if not out_prefix.endswith(".png") else out_prefix
    out_pdf = f"{os.path.splitext(out_png)[0]}.pdf"

    cv2.imwrite(out_png, page)
    Image.fromarray(page).save(out_pdf, "PDF", resolution=float(dpi))
    print(f" Saved single board successfully:")
    print(f"   - PNG: {out_png}")
    print(f"   - PDF: {out_pdf}")


def generate_multi_board(args):
    dpi = args.dpi
    mm2px = dpi / 25.4
    dict_type = DICT_MAP.get(args.dict, aruco.DICT_4X4_50)

    page_w_mm, page_h_mm = 210.0, 297.0  # A4
    page_w_px = int(page_w_mm * mm2px)
    page_h_px = int(page_h_mm * mm2px)
    page = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)

    configs = [
        {"sx": 4, "sy": 5, "sq_mm": 4.0, "mk_mm": 3.0, "title": "Board 1: 4x5 [sq=4mm, mk=3mm] (16x20mm)"},
        {"sx": 4, "sy": 5, "sq_mm": 5.0, "mk_mm": 3.75, "title": "Board 2: 4x5 [sq=5mm, mk=3.75mm] (20x25mm)"},
        {"sx": 4, "sy": 5, "sq_mm": 6.0, "mk_mm": 4.5, "title": "Board 3: 4x5 [sq=6mm, mk=4.5mm] (24x30mm)"},
        {"sx": 3, "sy": 4, "sq_mm": 4.0, "mk_mm": 3.0, "title": "Board 4: 3x4 [sq=4mm, mk=3mm] (12x16mm)"},
        {"sx": 3, "sy": 4, "sq_mm": 5.0, "mk_mm": 3.75, "title": "Board 5: 3x4 [sq=5mm, mk=3.75mm] (15x20mm)"},
        {"sx": 3, "sy": 4, "sq_mm": 6.0, "mk_mm": 4.5, "title": "Board 6: 3x4 [sq=6mm, mk=4.5mm] (18x24mm)"},
        {"sx": 3, "sy": 4, "sq_mm": 8.0, "mk_mm": 6.0, "title": "Board 7: 3x4 [sq=8mm, mk=6.0mm] (24x32mm)"},
    ]

    cols = 2
    rows = 4
    cell_w_px = page_w_px // cols
    cell_h_px = page_h_px // rows

    for idx, cfg in enumerate(configs):
        r = idx // cols
        c = idx % cols

        bw_px = int(cfg["sx"] * cfg["sq_mm"] * mm2px)
        bh_px = int(cfg["sy"] * cfg["sq_mm"] * mm2px)

        _, board = create_charuco_board(
            cfg["sx"], cfg["sy"], cfg["sq_mm"] / 1000.0, cfg["mk_mm"] / 1000.0, dict_type
        )
        b_img = draw_charuco_image(board, bw_px, bh_px)

        cell_cx = c * cell_w_px + cell_w_px // 2
        cell_cy = r * cell_h_px + cell_h_px // 2 + int(6 * mm2px)

        off_x = cell_cx - bw_px // 2
        off_y = cell_cy - bh_px // 2

        page[off_y : off_y + bh_px, off_x : off_x + bw_px] = b_img

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.7 * (dpi / 300.0)
        thickness = max(1, int(2 * (dpi / 300.0)))
        t_size = cv2.getTextSize(cfg["title"], font, scale, thickness)[0]
        t_x = cell_cx - t_size[0] // 2
        t_y = off_y - int(8 * mm2px)
        cv2.putText(page, cfg["title"], (t_x, t_y), font, scale, (0,), thickness, cv2.LINE_AA)

        m = int(4 * mm2px)
        for x, y in [
            (off_x, off_y),
            (off_x + bw_px, off_y),
            (off_x, off_y + bh_px),
            (off_x + bw_px, off_y + bh_px),
        ]:
            cv2.line(page, (x - m, y), (x + m, y), 0, thickness)
            cv2.line(page, (x, y - m), (x, y + m), 0, thickness)

    out_prefix = os.path.expanduser(args.output)
    out_png = f"{out_prefix}.png" if not out_prefix.endswith(".png") else out_prefix
    out_pdf = f"{os.path.splitext(out_png)[0]}.pdf"

    cv2.imwrite(out_png, page)
    Image.fromarray(page).save(out_pdf, "PDF", resolution=float(dpi))
    print(f" Saved multi-board sheet successfully:")
    print(f"   - PNG: {out_png}")
    print(f"   - PDF: {out_pdf}")


def main():
    parser = argparse.ArgumentParser(description="ChArUco Board Generator for Hand-Eye Calibration")
    parser.add_argument(
        "--mode", choices=["single", "multi"], default="single", help="Generation mode: single board or multi A4 sheet"
    )
    parser.add_argument("--squares_x", type=int, default=3, help="Number of squares in X (single mode)")
    parser.add_argument("--squares_y", type=int, default=4, help="Number of squares in Y (single mode)")
    parser.add_argument("--square_mm", type=float, default=6.0, help="Square side in mm (single mode)")
    parser.add_argument("--marker_mm", type=float, default=4.5, help="Marker side in mm (single mode)")
    parser.add_argument("--dict", type=str, default="DICT_4X4_50", choices=list(DICT_MAP.keys()), help="ArUco dictionary")
    parser.add_argument("--dpi", type=int, default=600, help="DPI resolution for print output")
    parser.add_argument("--paper", type=str, default="A4", choices=["A4", "A3"], help="Paper size")
    parser.add_argument("--margin_mm", type=float, default=15.0, help="Margin for single board")
    parser.add_argument("--output", type=str, default="/tmp/charuco_target", help="Output file path prefix without ext")

    args = parser.parse_args()

    if args.mode == "single":
        generate_single_board(args)
    else:
        generate_multi_board(args)


if __name__ == "__main__":
    main()
