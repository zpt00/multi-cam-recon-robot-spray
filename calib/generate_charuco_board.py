# -*- coding: utf-8 -*-
"""
生成 ChArUco 标定板图像

依赖:
    pip install opencv-contrib-python

参数配置在 config.py 中
"""

import cv2
import numpy as np
import os

import config as cfg


def mm_to_px(mm, dpi):
    return int(round(mm / 25.4 * dpi))


def main():
    aruco = cv2.aruco

    if not hasattr(aruco, cfg.ARUCO_DICT_NAME):
        raise ValueError(f"OpenCV 中不存在字典: {cfg.ARUCO_DICT_NAME}")

    dictionary_id = getattr(aruco, cfg.ARUCO_DICT_NAME)
    dictionary = aruco.getPredefinedDictionary(dictionary_id)

    if hasattr(aruco, "CharucoBoard") and callable(aruco.CharucoBoard):
        board = aruco.CharucoBoard(
            (cfg.SQUARES_X, cfg.SQUARES_Y),
            cfg.SQUARE_LENGTH_M * 1000,
            cfg.MARKER_LENGTH_M * 1000,
            dictionary
        )
    else:
        board = aruco.CharucoBoard_create(
            cfg.SQUARES_X, cfg.SQUARES_Y,
            cfg.SQUARE_LENGTH_M * 1000,
            cfg.MARKER_LENGTH_M * 1000,
            dictionary
        )

    paper_w_px = mm_to_px(cfg.BOARD_PAPER_W_MM, cfg.BOARD_DPI)
    paper_h_px = mm_to_px(cfg.BOARD_PAPER_H_MM, cfg.BOARD_DPI)
    margin_px = mm_to_px(cfg.BOARD_MARGIN_MM, cfg.BOARD_DPI)

    board_w_mm = cfg.SQUARES_X * cfg.SQUARE_LENGTH_M * 1000
    board_h_mm = cfg.SQUARES_Y * cfg.SQUARE_LENGTH_M * 1000

    max_draw_w_px = paper_w_px - 2 * margin_px
    max_draw_h_px = paper_h_px - 2 * margin_px

    scale = min(max_draw_w_px / board_w_mm, max_draw_h_px / board_h_mm)

    draw_w_px = int(board_w_mm * scale)
    draw_h_px = int(board_h_mm * scale)

    if hasattr(board, "generateImage"):
        board_img = board.generateImage((draw_w_px, draw_h_px), marginSize=0, borderBits=1)
    else:
        board_img = board.draw((draw_w_px, draw_h_px), marginSize=0, borderBits=1)

    canvas = np.ones((paper_h_px, paper_w_px), dtype=np.uint8) * 255
    start_x = (paper_w_px - draw_w_px) // 2
    start_y = (paper_h_px - draw_h_px) // 2
    canvas[start_y:start_y + draw_h_px, start_x:start_x + draw_w_px] = board_img

    cv2.imwrite(cfg.BOARD_OUTPUT_PATH, canvas)

    txt_path = os.path.splitext(cfg.BOARD_OUTPUT_PATH)[0] + "_info.txt"
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("ChArUco Board Parameters\n")
        f.write(f"squares_x = {cfg.SQUARES_X}\n")
        f.write(f"squares_y = {cfg.SQUARES_Y}\n")
        f.write(f"square_length_mm = {cfg.SQUARE_LENGTH_M * 1000}\n")
        f.write(f"marker_length_mm = {cfg.MARKER_LENGTH_M * 1000}\n")
        f.write(f"dictionary_name = {cfg.ARUCO_DICT_NAME}\n")
        f.write(f"dpi = {cfg.BOARD_DPI}\n")
        f.write(f"paper_width_mm = {cfg.BOARD_PAPER_W_MM}\n")
        f.write(f"paper_height_mm = {cfg.BOARD_PAPER_H_MM}\n")
        f.write(f"margin_mm = {cfg.BOARD_MARGIN_MM}\n")

    print("ChArUco 标定板已生成：")
    print(f"图像文件: {cfg.BOARD_OUTPUT_PATH}")
    print(f"参数说明: {txt_path}")
    print("建议打印时选择实际尺寸 / 100% 缩放，不勾选'适应页面'")


if __name__ == "__main__":
    main()
