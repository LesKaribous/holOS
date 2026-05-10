"""
LiveArucoPanel -- a real-time table widget showing all detected ArUco tags
in the current frame.

Columns :
    ID         : tag id
    Type       : ANCHOR / ROBOT / OTHER
    Pixel      : (cx, cy) center in image space
    World mm   : (x, y) in table coordinates (mm), via WorldMap
    Residual   : for anchors -- |poly_predicted - true| in mm (0 after correction)
    Status     : OK / OCCLUDED

Updated each frame from MainWindow._process_and_display.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTableWidget,
    QTableWidgetItem, QHeaderView, QSizePolicy,
)


class LiveArucoPanel(QWidget):
    """Live tag display, refreshed each frame."""

    HEADERS = ["ID", "Type", "Pixel", "World mm", "Residual", "Z mm"]

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self._lbl_summary = QLabel("Pas de detection")
        self._lbl_summary.setStyleSheet("color: #aaa; font-size: 11px;")
        layout.addWidget(self._lbl_summary)

        self._table = QTableWidget()
        self._table.setColumnCount(len(self.HEADERS))
        self._table.setHorizontalHeaderLabels(self.HEADERS)
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self._table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._table.setShowGrid(False)
        self._table.setStyleSheet(
            "QTableWidget { font-family: Consolas, monospace; font-size: 10px; }"
        )
        h = self._table.horizontalHeader()
        h.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        h.setStretchLastSection(True)
        layout.addWidget(self._table, stretch=1)

        # Camera info bar
        self._lbl_cam = QLabel("Camera : --")
        self._lbl_cam.setStyleSheet("color: #888; font-size: 10px;")
        layout.addWidget(self._lbl_cam)

    def update_live(
        self,
        result,            # DetectionResult
        anchors,           # list[TagAnchor]
        robots,            # list[RobotConfig]
        rectifier,         # TableRectifier (with poly + correction)
        tracker,           # RobotTracker (for cam_pos)
    ):
        """Refresh the table from a frame's detections."""
        if result is None:
            self._table.setRowCount(0)
            self._lbl_summary.setText("Pas de detection")
            return

        anchor_ids = {a.tag_id: a for a in anchors}
        robot_ids = {r.tag_id: r for r in robots}

        ids = list(result.ids)
        n_anchors_seen = sum(1 for i in ids if i in anchor_ids)
        n_robots_seen = sum(1 for i in ids if i in robot_ids)
        n_others = len(ids) - n_anchors_seen - n_robots_seen
        self._lbl_summary.setText(
            f"Tags : {len(ids)}    "
            f"anchors {n_anchors_seen}/{len(anchor_ids)}    "
            f"robots {n_robots_seen}/{len(robot_ids)}    "
            f"autres {n_others}"
        )

        # One row per detected ArUco
        rows = []
        for tid, corners_arr in zip(result.ids, result.corners):
            pts = corners_arr.reshape(4, 2)
            cx, cy = pts.mean(axis=0)
            if tid in anchor_ids:
                t = "ANCHOR"
                anchor = anchor_ids[tid]
                # Residual : compare poly_pred(pixel) with anchor.true_mm
                residual_mm = None
                if rectifier.has_poly:
                    poly = rectifier.poly
                    mm_pred = poly.pixel_to_mm_raw(np.array([[cx, cy]]))[0]
                    residual_mm = float(np.linalg.norm(
                        mm_pred - np.array([anchor.x_mm, anchor.y_mm])
                    ))
                rows.append((tid, t, cx, cy, anchor.x_mm, anchor.y_mm,
                             residual_mm, 0.0))
            elif tid in robot_ids:
                t = "ROBOT"
                cfg = robot_ids[tid]
                # Use tracker state if available (it has parallax-corrected mm)
                state = tracker.last_states.get(tid)
                if state is not None:
                    x_mm, y_mm = float(state.pos_mm[0]), float(state.pos_mm[1])
                else:
                    if rectifier.has_homography:
                        mm = rectifier.raw_pixel_to_table_mm(int(cx), int(cy))
                        x_mm, y_mm = (mm if mm else (0.0, 0.0))
                    else:
                        x_mm, y_mm = (0.0, 0.0)
                rows.append((tid, t, cx, cy, x_mm, y_mm, None, cfg.z_mm))
            else:
                t = "OTHER"
                if rectifier.has_homography:
                    mm = rectifier.raw_pixel_to_table_mm(int(cx), int(cy))
                    x_mm, y_mm = (mm if mm else (0.0, 0.0))
                else:
                    x_mm, y_mm = (0.0, 0.0)
                rows.append((tid, t, cx, cy, x_mm, y_mm, None, 0.0))

        # Sort: anchors first, then robots, then others, then by ID
        order = {"ANCHOR": 0, "ROBOT": 1, "OTHER": 2}
        rows.sort(key=lambda r: (order[r[1]], r[0]))

        self._table.setRowCount(len(rows))
        for row, (tid, t, cx, cy, x_mm, y_mm, res, z) in enumerate(rows):
            cells = [
                str(tid),
                t,
                f"{cx:7.1f}, {cy:7.1f}",
                f"{x_mm:7.0f}, {y_mm:7.0f}",
                "-" if res is None else f"{res:5.1f} mm",
                "-" if z == 0.0 else f"{z:.0f} mm",
            ]
            for col, txt in enumerate(cells):
                item = QTableWidgetItem(txt)
                if t == "ANCHOR":
                    item.setForeground(QColor("#fbbf24"))
                elif t == "ROBOT":
                    cfg = robot_ids[tid]
                    b, g, r = cfg.color_bgr
                    item.setForeground(QColor(r, g, b))
                else:
                    item.setForeground(QColor("#888"))
                self._table.setItem(row, col, item)

        # Camera info
        if tracker.cam_pos is not None:
            x, y, z = tracker.cam_pos
            src = tracker.cam_source
            self._lbl_cam.setText(
                f"Camera ({src}) : ({x:.0f}, {y:.0f}, {z:.0f}) mm"
            )
        else:
            self._lbl_cam.setText("Camera : non determinee")

    def clear(self):
        self._table.setRowCount(0)
        self._lbl_summary.setText("Pas de detection")
        self._lbl_cam.setText("Camera : --")
