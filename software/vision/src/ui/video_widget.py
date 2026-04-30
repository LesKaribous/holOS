"""
VideoWidget — QLabel amélioré pour afficher des frames OpenCV.
Supporte le clic pour récupérer les coordonnées image.
"""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
from PySide6.QtCore import Qt, Signal, QPoint
from PySide6.QtGui import QImage, QPixmap, QPainter, QColor, QFont
from PySide6.QtWidgets import QLabel, QSizePolicy


def bgr_to_qimage(frame: np.ndarray) -> QImage:
    """Convertit une image OpenCV BGR en QImage RGB."""
    h, w, ch = frame.shape
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return QImage(rgb.data, w, h, w * ch, QImage.Format.Format_RGB888).copy()


class VideoWidget(QLabel):
    """
    Widget d'affichage vidéo.
    - Redimensionne le frame en conservant le ratio.
    - Émet clicked(x, y) avec les coordonnées dans l'espace IMAGE (pas widget).
    """

    clicked = Signal(int, int)  # coordonnées image
    hovered = Signal(int, int)  # coordonnées image

    def __init__(self, title: str = "", parent=None):
        super().__init__(parent)
        self._title = title
        self._frame: Optional[np.ndarray] = None
        self._display_rect = None  # (x_off, y_off, w, h) dans le widget

        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(320, 180)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet("background-color: #1a1a2e; border: 1px solid #333;")
        self.setMouseTracking(True)

        self._show_placeholder()

    def _show_placeholder(self):
        self.setText(f"[ {self._title} ]" if self._title else "[ No source ]")
        self.setStyleSheet(
            "background-color: #1a1a2e; color: #444; border: 1px solid #333; "
            "font-size: 14px;"
        )

    def display_frame(self, frame: Optional[np.ndarray]):
        if frame is None:
            self._frame = None
            self._display_rect = None
            self._show_placeholder()
            return

        self._frame = frame
        self.setStyleSheet("background-color: #000; border: 1px solid #333;")

        img_h, img_w = frame.shape[:2]
        wgt_w, wgt_h = self.width(), self.height()

        # Calcul du ratio pour fit-in
        ratio = min(wgt_w / img_w, wgt_h / img_h)
        disp_w = int(img_w * ratio)
        disp_h = int(img_h * ratio)
        x_off = (wgt_w - disp_w) // 2
        y_off = (wgt_h - disp_h) // 2
        self._display_rect = (x_off, y_off, disp_w, disp_h)

        resized = cv2.resize(frame, (disp_w, disp_h), interpolation=cv2.INTER_LINEAR)
        qimg = bgr_to_qimage(resized)
        pixmap = QPixmap.fromImage(qimg)

        # On dessine sur un pixmap de la taille du widget pour centrer
        canvas = QPixmap(wgt_w, wgt_h)
        canvas.fill(QColor(0, 0, 0))
        painter = QPainter(canvas)
        painter.drawPixmap(x_off, y_off, pixmap)
        if self._title:
            painter.setPen(QColor(180, 180, 180))
            painter.setFont(QFont("Arial", 9))
            painter.drawText(6, 16, self._title)
        painter.end()

        self.setPixmap(canvas)

    def _widget_to_image(self, wx: int, wy: int) -> Optional[tuple[int, int]]:
        if self._display_rect is None or self._frame is None:
            return None
        x_off, y_off, disp_w, disp_h = self._display_rect
        img_h, img_w = self._frame.shape[:2]
        if disp_w == 0 or disp_h == 0:
            return None
        ix = int((wx - x_off) * img_w / disp_w)
        iy = int((wy - y_off) * img_h / disp_h)
        if 0 <= ix < img_w and 0 <= iy < img_h:
            return ix, iy
        return None

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            pos = self._widget_to_image(event.position().x(), event.position().y())
            if pos:
                self.clicked.emit(*pos)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        pos = self._widget_to_image(event.position().x(), event.position().y())
        if pos:
            self.hovered.emit(*pos)
        super().mouseMoveEvent(event)

    def resizeEvent(self, event):
        # Re-afficher avec la nouvelle taille
        if self._frame is not None:
            self.display_frame(self._frame)
        super().resizeEvent(event)
