"""
TableCalibDialog — calibration polynomiale de la table à partir d'une image
+ d'un mask rouge.

Workflow :
    1. L'utilisateur charge une image de la table (ou utilise le frame courant).
    2. L'utilisateur charge un mask rouge plein (même résolution).
    3. Le pipeline calibrate_from_mask() est lancé → modèle polynomial deg 3.
    4. Le résultat est affiché (RMS, coins détectés, BEV preview).
    5. Bouton « Sauvegarder » → JSON dans calibrations/.
    6. Signal `calibration_done(PolyCalibration)` émis pour intégration dans
       la fenêtre principale.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel, QPushButton,
    QFileDialog, QMessageBox, QGroupBox, QSpinBox, QSplitter,
)

from core.poly_calibration import (
    PolyCalibration, calibrate_from_mask, make_bev,
)
from ui.video_widget import VideoWidget


class TableCalibDialog(QDialog):
    """Dialog pour calibration polynomiale table."""

    calibration_done = Signal(object)   # PolyCalibration

    def __init__(self, parent=None, current_frame: Optional[np.ndarray] = None):
        super().__init__(parent)
        self.setWindowTitle("Calibration table polynomiale (mask rouge)")
        self.resize(1100, 720)
        self.setModal(False)

        self._image: Optional[np.ndarray] = None
        self._mask: Optional[np.ndarray] = None
        self._calibration: Optional[PolyCalibration] = None
        self._debug: dict = {}
        self._current_frame: Optional[np.ndarray] = current_frame

        self._build_ui()
        if current_frame is not None:
            self._image = current_frame.copy()
            self._update_image_preview()

    def _build_ui(self):
        root = QVBoxLayout(self)

        # ── Top : 3 boutons d'entrée ─────────────────────────────────────────
        top = QHBoxLayout()

        btn_load_img = QPushButton("📷 Charger image")
        btn_load_img.clicked.connect(self._load_image)
        top.addWidget(btn_load_img)

        btn_use_frame = QPushButton("🎬 Utiliser frame courante")
        btn_use_frame.clicked.connect(self._use_current_frame)
        top.addWidget(btn_use_frame)

        btn_load_mask = QPushButton("🟥 Charger mask rouge")
        btn_load_mask.clicked.connect(self._load_mask)
        top.addWidget(btn_load_mask)

        top.addStretch()

        self.sp_deg = QSpinBox()
        self.sp_deg.setRange(2, 5); self.sp_deg.setValue(3)
        self.sp_deg.setPrefix("deg ")
        top.addWidget(QLabel("Polynôme:"))
        top.addWidget(self.sp_deg)

        btn_run = QPushButton("▶ Calibrer")
        btn_run.setStyleSheet("font-weight: bold;")
        btn_run.clicked.connect(self._run_calibration)
        top.addWidget(btn_run)

        root.addLayout(top)

        # ── Center : preview images (3 colonnes : image, mask, preview résultat) ──
        splitter = QSplitter(Qt.Orientation.Horizontal)

        self.preview_image = VideoWidget("Image")
        self.preview_mask  = VideoWidget("Mask rouge")
        self.preview_result = VideoWidget("Résultat (coins + grille)")

        splitter.addWidget(self.preview_image)
        splitter.addWidget(self.preview_mask)
        splitter.addWidget(self.preview_result)
        splitter.setSizes([350, 350, 350])

        root.addWidget(splitter, stretch=1)

        # ── Status / résultats ───────────────────────────────────────────────
        self.lbl_status = QLabel("Charger une image et un mask pour commencer.")
        self.lbl_status.setStyleSheet("color: #888; font-size: 11px;")
        self.lbl_status.setWordWrap(True)
        self.lbl_status.setMinimumHeight(40)
        self.lbl_status.setFont(QFont("Monospace", 9))
        root.addWidget(self.lbl_status)

        # ── Bottom : sauvegarder + appliquer ─────────────────────────────────
        bot = QHBoxLayout()
        bot.addStretch()
        self.btn_save = QPushButton("💾 Sauvegarder JSON…")
        self.btn_save.setEnabled(False)
        self.btn_save.clicked.connect(self._save)
        bot.addWidget(self.btn_save)

        self.btn_apply = QPushButton("✓ Appliquer dans TwinVision")
        self.btn_apply.setEnabled(False)
        self.btn_apply.setStyleSheet("font-weight: bold;")
        self.btn_apply.clicked.connect(self._apply)
        bot.addWidget(self.btn_apply)

        btn_close = QPushButton("Fermer")
        btn_close.clicked.connect(self.reject)
        bot.addWidget(btn_close)

        root.addLayout(bot)

    # ── Slots image / mask ──────────────────────────────────────────────────

    def feed_frame(self, frame: np.ndarray):
        """Reçoit le frame courant depuis MainWindow (pour bouton 'utiliser frame')."""
        self._current_frame = frame

    def _load_image(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Charger une image de la table", "",
            "Images (*.png *.jpg *.jpeg *.bmp);;Tous (*.*)",
        )
        if not path:
            return
        img = cv2.imread(path)
        if img is None:
            QMessageBox.critical(self, "Erreur", f"Impossible de charger : {path}")
            return
        self._image = img
        self._update_image_preview()
        self._update_status_idle()

    def _use_current_frame(self):
        if self._current_frame is None:
            QMessageBox.warning(
                self, "Pas de frame",
                "Aucun frame courant. Ouvre d'abord une vidéo dans TwinVision.",
            )
            return
        self._image = self._current_frame.copy()
        self._update_image_preview()
        self._update_status_idle()

    def _load_mask(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Charger le mask rouge", "",
            "Images (*.png *.jpg *.jpeg);;Tous (*.*)",
        )
        if not path:
            return
        m = cv2.imread(path)
        if m is None:
            QMessageBox.critical(self, "Erreur", f"Impossible de charger : {path}")
            return
        self._mask = m
        self._update_mask_preview()
        self._update_status_idle()

    def _update_image_preview(self):
        if self._image is not None:
            self.preview_image.display_frame(self._image)

    def _update_mask_preview(self):
        if self._mask is not None:
            self.preview_mask.display_frame(self._mask)

    def _update_status_idle(self):
        if self._image is None:
            self.lbl_status.setText("→ Charger une image.")
        elif self._mask is None:
            self.lbl_status.setText(
                f"Image chargée : {self._image.shape[1]}×{self._image.shape[0]}. "
                "→ Charger un mask rouge.",
            )
        else:
            self.lbl_status.setText(
                f"Image et mask prêts. → Cliquer « Calibrer ».",
            )

    # ── Lancement de la calibration ─────────────────────────────────────────

    def _run_calibration(self):
        if self._image is None or self._mask is None:
            QMessageBox.warning(
                self, "Manquant",
                "Charge d'abord une image ET un mask rouge.",
            )
            return

        try:
            cal, debug = calibrate_from_mask(
                self._image, self._mask, deg=self.sp_deg.value(),
                note="Calibration TwinVision (UI)",
            )
        except Exception as e:
            QMessageBox.critical(
                self, "Erreur calibration",
                f"Échec :\n{type(e).__name__}: {e}",
            )
            return

        self._calibration = cal
        self._debug = debug

        # Preview résultat avec coins + grille rasterisée light
        result_img = self._render_result(self._image, cal, debug)
        self.preview_result.display_frame(result_img)

        msg = (
            f"✓ Calibration OK ({cal.image_size[0]}×{cal.image_size[1]}, "
            f"poly deg {cal.deg})\n"
            f"   Erreurs : poly RMS={cal.rms_mm:.2f} mm, max={cal.max_mm:.2f} mm, "
            f"med={cal.median_mm:.2f} mm\n"
            f"   Comparaison homographie : RMS={cal.homography_rms_mm:.2f} mm, "
            f"max={cal.homography_max_mm:.2f} mm"
        )
        self.lbl_status.setText(msg)
        self.lbl_status.setStyleSheet("color: #4ade80; font-size: 11px;")

        self.btn_save.setEnabled(True)
        self.btn_apply.setEnabled(True)

    def _render_result(
        self, image: np.ndarray, cal: PolyCalibration, debug: dict,
    ) -> np.ndarray:
        """Rendu d'aperçu : coins + grille rasterisée (lignes mm)."""
        out = debug.get("corners_vis", image).copy()

        # Grille via le polynôme : on rasterise X=cst et Y=cst
        try:
            X, Y = cal.world_grid(out.shape)
            mask_inside = debug.get("mask")
            if mask_inside is not None:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
                inside = cv2.dilate(mask_inside, kernel) > 0
            else:
                inside = np.ones(out.shape[:2], dtype=bool)

            # Lignes tous les 200 mm
            for x_t in range(0, 3001, 200):
                near = (np.abs(X - x_t) < 8.0) & inside
                out[near] = (0, 200, 0)
            for y_t in range(0, 2001, 200):
                near = (np.abs(Y - y_t) < 8.0) & inside
                out[near] = (0, 200, 0)
            # Périmètre rouge
            for x_t in (0, 3000):
                near = (np.abs(X - x_t) < 12.0) & inside
                out[near] = (0, 0, 255)
            for y_t in (0, 2000):
                near = (np.abs(Y - y_t) < 12.0) & inside
                out[near] = (0, 0, 255)
        except Exception:
            pass

        cv2.putText(
            out, f"Poly deg {cal.deg}  RMS={cal.rms_mm:.1f} mm",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
        )
        return out

    # ── Save / apply ────────────────────────────────────────────────────────

    def _save(self):
        if self._calibration is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Sauvegarder la calibration",
            "calibrations/poly_table.json",
            "JSON (*.json)",
        )
        if not path:
            return
        try:
            self._calibration.save(path)
            self.lbl_status.setText(self.lbl_status.text() + f"\n→ Sauvegardé : {path}")
        except Exception as e:
            QMessageBox.critical(
                self, "Erreur sauvegarde", f"{type(e).__name__}: {e}",
            )

    def _apply(self):
        if self._calibration is None:
            return
        self.calibration_done.emit(self._calibration)
        self.lbl_status.setText(self.lbl_status.text() + "\n→ Appliqué dans TwinVision.")
