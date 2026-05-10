"""ChArUco calibration wizard : multi-view capture + intrinsics fit."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFileDialog,
    QMessageBox, QSpinBox, QDoubleSpinBox, QComboBox, QFormLayout,
    QListWidget, QListWidgetItem, QSplitter, QCheckBox, QGroupBox,
    QWidget,
)

from core.charuco_calib import (
    make_charuco_board, render_board_image, detect_charuco,
    calibrate_intrinsics, CalibrationView, CameraIntrinsics,
    DEFAULT_SQUARES_X, DEFAULT_SQUARES_Y, DEFAULT_SQUARE_MM,
    DEFAULT_MARKER_MM, DEFAULT_DICT_NAME, _ARUCO_DICTS,
)
from ui.video_widget import VideoWidget


class CharucoWizardDialog(QDialog):
    """Modal-less wizard : capture views and run intrinsic calibration."""

    intrinsics_done = Signal(object)   # CameraIntrinsics

    def __init__(self, parent=None, current_frame=None):
        super().__init__(parent)
        self.setWindowTitle("Calibration camera (ChArUco)")
        self.resize(1200, 700)
        self.setModal(False)
        self._current_frame = current_frame
        self._views: list = []
        self._board = None
        self._aruco_dict = None
        self._intrinsics = None
        self._build_board()
        self._build_ui()
        # Populate status label with the current (default) params.
        self._refresh_board()
        if current_frame is not None:
            self.feed_frame(current_frame)

    def _build_board(self):
        self._board, self._aruco_dict = make_charuco_board()

    def _build_ui(self):
        root = QVBoxLayout(self)
        gp_top = QGroupBox("Parametres du board (avant impression)")
        gf = QFormLayout(gp_top)
        self.sp_sx = QSpinBox(); self.sp_sx.setRange(3, 20); self.sp_sx.setValue(DEFAULT_SQUARES_X)
        self.sp_sy = QSpinBox(); self.sp_sy.setRange(3, 20); self.sp_sy.setValue(DEFAULT_SQUARES_Y)
        self.sp_sq = QDoubleSpinBox(); self.sp_sq.setRange(5, 200); self.sp_sq.setValue(DEFAULT_SQUARE_MM); self.sp_sq.setSuffix(" mm")
        self.sp_mk = QDoubleSpinBox(); self.sp_mk.setRange(3, 200); self.sp_mk.setValue(DEFAULT_MARKER_MM); self.sp_mk.setSuffix(" mm")
        self.cb_dict = QComboBox()
        for k in _ARUCO_DICTS.keys():
            self.cb_dict.addItem(k)
        self.cb_dict.setCurrentText(DEFAULT_DICT_NAME)
        self.chk_legacy = QCheckBox("Layout legacy (calib.io / OpenCV < 4.6)")
        self.chk_legacy.setChecked(True)
        self.chk_legacy.setToolTip(
            "Calib.io et OpenCV < 4.6 placent les marqueurs dans une convention\n"
            "differente des versions recentes. Si ton board vient de calib.io,\n"
            "garde cette case cochee. Sinon decoche."
        )
        # Auto-update the internal board whenever any param changes (so a
        # printed board with non-default params is detected correctly).
        self.sp_sx.valueChanged.connect(self._refresh_board)
        self.sp_sy.valueChanged.connect(self._refresh_board)
        self.sp_sq.valueChanged.connect(self._refresh_board)
        self.sp_mk.valueChanged.connect(self._refresh_board)
        self.cb_dict.currentIndexChanged.connect(self._refresh_board)
        self.chk_legacy.toggled.connect(lambda _v: self._refresh_board())
        gf.addRow("Cases X :", self.sp_sx)
        gf.addRow("Cases Y :", self.sp_sy)
        gf.addRow("Taille case :", self.sp_sq)
        gf.addRow("Taille marker :", self.sp_mk)
        gf.addRow("Dictionnaire :", self.cb_dict)
        gf.addRow("Pattern :", self.chk_legacy)
        # Status label (OK / INVALIDE)
        self.lbl_board_status = QLabel("")
        self.lbl_board_status.setStyleSheet("font-size: 10px;")
        self.lbl_board_status.setWordWrap(True)
        gf.addRow(self.lbl_board_status)
        btn_gen = QPushButton("Generer image board (PNG)...")
        btn_gen.clicked.connect(self._on_generate_board)
        gf.addRow(btn_gen)
        root.addWidget(gp_top)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        self.preview_live = VideoWidget("Frame courant + detection")
        splitter.addWidget(self.preview_live)
        right_widget = QGroupBox("Vues capturees")
        right_box = QVBoxLayout(right_widget)
        self.lbl_count = QLabel("0 vues capturees")
        self.lbl_count.setStyleSheet("font-weight: bold;")
        right_box.addWidget(self.lbl_count)
        self.list_views = QListWidget()
        right_box.addWidget(self.list_views)
        btns_row = QHBoxLayout()
        btn_capture = QPushButton("+ Capturer cette vue")
        btn_capture.setStyleSheet("font-weight: bold; background-color: #4ade80;")
        btn_capture.clicked.connect(self._on_capture_view)
        btn_remove = QPushButton("Retirer selectionnee")
        btn_remove.clicked.connect(self._on_remove_selected)
        btn_clear = QPushButton("Tout effacer")
        btn_clear.clicked.connect(self._on_clear_all)
        btn_load_imgs = QPushButton("Charger photos disque...")
        btn_load_imgs.setToolTip(
            "Selectionne plusieurs images du board prises avec ta camera."
            " Utile quand tu as deja une serie de photos."
        )
        btn_load_imgs.clicked.connect(self._on_load_images)
        btns_row.addWidget(btn_capture)
        btns_row.addWidget(btn_load_imgs)
        btns_row.addWidget(btn_remove)
        btns_row.addWidget(btn_clear)
        right_box.addLayout(btns_row)
        splitter.addWidget(right_widget)
        splitter.setSizes([700, 500])
        root.addWidget(splitter, stretch=1)

        run_row = QHBoxLayout()
        self.chk_fisheye = QCheckBox("Modele fisheye (cam grand angle)")
        self.chk_fisheye.setChecked(False)
        run_row.addWidget(self.chk_fisheye)
        run_row.addStretch()
        self.btn_calibrate = QPushButton("Lancer calibration")
        self.btn_calibrate.setStyleSheet("font-weight: bold; background-color: #fbbf24; color: #000;")
        self.btn_calibrate.clicked.connect(self._on_run_calibration)
        run_row.addWidget(self.btn_calibrate)
        root.addLayout(run_row)

        self.lbl_result = QLabel("Capture des vues puis lancer la calibration.")
        self.lbl_result.setStyleSheet("color: #888; font-family: monospace;")
        self.lbl_result.setWordWrap(True)
        self.lbl_result.setMinimumHeight(60)
        root.addWidget(self.lbl_result)

        bot_row = QHBoxLayout()
        bot_row.addStretch()
        self.btn_save = QPushButton("Sauvegarder JSON...")
        self.btn_save.setEnabled(False)
        self.btn_save.clicked.connect(self._on_save)
        bot_row.addWidget(self.btn_save)
        self.btn_apply = QPushButton("Appliquer dans TwinVision")
        self.btn_apply.setEnabled(False)
        self.btn_apply.setStyleSheet("font-weight: bold;")
        self.btn_apply.clicked.connect(self._on_apply)
        bot_row.addWidget(self.btn_apply)
        btn_close = QPushButton("Fermer")
        btn_close.clicked.connect(self.reject)
        bot_row.addWidget(btn_close)
        root.addLayout(bot_row)

    def feed_frame(self, frame):
        self._current_frame = frame
        if frame is None:
            return
        try:
            corners, ids = detect_charuco(frame, self._board, self._aruco_dict)
        except Exception:
            corners, ids = None, None
        out = frame.copy()
        if corners is not None and ids is not None:
            cv2.aruco.drawDetectedCornersCharuco(out, corners, ids)
            cv2.putText(out, "DETECTED : " + str(len(ids)) + " corners",
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(out, "NO BOARD DETECTED",
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        self.preview_live.display_frame(out)

    def _on_generate_board(self):
        self._board, self._aruco_dict = make_charuco_board(
            squares_x=self.sp_sx.value(),
            squares_y=self.sp_sy.value(),
            square_mm=self.sp_sq.value(),
            marker_mm=self.sp_mk.value(),
            dict_name=self.cb_dict.currentText(),
            legacy_pattern=self.chk_legacy.isChecked(),
        )
        self._views = []
        self._refresh_views_ui()
        img = render_board_image(self._board, px_per_mm=6.0, margin_px=30)
        default_name = "charuco_" + str(self.sp_sx.value()) + "x" + str(self.sp_sy.value()) + "_" + str(int(self.sp_sq.value())) + "mm.png"
        path, _ = QFileDialog.getSaveFileName(self, "Sauvegarder l\u0027image du board",
                                              default_name, "PNG (*.png);;Tous (*.*)")
        if not path:
            return
        cv2.imwrite(path, img)
        msg = "Image sauvegardee : " + path + chr(10) + chr(10)
        msg += "Imprimer en taille reelle (verifier que la case mesure "
        msg += str(self.sp_sq.value()) + " mm apres impression)."
        QMessageBox.information(self, "Board genere", msg)

    def _on_capture_view(self):
        if self._current_frame is None:
            QMessageBox.warning(self, "Capture", "Aucun frame courant.")
            return
        corners, ids = detect_charuco(self._current_frame, self._board, self._aruco_dict)
        if corners is None or ids is None:
            QMessageBox.warning(self, "Capture",
                                "Board pas detecte. Bouge le board ou avance/recule.")
            return
        h, w = self._current_frame.shape[:2]
        view = CalibrationView(corners=corners, ids=ids, image_size=(w, h))
        self._views.append(view)
        self._refresh_views_ui()

    def _on_remove_selected(self):
        row = self.list_views.currentRow()
        if 0 <= row < len(self._views):
            del self._views[row]
            self._refresh_views_ui()

    def _on_clear_all(self):
        if not self._views:
            return
        if QMessageBox.question(self, "Tout effacer",
                                 "Effacer toutes les vues capturees ?") == QMessageBox.Yes:
            self._views = []
            self._refresh_views_ui()

    def _refresh_views_ui(self):
        self.list_views.clear()
        for i, v in enumerate(self._views):
            self.list_views.addItem(QListWidgetItem(
                "Vue " + str(i + 1) + " -- " + str(v.n_corners) + " coins detectes"))
        self.lbl_count.setText(str(len(self._views)) + " vues capturees")

    def _on_run_calibration(self):
        if len(self._views) < 5:
            QMessageBox.warning(self, "Calibration",
                "Capturer au moins 5 vues (idealement 15-30). Vues actuelles : "
                + str(len(self._views)))
            return
        try:
            K, dist, rms, _ = calibrate_intrinsics(
                self._views, self._board,
                use_fisheye=self.chk_fisheye.isChecked())
        except Exception as e:
            QMessageBox.critical(self, "Calibration",
                                  type(e).__name__ + ": " + str(e))
            return
        import time
        self._intrinsics = CameraIntrinsics(
            K=K, dist=dist,
            image_size=self._views[0].image_size,
            rms_px=rms, n_views=len(self._views),
            model="fisheye" if self.chk_fisheye.isChecked() else "standard",
            board_squares_x=self.sp_sx.value(),
            board_squares_y=self.sp_sy.value(),
            board_square_mm=self.sp_sq.value(),
            board_marker_mm=self.sp_mk.value(),
            board_dict=self.cb_dict.currentText(),
            calibrated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )
        nl = chr(10)
        msg_lines = [
            "Calibration OK !",
            "  RMS = " + str(round(rms, 3)) + " px (lower is better, < 1 px = excellent)",
            "  fx = " + str(round(K[0,0], 1)) + "   fy = " + str(round(K[1,1], 1)),
            "  cx = " + str(round(K[0,2], 1)) + "   cy = " + str(round(K[1,2], 1)),
            "  dist[:5] = " + str(dist.ravel()[:5].round(4).tolist()),
            "  " + str(len(self._views)) + " vues, modele " + self._intrinsics.model,
        ]
        self.lbl_result.setText(nl.join(msg_lines))
        self.lbl_result.setStyleSheet("color: #4ade80; font-family: monospace;")
        self.btn_save.setEnabled(True)
        self.btn_apply.setEnabled(True)

    def _on_save(self):
        if self._intrinsics is None:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Sauvegarder intrinsics camera",
                                              "calibrations/camera_intrinsics.json", "JSON (*.json)")
        if not path:
            return
        try:
            self._intrinsics.save(path)
            self.lbl_result.setText(self.lbl_result.text() + chr(10) + "-> Sauve : " + path)
        except Exception as e:
            QMessageBox.critical(self, "Sauvegarde", type(e).__name__ + ": " + str(e))

    def _on_apply(self):
        if self._intrinsics is None:
            return
        self.intrinsics_done.emit(self._intrinsics)
        self.lbl_result.setText(self.lbl_result.text() + chr(10) + "-> Applique dans TwinVision.")

    def _refresh_board(self, *args):
        """Rebuild internal board when params change. Reports status."""
        sx = self.sp_sx.value()
        sy = self.sp_sy.value()
        sq = self.sp_sq.value()
        mk = self.sp_mk.value()
        dict_name = self.cb_dict.currentText()
        # Validate before calling cv2 (which would raise SystemError).
        msg = None
        if sx < 3 or sy < 3:
            msg = "Cases X et Y doivent etre >= 3"
        elif sq <= 0:
            msg = "Taille case doit etre > 0"
        elif mk <= 0:
            msg = "Taille marker doit etre > 0"
        elif mk >= sq:
            msg = ("Marker (" + str(mk) + " mm) doit etre STRICTEMENT < case ("
                   + str(sq) + " mm). Sur calib.io le marker est typiquement "
                   "70-80% de la case (ex : case 15 mm -> marker 11 mm).")
        if msg is not None:
            self.lbl_board_status.setText("[INVALIDE] " + msg)
            self.lbl_board_status.setStyleSheet("color: #f87171; font-size: 10px;")
            return
        try:
            self._board, self._aruco_dict = make_charuco_board(
                squares_x=sx, squares_y=sy,
                square_mm=sq, marker_mm=mk,
                dict_name=dict_name,
                legacy_pattern=self.chk_legacy.isChecked(),
            )
        except Exception as e:
            self.lbl_board_status.setText(
                "[INVALIDE] cv2 rejete : " + str(e)
            )
            self.lbl_board_status.setStyleSheet("color: #f87171; font-size: 10px;")
            return
        self.lbl_board_status.setText(
            "[OK] Board " + str(sx) + "x" + str(sy) + ", case " + str(sq)
            + " mm, marker " + str(mk) + " mm, dict " + dict_name
        )
        self.lbl_board_status.setStyleSheet("color: #4ade80; font-size: 10px;")
        if self._views:
            self._views = []
            self._refresh_views_ui()
        if self._current_frame is not None:
            self.feed_frame(self._current_frame)

    def _on_load_images(self):
        """Load multiple images from disk and run detection on each."""
        paths, _ = QFileDialog.getOpenFileNames(
            self, "Charger images du board",
            "",
            "Images (*.png *.jpg *.jpeg *.bmp);;Tous (*.*)",
        )
        if not paths:
            return
        n_loaded = 0
        n_detected = 0
        for path in paths:
            img = cv2.imread(path)
            if img is None:
                continue
            n_loaded += 1
            corners, ids = detect_charuco(img, self._board, self._aruco_dict)
            if corners is None or ids is None:
                continue
            h, w = img.shape[:2]
            view = CalibrationView(corners=corners, ids=ids, image_size=(w, h))
            self._views.append(view)
            n_detected += 1
        self._refresh_views_ui()
        if n_detected == 0:
            QMessageBox.warning(
                self, "Chargement",
                str(n_loaded) + " images chargees mais 0 detectees." + chr(10)
                + "Verifie : nombre de cases X/Y, taille case/marker (le marker est plus petit !), dictionnaire." + chr(10)
                + "Lis le bas de ton impression calib.io : 'Checker Size XX mm Marker Size YY mm Dictionary ZZZ'."
            )
        else:
            QMessageBox.information(
                self, "Chargement",
                str(n_loaded) + " images chargees, " + str(n_detected) + " avec board detecte."
            )

