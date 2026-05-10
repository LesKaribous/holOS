"""
CalibrationDialog — workflow de calibration caméra.

Onglet 1 — Échiquier        : pattern classique
Onglet 2 — ChArUco ★        : recommandé grand-angle (sans imprimante OK si vous avez les boards)
Onglet 3 — Preset / Manuel  : presets de caméras courantes + saisie manuelle
                               → à utiliser si vous n'avez pas de grille d'étalonnage
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Union

import cv2
import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QPushButton, QSpinBox, QDoubleSpinBox,
    QProgressBar, QFileDialog, QMessageBox, QGroupBox,
    QCheckBox, QTabWidget, QWidget, QComboBox, QFrame,
    QSizePolicy, QScrollArea, QGridLayout,
)

from core.calibration import (
    CalibrationData,
    ChessboardCalibrator,
    CharucoCalibrator,
    _BaseCalibrator,
)
from ui.video_widget import VideoWidget


_Calibrator = Union[ChessboardCalibrator, CharucoCalibrator]

# ──────────────────────────────────────────────────────────────────────────────
# Presets de caméras courantes
# fx_scale / fy_scale : multipliés par la largeur de l'image pour obtenir fx/fy
# cx_scale / cy_scale : multipliés par w/h (0.5 = centre)
# dist_standard : [k1, k2, p1, p2, k3]
# dist_fisheye  : [k1, k2, k3, k4]
# ──────────────────────────────────────────────────────────────────────────────
CAMERA_PRESETS: dict[str, dict] = {
    "— Sélectionner un preset —": {},

    "Aucune correction (identité)": {
        "model": "standard",
        "note": "Pas de distorsion. Utile comme baseline.",
        "fx_scale": 1.0, "fy_scale": 1.0,
        "dist_standard": [0.0, 0.0, 0.0, 0.0, 0.0],
    },

    "Grand-angle ~90° (standard)": {
        "model": "standard",
        "note": "Objectif grand-angle avec légère distorsion en barrique.",
        "fx_scale": 0.65, "fy_scale": 0.65,
        "dist_standard": [-0.20, 0.05, 0.0, 0.0, -0.005],
    },

    "Grand-angle ~120° (standard)": {
        "model": "standard",
        "note": "Forte distorsion en barrique, FOV ~120°.",
        "fx_scale": 0.50, "fy_scale": 0.50,
        "dist_standard": [-0.38, 0.14, 0.0, 0.0, -0.02],
    },

    "Grand-angle ~120° (fisheye)": {
        "model": "fisheye",
        "note": "Modèle équidistant fisheye, ~120° FOV.",
        "fx_scale": 0.48, "fy_scale": 0.48,
        "dist_fisheye": [-0.04, 0.0, 0.0, 0.0],
    },

    "Grand-angle ~150° (fisheye)": {
        "model": "fisheye",
        "note": "Très grand-angle fisheye, ~150° FOV.",
        "fx_scale": 0.36, "fy_scale": 0.36,
        "dist_fisheye": [-0.08, 0.01, 0.0, 0.0],
    },

    "GoPro Hero (Wide, 1280×720)": {
        "model": "standard",
        "note": "GoPro HERO5/6/7/8 mode Wide, 1280×720. Params approx.",
        "fx_scale": 0.52, "fy_scale": 0.52,
        "dist_standard": [-0.27, 0.08, 0.0, 0.0, -0.01],
    },

    "GoPro Hero (Linear, 1280×720)": {
        "model": "standard",
        "note": "GoPro HERO5/6/7/8 mode Linear (moins de distorsion).",
        "fx_scale": 0.72, "fy_scale": 0.72,
        "dist_standard": [-0.10, 0.02, 0.0, 0.0, 0.0],
    },

    "Raspberry Pi Cam V2 (standard)": {
        "model": "standard",
        "note": "RPi Camera Module V2 (Sony IMX219), pas de fisheye.",
        "fx_scale": 0.92, "fy_scale": 0.92,
        "dist_standard": [-0.35, 0.11, 0.0, 0.0, 0.0],
    },

    "Logitech C920/C930 (standard)": {
        "model": "standard",
        "note": "Webcam USB grand-angle classique.",
        "fx_scale": 0.75, "fy_scale": 0.75,
        "dist_standard": [-0.17, 0.03, 0.0, 0.0, 0.0],
    },
}


class CalibrationDialog(QDialog):
    """
    Fenêtre de calibration caméra.
    Le user sélectionne pattern + modèle, capture des frames, puis calibre.
    Ou bien charge directement un preset / entre les paramètres manuellement.
    """

    calibration_done = Signal(object)   # émet CalibrationData

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Calibration caméra")
        self.setMinimumSize(860, 620)

        self._calibrator: Optional[_Calibrator] = None
        self._current_frame: Optional[np.ndarray] = None
        self._result: Optional[CalibrationData] = None
        self._image_size: tuple[int, int] = (1280, 720)

        self._auto_timer = QTimer(self)
        self._auto_timer.timeout.connect(self._try_auto_capture)

        self._build_ui()

    # ──────────────────────────────────────────────────────────────────────────
    # Construction UI
    # ──────────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QHBoxLayout(self)

        # ── Gauche : aperçu ──────────────────────────────────────────────────
        left = QVBoxLayout()
        self._preview = VideoWidget("Aperçu")
        self._preview.setMinimumSize(520, 380)
        left.addWidget(self._preview)

        btn_load_img = QPushButton("📂  Charger une image de référence")
        btn_load_img.setToolTip("Charger une image fixe pour tester/régler la calibration\n"
                                "(inutile si une vidéo est ouverte dans la fenêtre principale)")
        btn_load_img.clicked.connect(self._load_reference_image)
        left.addWidget(btn_load_img)

        self._status_lbl = QLabel("Choisissez un onglet pour commencer.")
        self._status_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._status_lbl.setStyleSheet("color:#aaa; font-size:11px;")
        left.addWidget(self._status_lbl)

        self._progress = QProgressBar()
        self._progress.setRange(0, 20)
        self._progress.setValue(0)
        self._progress.setFormat("%v / %m frames")
        left.addWidget(self._progress)
        root.addLayout(left, stretch=3)

        # ── Droite : contrôles ───────────────────────────────────────────────
        right = QVBoxLayout()

        # ── Modèle caméra (utilisé par les onglets échiquier/charuco) ────────
        model_group = QGroupBox("Modèle de caméra")
        model_form = QFormLayout(model_group)
        self._combo_model = QComboBox()
        self._combo_model.addItem("Standard  (objectif normal)", "standard")
        self._combo_model.addItem("Fisheye  (grand-angle, FOV > 120°)", "fisheye")
        model_form.addRow("Modèle:", self._combo_model)
        right.addWidget(model_group)

        # ── Onglets ──────────────────────────────────────────────────────────
        self._tabs = QTabWidget()
        self._tabs.addTab(self._build_chess_tab(),   "Échiquier")
        self._tabs.addTab(self._build_charuco_tab(), "ChArUco ★")
        self._tabs.addTab(self._build_preset_tab(),  "Preset / Manuel ⚡")
        self._tabs.setCurrentIndex(2)   # Preset par défaut (le plus accessible)
        self._tabs.currentChanged.connect(self._on_tab_changed)
        right.addWidget(self._tabs)

        # ── Capture (masqué sur l'onglet preset) ─────────────────────────────
        self._cap_group = QGroupBox("Capture")
        cap_layout = QVBoxLayout(self._cap_group)

        self._chk_auto = QCheckBox("Auto-capture (toutes les 2 s)")
        self._chk_auto.stateChanged.connect(self._toggle_auto)
        cap_layout.addWidget(self._chk_auto)

        row = QHBoxLayout()
        self._btn_capture = QPushButton("📷  Capturer")
        self._btn_capture.setMinimumHeight(34)
        self._btn_capture.clicked.connect(self._capture)
        self._btn_capture.setEnabled(False)
        row.addWidget(self._btn_capture)
        self._btn_reset = QPushButton("🗑  Reset")
        self._btn_reset.clicked.connect(self._reset)
        row.addWidget(self._btn_reset)
        cap_layout.addLayout(row)

        self._lbl_count = QLabel("Frames : 0")
        self._lbl_count.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_count.setFont(QFont("Arial", 11, QFont.Weight.Bold))
        cap_layout.addWidget(self._lbl_count)
        right.addWidget(self._cap_group)

        # ── Résultats (pattern calibration) ──────────────────────────────────
        self._cal_group = QGroupBox("Résultat calibration")
        cal_layout = QVBoxLayout(self._cal_group)

        self._btn_calibrate = QPushButton("⚙  Calibrer")
        self._btn_calibrate.setMinimumHeight(34)
        self._btn_calibrate.setEnabled(False)
        self._btn_calibrate.clicked.connect(self._calibrate)
        cal_layout.addWidget(self._btn_calibrate)

        self._lbl_rms = QLabel("RMS : —")
        self._lbl_rms.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cal_layout.addWidget(self._lbl_rms)

        row2 = QHBoxLayout()
        self._btn_save = QPushButton("💾  Sauver")
        self._btn_save.setEnabled(False)
        self._btn_save.clicked.connect(self._save)
        row2.addWidget(self._btn_save)
        self._btn_generate = QPushButton("🖨  Board")
        self._btn_generate.clicked.connect(self._generate_board)
        row2.addWidget(self._btn_generate)
        cal_layout.addLayout(row2)

        self._btn_accept = QPushButton("✅  Appliquer et fermer")
        self._btn_accept.setEnabled(False)
        self._btn_accept.clicked.connect(self._accept_result)
        cal_layout.addWidget(self._btn_accept)
        right.addWidget(self._cal_group)

        right.addStretch()
        self._btn_close = QPushButton("Fermer")
        self._btn_close.clicked.connect(self.reject)
        right.addWidget(self._btn_close)

        root.addLayout(right, stretch=1)
        self._on_tab_changed(2)   # init visibility

    # ── Construction des onglets ───────────────────────────────────────────────

    def _build_chess_tab(self) -> QWidget:
        tab = QWidget()
        form = QFormLayout(tab)
        self._chess_cols = QSpinBox(); self._chess_cols.setRange(3, 20); self._chess_cols.setValue(9)
        self._chess_rows = QSpinBox(); self._chess_rows.setRange(3, 20); self._chess_rows.setValue(6)
        self._chess_sq = QDoubleSpinBox(); self._chess_sq.setRange(1, 500); self._chess_sq.setValue(25); self._chess_sq.setSuffix(" mm")
        form.addRow("Colonnes (coins internes):", self._chess_cols)
        form.addRow("Lignes (coins internes):",   self._chess_rows)
        form.addRow("Taille case:",               self._chess_sq)
        note = QLabel("⚠ Peut échouer si les bords sont hors-champ.\nPrivilégiez ChArUco ou Preset.")
        note.setStyleSheet("color:#facc15; font-size:10px;")
        form.addRow(note)
        return tab

    def _build_charuco_tab(self) -> QWidget:
        tab = QWidget()
        form = QFormLayout(tab)
        self._charuco_cols = QSpinBox(); self._charuco_cols.setRange(3, 20); self._charuco_cols.setValue(10)
        self._charuco_rows = QSpinBox(); self._charuco_rows.setRange(3, 20); self._charuco_rows.setValue(7)
        self._charuco_sq = QDoubleSpinBox(); self._charuco_sq.setRange(1, 500); self._charuco_sq.setValue(30); self._charuco_sq.setSuffix(" mm")
        self._charuco_mk = QDoubleSpinBox(); self._charuco_mk.setRange(1, 500); self._charuco_mk.setValue(22); self._charuco_mk.setSuffix(" mm")
        self._charuco_dict = QComboBox()
        for name in CharucoCalibrator.DICTS:
            self._charuco_dict.addItem(name)
        self._charuco_dict.setCurrentText("5x5_100")
        form.addRow("Colonnes (cases):",    self._charuco_cols)
        form.addRow("Lignes (cases):",      self._charuco_rows)
        form.addRow("Taille case:",         self._charuco_sq)
        form.addRow("Taille marqueur:",     self._charuco_mk)
        form.addRow("Dictionnaire ArUco:",  self._charuco_dict)
        note = QLabel("✓ Robuste aux vues partielles.\n   Cliquer '🖨 Board' pour générer le pattern à imprimer.")
        note.setStyleSheet("color:#4ade80; font-size:10px;")
        form.addRow(note)
        return tab

    def _build_preset_tab(self) -> QWidget:
        """Onglet Preset / Manuel — ne requiert pas de grille d'étalonnage."""
        tab = QWidget()
        vbox = QVBoxLayout(tab)
        vbox.setSpacing(6)

        # ── Presets ───────────────────────────────────────────────────────────
        preset_group = QGroupBox("Presets caméras courantes")
        pform = QFormLayout(preset_group)

        self._combo_preset = QComboBox()
        for name in CAMERA_PRESETS:
            self._combo_preset.addItem(name)
        self._combo_preset.setCurrentIndex(0)
        self._combo_preset.currentIndexChanged.connect(self._on_preset_changed)
        pform.addRow("Preset:", self._combo_preset)

        self._lbl_preset_note = QLabel("")
        self._lbl_preset_note.setStyleSheet("color:#aaa; font-size:10px;")
        self._lbl_preset_note.setWordWrap(True)
        pform.addRow(self._lbl_preset_note)

        btn_apply_preset = QPushButton("⬇  Charger le preset dans les paramètres")
        btn_apply_preset.clicked.connect(self._apply_preset)
        pform.addRow(btn_apply_preset)
        vbox.addWidget(preset_group)

        # ── Paramètres manuels ────────────────────────────────────────────────
        manual_group = QGroupBox("Paramètres manuels")
        mform = QFormLayout(manual_group)

        # Modèle
        self._combo_preset_model = QComboBox()
        self._combo_preset_model.addItem("Standard  (k1,k2,p1,p2,k3)", "standard")
        self._combo_preset_model.addItem("Fisheye  (k1,k2,k3,k4)",     "fisheye")
        self._combo_preset_model.currentIndexChanged.connect(self._on_preset_model_changed)
        mform.addRow("Modèle:", self._combo_preset_model)

        # Taille image
        size_row = QHBoxLayout()
        self._spin_img_w = QSpinBox(); self._spin_img_w.setRange(1, 9999); self._spin_img_w.setValue(1280); self._spin_img_w.setSuffix(" px")
        self._spin_img_h = QSpinBox(); self._spin_img_h.setRange(1, 9999); self._spin_img_h.setValue(720);  self._spin_img_h.setSuffix(" px")
        size_row.addWidget(self._spin_img_w); size_row.addWidget(QLabel("×")); size_row.addWidget(self._spin_img_h)
        mform.addRow("Taille image:", size_row)

        # Focale
        self._spin_fx = QDoubleSpinBox(); self._spin_fx.setRange(50, 9999); self._spin_fx.setValue(640); self._spin_fx.setSuffix(" px"); self._spin_fx.setDecimals(1)
        self._spin_fy = QDoubleSpinBox(); self._spin_fy.setRange(50, 9999); self._spin_fy.setValue(640); self._spin_fy.setSuffix(" px"); self._spin_fy.setDecimals(1)
        self._chk_square = QCheckBox("fy = fx")
        self._chk_square.setChecked(True)
        self._chk_square.stateChanged.connect(self._on_square_changed)
        self._spin_fx.valueChanged.connect(self._on_fx_changed)
        fx_row = QHBoxLayout(); fx_row.addWidget(self._spin_fx); fx_row.addWidget(self._chk_square)
        mform.addRow("fx:", fx_row)
        mform.addRow("fy:", self._spin_fy)

        # Centre optique
        self._spin_cx = QDoubleSpinBox(); self._spin_cx.setRange(0, 9999); self._spin_cx.setValue(640); self._spin_cx.setSuffix(" px"); self._spin_cx.setDecimals(1)
        self._spin_cy = QDoubleSpinBox(); self._spin_cy.setRange(0, 9999); self._spin_cy.setValue(360); self._spin_cy.setSuffix(" px"); self._spin_cy.setDecimals(1)
        mform.addRow("cx (centre X):", self._spin_cx)
        mform.addRow("cy (centre Y):", self._spin_cy)

        # Distorsion — conteneur avec deux états (standard / fisheye)
        self._dist_stack: dict[str, list[QDoubleSpinBox]] = {}

        # Standard : k1 k2 p1 p2 k3
        self._dist_standard_widgets = []
        dist_std_group = QGroupBox("Distorsion (Standard)")
        dist_std_form = QFormLayout(dist_std_group)
        for name, val, lo, hi in [("k1", -0.3, -3.0, 3.0),
                                    ("k2",  0.0, -3.0, 3.0),
                                    ("p1",  0.0, -0.5, 0.5),
                                    ("p2",  0.0, -0.5, 0.5),
                                    ("k3",  0.0, -1.0, 1.0)]:
            sp = QDoubleSpinBox(); sp.setRange(lo, hi); sp.setValue(val); sp.setDecimals(4); sp.setSingleStep(0.01)
            dist_std_form.addRow(f"{name}:", sp)
            self._dist_standard_widgets.append(sp)
        self._dist_standard_group = dist_std_group
        mform.addRow(dist_std_group)

        # Fisheye : k1 k2 k3 k4
        self._dist_fisheye_widgets = []
        dist_fish_group = QGroupBox("Distorsion (Fisheye)")
        dist_fish_form = QFormLayout(dist_fish_group)
        for name, val in [("k1", -0.05), ("k2", 0.0), ("k3", 0.0), ("k4", 0.0)]:
            sp = QDoubleSpinBox(); sp.setRange(-3.0, 3.0); sp.setValue(val); sp.setDecimals(4); sp.setSingleStep(0.01)
            dist_fish_form.addRow(f"{name}:", sp)
            self._dist_fisheye_widgets.append(sp)
        self._dist_fisheye_group = dist_fish_group
        mform.addRow(dist_fish_group)

        vbox.addWidget(manual_group)

        # ── Boutons preset tab ────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        self._btn_preview_undist = QPushButton("👁  Voir l'effet sur le frame actuel")
        self._btn_preview_undist.clicked.connect(self._preview_undistort)
        btn_row.addWidget(self._btn_preview_undist)
        vbox.addLayout(btn_row)

        btn_row2 = QHBoxLayout()
        self._btn_accept_preset = QPushButton("✅  Appliquer ces paramètres")
        self._btn_accept_preset.setMinimumHeight(34)
        self._btn_accept_preset.clicked.connect(self._accept_manual)
        btn_row2.addWidget(self._btn_accept_preset)
        vbox.addLayout(btn_row2)

        # Init visibility
        self._on_preset_model_changed(0)

        # Scroll wrapper car le contenu peut être long
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QWidget(); inner.setLayout(vbox)
        scroll.setWidget(inner)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        return scroll

    # ──────────────────────────────────────────────────────────────────────────
    # API publique
    # ──────────────────────────────────────────────────────────────────────────

    def _load_reference_image(self):
        """Charge une image fixe comme frame de référence (indépendant de la vidéo principale)."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Charger une image de référence", "",
            "Images (*.png *.jpg *.jpeg *.bmp *.tiff)"
        )
        if not path:
            return
        img = cv2.imread(path)
        if img is None:
            QMessageBox.warning(self, "Erreur", f"Impossible de charger :\n{path}")
            return
        self.feed_frame(img)
        self._status_lbl.setText(f"Image chargée : {Path(path).name}")
        self._status_lbl.setStyleSheet("color:#4ade80; font-size:11px;")

    def feed_frame(self, frame: np.ndarray):
        """Appelé par MainWindow à chaque nouveau frame quand la dialog est visible."""
        self._current_frame = frame.copy()
        h, w = frame.shape[:2]
        self._image_size = (w, h)
        self._spin_img_w.setValue(w)
        self._spin_img_h.setValue(h)
        self._btn_capture.setEnabled(True)

        idx = self._tabs.currentIndex()
        if idx == 2:
            # Onglet preset : afficher le frame brut (pas de détection pattern)
            self._preview.display_frame(frame)
        else:
            # Onglets échiquier / charuco : détection live
            _tmp = self._make_preview_calibrator()
            _, preview = _tmp.detect(frame)
            self._preview.display_frame(preview)

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers internes
    # ──────────────────────────────────────────────────────────────────────────

    @property
    def _model(self) -> str:
        return self._combo_model.currentData()

    @property
    def _is_charuco(self) -> bool:
        return self._tabs.currentIndex() == 1

    def _make_preview_calibrator(self) -> _BaseCalibrator:
        if self._is_charuco:
            return CharucoCalibrator(
                cols=self._charuco_cols.value(), rows=self._charuco_rows.value(),
                square_mm=self._charuco_sq.value(), marker_mm=self._charuco_mk.value(),
                dict_name=self._charuco_dict.currentText(), model=self._model,
            )
        return ChessboardCalibrator(
            cols=self._chess_cols.value(), rows=self._chess_rows.value(),
            square_mm=self._chess_sq.value(), model=self._model,
        )

    def _ensure_calibrator(self):
        if self._calibrator is None:
            self._calibrator = self._make_preview_calibrator()

    def _build_manual_calibration(self) -> CalibrationData:
        """Construit un CalibrationData depuis les spinboxes manuels."""
        w = self._spin_img_w.value()
        h = self._spin_img_h.value()
        fx = self._spin_fx.value()
        fy = self._spin_fy.value()
        cx = self._spin_cx.value()
        cy = self._spin_cy.value()
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

        model = self._combo_preset_model.currentData()
        if model == "fisheye":
            D = np.array([[sp.value()] for sp in self._dist_fisheye_widgets], dtype=np.float64)  # (4,1)
        else:
            D = np.array([[sp.value() for sp in self._dist_standard_widgets]], dtype=np.float64)  # (1,5)

        return CalibrationData(
            camera_matrix=K, dist_coeffs=D,
            model=model, image_size=(w, h),
            rms_error=0.0,
            calibrated_at="manuel",
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Slots — onglets pattern
    # ──────────────────────────────────────────────────────────────────────────

    def _on_tab_changed(self, idx: int):
        is_preset = (idx == 2)
        self._cap_group.setVisible(not is_preset)
        self._cal_group.setVisible(not is_preset)
        if is_preset:
            self._status_lbl.setText("Choisissez un preset ou entrez les paramètres manuellement.")
            self._progress.setValue(0)

    def _capture(self):
        if self._current_frame is None:
            return
        self._ensure_calibrator()
        found, preview = self._calibrator.add_frame(self._current_frame)
        self._preview.display_frame(preview)
        n = self._calibrator.sample_count
        self._lbl_count.setText(f"Frames : {n}")
        self._progress.setValue(min(n, self._progress.maximum()))
        if found:
            self._status_lbl.setText(f"✓ Pattern détecté — {n} frames stockés")
            self._status_lbl.setStyleSheet("color:#4ade80; font-size:11px;")
        else:
            self._status_lbl.setText("✗ Pattern non détecté sur ce frame")
            self._status_lbl.setStyleSheet("color:#f87171; font-size:11px;")
        self._btn_calibrate.setEnabled(n >= 5)

    def _toggle_auto(self, state: int):
        if state:
            self._auto_timer.start(2000)
        else:
            self._auto_timer.stop()

    def _try_auto_capture(self):
        if self._current_frame is not None:
            self._capture()

    def _reset(self):
        self._auto_timer.stop()
        self._chk_auto.setChecked(False)
        self._calibrator = None
        self._result = None
        self._lbl_count.setText("Frames : 0")
        self._progress.setValue(0)
        self._lbl_rms.setText("RMS : —")
        self._lbl_rms.setStyleSheet("")
        self._btn_calibrate.setEnabled(False)
        self._btn_save.setEnabled(False)
        self._btn_accept.setEnabled(False)
        self._status_lbl.setText("Remis à zéro.")
        self._status_lbl.setStyleSheet("color:#aaa; font-size:11px;")

    def _calibrate(self):
        if self._calibrator is None:
            return
        try:
            result = self._calibrator.calibrate()
        except RuntimeError as e:
            QMessageBox.critical(self, "Erreur", str(e))
            return
        if result is None:
            QMessageBox.warning(self, "Insuffisant", "Pas assez de frames (minimum 5).")
            return
        self._result = result
        rms = result.rms_error
        color = "#4ade80" if rms < 0.5 else "#facc15" if rms < 1.5 else "#f87171"
        quality = "Excellente" if rms < 0.5 else "Bonne" if rms < 1.5 else "Mauvaise"
        self._lbl_rms.setText(f"RMS : {rms:.4f} px  ({quality})")
        self._lbl_rms.setStyleSheet(f"color:{color}; font-weight:bold;")
        self._btn_save.setEnabled(True)
        self._btn_accept.setEnabled(True)

    def _save(self):
        if self._result is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Sauvegarder", f"calibrations/calib_{self._result.model}.json", "JSON (*.json)")
        if path:
            self._result.save(path)
            QMessageBox.information(self, "OK", f"Sauvegardé :\n{path}")

    def _generate_board(self):
        if not self._is_charuco:
            QMessageBox.information(self, "Board", "Disponible seulement en mode ChArUco.")
            return
        tmp = CharucoCalibrator(cols=self._charuco_cols.value(), rows=self._charuco_rows.value(),
                                 square_mm=self._charuco_sq.value(), marker_mm=self._charuco_mk.value(),
                                 dict_name=self._charuco_dict.currentText())
        img = tmp.generate_board_image(px_per_mm=8.0)
        path, _ = QFileDialog.getSaveFileName(
            self, "Sauver le board", f"charuco_{tmp.cols}x{tmp.rows}.png", "Images (*.png)")
        if path:
            cv2.imwrite(path, img)
            QMessageBox.information(self, "Board généré",
                f"{path}\nTaille case : {tmp.square_mm:.0f} mm — Imprimer à l'échelle 1:1.")

    def _accept_result(self):
        if self._result is not None:
            self.calibration_done.emit(self._result)
        self.accept()

    # ──────────────────────────────────────────────────────────────────────────
    # Slots — onglet Preset / Manuel
    # ──────────────────────────────────────────────────────────────────────────

    def _on_preset_changed(self, idx: int):
        name = self._combo_preset.currentText()
        p = CAMERA_PRESETS.get(name, {})
        self._lbl_preset_note.setText(p.get("note", ""))

    def _apply_preset(self):
        name = self._combo_preset.currentText()
        p = CAMERA_PRESETS.get(name, {})
        if not p:
            return

        w, h = self._image_size
        fx = p.get("fx_scale", 0.7) * w
        fy = p.get("fy_scale", p.get("fx_scale", 0.7)) * w
        cx = p.get("cx_scale", 0.5) * w
        cy = p.get("cy_scale", 0.5) * h
        model = p.get("model", "standard")

        # Bloquer les signaux pour éviter les mises à jour en cascade
        for sp in [self._spin_fx, self._spin_fy, self._spin_cx, self._spin_cy]:
            sp.blockSignals(True)

        self._spin_fx.setValue(fx)
        self._spin_fy.setValue(fy)
        self._spin_cx.setValue(cx)
        self._spin_cy.setValue(cy)

        for sp in [self._spin_fx, self._spin_fy, self._spin_cx, self._spin_cy]:
            sp.blockSignals(False)

        # Modèle
        model_idx = 1 if model == "fisheye" else 0
        self._combo_preset_model.setCurrentIndex(model_idx)

        # Distorsion
        if model == "fisheye":
            d = p.get("dist_fisheye", [0, 0, 0, 0])
            for sp, val in zip(self._dist_fisheye_widgets, d):
                sp.setValue(val)
        else:
            d = p.get("dist_standard", [0, 0, 0, 0, 0])
            for sp, val in zip(self._dist_standard_widgets, d):
                sp.setValue(val)

        self._status_lbl.setText(f"Preset chargé : {name}")
        self._status_lbl.setStyleSheet("color:#4ade80; font-size:11px;")

    def _on_preset_model_changed(self, idx: int):
        model = self._combo_preset_model.currentData()
        self._dist_standard_group.setVisible(model == "standard")
        self._dist_fisheye_group.setVisible(model == "fisheye")

    def _on_square_changed(self, state: int):
        self._spin_fy.setEnabled(not bool(state))
        if state:
            self._spin_fy.setValue(self._spin_fx.value())

    def _on_fx_changed(self, val: float):
        if self._chk_square.isChecked():
            self._spin_fy.blockSignals(True)
            self._spin_fy.setValue(val)
            self._spin_fy.blockSignals(False)

    def _preview_undistort(self):
        """Applique les paramètres manuels sur le frame actuel et l'affiche."""
        if self._current_frame is None:
            self._status_lbl.setText("Aucun frame disponible — ouvrez une vidéo d'abord.")
            self._status_lbl.setStyleSheet("color:#f87171; font-size:11px;")
            return
        try:
            cal = self._build_manual_calibration()
            undistorted = cal.undistort(self._current_frame)
            self._preview.display_frame(undistorted)
            self._status_lbl.setText("Preview undistordu affiché. Ajustez les paramètres si besoin.")
            self._status_lbl.setStyleSheet("color:#4ade80; font-size:11px;")
        except Exception as e:
            self._status_lbl.setText(f"Erreur : {e}")
            self._status_lbl.setStyleSheet("color:#f87171; font-size:11px;")

    def _accept_manual(self):
        """Accepte les paramètres manuels comme calibration."""
        cal = self._build_manual_calibration()
        self.calibration_done.emit(cal)
        self._status_lbl.setText("✅ Paramètres appliqués.")
        self._status_lbl.setStyleSheet("color:#4ade80; font-size:11px;")
        self.accept()
