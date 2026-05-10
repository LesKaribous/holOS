"""
MainWindow — Fenêtre principale TwinVision.

Layout :
  ┌─[Toolbar]───────────────────────────────────────────────────────────────┐
  │ ┌──[Vue brute]──────────────┐ ┌──[Vue rectifiée]──────────────────────┐ │
  │ │                           │ │                                       │ │
  │ │  Image + overlays ArUco   │ │  Table vue de dessus + grille         │ │
  │ │                           │ │                                       │ │
  │ └───────────────────────────┘ └───────────────────────────────────────┘ │
  │ ┌──[Contrôles playback]──────────────────────────────────────────────── │
  │ │  [|<] [<] [▶/||] [>] [>|]   Slider   [Vitesse]                      │ │
  │ │  Frame: 1234 / 25871   0:41 / 14:22                                  │ │
  │ └───────────────────────────────────────────────────────────────────────┘ │
  └─[StatusBar]──────────────────────────────────────────────────────────────┘

  + DockWidget (droite) : réglages ArUco, calibration, table
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from PySide6.QtCore import Qt, QTimer, QSettings, Signal, Slot
from PySide6.QtGui import QAction, QIcon, QKeySequence, QFont
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QSplitter, QVBoxLayout, QHBoxLayout,
    QToolBar, QDockWidget, QFileDialog, QMessageBox, QSlider, QLabel,
    QPushButton, QComboBox, QSpinBox, QCheckBox, QGroupBox, QFormLayout,
    QStatusBar, QSizePolicy, QFrame, QDoubleSpinBox, QGridLayout,
    QScrollArea, QTabWidget,
)

from core.video_source import VideoSource
from core.aruco_detector import ArucoDetector, DetectionResult, ARUCO_DICTS
from core.table_rectifier import TableRectifier, CornerConfig, TagAnchor, TABLE_W_MM, TABLE_H_MM
from core.calibration import CalibrationData
from core.preprocessor import Preprocessor
from core.robot_tracker import (
    RobotTracker, RobotConfig, solve_camera_xy_from_known_position,
)
from core.poly_calibration import PolyCalibration
from core.charuco_calib import CameraIntrinsics
from ui.video_widget import VideoWidget
from ui.calibration_dialog import CalibrationDialog
from ui.preprocess_panel import PreprocessPanel
from ui.tracking_panel import TrackingPanel
from ui.table_calib_dialog import TableCalibDialog
from ui.charuco_wizard import CharucoWizardDialog

# Chemin absolu : toujours à la racine du projet (parent de src/)
_PROJECT_ROOT = Path(os.path.dirname(os.path.abspath(__file__))).parent.parent
SETTINGS_FILE = str(_PROJECT_ROOT / "twinvision_settings.json")

PLAYBACK_SPEEDS = [0.25, 0.5, 1.0, 1.5, 2.0, 4.0]


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TwinVision")
        self.setMinimumSize(1100, 680)

        # ── État ──────────────────────────────────────────────────────────────
        self._source: Optional[VideoSource] = None
        self._detector = ArucoDetector("4x4_50")
        self._rectifier = TableRectifier()
        self._calibration: Optional[CalibrationData] = None
        self._preprocessor = Preprocessor()
        self._tracker = RobotTracker(
            robots=[
                RobotConfig(tag_id=2, label="USER",     color_bgr=(0, 255, 0), z_mm=530.0),
                RobotConfig(tag_id=7, label="OPPONENT", color_bgr=(0, 0, 255), z_mm=530.0),
            ],
            cam_mode="manual",
            cam_override=(275.0, -200.0, 1000.0),
            trail_len=1,   # trails desactives (user feedback : useless)
        )
        self._poly_cal: Optional[PolyCalibration] = None

        self._playing = False
        self._speed_idx = 2        # index dans PLAYBACK_SPEEDS (1.0x par défaut)
        self._last_result: Optional[DetectionResult] = None
        self._current_raw: Optional[np.ndarray] = None
        self._current_processed: Optional[np.ndarray] = None
        self._seeking = False      # vrai quand l'utilisateur drag le slider

        # ── Timer de lecture ──────────────────────────────────────────────────
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timer)

        # ── UI ────────────────────────────────────────────────────────────────
        self._build_toolbar()
        self._build_central()
        self._build_dock()
        self._build_statusbar()

        # ── Calibration dialog (caché) ─────────────────────────────────────
        self._cal_dialog: Optional[CalibrationDialog] = None
        self._table_cal_dialog: Optional[TableCalibDialog] = None
        self._charuco_wizard: Optional[CharucoWizardDialog] = None
        self._intrinsics: Optional[CameraIntrinsics] = None
        self._intrinsics_path: Optional[str] = None

        self._load_settings()
        self._update_controls_enabled()

    # ══════════════════════════════════════════════════════════════════════════
    # Construction UI
    # ══════════════════════════════════════════════════════════════════════════

    def _trigger_rerender(self):
        """Re-process the current frame so toggle changes take effect immediately
        (when paused). When playing, the next frame will pick up the new state."""
        if self._current_raw is not None and not self._playing:
            self._process_and_display(self._current_raw)

    def _request_save_settings(self):
        """Debounce settings writes : 500 ms after the last change."""
        if not hasattr(self, "_save_timer") or self._save_timer is None:
            self._save_timer = QTimer(self)
            self._save_timer.setSingleShot(True)
            self._save_timer.timeout.connect(self._save_settings)
        self._save_timer.start(500)

    def _build_toolbar(self):
        tb = QToolBar("Principal", self)
        tb.setMovable(False)
        tb.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.addToolBar(tb)

        self._act_open_video = QAction("📂  Ouvrir vidéo", self)
        self._act_open_video.setShortcut(QKeySequence("Ctrl+O"))
        self._act_open_video.triggered.connect(self._open_video)
        tb.addAction(self._act_open_video)

        self._act_open_camera = QAction("🎥  Caméra", self)
        self._act_open_camera.triggered.connect(self._open_camera)
        tb.addAction(self._act_open_camera)

        tb.addSeparator()

        # Legacy chessboard/charuco calibration -- hidden but kept for fallback.
        # The polynomial table calibration (red mask) is the source of truth.
        self._act_cal_dialog = QAction("🔧  Calibrer caméra", self)
        self._act_cal_dialog.triggered.connect(self._open_cal_dialog)
        # tb.addAction(self._act_cal_dialog)  # disabled

        self._act_load_cal = QAction("📥  Charger calibration", self)
        self._act_load_cal.triggered.connect(self._load_calibration)
        # tb.addAction(self._act_load_cal)  # disabled

        # Legacy red-mask polynomial wizard -- kept in code but hidden from
        # toolbar. The clean robotics workflow is ChArUco calibration below.
        self._act_table_cal = QAction("🟥  Calibrer table (poly, legacy)", self)
        self._act_table_cal.setToolTip("Calibration polynomiale table depuis image + mask rouge")
        self._act_table_cal.triggered.connect(self._open_table_cal_dialog)
        # tb.addAction(self._act_table_cal)  # disabled : prefer ChArUco below

        self._act_charuco = QAction("🎯  Calibrer caméra (ChArUco)", self)
        self._act_charuco.setToolTip(
            "Wizard de calibration intrinsèque (K, dist) à partir d'un damier ChArUco."
            " Une seule fois par caméra+objectif. Ensuite la pose se calcule live"
            " par solvePnP sur les 4 ArUco anchors."
        )
        self._act_charuco.triggered.connect(self._open_charuco_wizard)
        tb.addAction(self._act_charuco)

        # Charger un JSON d'intrinsics deja calibre (sans relancer le wizard).
        self._act_load_intr = QAction("📂  Charger intrinsics...", self)
        self._act_load_intr.setToolTip(
            "Charger un fichier camera_intrinsics.json deja calibre"
            " (active immediatement le mode pose-based / solvePnP)."
        )
        self._act_load_intr.triggered.connect(self._load_intrinsics_from_disk)
        tb.addAction(self._act_load_intr)

        # Toggle on/off du polynome. Quand desactive, seul le mode pose-based
        # ou le fallback 4-tag homography est utilise pour la rectification.
        self._act_use_poly = QAction("🟥  Polynomial actif", self)
        self._act_use_poly.setCheckable(True)
        self._act_use_poly.setChecked(True)
        self._act_use_poly.setToolTip(
            "Active/desactive l'utilisation du polynome de calibration table."
            " A desactiver une fois les intrinsics ChArUco charges."
        )
        self._act_use_poly.toggled.connect(self._on_poly_action_toggled)
        tb.addAction(self._act_use_poly)
        # Ce QAction REMPLACE le shim _AlwaysTrueChk -- les anciens appels
        # self._chk_use_poly.isChecked() / setChecked() restent compatibles.
        self._chk_use_poly = self._act_use_poly

        self._act_save_frame = QAction("🖼  Sauver frame", self)
        self._act_save_frame.setShortcut(QKeySequence("Ctrl+S"))
        self._act_save_frame.triggered.connect(self._save_current_frame)
        tb.addAction(self._act_save_frame)

    def _build_central(self):
        central = QWidget()
        self.setCentralWidget(central)
        vbox = QVBoxLayout(central)
        vbox.setContentsMargins(4, 4, 4, 4)
        vbox.setSpacing(4)

        # ── Deux vues vidéo ──────────────────────────────────────────────────
        splitter = QSplitter(Qt.Orientation.Horizontal)

        self._view_raw = VideoWidget("Brut")
        self._view_rect = VideoWidget("Table rectifiée")
        self._view_raw.hovered.connect(self._on_raw_hover)
        self._view_rect.hovered.connect(self._on_rect_hover)

        splitter.addWidget(self._view_raw)
        splitter.addWidget(self._view_rect)
        splitter.setSizes([600, 500])
        vbox.addWidget(splitter, stretch=1)

        # ── Contrôles playback ───────────────────────────────────────────────
        ctrl_frame = QFrame()
        ctrl_frame.setFrameShape(QFrame.Shape.StyledPanel)
        ctrl_frame.setMaximumHeight(80)
        ctrl_layout = QVBoxLayout(ctrl_frame)
        ctrl_layout.setContentsMargins(6, 4, 6, 4)
        ctrl_layout.setSpacing(4)

        # Ligne 1 : boutons + slider
        row1 = QHBoxLayout()

        self._btn_first = QPushButton("|◀")
        self._btn_prev  = QPushButton("◀")
        self._btn_play  = QPushButton("▶")
        self._btn_next  = QPushButton("▶|")
        self._btn_last  = QPushButton("▶|")

        for btn, tip, cb in [
            (self._btn_first, "Premier frame (Home)",   self._go_first),
            (self._btn_prev,  "Frame précédent (←)",    self._go_prev),
            (self._btn_play,  "Lecture / Pause (Espace)",self._toggle_play),
            (self._btn_next,  "Frame suivant (→)",       self._go_next),
            (self._btn_last,  "Dernier frame (End)",     self._go_last),
        ]:
            btn.setFixedWidth(34)
            btn.setFixedHeight(28)
            btn.setToolTip(tip)
            btn.clicked.connect(cb)
            row1.addWidget(btn)

        self._btn_first.setText("|◀")
        self._btn_prev.setText("◀")
        self._btn_play.setText("▶")
        self._btn_next.setText("▶|")
        self._btn_last.setText("▶|")
        self._btn_last.setText("◀|")
        self._btn_last.setText("|▶")

        # Slider seek
        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setRange(0, 1000)
        self._slider.setValue(0)
        self._slider.sliderPressed.connect(lambda: setattr(self, "_seeking", True))
        self._slider.sliderReleased.connect(self._on_slider_released)
        row1.addWidget(self._slider, stretch=1)

        # Vitesse
        row1.addWidget(QLabel("Vitesse:"))
        self._speed_combo = QComboBox()
        for s in PLAYBACK_SPEEDS:
            self._speed_combo.addItem(f"{s}x")
        self._speed_combo.setCurrentIndex(self._speed_idx)
        self._speed_combo.setFixedWidth(64)
        self._speed_combo.currentIndexChanged.connect(self._on_speed_changed)
        row1.addWidget(self._speed_combo)

        ctrl_layout.addLayout(row1)

        # Ligne 2 : infos frame/temps
        row2 = QHBoxLayout()
        self._lbl_frame = QLabel("Frame: — / —")
        self._lbl_frame.setFont(QFont("Monospace", 9))
        self._lbl_time  = QLabel("— / —")
        self._lbl_time.setFont(QFont("Monospace", 9))
        row2.addWidget(self._lbl_frame)
        row2.addStretch()
        row2.addWidget(self._lbl_time)
        ctrl_layout.addLayout(row2)

        vbox.addWidget(ctrl_frame)

    def _build_dock(self):
        dock = QDockWidget("Réglages", self)
        dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable |
                         QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        dock.setMinimumWidth(280)

        # Tab widget regroupant : Détection / Préprocessing / Tracking
        tabs = QTabWidget()
        tabs.setTabPosition(QTabWidget.TabPosition.North)

        # Onglet 1 : DÉTECTION (contenu actuel : ArUco, rectif, calib caméra)
        tab_detect = QWidget()
        tab_detect_scroll = QScrollArea()
        tab_detect_scroll.setWidgetResizable(True)
        tab_detect_scroll.setWidget(tab_detect)
        layout = QVBoxLayout(tab_detect)
        layout.setSpacing(8)

        # ── Traitement ─────────────────────────────────────────────────────
        proc_group = QGroupBox("Traitement")
        proc_form = QFormLayout(proc_group)

        # Legacy undistort -- hidden (poly calibration handles distortion).
        self._chk_undistort = QCheckBox("Undistort (legacy)")
        self._chk_undistort.setChecked(False)
        self._chk_undistort.setVisible(False)
        self._chk_undistort.toggled.connect(self._trigger_rerender)
        # proc_form.addRow(self._chk_undistort)  # hidden

        self._chk_aruco = QCheckBox("Détection ArUco")
        self._chk_aruco.setChecked(True)
        self._chk_aruco.toggled.connect(self._trigger_rerender)
        proc_form.addRow(self._chk_aruco)

        self._chk_aruco_ids = QCheckBox("Afficher IDs")
        self._chk_aruco_ids.setChecked(True)
        self._chk_aruco_ids.toggled.connect(self._trigger_rerender)
        proc_form.addRow(self._chk_aruco_ids)

        self._chk_rejected = QCheckBox("Afficher rejetés (debug)")
        self._chk_rejected.setChecked(False)
        self._chk_rejected.toggled.connect(self._trigger_rerender)
        proc_form.addRow(self._chk_rejected)

        self._combo_dict = QComboBox()
        for name in ARUCO_DICTS:
            self._combo_dict.addItem(name)
        self._combo_dict.currentTextChanged.connect(self._on_dict_changed)
        proc_form.addRow("Dictionnaire:", self._combo_dict)

        layout.addWidget(proc_group)

        # ── Rectification table ─────────────────────────────────────────────
        rect_group = QGroupBox("Rectification table")
        rect_form = QFormLayout(rect_group)

        self._chk_rectify = QCheckBox("Activer la rectification")
        self._chk_rectify.setChecked(False)
        self._chk_rectify.toggled.connect(self._trigger_rerender)
        rect_form.addRow(self._chk_rectify)

        self._chk_grid = QCheckBox("Grille mm")
        self._chk_grid.setChecked(True)
        self._chk_grid.toggled.connect(self._trigger_rerender)
        rect_form.addRow(self._chk_grid)

        self._chk_table_overlay = QCheckBox("Contour table sur vue brute")
        self._chk_table_overlay.setChecked(True)
        self._chk_table_overlay.toggled.connect(self._trigger_rerender)
        rect_form.addRow(self._chk_table_overlay)

        # ── Grille de configuration des tags (ID + position mm) ─────────────
        tag_grid = QGridLayout()
        tag_grid.setSpacing(3)

        # Headers
        for col, txt in enumerate(["", "ID", "X (mm)", "Y (mm)"]):
            hdr = QLabel(txt)
            hdr.setStyleSheet("color: #888; font-size: 10px;")
            tag_grid.addWidget(hdr, 0, col)

        # Rangées : TL, TR, BR, BL
        self._anchor_spins: dict[str, dict] = {}   # {"TL": {"id":, "x":, "y":}}
        # 4 ArUco anchors at 600 mm from each table corner.
        defaults = {
            "TL": (23,  600,  600),
            "TR": (22, 2400,  600),
            "BR": (20, 2400, 1400),
            "BL": (21,  600, 1400),
        }
        for row_idx, (corner, (def_id, def_x, def_y)) in enumerate(defaults.items(), start=1):
            lbl = QLabel(corner)
            lbl.setStyleSheet("font-weight: bold; font-size: 10px;")
            tag_grid.addWidget(lbl, row_idx, 0)

            sp_id = QSpinBox()
            sp_id.setRange(0, 999); sp_id.setValue(def_id); sp_id.setFixedWidth(46)
            tag_grid.addWidget(sp_id, row_idx, 1)

            sp_x = QDoubleSpinBox()
            sp_x.setRange(0, TABLE_W_MM); sp_x.setValue(def_x)
            sp_x.setDecimals(0); sp_x.setSingleStep(10); sp_x.setFixedWidth(66)
            tag_grid.addWidget(sp_x, row_idx, 2)

            sp_y = QDoubleSpinBox()
            sp_y.setRange(0, TABLE_H_MM); sp_y.setValue(def_y)
            sp_y.setDecimals(0); sp_y.setSingleStep(10); sp_y.setFixedWidth(66)
            tag_grid.addWidget(sp_y, row_idx, 3)

            self._anchor_spins[corner] = {"id": sp_id, "x": sp_x, "y": sp_y}
            sp_id.valueChanged.connect(self._on_corner_ids_changed)
            sp_x.valueChanged.connect(self._on_corner_ids_changed)
            sp_y.valueChanged.connect(self._on_corner_ids_changed)

        rect_form.addRow(tag_grid)

        # Shortcuts pour le code legacy (référencés dans _load_settings)
        self._spin_tl = self._anchor_spins["TL"]["id"]
        self._spin_tr = self._anchor_spins["TR"]["id"]
        self._spin_br = self._anchor_spins["BR"]["id"]
        self._spin_bl = self._anchor_spins["BL"]["id"]

        layout.addWidget(rect_group)

        # Legacy chessboard/charuco calibration block -- hidden.
        # The polynomial table calibration (red mask) is sufficient :
        # it produces a direct pixel<->mm mapping that already captures the
        # camera's distortion. Keep the widgets alive for compatibility but
        # don't expose them in the UI.
        self._lbl_cal_status = QLabel("")
        self._lbl_cal_status.setVisible(False)

        # ── Calibration polynomiale table (sous-section) ───────────────────
        poly_group = QGroupBox("Calibration table polynomiale")
        poly_layout = QVBoxLayout(poly_group)

        self._lbl_poly_status = QLabel("Aucune calibration polynomiale chargée")
        self._lbl_poly_status.setStyleSheet("color: #888; font-size: 10px;")
        self._lbl_poly_status.setWordWrap(True)
        poly_layout.addWidget(self._lbl_poly_status)

        # Note : quand un polynome est charge, il est automatiquement utilise
        # pour la rectification et toutes les conversions pixel<->mm. La
        # correction live des 4 anchors ArUco compense la derive de la camera.
        # Plus de toggle manuel : un seul pipeline.
        info = QLabel(
            "Quand charge : utilise pour rectification + coords + tracking,\n"
            "avec correction live mm-space depuis les 4 anchors ArUco."
        )
        info.setStyleSheet("color: #777; font-size: 9px; font-style: italic;")
        info.setWordWrap(True)
        poly_layout.addWidget(info)
        # _chk_use_poly est maintenant le QAction "Polynomial actif" de la
        # toolbar (cree dans _build_toolbar). API identique : isChecked(),
        # setChecked(), setEnabled().

        poly_btn_row = QHBoxLayout()
        btn_load_poly = QPushButton("📥 Charger")
        btn_load_poly.clicked.connect(self._load_poly_calibration)
        btn_new_poly = QPushButton("🟥 Calibrer")
        btn_new_poly.clicked.connect(self._open_table_cal_dialog)
        poly_btn_row.addWidget(btn_load_poly)
        poly_btn_row.addWidget(btn_new_poly)
        poly_layout.addLayout(poly_btn_row)

        # Bake correction + extract intrinsics
        poly_advanced_row = QHBoxLayout()
        btn_bake = QPushButton("Baker correction")
        btn_bake.setToolTip(
            "Absorbe la correction live (rotation/translation) dans les "
            "coefficients du polynome. La rectif sera definitivement en "
            "ton repere monde, plus de cout runtime."
        )
        btn_bake.clicked.connect(self._on_bake_correction)
        btn_intrinsics = QPushButton("Extraire intrinsics camera")
        btn_intrinsics.setToolTip(
            "Fit K + dist depuis le polynome (~14 px RMS). Permet l'usage "
            "live de solvePnP sur les 4 anchors sans le polynome ni le mask."
        )
        btn_intrinsics.clicked.connect(self._on_extract_intrinsics)
        poly_advanced_row.addWidget(btn_bake)
        poly_advanced_row.addWidget(btn_intrinsics)
        poly_layout.addLayout(poly_advanced_row)

        # Refine polynomial with detected anchors
        btn_refine = QPushButton("Affiner avec anchors detectes")
        btn_refine.setToolTip(
            "Refit le polynome en combinant les contraintes du mask (bords) "
            "et les positions vraies des 4 anchors detectes. Donne un "
            "polynome qui satisfait BORD + ANCHORS simultanement."
        )
        btn_refine.clicked.connect(self._on_refine_with_anchors)
        poly_layout.addWidget(btn_refine)

        self._lbl_intrinsics = QLabel("")
        self._lbl_intrinsics.setStyleSheet(
            "color: #4ade80; font-size: 9px; font-family: monospace;"
        )
        self._lbl_intrinsics.setWordWrap(True)
        poly_layout.addWidget(self._lbl_intrinsics)

        layout.addWidget(poly_group)
        layout.addStretch()

        tabs.addTab(tab_detect_scroll, "🎯 Détection")

        # Onglet 2 : PRÉPROCESSING
        self._preprocess_panel = PreprocessPanel(self._preprocessor)
        self._preprocess_panel.changed.connect(self._on_preprocess_changed)
        pp_scroll = QScrollArea()
        pp_scroll.setWidgetResizable(True)
        pp_scroll.setWidget(self._preprocess_panel)
        tabs.addTab(pp_scroll, "🎚 Préprocessing")

        # Onglet 3 : TRACKING ROBOTS
        self._tracking_panel = TrackingPanel(self._tracker)
        self._tracking_panel.robots_changed.connect(self._on_robots_changed)
        self._tracking_panel.cam_changed.connect(self._on_cam_changed)
        self._tracking_panel.request_calibrate_from_known.connect(
            self._on_calibrate_from_known
        )
        self._tracking_panel.request_reset_trails.connect(self._tracker.reset_trails)
        tr_scroll = QScrollArea()
        tr_scroll.setWidgetResizable(True)
        tr_scroll.setWidget(self._tracking_panel)
        tabs.addTab(tr_scroll, "🤖 Tracking")

        # Onglet 4 : LIVE ARUCO (table temps reel)
        from ui.live_aruco_panel import LiveArucoPanel
        self._live_panel = LiveArucoPanel()
        tabs.addTab(self._live_panel, "📡 Live")

        dock.setWidget(tabs)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)

    def _build_statusbar(self):
        self._sb = QStatusBar()
        self.setStatusBar(self._sb)
        self._sb_source  = QLabel("Pas de source")
        self._sb_aruco   = QLabel("")
        self._sb_homog   = QLabel("")
        self._sb_robots  = QLabel("")
        self._sb_coords  = QLabel("")
        self._sb_pp      = QLabel("")
        for w in [self._sb_source, self._sb_aruco, self._sb_homog,
                  self._sb_robots, self._sb_pp, self._sb_coords]:
            self._sb.addWidget(w)
            self._sb.addPermanentWidget(QLabel("│"))

    # ══════════════════════════════════════════════════════════════════════════
    # Persistance settings
    # ══════════════════════════════════════════════════════════════════════════

    def _load_settings(self):
        try:
            with open(SETTINGS_FILE) as f:
                d = json.load(f)
            self._combo_dict.setCurrentText(d.get("aruco_dict", "4x4_50"))
            self._chk_aruco.setChecked(d.get("aruco_enabled", True))
            self._chk_undistort.setChecked(d.get("undistort_enabled", False))
            self._chk_rectify.setChecked(d.get("rectify_enabled", False))
            self._chk_grid.setChecked(d.get("grid_enabled", True))
            # Anchors
            if "anchors" in d:
                for corner, spins in self._anchor_spins.items():
                    a = d["anchors"].get(corner, {})
                    if a:
                        spins["id"].setValue(int(a.get("tag_id", spins["id"].value())))
                        spins["x"].setValue(float(a.get("x_mm", spins["x"].value())))
                        spins["y"].setValue(float(a.get("y_mm", spins["y"].value())))
            if "calibration" in d:
                try:
                    self._calibration = CalibrationData.from_dict(d["calibration"])
                    self._update_cal_status()
                except Exception:
                    pass
            # Préprocesseur
            if "preprocessing" in d:
                self._preprocessor.load_dict(d["preprocessing"])
                if hasattr(self, "_preprocess_panel"):
                    self._preprocess_panel.refresh_from_preproc()
            # Tracker
            if "tracker" in d:
                self._tracker.load_dict(d["tracker"])
                if hasattr(self, "_tracking_panel"):
                    self._tracking_panel.refresh_from_tracker()
            # Calibration polynomiale
            if "poly_calibration" in d and d["poly_calibration"]:
                try:
                    self._poly_cal = PolyCalibration.from_dict(d["poly_calibration"])
                    self._chk_use_poly.setEnabled(True)
                    self._chk_use_poly.setChecked(d.get("poly_calibration_use", True))
                    self._apply_poly_to_rectifier()
                    self._lbl_poly_status.setText(
                        f"✓ Polynôme deg {self._poly_cal.deg}, "
                        f"RMS={self._poly_cal.rms_mm:.2f} mm"
                    )
                    self._lbl_poly_status.setStyleSheet(
                        "color: #4ade80; font-size: 10px;"
                    )
                except Exception:
                    pass
            # Auto-load des intrinsics camera (ChArUco) si un chemin est sauve.
            ip = d.get("camera_intrinsics_path")
            if ip and os.path.exists(ip):
                try:
                    intr = CameraIntrinsics.load(ip)
                    self._intrinsics = intr
                    self._intrinsics_path = ip
                    self._rectifier.set_intrinsics(intr)
                    # Si on a des intrinsics ChArUco fiables, on coupe le poly
                    # par defaut pour utiliser le mode pose-based.
                    self._chk_use_poly.setChecked(False)
                    self._apply_poly_to_rectifier()
                except Exception:
                    pass
        except Exception:
            pass

    def _save_settings(self):
        anchors_data = {}
        for corner, spins in self._anchor_spins.items():
            anchors_data[corner] = {
                "tag_id": spins["id"].value(),
                "x_mm":   spins["x"].value(),
                "y_mm":   spins["y"].value(),
            }
        d = {
            "aruco_dict":        self._combo_dict.currentText(),
            "anchors":           anchors_data,
            "aruco_enabled":     self._chk_aruco.isChecked(),
            "undistort_enabled": self._chk_undistort.isChecked(),
            "rectify_enabled":   self._chk_rectify.isChecked(),
            "grid_enabled":      self._chk_grid.isChecked(),
            "preprocessing":     self._preprocessor.to_dict(),
            "tracker":           self._tracker.to_dict(),
        }
        if self._calibration is not None:
            d["calibration"] = self._calibration.to_dict()
        if self._poly_cal is not None:
            d["poly_calibration"]     = self._poly_cal.to_dict()
            d["poly_calibration_use"] = self._chk_use_poly.isChecked()
        # Persist le chemin du JSON intrinsics pour le recharger au demarrage.
        if self._intrinsics_path is not None:
            d["camera_intrinsics_path"] = self._intrinsics_path
        try:
            with open(SETTINGS_FILE, "w") as f:
                json.dump(d, f, indent=2)
        except Exception:
            pass

    def closeEvent(self, event):
        self._save_settings()
        if self._source:
            self._source.release()
        super().closeEvent(event)

    # ══════════════════════════════════════════════════════════════════════════
    # Ouverture de source
    # ══════════════════════════════════════════════════════════════════════════

    def _open_video(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Ouvrir une vidéo", "",
            "Vidéos (*.mp4 *.mkv *.avi *.mov *.ts);;Tous (*.*)"
        )
        if not path:
            return
        self._open_source(path)

    def _open_camera(self):
        # Pour l'instant on ouvre la caméra 0
        # TODO: dialog pour choisir l'index ou une URL RTSP
        self._open_source(0)

    def _open_source(self, source):
        self._stop()
        if self._source:
            self._source.release()

        vs = VideoSource(source)
        if not vs.open():
            QMessageBox.critical(self, "Erreur", f"Impossible d'ouvrir : {source}")
            return

        self._source = vs
        info = vs.info
        name = Path(info.source_name).name if not info.is_live else f"Caméra {source}"
        self.setWindowTitle(f"TwinVision — {name}")
        self._sb_source.setText(f"{'📹' if info.is_live else '🎬'} {name}  {info.width}×{info.height} @ {info.fps:.1f} fps")

        if not info.is_live:
            self._slider.setRange(0, info.frame_count - 1)
        else:
            self._slider.setRange(0, 0)

        self._update_controls_enabled()
        self._advance_frame()  # Afficher le premier frame

    # ══════════════════════════════════════════════════════════════════════════
    # Playback
    # ══════════════════════════════════════════════════════════════════════════

    def _toggle_play(self):
        if self._source is None:
            return
        if self._playing:
            self._stop()
        else:
            self._play()

    def _play(self):
        if self._source is None:
            return
        self._playing = True
        self._btn_play.setText("⏸")
        fps = self._source.info.fps if self._source.info else 30.0
        speed = PLAYBACK_SPEEDS[self._speed_idx]
        interval_ms = max(1, int(1000 / fps / speed))
        self._timer.start(interval_ms)

    def _stop(self):
        self._playing = False
        self._timer.stop()
        self._btn_play.setText("▶")

    def _on_timer(self):
        self._advance_frame()

    def _advance_frame(self):
        if self._source is None:
            return
        frame = self._source.read()
        if frame is None:
            # Fin de vidéo
            self._stop()
            return
        self._process_and_display(frame)
        self._update_frame_info()

    def _process_and_display(self, frame: np.ndarray):
        self._current_raw = frame.copy()

        working = frame.copy()

        # 1. Undistort
        if self._chk_undistort.isChecked() and self._calibration is not None:
            working = self._calibration.undistort(working)

        # 1.5. Preprocessing (CLAHE, blur, gamma...)
        active_stages = [k for k, v in self._preprocessor.opts.items() if v.get("on")]
        pp_active = bool(active_stages)
        if pp_active:
            working = self._preprocessor.apply(working)
            self._sb_pp.setText(f"PP: {', '.join(active_stages)}")
        else:
            self._sb_pp.setText("")
        self._current_processed = working

        # 2. Détection ArUco
        result: Optional[DetectionResult] = None
        if self._chk_aruco.isChecked():
            result = self._detector.detect(working)
            self._last_result = result
            n = result.count
            self._sb_aruco.setText(f"🏷 {n} tag{'s' if n != 1 else ''}")
        else:
            self._last_result = None
            self._sb_aruco.setText("")

        # 3. Mise a jour rectification :
        #    - Toujours faire l'update H 4-tags (utile pour solvePnP auto cam)
        #    - Si poly charge : recalculer la correction live mm-space depuis
        #      les 4 anchors detectes -> ajuste la calibration polynomiale au
        #      mouvement de la camera frame par frame.
        if result is not None and self._chk_rectify.isChecked():
            self._rectifier.update(result)
            self._update_poly_correction(result)
            # Pose-based rectification : per-frame solvePnP from 4 anchors.
            # Takes priority over poly when intrinsics are loaded.
            if self._rectifier.has_intrinsics:
                self._rectifier.update_pose(result)
            if self._rectifier.has_poly:
                if self._poly_cal is not None and self._poly_cal.has_correction:
                    rms = float(np.sqrt(np.mean(
                        np.array(self._poly_cal.correction_residuals_mm) ** 2
                    ))) if self._poly_cal.correction_residuals_mm else 0.0
                    self._sb_homog.setText(f"Poly+corr ({rms:.1f}mm)")
                else:
                    self._sb_homog.setText("Poly")
            elif not self._rectifier.has_homography:
                self._sb_homog.setText("H ✗")
            elif self._rectifier.homography_is_fresh:
                self._sb_homog.setText("H ✓")
            else:
                self._sb_homog.setText("H cache")
        elif not self._chk_rectify.isChecked():
            self._sb_homog.setText("")

        # 3.5. Tracking robots (parallaxe)
        h, w = working.shape[:2]
        if result is not None and self._rectifier.has_homography:
            self._tracker.process(result, self._rectifier, image_size=(w, h))
            n_robots = len(self._tracker.last_states)
            n_total = len(self._tracker.robots)
            self._sb_robots.setText(f"🤖 {n_robots}/{n_total}")
            if hasattr(self, "_tracking_panel"):
                self._tracking_panel.update_cam_status()
        else:
            self._sb_robots.setText("")

        # 3.6. Mise a jour panel live ArUco -- throttled to ~5 Hz to avoid
        #     QTableWidget rebuilds on every video frame (was causing lag spikes).
        if hasattr(self, "_live_panel"):
            import time as _t
            now = _t.monotonic()
            last = getattr(self, "_live_panel_last_update", 0.0)
            if now - last >= 0.20:   # 5 Hz
                self._live_panel.update_live(
                    result,
                    self._rectifier.config.anchors(),
                    self._tracker.robots,
                    self._rectifier,
                    self._tracker,
                )
                self._live_panel_last_update = now

        # 4. Vue brute : overlay ArUco + contour table + robots
        raw_display = working.copy()
        if result is not None:
            raw_display = self._detector.draw(
                raw_display, result,
                draw_ids=self._chk_aruco_ids.isChecked(),
                draw_rejected=self._chk_rejected.isChecked(),
                highlight_ids=[
                    self._spin_tl.value(), self._spin_tr.value(),
                    self._spin_br.value(), self._spin_bl.value(),
                ]
            )
        if self._chk_table_overlay.isChecked() and self._rectifier.has_homography:
            raw_display = self._rectifier.draw_overlay_on_raw(raw_display, result)

        # Robots sur la vue brute : croix au pixel détecté + label position mm
        _draw_robots_on_raw(raw_display, self._tracker, self._rectifier)

        # Visual indicator: red banner when preprocessing is active. Removes
        # all doubt about whether the pipeline is running (instead of relying
        # on subtle visual differences like CLAHE/median which the user might
        # not notice). Always shown when at least one stage is on.
        if pp_active:
            h, w = raw_display.shape[:2]
            cv2.rectangle(raw_display, (0, 0), (w - 1, h - 1), (0, 0, 255), 6)
            label = f"PREPROCESS ON: {', '.join(active_stages)}"
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2,
            )
            cv2.rectangle(raw_display, (8, 8), (8 + tw + 12, 8 + th + 12),
                          (0, 0, 0), -1)
            cv2.putText(raw_display, label, (14, 8 + th + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                        cv2.LINE_AA)

        self._view_raw.display_frame(raw_display)

        # 5. Vue rectifiée
        if self._chk_rectify.isChecked() and self._rectifier.has_homography:
            rect = self._rectifier.rectify(working)
            if rect is not None:
                if self._chk_grid.isChecked():
                    rect = self._rectifier.draw_grid(rect)
                # Robots overlay sur la rectifiée
                _draw_robots_on_rectified(rect, self._tracker, self._rectifier)
                # Indicateur visuel si H provient du cache (tag(s) occultés)
                if self._rectifier.homography_is_cached:
                    _draw_cached_banner(rect)
                self._view_rect.display_frame(rect)
        else:
            self._view_rect.display_frame(None)

        # Feed au dialog de calibration si ouvert
        if self._cal_dialog and self._cal_dialog.isVisible():
            self._cal_dialog.feed_frame(frame)
        if self._table_cal_dialog and self._table_cal_dialog.isVisible():
            self._table_cal_dialog.feed_frame(frame)
        if self._charuco_wizard and self._charuco_wizard.isVisible():
            self._charuco_wizard.feed_frame(frame)

    def _update_frame_info(self):
        if self._source is None or self._source.info is None:
            return
        info = self._source.info
        idx = self._source.current_frame_idx

        if not info.is_live:
            self._lbl_frame.setText(f"Frame: {idx:,} / {info.frame_count:,}")
            elapsed = idx / info.fps
            total = info.duration_s
            self._lbl_time.setText(
                f"{_fmt_time(elapsed)} / {_fmt_time(total)}"
            )
            if not self._seeking:
                self._slider.blockSignals(True)
                self._slider.setValue(idx)
                self._slider.blockSignals(False)
        else:
            self._lbl_frame.setText(f"Frame: {idx:,}")
            self._lbl_time.setText("Live")

    # ── Navigation ──────────────────────────────────────────────────────────

    def _go_first(self):
        if self._source:
            self._source.seek(0)
            self._advance_frame()

    def _go_last(self):
        if self._source and self._source.info:
            self._source.seek(self._source.info.frame_count - 1)
            self._advance_frame()

    def _go_prev(self):
        if self._source:
            self._source.seek(max(0, self._source.current_frame_idx - 2))
            self._advance_frame()

    def _go_next(self):
        if self._source:
            self._advance_frame()

    def _on_slider_released(self):
        self._seeking = False
        if self._source:
            self._source.seek(self._slider.value())
            self._advance_frame()

    def _on_speed_changed(self, idx: int):
        self._speed_idx = idx
        if self._playing:
            self._stop()
            self._play()

    # =====================================================================
    # Keyboard
    # =====================================================================

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key.Key_Space:
            self._toggle_play()
        elif key == Qt.Key.Key_Left:
            self._stop()
            self._go_prev()
        elif key == Qt.Key.Key_Right:
            self._stop()
            self._go_next()
        elif key == Qt.Key.Key_Home:
            self._stop()
            self._go_first()
        elif key == Qt.Key.Key_End:
            self._stop()
            self._go_last()
        else:
            super().keyPressEvent(event)

    # =====================================================================
    # Calibration camera (chessboard / charuco)
    # =====================================================================

    def _open_cal_dialog(self):
        if self._cal_dialog is None:
            self._cal_dialog = CalibrationDialog(self)
            self._cal_dialog.calibration_done.connect(self._on_calibration_done)
        if self._current_raw is not None:
            self._cal_dialog.feed_frame(self._current_raw)
        self._cal_dialog.show()
        self._cal_dialog.raise_()

    def _load_calibration(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Charger une calibration", "calibrations/",
            "JSON (*.json)"
        )
        if not path:
            return
        try:
            self._calibration = CalibrationData.load(path)
            self._update_cal_status()
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Impossible de charger :\n{e}")

    def _on_calibration_done(self, data: CalibrationData):
        self._calibration = data
        self._update_cal_status()

    def _update_cal_status(self):
        if self._calibration is None:
            self._lbl_cal_status.setText("Aucune calibration chargee")
            self._lbl_cal_status.setStyleSheet("color: #f87171; font-size: 10px;")
        else:
            c = self._calibration
            txt = (f"OK [{c.model_label}]  RMS={c.rms_error:.3f}px\n"
                   f"   {c.image_size[0]}x{c.image_size[1]}")
            if c.calibrated_at:
                txt += f"  {c.calibrated_at}"
            self._lbl_cal_status.setText(txt)
            self._lbl_cal_status.setStyleSheet("color: #4ade80; font-size: 10px;")

    # =====================================================================
    # Misc slots
    # =====================================================================

    def _on_dict_changed(self, name: str):
        self._detector.set_dictionary(name)
        self._trigger_rerender()

    def _on_corner_ids_changed(self):
        def _anchor(corner: str) -> TagAnchor:
            s = self._anchor_spins[corner]
            return TagAnchor(
                tag_id=s["id"].value(),
                x_mm=s["x"].value(),
                y_mm=s["y"].value(),
            )
        config = CornerConfig(
            top_left=     _anchor("TL"),
            top_right=    _anchor("TR"),
            bottom_right= _anchor("BR"),
            bottom_left=  _anchor("BL"),
        )
        # Visually flag duplicate IDs (rectif fails if 2 anchors share the same id)
        ids = [a.tag_id for a in config.anchors()]
        for corner, spins in self._anchor_spins.items():
            duplicate = ids.count(spins["id"].value()) > 1
            spins["id"].setStyleSheet(
                "background-color: #fbbf24; color: #000;" if duplicate else ""
            )
        self._rectifier.config = config
        self._request_save_settings()
        # Re-render so the change is visible immediately (when paused)
        self._trigger_rerender()

    def _on_raw_hover(self, x: int, y: int):
        if self._poly_cal is not None and self._chk_use_poly.isChecked():
            mm = self._poly_cal.pixel_to_mm(np.array([[x, y]], dtype=np.float64))[0]
            self._sb_coords.setText(
                f"Raw ({x}, {y}) -> Table-poly ({mm[0]:.0f}, {mm[1]:.0f}) mm"
            )
            return
        mm = self._rectifier.raw_pixel_to_table_mm(x, y)
        if mm:
            self._sb_coords.setText(f"Raw ({x}, {y}) -> Table ({mm[0]:.0f}, {mm[1]:.0f}) mm")
        else:
            self._sb_coords.setText(f"Raw ({x}, {y})")

    def _on_rect_hover(self, x: int, y: int):
        mm = self._rectifier.image_to_table_mm(x, y)
        if mm:
            self._sb_coords.setText(f"Table ({mm[0]:.0f} mm, {mm[1]:.0f} mm)")
        else:
            self._sb_coords.setText(f"Rect ({x}, {y})")

    # =====================================================================
    # Slots tracking & preprocess
    # =====================================================================

    def _on_preprocess_changed(self):
        self._request_save_settings()
        if self._current_raw is not None and not self._playing:
            self._process_and_display(self._current_raw)

    def _on_robots_changed(self):
        self._request_save_settings()
        if self._current_raw is not None and not self._playing:
            self._process_and_display(self._current_raw)

    def _on_cam_changed(self):
        self._request_save_settings()
        if self._current_raw is not None and not self._playing:
            self._process_and_display(self._current_raw)

    def _on_calibrate_from_known(self, tag_id: int, real_x: float, real_y: float):
        state = self._tracker.last_states.get(tag_id)
        if state is None:
            QMessageBox.warning(
                self, "Calibration",
                f"Tag {tag_id} non detecte sur ce frame. "
                "Avance jusqu'a un frame ou il est visible.",
            )
            return
        cfg = self._tracker.get_robot_config(tag_id)
        if cfg is None:
            return
        Zc = self._tracking_panel.sp_zc.value()
        z_object = cfg.z_mm
        cam = solve_camera_xy_from_known_position(
            state.naive_mm, (real_x, real_y), Zc, z_object,
        )
        if cam is None:
            QMessageBox.critical(
                self, "Calibration",
                f"Echec : Zc ({Zc:.0f}) trop proche de la hauteur du tag ({z_object:.0f}).",
            )
            return
        self._tracking_panel.set_camera_xyz((cam[0], cam[1], cam[2]))
        QMessageBox.information(
            self, "Calibration camera",
            f"Position camera calculee :\n"
            f"  Xc = {cam[0]:.0f} mm\n"
            f"  Yc = {cam[1]:.0f} mm\n"
            f"  Zc = {cam[2]:.0f} mm\n\n"
            f"Mode 'Manuel' active.",
        )

    # =====================================================================
    # Calibration polynomiale table
    # =====================================================================

    def _open_table_cal_dialog(self):
        if self._table_cal_dialog is None:
            self._table_cal_dialog = TableCalibDialog(
                self, current_frame=self._current_raw,
            )
            self._table_cal_dialog.calibration_done.connect(
                self._on_table_calibration_done
            )
        elif self._current_raw is not None:
            self._table_cal_dialog.feed_frame(self._current_raw)
        self._table_cal_dialog.show()
        self._table_cal_dialog.raise_()

    # =====================================================================
    # ChArUco camera intrinsics wizard (clean robotics workflow)
    # =====================================================================

    def _open_charuco_wizard(self):
        if self._charuco_wizard is None:
            self._charuco_wizard = CharucoWizardDialog(
                self, current_frame=self._current_raw,
            )
            self._charuco_wizard.intrinsics_done.connect(self._on_charuco_done)
        elif self._current_raw is not None:
            self._charuco_wizard.feed_frame(self._current_raw)
        self._charuco_wizard.show()
        self._charuco_wizard.raise_()

    def _on_charuco_done(self, intr):
        """Called when the ChArUco wizard finishes : K, dist are now available."""
        self._intrinsics = intr
        self._rectifier.set_intrinsics(intr)
        # Une fois les intrinsics charges, on peut basculer le rectif en mode
        # pose-based (solvePnP par frame sur les 4 anchors) -- on coupe donc
        # automatiquement le polynome.
        self._chk_use_poly.setChecked(False)
        self._apply_poly_to_rectifier()
        # ArUco DOIT etre actif : la pose se calcule live depuis les 4 anchors.
        # Sans ArUco -> pas de detection -> pas de pose -> rectif vide.
        if not self._chk_aruco.isChecked():
            self._chk_aruco.setChecked(True)
        QMessageBox.information(
            self, "ChArUco appliqué",
            f"Intrinsics chargés : RMS = {intr.rms_px:.2f} px, "
            f"fx = {intr.K[0,0]:.0f}, dist[:5] = {intr.dist.ravel()[:5].round(3).tolist()}.\n\n"
            "Le mode pose-based (live solvePnP sur 4 anchors) est maintenant actif.\n"
            "Le polynomial a été désactivé automatiquement."
        )
        self._request_save_settings()
        self._trigger_rerender()

    def _load_intrinsics_from_disk(self):
        """Load a previously saved camera_intrinsics.json (no wizard needed)."""
        default_dir = str(_PROJECT_ROOT / "calibrations")
        path, _ = QFileDialog.getOpenFileName(
            self, "Charger intrinsics camera",
            default_dir, "JSON (*.json);;Tous (*.*)",
        )
        if not path:
            return
        try:
            intr = CameraIntrinsics.load(path)
        except Exception as e:
            QMessageBox.critical(
                self, "Erreur",
                f"Impossible de charger : {type(e).__name__}: {e}",
            )
            return
        self._intrinsics_path = path
        self._on_charuco_done(intr)

    def _on_poly_action_toggled(self, checked: bool):
        """Toolbar toggle : enable/disable polynomial calibration usage."""
        self._apply_poly_to_rectifier()
        self._request_save_settings()
        if self._current_raw is not None and not self._playing:
            self._process_and_display(self._current_raw)

    def _load_poly_calibration(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Charger une calibration polynomiale", "calibrations/",
            "JSON (*.json)",
        )
        if not path:
            return
        try:
            cal = PolyCalibration.load(path)
        except Exception as e:
            QMessageBox.critical(
                self, "Erreur", f"Impossible de charger :\n{type(e).__name__}: {e}",
            )
            return
        self._on_table_calibration_done(cal)

    def _on_table_calibration_done(self, cal: PolyCalibration):
        self._poly_cal = cal
        self._chk_use_poly.setEnabled(True)
        self._chk_use_poly.setChecked(True)
        cal.invalidate_caches()
        self._apply_poly_to_rectifier()
        self._lbl_poly_status.setText(
            f"OK Polynome deg {cal.deg}, RMS={cal.rms_mm:.2f} mm "
            f"({cal.image_size[0]}x{cal.image_size[1]})"
        )
        self._lbl_poly_status.setStyleSheet("color: #4ade80; font-size: 10px;")
        self._chk_rectify.setChecked(True)
        if self._current_raw is not None and not self._playing:
            self._process_and_display(self._current_raw)

    def _on_poly_toggle(self, checked: bool):
        self._apply_poly_to_rectifier()
        self._save_settings()
        if self._current_raw is not None and not self._playing:
            self._process_and_display(self._current_raw)

    def _apply_poly_to_rectifier(self):
        if self._poly_cal is not None and self._chk_use_poly.isChecked():
            self._rectifier.set_poly(self._poly_cal)
        else:
            self._rectifier.set_poly(None)

    def _on_bake_correction(self):
        """Bake the live mm-space correction into the polynomial coefficients
        permanently. The rectif then runs in user-world without runtime cost."""
        if self._poly_cal is None:
            QMessageBox.information(
                self, "Baker correction",
                "Aucun polynome charge.",
            )
            return
        if not self._poly_cal.has_correction:
            QMessageBox.information(
                self, "Baker correction",
                "Pas de correction active. Place toi sur un frame ou les 4 "
                "anchors sont detectes pour generer la correction.",
            )
            return
        ok = self._poly_cal.bake_correction()
        if ok:
            self._poly_cal.invalidate_caches()
            QMessageBox.information(
                self, "Baker correction",
                "Correction bakee dans les coefficients du polynome.\n"
                "Le rectif est maintenant directement en repere monde.",
            )
            self._request_save_settings()
            self._trigger_rerender()
        else:
            QMessageBox.warning(
                self, "Baker correction",
                "Le baking a echoue (homographie degeneree).",
            )

    def _on_extract_intrinsics(self):
        """Run fit_intrinsics on the loaded polynomial calibration."""
        if self._poly_cal is None:
            QMessageBox.information(
                self, "Extraire intrinsics",
                "Aucun polynome charge.",
            )
            return
        try:
            K, dist, rms = self._poly_cal.fit_intrinsics()
        except Exception as e:
            QMessageBox.critical(
                self, "Extraire intrinsics",
                f"Echec : {type(e).__name__}: {e}",
            )
            return
        # Display nicely : focal lengths + distortion coefs
        lines = [
            f"Fit OK -- RMS = {rms:.1f} px",
            f"  fx = {K[0,0]:.0f}   fy = {K[1,1]:.0f}",
            f"  cx = {K[0,2]:.1f}   cy = {K[1,2]:.1f}",
            f"  dist = {dist.ravel()[:5].round(4).tolist()}",
        ]
        self._lbl_intrinsics.setText("\n".join(lines))
        self._request_save_settings()

    def _on_refine_with_anchors(self):
        """Refit the polynomial using mask correspondences (preserved via dense
        sampling of the current poly) plus the 4 detected ArUco anchors with
        high weight. Yields a polynomial that satisfies BORDER + ANCHORS at
        the same time."""
        if self._poly_cal is None:
            QMessageBox.information(
                self, "Affiner avec anchors",
                "Aucun polynome charge.",
            )
            return
        if self._last_result is None:
            QMessageBox.information(
                self, "Affiner avec anchors",
                "Aucune detection ArUco. Active la detection et place toi sur "
                "un frame ou les 4 anchors sont visibles.",
            )
            return
        # Collect the 4 anchor (pixel, mm) correspondences
        pix_pts = []
        mm_pts = []
        missing = []
        for a in self._rectifier.config.anchors():
            c = self._last_result.get_center_for_id(a.tag_id)
            if c is None:
                missing.append(a.tag_id)
                continue
            pix_pts.append([float(c[0]), float(c[1])])
            mm_pts.append([a.x_mm, a.y_mm])
        if missing:
            QMessageBox.warning(
                self, "Affiner avec anchors",
                f"Anchors manquants : {missing}. Tous les 4 doivent etre "
                "detectes pour le refit.",
            )
            return

        try:
            rms, mx = self._poly_cal.refine_with_anchors(
                np.array(pix_pts, dtype=np.float64),
                np.array(mm_pts, dtype=np.float64),
                weight=200,
            )
        except Exception as e:
            QMessageBox.critical(
                self, "Affiner avec anchors",
                f"Echec : {type(e).__name__}: {e}",
            )
            return

        # Clear live correction (the polynomial now fits the anchors directly)
        self._poly_cal.clear_live_correction()
        self._poly_cal.invalidate_caches()

        QMessageBox.information(
            self, "Affiner avec anchors",
            f"Refit OK.\n"
            f"  Residus aux 4 anchors : RMS = {rms:.2f} mm, max = {mx:.2f} mm\n"
            f"  La correction live a ete remise a zero (le polynome integre "
            f"deja les anchors).",
        )
        self._request_save_settings()
        self._trigger_rerender()

    def _update_poly_correction(self, result):
        """Compute the live mm-space homography correction from the 4 detected
        anchor tags. When all 4 anchors are visible, refit; when one or more
        is missing, keep the previous correction."""
        if self._poly_cal is None:
            return
        anchors = self._rectifier.config.anchors()
        mm_pred_list = []
        mm_true_list = []
        for a in anchors:
            center = result.get_center_for_id(a.tag_id)
            if center is None:
                return
            mm_p = self._poly_cal.pixel_to_mm_raw(
                np.array([[float(center[0]), float(center[1])]])
            )[0]
            mm_pred_list.append(mm_p.tolist())
            mm_true_list.append([a.x_mm, a.y_mm])
        self._poly_cal.update_live_correction(
            np.array(mm_pred_list, dtype=np.float64),
            np.array(mm_true_list, dtype=np.float64),
        )

    def _save_current_frame(self):
        if self._current_raw is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Sauver le frame", "frame.png", "Images (*.png *.jpg)",
        )
        if path:
            cv2.imwrite(path, self._current_raw)
            self._sb.showMessage(f"Frame sauve : {path}", 3000)

    def _update_controls_enabled(self):
        has = self._source is not None
        is_file = has and (self._source.info is not None) and not self._source.info.is_live
        for btn in [self._btn_play, self._btn_next, self._btn_prev,
                    self._btn_first, self._btn_last]:
            btn.setEnabled(has)
        self._slider.setEnabled(is_file)


# =========================================================================
# Helpers module-level
# =========================================================================

def _draw_cached_banner(image: np.ndarray) -> None:
    h, w = image.shape[:2]
    banner_h = 18
    overlay = image[:banner_h, :].copy()
    overlay[:] = (0, 100, 220)
    cv2.addWeighted(overlay, 0.55, image[:banner_h, :], 0.45, 0, image[:banner_h, :])
    cv2.putText(
        image, "H EN CACHE - tag(s) occulte(s)",
        (6, banner_h - 4),
        cv2.FONT_HERSHEY_SIMPLEX, 0.38,
        (255, 255, 255), 1, cv2.LINE_AA,
    )


def _fmt_time(s: float) -> str:
    s = int(s)
    m, sec = divmod(s, 60)
    h, m = divmod(m, 60)
    if h:
        return f"{h}:{m:02d}:{sec:02d}"
    return f"{m}:{sec:02d}"


def _draw_robots_on_raw(image, tracker, rectifier=None) -> None:
    import math
    for state in tracker.last_states.values():
        cx, cy = state.pixel_center
        if not (math.isfinite(cx) and math.isfinite(cy)):
            continue
        cx, cy = int(round(cx)), int(round(cy))
        col = state.color_bgr
        cv2.drawMarker(image, (cx, cy), col, cv2.MARKER_TILTED_CROSS, 22, 2)
        cv2.circle(image, (cx, cy), 14, col, 2)
        x_mm, y_mm = state.pos_mm
        # Heading arrow projetee depuis le 3D si on est en mode pose-based.
        # Sinon on l'omet sur la vue brute (pas precis sans pose).
        theta = state.theta_rad
        if (math.isfinite(theta) and rectifier is not None
                and rectifier.has_intrinsics
                and rectifier._pose_rvec is not None):
            try:
                # Cherche le z_mm du robot dans la config tracker
                z_mm = 0.0
                for cfg in tracker.robots:
                    if cfg.tag_id == state.tag_id:
                        z_mm = float(cfg.z_mm)
                        break
                L = 200.0  # longueur de la fleche en mm
                p_tip_3d = np.array([[[
                    float(x_mm) + L * math.cos(theta),
                    float(y_mm) + L * math.sin(theta),
                    z_mm,
                ]]], dtype=np.float32)
                p_base_3d = np.array([[[float(x_mm), float(y_mm), z_mm]]],
                                      dtype=np.float32)
                proj_tip, _ = cv2.projectPoints(
                    p_tip_3d, rectifier._pose_rvec, rectifier._pose_tvec,
                    rectifier._intrinsics.K, rectifier._intrinsics.dist,
                )
                proj_base, _ = cv2.projectPoints(
                    p_base_3d, rectifier._pose_rvec, rectifier._pose_tvec,
                    rectifier._intrinsics.K, rectifier._intrinsics.dist,
                )
                tx, ty = int(proj_tip[0, 0, 0]), int(proj_tip[0, 0, 1])
                bx, by = int(proj_base[0, 0, 0]), int(proj_base[0, 0, 1])
                cv2.arrowedLine(image, (bx, by), (tx, ty), col, 3,
                                tipLength=0.25)
            except Exception:
                pass
        # Label avec angle si dispo
        if math.isfinite(theta):
            theta_deg = math.degrees(theta)
            txt = f"{state.label} ({x_mm:.0f},{y_mm:.0f}) {theta_deg:+.0f}°"
        else:
            txt = f"{state.label} ({x_mm:.0f},{y_mm:.0f})"
        cv2.putText(image, txt, (cx + 18, cy - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, txt, (cx + 18, cy - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)


def _draw_robots_on_rectified(image: np.ndarray, tracker, rectifier) -> None:
    if not rectifier.has_homography:
        return

    def w2px(x_mm: float, y_mm: float):
        px, py = rectifier.table_mm_to_rectified(x_mm, y_mm)
        return int(px), int(py)

    h, w = image.shape[:2]
    for cfg in tracker.robots:
        trail = tracker.trails(cfg.tag_id)
        if len(trail) < 2:
            continue
        col = cfg.color_bgr
        for i in range(1, len(trail)):
            a = w2px(trail[i - 1][0], trail[i - 1][1])
            b = w2px(trail[i][0],     trail[i][1])
            alpha = i / len(trail)
            faded = tuple(int(c * alpha) for c in col)
            cv2.line(image, a, b, faded, 2)

    import math
    for state in tracker.last_states.values():
        x_mm, y_mm = state.pos_mm
        p = w2px(x_mm, y_mm)
        col = state.color_bgr
        cv2.circle(image, p, 5, col, -1)
        cv2.circle(image, p, 11, col, 2)
        # Heading arrow : longueur en mm convertie en pixels BEV.
        theta = state.theta_rad
        if math.isfinite(theta):
            L = 200.0  # mm
            tip_mm = (x_mm + L * math.cos(theta), y_mm + L * math.sin(theta))
            tip_px = w2px(tip_mm[0], tip_mm[1])
            cv2.arrowedLine(image, p, tip_px, col, 2, tipLength=0.25)
            theta_deg = math.degrees(theta)
            txt = f"{state.label} ({x_mm:.0f},{y_mm:.0f}) {theta_deg:+.0f}°"
        else:
            txt = f"{state.label} ({x_mm:.0f},{y_mm:.0f})"
        cv2.putText(image, txt, (p[0] + 13, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, txt, (p[0] + 13, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
        if not (0 <= p[0] < w and 0 <= p[1] < h):
            cx, cy = w // 2, h // 2
            qx = max(8, min(w - 8, p[0]))
            qy = max(8, min(h - 8, p[1]))
            cv2.arrowedLine(image, (cx, cy), (qx, qy), col, 2, tipLength=0.05)
