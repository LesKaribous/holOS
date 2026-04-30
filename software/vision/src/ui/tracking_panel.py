"""
TrackingPanel — panneau de configuration des robots suivis et de la position
caméra.

Sections :
    1. Robots         : ID + label + couleur + hauteur Z (mm) pour chaque robot.
                        Boutons +/− pour ajouter/supprimer un robot.
    2. Position cam   : mode auto/manuel, valeurs Xc/Yc/Zc.
    3. Calibrer cam   : depuis position connue d'un robot (inversion parallaxe).

Émet :
    robots_changed()                          : la liste des RobotConfig a changé
    cam_changed()                             : mode/override caméra a changé
    request_calibrate_from_known(tag_id, X, Y): l'utilisateur clique « calibrer »
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel, QGroupBox,
    QRadioButton, QButtonGroup, QSpinBox, QDoubleSpinBox, QPushButton,
    QGridLayout, QColorDialog, QFrame, QLineEdit, QComboBox, QMessageBox,
)

from core.robot_tracker import RobotTracker, RobotConfig


_DEFAULT_ROBOTS = [
    RobotConfig(tag_id=2, label="USER",     color_bgr=(0, 255, 0), z_mm=530.0),
    RobotConfig(tag_id=7, label="OPPONENT", color_bgr=(0, 0, 255), z_mm=530.0),
]


class _RobotRow(QFrame):
    """Une ligne de configuration robot (id, label, couleur, Z)."""

    changed = Signal()
    remove_requested = Signal(object)  # self

    def __init__(self, cfg: RobotConfig, parent=None):
        super().__init__(parent)
        self._cfg = cfg
        self.setFrameShape(QFrame.Shape.StyledPanel)

        layout = QGridLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(4)

        # Row 0: ID + label
        layout.addWidget(QLabel("ID"), 0, 0)
        self.sp_id = QSpinBox()
        self.sp_id.setRange(0, 999); self.sp_id.setValue(cfg.tag_id)
        self.sp_id.setFixedWidth(60)
        self.sp_id.valueChanged.connect(self._emit_changed)
        layout.addWidget(self.sp_id, 0, 1)

        layout.addWidget(QLabel("Label"), 0, 2)
        self.le_label = QLineEdit(cfg.label)
        self.le_label.setMaximumWidth(120)
        self.le_label.editingFinished.connect(self._emit_changed)
        layout.addWidget(self.le_label, 0, 3)

        # Couleur
        self.btn_color = QPushButton()
        self.btn_color.setFixedSize(28, 22)
        self._apply_color(cfg.color_bgr)
        self.btn_color.clicked.connect(self._pick_color)
        layout.addWidget(self.btn_color, 0, 4)

        # Bouton supprimer
        btn_del = QPushButton("✕")
        btn_del.setFixedSize(22, 22)
        btn_del.setToolTip("Supprimer ce robot")
        btn_del.clicked.connect(lambda: self.remove_requested.emit(self))
        layout.addWidget(btn_del, 0, 5)

        # Row 1: hauteur Z
        layout.addWidget(QLabel("Z (mm)"), 1, 0)
        self.sp_z = QDoubleSpinBox()
        self.sp_z.setRange(0, 2000); self.sp_z.setValue(cfg.z_mm)
        self.sp_z.setDecimals(0); self.sp_z.setSingleStep(10)
        self.sp_z.setFixedWidth(80)
        self.sp_z.valueChanged.connect(self._emit_changed)
        layout.addWidget(self.sp_z, 1, 1, 1, 2)

    def _apply_color(self, bgr: tuple[int, int, int]):
        b, g, r = bgr
        self.btn_color.setStyleSheet(
            f"background-color: rgb({r},{g},{b}); border: 1px solid #888;"
        )
        self._cfg.color_bgr = bgr

    def _pick_color(self):
        b, g, r = self._cfg.color_bgr
        col = QColorDialog.getColor(QColor(r, g, b), self, "Couleur du robot")
        if col.isValid():
            self._apply_color((col.blue(), col.green(), col.red()))
            self._emit_changed()

    def _emit_changed(self):
        self.changed.emit()

    def to_config(self) -> RobotConfig:
        return RobotConfig(
            tag_id=self.sp_id.value(),
            label=self.le_label.text() or f"ROBOT_{self.sp_id.value()}",
            color_bgr=self._cfg.color_bgr,
            z_mm=self.sp_z.value(),
        )


class TrackingPanel(QWidget):
    """Panneau de tracking (robots + cam)."""

    robots_changed = Signal()
    cam_changed = Signal()
    request_calibrate_from_known = Signal(int, float, float)  # tag_id, X_mm, Y_mm
    request_reset_trails = Signal()

    def __init__(self, tracker: RobotTracker, parent=None):
        super().__init__(parent)
        self._tracker = tracker
        self._building = False
        self._rows: list[_RobotRow] = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # ── Robots ──────────────────────────────────────────────────────────
        gb = QGroupBox("Robots suivis")
        gv = QVBoxLayout(gb)
        gv.setSpacing(2)

        self._robots_container = QVBoxLayout()
        self._robots_container.setSpacing(2)
        gv.addLayout(self._robots_container)

        btn_row = QHBoxLayout()
        btn_add = QPushButton("➕ Ajouter")
        btn_add.clicked.connect(self._on_add_robot)
        btn_row.addWidget(btn_add)
        btn_clear = QPushButton("⟲ Reset trails")
        btn_clear.clicked.connect(self.request_reset_trails.emit)
        btn_row.addWidget(btn_clear)
        gv.addLayout(btn_row)

        layout.addWidget(gb)

        # ── Caméra ──────────────────────────────────────────────────────────
        gb2 = QGroupBox("Position caméra (mm)")
        gf = QFormLayout(gb2)

        # Mode
        self.rb_auto = QRadioButton("Auto (solvePnP)")
        self.rb_manual = QRadioButton("Manuel (override)")
        self.rb_manual.setChecked(True)
        bg = QButtonGroup(self)
        bg.addButton(self.rb_auto)
        bg.addButton(self.rb_manual)
        self.rb_auto.toggled.connect(self._emit_cam_changed)
        self.rb_manual.toggled.connect(self._emit_cam_changed)

        mode_row = QHBoxLayout()
        mode_row.addWidget(self.rb_auto)
        mode_row.addWidget(self.rb_manual)
        mode_row.addStretch()
        gf.addRow("Mode :", _wrap_layout(mode_row))

        # Xc, Yc, Zc
        self.sp_xc = QDoubleSpinBox()
        self.sp_xc.setRange(-5000, 8000); self.sp_xc.setValue(275.0)
        self.sp_xc.setDecimals(0); self.sp_xc.setSingleStep(10); self.sp_xc.setSuffix(" mm")
        self.sp_xc.valueChanged.connect(self._emit_cam_changed)
        gf.addRow("Xc :", self.sp_xc)

        self.sp_yc = QDoubleSpinBox()
        self.sp_yc.setRange(-5000, 8000); self.sp_yc.setValue(-200.0)
        self.sp_yc.setDecimals(0); self.sp_yc.setSingleStep(10); self.sp_yc.setSuffix(" mm")
        self.sp_yc.valueChanged.connect(self._emit_cam_changed)
        gf.addRow("Yc :", self.sp_yc)

        self.sp_zc = QDoubleSpinBox()
        self.sp_zc.setRange(100, 5000); self.sp_zc.setValue(1000.0)
        self.sp_zc.setDecimals(0); self.sp_zc.setSingleStep(10); self.sp_zc.setSuffix(" mm")
        self.sp_zc.valueChanged.connect(self._emit_cam_changed)
        gf.addRow("Zc :", self.sp_zc)

        self.lbl_cam_status = QLabel("—")
        self.lbl_cam_status.setStyleSheet("color: #888; font-size: 10px;")
        self.lbl_cam_status.setWordWrap(True)
        gf.addRow(self.lbl_cam_status)

        layout.addWidget(gb2)

        # ── Calibrer caméra depuis position connue d'un robot ──────────────
        gb3 = QGroupBox("Calibrer la caméra (depuis robot connu)")
        gf3 = QFormLayout(gb3)

        self.cb_calib_id = QComboBox()
        self.cb_calib_id.setEditable(True)
        gf3.addRow("Tag robot :", self.cb_calib_id)

        self.sp_calib_x = QDoubleSpinBox()
        self.sp_calib_x.setRange(-5000, 8000); self.sp_calib_x.setValue(150.0)
        self.sp_calib_x.setDecimals(0); self.sp_calib_x.setSingleStep(10); self.sp_calib_x.setSuffix(" mm")
        gf3.addRow("X réel :", self.sp_calib_x)

        self.sp_calib_y = QDoubleSpinBox()
        self.sp_calib_y.setRange(-5000, 8000); self.sp_calib_y.setValue(2850.0)
        self.sp_calib_y.setDecimals(0); self.sp_calib_y.setSingleStep(10); self.sp_calib_y.setSuffix(" mm")
        gf3.addRow("Y réel :", self.sp_calib_y)

        btn_calib = QPushButton("Calculer (Xc, Yc) à partir de cette position")
        btn_calib.clicked.connect(self._on_calibrate_clicked)
        gf3.addRow(btn_calib)

        info = QLabel("Zc et hauteur Z du robot proviennent des champs ci-dessus.")
        info.setStyleSheet("color: #888; font-size: 10px;")
        info.setWordWrap(True)
        gf3.addRow(info)

        layout.addWidget(gb3)
        layout.addStretch()

        # Initial state
        self._building = True
        if not tracker.robots:
            tracker.set_robots([
                RobotConfig(tag_id=r.tag_id, label=r.label,
                            color_bgr=r.color_bgr, z_mm=r.z_mm)
                for r in _DEFAULT_ROBOTS
            ])
        self._rebuild_robot_rows()
        self._pull_camera_from_tracker()
        self._refresh_calib_combo()
        self._building = False

    # ── Robot rows ─────────────────────────────────────────────────────────

    def _clear_robot_rows(self):
        while self._robots_container.count():
            item = self._robots_container.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._rows.clear()

    def _rebuild_robot_rows(self):
        self._clear_robot_rows()
        for cfg in self._tracker.robots:
            row = _RobotRow(cfg)
            row.changed.connect(self._on_row_changed)
            row.remove_requested.connect(self._on_row_remove)
            self._robots_container.addWidget(row)
            self._rows.append(row)
        self._refresh_calib_combo()

    def _refresh_calib_combo(self):
        cur = self.cb_calib_id.currentText()
        self.cb_calib_id.blockSignals(True)
        self.cb_calib_id.clear()
        for cfg in self._tracker.robots:
            self.cb_calib_id.addItem(f"{cfg.tag_id}  ({cfg.label})", cfg.tag_id)
        # Restaurer
        idx = self.cb_calib_id.findText(cur)
        if idx >= 0:
            self.cb_calib_id.setCurrentIndex(idx)
        self.cb_calib_id.blockSignals(False)

    def _on_add_robot(self):
        used = {cfg.tag_id for cfg in self._tracker.robots}
        new_id = next((i for i in range(0, 100) if i not in used), 99)
        new = RobotConfig(
            tag_id=new_id, label=f"ROBOT_{new_id}",
            color_bgr=(255, 255, 0), z_mm=530.0,
        )
        self._tracker.set_robots(self._tracker.robots + [new])
        self._rebuild_robot_rows()
        self.robots_changed.emit()

    def _on_row_remove(self, row: _RobotRow):
        cfgs = [r.to_config() for r in self._rows if r is not row]
        self._tracker.set_robots(cfgs)
        self._rebuild_robot_rows()
        self.robots_changed.emit()

    def _on_row_changed(self):
        if self._building:
            return
        self._tracker.set_robots([r.to_config() for r in self._rows])
        self._refresh_calib_combo()
        self.robots_changed.emit()

    # ── Cam ────────────────────────────────────────────────────────────────

    def _pull_camera_from_tracker(self):
        self._building = True
        if self._tracker.cam_mode == "auto":
            self.rb_auto.setChecked(True)
        else:
            self.rb_manual.setChecked(True)
        if self._tracker.cam_override is not None:
            x, y, z = self._tracker.cam_override
            self.sp_xc.setValue(float(x))
            self.sp_yc.setValue(float(y))
            self.sp_zc.setValue(float(z))
        self._building = False

    def _push_camera_to_tracker(self):
        self._tracker.set_cam_mode("auto" if self.rb_auto.isChecked() else "manual")
        self._tracker.set_cam_override(
            (self.sp_xc.value(), self.sp_yc.value(), self.sp_zc.value())
        )

    def _emit_cam_changed(self):
        if self._building:
            return
        self._push_camera_to_tracker()
        self.cam_changed.emit()

    def update_cam_status(self):
        """À appeler après chaque frame pour rafraîchir le label de statut."""
        cs = self._tracker.cam_source
        cp = self._tracker.cam_pos
        labels = {
            "manual":   ("✓ Manuel",   "#4ade80"),
            "auto_K":   ("✓ Auto (K cal.)", "#4ade80"),
            "auto_fx":  ("⚠ Auto (fx estimé)", "#fbbf24"),
            "none":     ("✗ Indisponible", "#f87171"),
        }
        label, color = labels.get(cs, ("?", "#888"))
        if cp is not None:
            txt = f"{label}  ({cp[0]:.0f}, {cp[1]:.0f}, {cp[2]:.0f})"
        else:
            txt = label
        self.lbl_cam_status.setText(txt)
        self.lbl_cam_status.setStyleSheet(f"color: {color}; font-size: 10px;")

    def set_camera_xyz(self, xyz: tuple[float, float, float]):
        """Force les valeurs des spinbox (utilisé après calibrate-from-known)."""
        self._building = True
        self.sp_xc.setValue(float(xyz[0]))
        self.sp_yc.setValue(float(xyz[1]))
        self.sp_zc.setValue(float(xyz[2]))
        self.rb_manual.setChecked(True)
        self._building = False
        self._push_camera_to_tracker()
        self.cam_changed.emit()

    def _on_calibrate_clicked(self):
        if self.cb_calib_id.count() == 0:
            QMessageBox.warning(self, "Calibration", "Aucun robot configuré.")
            return
        try:
            tag_id = int(self.cb_calib_id.currentData()
                         if self.cb_calib_id.currentData() is not None
                         else self.cb_calib_id.currentText().split()[0])
        except Exception:
            QMessageBox.warning(self, "Calibration", "ID robot invalide.")
            return
        self.request_calibrate_from_known.emit(
            tag_id, self.sp_calib_x.value(), self.sp_calib_y.value(),
        )

    # ── Sync externe ───────────────────────────────────────────────────────

    def refresh_from_tracker(self):
        """Rafraîchit l'UI depuis l'état du tracker (ex: après load settings)."""
        self._building = True
        self._rebuild_robot_rows()
        self._pull_camera_from_tracker()
        self._building = False


def _wrap_layout(layout) -> QWidget:
    w = QWidget()
    w.setLayout(layout)
    return w
