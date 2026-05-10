"""
PreprocessPanel -- panneau de reglage du Preprocessor.

Le panneau modifie directement preprocessor.opts puis emet `changed`.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox,
    QDoubleSpinBox, QSpinBox, QFormLayout, QPushButton, QFrame,
)

from core.preprocessor import Preprocessor


class _Stage(QFrame):
    """One pipeline step : checkbox + parameters in a compact GroupBox."""

    changed = Signal()

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.NoFrame)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(2)

        self.chk = QCheckBox(title)
        self.chk.setStyleSheet("font-weight: bold;")
        # Use a no-arg lambda : QCheckBox.toggled emits a bool which would be
        # forwarded to Signal().emit(bool) -- some Qt bindings choke on the
        # extra arg and silently drop the emission.
        self.chk.toggled.connect(lambda _checked: self.changed.emit())
        outer.addWidget(self.chk)

        self.params = QFormLayout()
        self.params.setContentsMargins(16, 0, 0, 0)
        self.params.setSpacing(2)
        outer.addLayout(self.params)

    def add_double(self, label, default, vmin, vmax, step=0.1, decimals=2):
        sp = QDoubleSpinBox()
        sp.setRange(vmin, vmax); sp.setValue(default)
        sp.setSingleStep(step); sp.setDecimals(decimals); sp.setFixedWidth(80)
        sp.valueChanged.connect(lambda _val: self.changed.emit())
        self.params.addRow(label, sp)
        return sp

    def add_int(self, label, default, vmin, vmax, step=1):
        sp = QSpinBox()
        sp.setRange(vmin, vmax); sp.setValue(default)
        sp.setSingleStep(step); sp.setFixedWidth(80)
        sp.valueChanged.connect(lambda _val: self.changed.emit())
        self.params.addRow(label, sp)
        return sp


class PreprocessPanel(QWidget):
    """UI pour activer/configurer chaque etape du Preprocessor."""

    changed = Signal()

    def __init__(self, preprocessor: Preprocessor, parent=None):
        super().__init__(parent)
        self._pp = preprocessor
        self._building = True

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        self._stages: dict[str, _Stage] = {}
        self._w: dict[str, dict] = {}

        # Median blur
        s = _Stage("Median blur (impulse noise)")
        self._w["median_blur"] = {
            "ksize": s.add_int("ksize (odd):", 3, 3, 31, 2),
        }
        self._stages["median_blur"] = s; layout.addWidget(s)

        # Gaussian blur
        s = _Stage("Gaussian blur")
        self._w["gaussian_blur"] = {
            "sigma": s.add_double("sigma:", 1.5, 0.1, 10.0, 0.1, 2),
        }
        self._stages["gaussian_blur"] = s; layout.addWidget(s)

        # Bilateral
        s = _Stage("Bilateral (denoise)")
        self._w["bilateral"] = {
            "d": s.add_int("d:", 9, 3, 31, 2),
            "sigma_color": s.add_double("sigma color:", 75, 1, 200, 5, 1),
            "sigma_space": s.add_double("sigma space:", 75, 1, 200, 5, 1),
        }
        self._stages["bilateral"] = s; layout.addWidget(s)

        # NLM denoise
        s = _Stage("NLM denoise (lent mais puissant)")
        self._w["nlm_denoise"] = {
            "h": s.add_double("h (force):", 7, 1, 30, 0.5, 1),
            "template": s.add_int("template (odd):", 7, 3, 21, 2),
            "search": s.add_int("search (odd):", 21, 3, 51, 2),
        }
        self._stages["nlm_denoise"] = s; layout.addWidget(s)

        # Motion deblur
        s = _Stage("Motion deblur (Wiener)")
        self._w["motion_deblur"] = {
            "length": s.add_double("length px:", 12.0, 1, 50, 0.5, 1),
            "angle":  s.add_double("angle deg:", 0.0, -180, 180, 1, 1),
            "snr_db": s.add_double("SNR dB:", 25.0, 5, 50, 1, 1),
        }
        self._stages["motion_deblur"] = s; layout.addWidget(s)

        # Brightness/contrast
        s = _Stage("Brightness / Contrast")
        self._w["brightness"] = {
            "alpha": s.add_double("alpha (contrast):", 1.0, 0.3, 3.0, 0.05, 2),
            "beta":  s.add_double("beta (luminosite):", 0.0, -100, 100, 1, 1),
        }
        self._stages["brightness"] = s; layout.addWidget(s)

        # Gamma
        s = _Stage("Gamma")
        self._w["gamma"] = {
            "value": s.add_double("gamma:", 1.0, 0.3, 3.0, 0.05, 2),
        }
        self._stages["gamma"] = s; layout.addWidget(s)

        # CLAHE
        s = _Stage("CLAHE (egalisation locale)")
        self._w["clahe"] = {
            "clip": s.add_double("clip:", 3.0, 1.0, 10.0, 0.1, 2),
            "tile": s.add_int("tile:", 8, 2, 32, 1),
        }
        self._stages["clahe"] = s; layout.addWidget(s)

        # Unsharp
        s = _Stage("Unsharp mask (sharpen)")
        self._w["unsharp"] = {
            "amount": s.add_double("amount:", 1.0, 0.0, 3.0, 0.1, 2),
            "radius": s.add_double("radius:", 1.5, 0.3, 10.0, 0.1, 2),
        }
        self._stages["unsharp"] = s; layout.addWidget(s)

        # Edge enhance
        s = _Stage("Edge enhance (Sobel boost)")
        self._w["edge_enhance"] = {
            "amount": s.add_double("amount:", 0.5, 0.0, 2.0, 0.05, 2),
        }
        self._stages["edge_enhance"] = s; layout.addWidget(s)

        # Adaptive threshold
        s = _Stage("Adaptive threshold (binarisation)")
        self._w["adaptive_thresh"] = {
            "block": s.add_int("block (odd):", 31, 3, 99, 2),
            "C": s.add_int("C:", 5, -20, 30, 1),
        }
        self._stages["adaptive_thresh"] = s; layout.addWidget(s)

        # Grayscale
        s = _Stage("Grayscale (forcer)")
        self._w["grayscale"] = {}
        self._stages["grayscale"] = s; layout.addWidget(s)

        # Reset button
        btn_row = QHBoxLayout()
        btn_reset = QPushButton("Reset preprocessing")
        btn_reset.clicked.connect(self._on_reset)
        btn_row.addWidget(btn_reset)
        layout.addLayout(btn_row)

        layout.addStretch()

        # Wire stages
        for s in self._stages.values():
            s.changed.connect(self._on_any_changed)

        self._building = False
        self._pull_from_preproc()
        self._on_any_changed()

    # ---- Sync ----------------------------------------------------------

    def _pull_from_preproc(self):
        """Set widget values from preprocessor.opts."""
        self._building = True
        o = self._pp.opts
        for stage_name, widgets in self._w.items():
            if stage_name not in o:
                continue
            self._stages[stage_name].chk.setChecked(o[stage_name].get("on", False))
            for key, w in widgets.items():
                if key in o[stage_name]:
                    val = o[stage_name][key]
                    if isinstance(w, QSpinBox):
                        w.setValue(int(val))
                    else:
                        w.setValue(float(val))
        self._building = False

    def _push_to_preproc(self):
        o = self._pp.opts
        for stage_name, widgets in self._w.items():
            if stage_name not in o:
                continue
            o[stage_name]["on"] = self._stages[stage_name].chk.isChecked()
            for key, w in widgets.items():
                if isinstance(w, QSpinBox):
                    o[stage_name][key] = w.value()
                else:
                    o[stage_name][key] = w.value()

    # ---- Slots ---------------------------------------------------------

    def _on_any_changed(self):
        if self._building:
            return
        self._push_to_preproc()
        self.changed.emit()

    def _on_reset(self):
        self._pp.reset()
        self._pull_from_preproc()
        self.changed.emit()

    def refresh_from_preproc(self):
        self._pull_from_preproc()
