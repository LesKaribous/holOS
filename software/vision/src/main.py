"""
TwinVision — Application de test d'algorithmes de vision pour robot de compétition.
Point d'entrée principal.

Usage :
    python src/main.py
    python src/main.py video.mkv        # ouvre directement un fichier
    python src/main.py --camera 0       # ouvre la caméra 0
"""

import sys
import os

# Ajout du répertoire src au path pour les imports relatifs
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt
from PySide6.QtGui import QPalette, QColor

from ui.main_window import MainWindow


def apply_dark_palette(app: QApplication):
    """Thème sombre custom."""
    palette = QPalette()
    dark   = QColor(30, 30, 46)
    mid    = QColor(49, 50, 68)
    text   = QColor(205, 214, 244)
    accent = QColor(137, 180, 250)
    hl     = QColor(69, 71, 90)

    palette.setColor(QPalette.ColorRole.Window,          dark)
    palette.setColor(QPalette.ColorRole.WindowText,      text)
    palette.setColor(QPalette.ColorRole.Base,            QColor(24, 24, 37))
    palette.setColor(QPalette.ColorRole.AlternateBase,   mid)
    palette.setColor(QPalette.ColorRole.ToolTipBase,     mid)
    palette.setColor(QPalette.ColorRole.ToolTipText,     text)
    palette.setColor(QPalette.ColorRole.Text,            text)
    palette.setColor(QPalette.ColorRole.Button,          mid)
    palette.setColor(QPalette.ColorRole.ButtonText,      text)
    palette.setColor(QPalette.ColorRole.BrightText,      QColor(255, 85, 85))
    palette.setColor(QPalette.ColorRole.Highlight,       accent)
    palette.setColor(QPalette.ColorRole.HighlightedText, dark)
    palette.setColor(QPalette.ColorRole.Link,            accent)

    app.setPalette(palette)


def main():
    # HiDPI
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication(sys.argv)
    app.setApplicationName("TwinVision")
    app.setOrganizationName("CDR")
    app.setStyle("Fusion")
    apply_dark_palette(app)

    window = MainWindow()
    window.show()

    # Ouvrir une source depuis la ligne de commande
    args = sys.argv[1:]
    if args:
        if args[0] == "--camera" and len(args) >= 2:
            window._open_source(int(args[1]))
        elif not args[0].startswith("-"):
            window._open_source(args[0])

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
