from __future__ import annotations

import sys

from PySide6.QtCore import QCoreApplication, Qt
from PySide6.QtWidgets import QApplication

from drone_mission_planner.common.logging_config import configure_logging
from drone_mission_planner.ui.main_window import MainWindow
from drone_mission_planner.ui.theme import APP_STYLESHEET


def create_application(argv: list[str] | None = None) -> QApplication:
    QCoreApplication.setOrganizationName("Drone Mission Planner")
    QCoreApplication.setApplicationName("Drone Mission Planner")
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )
    app = QApplication(argv if argv is not None else sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)
    return app


def main() -> int:
    configure_logging()
    app = create_application()
    window = MainWindow()
    window.show()
    window.map_view.reset_view()
    return app.exec()
