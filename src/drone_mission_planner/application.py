from __future__ import annotations

import logging
import sys
from types import TracebackType

from PySide6.QtCore import QCoreApplication, QStandardPaths, Qt, QTimer
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication, QMessageBox

from drone_mission_planner.app.workspace_state import WorkspaceState
from drone_mission_planner.common.logging_config import configure_logging
from drone_mission_planner.common.resources import resource_path
from drone_mission_planner.ui.main_window import MainWindow
from drone_mission_planner.ui.theme import APP_STYLESHEET

LOGGER = logging.getLogger(__name__)


def handle_unexpected_exception(
    exception_type: type[BaseException],
    exception: BaseException,
    traceback: TracebackType | None,
) -> None:
    LOGGER.critical("Unexpected application error", exc_info=(exception_type, exception, traceback))
    if QApplication.instance() is not None:
        QMessageBox.critical(
            QApplication.activeWindow(),
            "Unexpected error",
            "The operation could not be completed. Your project remains open.\n\n"
            f"{type(exception).__name__}: {exception}",
        )
    sys.__excepthook__(exception_type, exception, traceback)


def create_application(argv: list[str] | None = None) -> QApplication:
    QCoreApplication.setOrganizationName("Drone Mission Planner")
    QCoreApplication.setApplicationName("Drone Mission Planner")
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )
    app = QApplication(argv if argv is not None else sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(APP_STYLESHEET)
    app.setWindowIcon(QIcon(str(resource_path("assets/icons/drone-mission-planner.svg"))))
    return app


def main(argv: list[str] | None = None) -> int:
    application_argv = list(sys.argv if argv is None else argv)
    smoke_test = "--packaged-smoke-test" in application_argv
    application_argv = [arg for arg in application_argv if arg != "--packaged-smoke-test"]
    configure_logging()
    sys.excepthook = handle_unexpected_exception
    app = create_application(application_argv)
    state_root = QStandardPaths.writableLocation(
        QStandardPaths.StandardLocation.AppLocalDataLocation
    )
    state_store = WorkspaceState(state_root)
    window = MainWindow(state_store=state_store)
    window.show()
    window.map_view.reset_view()
    if smoke_test:
        QTimer.singleShot(250, app.quit)
    else:
        QTimer.singleShot(0, window.offer_recovery)
    return app.exec()
