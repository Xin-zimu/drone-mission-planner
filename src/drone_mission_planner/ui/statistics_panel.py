from __future__ import annotations

from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from drone_mission_planner.simulation.reporting import SimulationReport


class StatisticsPanel(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        header = QHBoxLayout()
        self.summary = QLabel("No simulation data")
        self.summary.setStyleSheet("color: #9eacc0; font-weight: 600;")
        header.addWidget(self.summary, 2)
        header.addWidget(QLabel("Tasks"))
        self.task_progress = QProgressBar()
        self.task_progress.setRange(0, 1000)
        self.task_progress.setFixedWidth(160)
        header.addWidget(self.task_progress)
        header.addWidget(QLabel("Coverage"))
        self.coverage_progress = QProgressBar()
        self.coverage_progress.setRange(0, 1000)
        self.coverage_progress.setFixedWidth(160)
        header.addWidget(self.coverage_progress)
        layout.addLayout(header)
        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(
            ["Drone", "Status", "Distance", "Flight", "Wait", "Energy", "Battery", "Link"]
        )
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.table)

    def set_report(self, report: SimulationReport) -> None:
        self.summary.setText(
            f"T+{report.mission_time:.1f}s  •  {report.total_distance:.1f} m  •  "
            f"{report.total_energy_used:.1f} energy  •  {report.replan_count} replans  •  "
            f"{report.conflict_count} safety holds"
        )
        self.task_progress.setValue(round(report.completion_rate * 1000))
        self.task_progress.setFormat(f"{report.completed_tasks}/{report.total_tasks}  %p%")
        coverage = report.coverage[0].coverage if report.coverage else 0.0
        self.coverage_progress.setValue(round(coverage * 1000))
        self.coverage_progress.setFormat(f"{coverage:.1%}")
        self.table.setRowCount(len(report.drones))
        for row, drone in enumerate(report.drones):
            values = [
                drone.drone_id,
                drone.status.replace("_", " ").title(),
                f"{drone.distance_flown:.1f} m",
                f"{drone.flight_time:.1f} s",
                f"{drone.waiting_time:.1f} s",
                f"{drone.battery_used:.2f}",
                f"{drone.remaining_battery:.2f}",
                (
                    f"{drone.base_link} · {drone.hop_count} hop"
                    if drone.hop_count is not None
                    else drone.base_link
                ),
            ]
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(value))
        self.table.resizeColumnsToContents()
