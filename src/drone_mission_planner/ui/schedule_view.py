"""Read-only mission Gantt (plan §10.8).

One row per aircraft, with a planned lane and an actual lane. Planned bars come
from ``AssignmentResult.schedule``; actual bars come from the engine's task
timelines (waiting, service). Clicking a bar only emits ``task_selected`` — the
view never writes to the project, and there is no drag or edit affordance.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QMouseEvent, QPainter, QPaintEvent, QPen
from PySide6.QtWidgets import QWidget

from drone_mission_planner.domain.models import MissionTask
from drone_mission_planner.planning.scheduling import ScheduleResult
from drone_mission_planner.simulation.task_timeline import TaskTimeline

PLANNED_COLOR = QColor("#3f7fd6")
TRAVEL_COLOR = QColor("#5c8fd6")
WAIT_COLOR = QColor("#d9a13b")
SERVICE_COLOR = QColor("#3f9d63")
BLOCKED_COLOR = QColor("#b45353")
GRID_COLOR = QColor("#c9d3e2")
TEXT_COLOR = QColor("#172033")

LANE_PLANNED = "planned"
LANE_TRAVEL = "travel"
LANE_WAIT = "wait"
LANE_SERVICE = "service"


@dataclass(frozen=True, slots=True)
class ScheduleBar:
    """One drawn interval; ``kind`` is one of the LANE_* values."""

    task_id: str
    drone_id: str
    kind: str
    start: float
    finish: float
    label: str = ""

    @property
    def duration(self) -> float:
        return max(0.0, self.finish - self.start)


class ScheduleView(QWidget):
    """Read-only Gantt of planned and actual mission times."""

    task_selected = Signal(str)

    ROW_HEIGHT = 46
    HEADER_HEIGHT = 26
    LEFT_MARGIN = 96
    RIGHT_MARGIN = 18

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(220)
        self.setMouseTracking(True)
        self._bars: list[ScheduleBar] = []
        self._rows: list[str] = []
        self._max_time = 1.0
        self._hit_boxes: list[tuple[QRectF, str]] = []
        self._empty_reason = "No schedule yet"

    # --- read-only inspection helpers (used by tests and tooltips) ---------
    @property
    def read_only(self) -> bool:
        return True

    def bars(self) -> tuple[ScheduleBar, ...]:
        return tuple(self._bars)

    def row_count(self) -> int:
        return len(self._rows)

    def empty_reason(self) -> str:
        return self._empty_reason

    def bar_rect(self, task_id: str, kind: str) -> QRectF | None:
        for rect, hit_id in self._hit_boxes:
            if hit_id == f"{task_id}:{kind}":
                return rect
        return None

    # --- data --------------------------------------------------------------
    def clear(self, reason: str = "No schedule yet") -> None:
        self._bars = []
        self._rows = []
        self._max_time = 1.0
        self._hit_boxes = []
        self._empty_reason = reason
        self.update()

    def set_schedule(
        self,
        schedule: ScheduleResult | None,
        timelines: Sequence[TaskTimeline] = (),
        tasks: Mapping[str, MissionTask] | None = None,
        drone_ids: Sequence[str] = (),
    ) -> None:
        """Attach a plan and/or actual timelines; nothing is mutated."""

        tasks = tasks or {}
        names = {task_id: task.name for task_id, task in tasks.items()}
        bars: list[ScheduleBar] = []
        rows: list[str] = list(drone_ids)

        if schedule is not None:
            for entry in schedule.entries:
                if entry.status != "scheduled" or entry.drone_id is None:
                    continue
                bars.append(
                    ScheduleBar(
                        entry.task_id,
                        entry.drone_id,
                        LANE_PLANNED,
                        entry.start_time,
                        entry.finish_time,
                        names.get(entry.task_id, entry.task_id),
                    )
                )
                if entry.drone_id not in rows:
                    rows.append(entry.drone_id)

        for timeline in timelines:
            if timeline.drone_id is None:
                continue
            if timeline.drone_id not in rows:
                rows.append(timeline.drone_id)
            label = names.get(timeline.task_id, timeline.task_id)
            if (
                timeline.wait_started_at is not None
                and timeline.wait_ended_at is not None
                and timeline.wait_ended_at > timeline.wait_started_at
            ):
                bars.append(
                    ScheduleBar(
                        timeline.task_id,
                        timeline.drone_id,
                        LANE_WAIT,
                        timeline.wait_started_at,
                        timeline.wait_ended_at,
                        f"{label} waits",
                    )
                )
            if (
                timeline.arrived_at is not None
                and timeline.service_started_at is not None
                and timeline.service_started_at > timeline.arrived_at
            ):
                bars.append(
                    ScheduleBar(
                        timeline.task_id,
                        timeline.drone_id,
                        LANE_TRAVEL,
                        timeline.arrived_at,
                        timeline.service_started_at,
                        f"{label} holds",
                    )
                )
            if (
                timeline.service_started_at is not None
                and timeline.service_finished_at is not None
            ):
                bars.append(
                    ScheduleBar(
                        timeline.task_id,
                        timeline.drone_id,
                        LANE_SERVICE,
                        timeline.service_started_at,
                        timeline.service_finished_at,
                        label,
                    )
                )

        self._bars = bars
        self._rows = rows
        self._max_time = max((bar.finish for bar in bars), default=1.0)
        self._empty_reason = "No schedule yet"
        self.update()

    # --- painting ----------------------------------------------------------
    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#fbfcfe"))
        self._hit_boxes = []
        if not self._bars:
            painter.setPen(QPen(TEXT_COLOR))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self._empty_reason)
            return
        self._draw_time_axis(painter)
        for index, drone_id in enumerate(self._rows):
            top = self.HEADER_HEIGHT + index * self.ROW_HEIGHT
            self._draw_row(painter, drone_id, top)
        painter.end()

    def _draw_time_axis(self, painter: QPainter) -> None:
        width = max(1.0, self.width() - self.LEFT_MARGIN - self.RIGHT_MARGIN)
        step = _axis_step(self._max_time)
        painter.setPen(QPen(GRID_COLOR, 1))
        tick = 0.0
        while tick <= self._max_time + step:
            x = self.LEFT_MARGIN + width * (tick / max(self._max_time, 1e-9))
            painter.drawLine(QPointF(x, self.HEADER_HEIGHT), QPointF(x, self.height()))
            painter.setPen(QPen(TEXT_COLOR))
            painter.drawText(QPointF(x + 3.0, self.HEADER_HEIGHT - 8.0), f"{tick:.0f}s")
            painter.setPen(QPen(GRID_COLOR, 1))
            tick += step

    def _draw_row(self, painter: QPainter, drone_id: str, top: float) -> None:
        width = max(1.0, self.width() - self.LEFT_MARGIN - self.RIGHT_MARGIN)
        painter.setPen(QPen(TEXT_COLOR))
        painter.drawText(QPointF(8.0, top + self.ROW_HEIGHT / 2), drone_id)
        planned_y = top + 8.0
        actual_y = top + 26.0
        for bar in self._bars:
            if bar.drone_id != drone_id:
                continue
            x = self.LEFT_MARGIN + width * (bar.start / max(self._max_time, 1e-9))
            w = max(2.0, width * (bar.duration / max(self._max_time, 1e-9)))
            if bar.kind == LANE_PLANNED:
                rect = QRectF(x, planned_y, w, 10.0)
                color = PLANNED_COLOR
            elif bar.kind == LANE_WAIT:
                rect = QRectF(x, actual_y, w, 10.0)
                color = WAIT_COLOR
            elif bar.kind == LANE_SERVICE:
                rect = QRectF(x, actual_y, w, 10.0)
                color = SERVICE_COLOR
            else:
                rect = QRectF(x, actual_y, w, 10.0)
                color = TRAVEL_COLOR
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(color)
            painter.drawRoundedRect(rect, 3.0, 3.0)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            self._hit_boxes.append((rect, f"{bar.task_id}:{bar.kind}"))

    # --- interaction -------------------------------------------------------
    def mousePressEvent(self, event: QMouseEvent) -> None:
        position = event.position()
        for rect, hit_id in reversed(self._hit_boxes):
            if rect.adjusted(-2.0, -4.0, 2.0, 4.0).contains(position):
                task_id = hit_id.split(":", 1)[0]
                self.task_selected.emit(task_id)
                return
        super().mousePressEvent(event)


def _axis_step(max_time: float) -> float:
    for candidate in (5.0, 10.0, 20.0, 30.0, 60.0, 120.0, 300.0, 600.0):
        if max_time / candidate <= 12.0:
            return candidate
    return max_time / 12.0
