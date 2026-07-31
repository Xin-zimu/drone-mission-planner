from __future__ import annotations

from dataclasses import fields
from enum import Enum
from typing import Any

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QLabel,
    QLineEdit,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.domain.models import MapObject

EDITABLE = {
    "name",
    "communication_range",
    "max_speed",
    "battery_capacity",
    "remaining_battery",
    "energy_per_meter",
    "payload_capacity",
    "current_payload",
    "safety_radius",
    "priority",
    "required_payload",
    "execution_duration",
}

DISPLAY_NAMES = {
    "id": "ID",
    "name": "Name",
    "position": "Position (m)",
    "status": "Status",
    "task_type": "Task type",
    "priority": "Priority",
    "communication_range": "Comm. range (m)",
    "max_speed": "Max speed (m/s)",
    "battery_capacity": "Battery capacity",
    "remaining_battery": "Battery remaining",
    "energy_per_meter": "Energy per metre",
    "payload_capacity": "Payload capacity (kg)",
    "current_payload": "Current payload (kg)",
    "safety_radius": "Safety radius (m)",
    "required_payload": "Required payload (kg)",
    "execution_duration": "Execution time (s)",
    "home_base_id": "Home base",
    "bounds": "Bounds (m)",
    "shape": "Shape",
}


class PropertyPanel(QScrollArea):
    property_changed = Signal(str, str, object)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setFrameShape(QScrollArea.Shape.NoFrame)
        self._container = QWidget()
        self._layout = QVBoxLayout(self._container)
        self._layout.setContentsMargins(14, 14, 14, 14)
        self._layout.setSpacing(10)
        self.setWidget(self._container)
        self.show_empty()

    def _clear(self) -> None:
        while self._layout.count():
            item = self._layout.takeAt(0)
            if item is None:
                continue
            widget = item.widget()
            if widget is not None:
                widget.hide()
                widget.deleteLater()

    def show_empty(self) -> None:
        self._clear()
        heading = QLabel("PROPERTIES")
        heading.setObjectName("SectionLabel")
        self._layout.addWidget(heading)
        hint = QLabel("Select a map object to inspect and edit its mission parameters.")
        hint.setWordWrap(True)
        hint.setObjectName("EmptyHint")
        self._layout.addWidget(hint)
        self._layout.addStretch()

    def set_object(self, item: MapObject) -> None:
        self._clear()
        heading = QLabel(type(item).__name__.replace("Station", " station").upper())
        heading.setObjectName("SectionLabel")
        self._layout.addWidget(heading)
        title = QLabel(item.name)
        title.setStyleSheet("font-size: 16pt; font-weight: 700; color: #f3f7ff;")
        self._layout.addWidget(title)
        form_widget = QWidget()
        form = QFormLayout(form_widget)
        form.setContentsMargins(0, 8, 0, 0)
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(10)
        for field in fields(item):
            if field.name in {"assigned_tasks", "planned_path", "points"}:
                continue
            value = getattr(item, field.name)
            label = DISPLAY_NAMES.get(field.name, field.name.replace("_", " ").title())
            editor = self._editor(item.id, field.name, value)
            form.addRow(label, editor)
        self._layout.addWidget(form_widget)
        self._layout.addStretch()

    def _editor(self, object_id: str, name: str, value: Any) -> QWidget:
        if name in EDITABLE and isinstance(value, str):
            edit = QLineEdit(value)
            edit.editingFinished.connect(
                lambda edit=edit: self.property_changed.emit(object_id, name, edit.text())
            )
            return edit
        if name in EDITABLE and isinstance(value, int) and not isinstance(value, bool):
            spin = QSpinBox()
            spin.setRange(0, 1_000_000)
            spin.setValue(value)
            spin.valueChanged.connect(
                lambda changed, object_id=object_id, name=name: self.property_changed.emit(
                    object_id, name, changed
                )
            )
            return spin
        if name in EDITABLE and isinstance(value, float):
            spin_float = QDoubleSpinBox()
            spin_float.setRange(0.0, 1_000_000.0)
            spin_float.setDecimals(3)
            spin_float.setValue(value)
            spin_float.valueChanged.connect(
                lambda changed, object_id=object_id, name=name: self.property_changed.emit(
                    object_id, name, changed
                )
            )
            return spin_float
        text = self._format_value(value)
        label = QLabel(text)
        label.setWordWrap(True)
        label.setStyleSheet("color: #9eacc0; padding: 6px 4px;")
        return label

    @staticmethod
    def _format_value(value: Any) -> str:
        if isinstance(value, Point):
            return f"{value.x:.1f}, {value.y:.1f}"
        if isinstance(value, Rect):
            return f"x={value.x:.1f}, y={value.y:.1f}\n{value.width:.1f} x {value.height:.1f}"
        if isinstance(value, Enum):
            return str(value.value).replace("_", " ").title()
        if value is None:
            return "—"
        return str(value)
