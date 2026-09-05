"""Waypoints tab: inspect and edit the three-dimensional route of each drone.

The panel renders every drone waypoint as a table row. Editing is limited to
altitude, altitude mode, speed, action, and hold time; structural changes go
through the guarded delete request. All mutations are applied by
:class:`~drone_mission_planner.app.project_service.ProjectService`, which
validates and rolls back rejected edits.
"""

from __future__ import annotations

from collections.abc import Sequence

from PySide6.QtCore import (
    QAbstractItemModel,
    QModelIndex,
    QPersistentModelIndex,
    Qt,
    Signal,
)
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QHBoxLayout,
    QPushButton,
    QStyledItemDelegate,
    QStyleOptionViewItem,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from drone_mission_planner.domain.enums import AltitudeMode, WaypointAction
from drone_mission_planner.domain.models import Drone, MapModel
from drone_mission_planner.domain.terrain import TerrainModel
from drone_mission_planner.domain.waypoint import Waypoint

COLUMNS = (
    "Index",
    "Drone",
    "X (m)",
    "Y (m)",
    "Altitude (m)",
    "Mode",
    "Speed (m/s)",
    "Action",
    "Hold (s)",
    "Task",
)

_COLUMN_FIELDS: dict[int, str] = {
    4: "altitude",
    5: "altitude_mode",
    6: "speed",
    7: "action",
    8: "hold_seconds",
}

_ALTITUDE_MODE_TEXTS = [mode.value for mode in AltitudeMode]
_ACTION_TEXTS = [action.value for action in WaypointAction]
_COVERAGE_EDITABLE_FIELDS = {"altitude", "speed"}


class _ComboDelegate(QStyledItemDelegate):
    """Combo-box editor for enum-like waypoint columns."""

    def __init__(self, values: Sequence[str], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._values = list(values)

    def createEditor(
        self,
        parent: QWidget,
        option: QStyleOptionViewItem,
        index: QModelIndex | QPersistentModelIndex,
    ) -> QComboBox:
        combo = QComboBox(parent)
        combo.addItems(self._values)
        return combo

    def setEditorData(self, editor: QWidget, index: QModelIndex | QPersistentModelIndex) -> None:
        if isinstance(editor, QComboBox):
            editor.setCurrentText(str(index.data(Qt.ItemDataRole.DisplayRole)))

    def setModelData(
        self,
        editor: QWidget,
        model: QAbstractItemModel,
        index: QModelIndex | QPersistentModelIndex,
    ) -> None:
        if isinstance(editor, QComboBox):
            model.setData(index, editor.currentText())


class WaypointPanel(QWidget):
    """Table view over every drone waypoint with guarded inline editing."""

    waypoint_edited = Signal(str, int, str, object)
    waypoint_selected = Signal(str, int)
    waypoint_delete_requested = Signal(str, int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._rows: list[tuple[str, int]] = []
        self._coverage_drones: set[str] = set()
        self._building = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(list(COLUMNS))
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setItemDelegateForColumn(5, _ComboDelegate(_ALTITUDE_MODE_TEXTS, self.table))
        self.table.setItemDelegateForColumn(7, _ComboDelegate(_ACTION_TEXTS, self.table))
        self.table.itemChanged.connect(self._on_item_changed)
        self.table.currentCellChanged.connect(self._on_current_cell_changed)
        layout.addWidget(self.table)
        buttons = QHBoxLayout()
        self.delete_button = QPushButton("Delete selected waypoint")
        self.delete_button.setToolTip(
            "Departure, mission, and return waypoints as well as coverage scan "
            "routes cannot be deleted"
        )
        self.delete_button.clicked.connect(self._emit_delete_request)
        buttons.addWidget(self.delete_button)
        buttons.addStretch()
        layout.addLayout(buttons)

    # ------------------------------------------------------------------ API

    def set_project_map(self, map_model: MapModel) -> None:
        """Rebuild the table, preserving the current waypoint selection."""

        selected = self.current_waypoint()
        self._building = True
        self.table.setRowCount(0)
        self._rows = []
        self._coverage_drones = {
            drone.id
            for drone in map_model.drones
            if any(waypoint.action == WaypointAction.SCAN for waypoint in drone.waypoints)
        }
        for drone in sorted(map_model.drones, key=lambda item: item.id):
            for index, waypoint in enumerate(drone.waypoints):
                row = self.table.rowCount()
                self.table.insertRow(row)
                self._rows.append((drone.id, index))
                self._fill_row(row, drone, index, waypoint, map_model.terrain)
        self._building = False
        self.table.resizeColumnsToContents()
        if selected is not None:
            self.set_selected_waypoint(selected[0], selected[1])
        elif self._rows:
            self._select_row(0)

    def set_selected_waypoint(self, drone_id: str, index: int) -> None:
        """Highlight the row of one waypoint without re-emitting selection."""

        try:
            row = self._rows.index((drone_id, index))
        except ValueError:
            return
        self._select_row(row)

    def current_waypoint(self) -> tuple[str, int] | None:
        row = self.table.currentRow()
        if 0 <= row < len(self._rows):
            return self._rows[row]
        return None

    # ------------------------------------------------------------ internals

    def _fill_row(
        self,
        row: int,
        drone: Drone,
        index: int,
        waypoint: Waypoint,
        terrain: TerrainModel,
    ) -> None:
        del terrain  # altitudes are shown as stored; mode decides their meaning
        coverage_drone = drone.id in self._coverage_drones
        values = (
            str(index + 1),
            drone.id,
            f"{waypoint.x:.1f}",
            f"{waypoint.y:.1f}",
            f"{waypoint.altitude:g}",
            waypoint.altitude_mode.value,
            "auto" if waypoint.speed is None else f"{waypoint.speed:g}",
            waypoint.action.value,
            f"{waypoint.hold_seconds:g}",
            waypoint.task_id or "",
        )
        for column, text in enumerate(values):
            item = QTableWidgetItem(text)
            field = _COLUMN_FIELDS.get(column)
            if field is None or (coverage_drone and field not in _COVERAGE_EDITABLE_FIELDS):
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            else:
                item.setToolTip("Double-click to edit")
            self.table.setItem(row, column, item)

    def _select_row(self, row: int) -> None:
        self._building = True
        self.table.selectRow(row)
        self._building = False

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        if self._building or not 0 <= item.row() < len(self._rows):
            return
        field = _COLUMN_FIELDS.get(item.column())
        if field is None:
            return
        drone_id, index = self._rows[item.row()]
        self.waypoint_edited.emit(
            drone_id, index, field, item.data(Qt.ItemDataRole.DisplayRole)
        )

    def _on_current_cell_changed(
        self,
        row: int,
        _column: int,
        _previous_row: int,
        _previous_column: int,
    ) -> None:
        if self._building or not 0 <= row < len(self._rows):
            return
        drone_id, index = self._rows[row]
        self.waypoint_selected.emit(drone_id, index)

    def _emit_delete_request(self) -> None:
        current = self.current_waypoint()
        if current is not None:
            self.waypoint_delete_requested.emit(*current)
