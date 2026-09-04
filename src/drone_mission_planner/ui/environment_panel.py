from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTableWidget,
    QVBoxLayout,
    QWidget,
)

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import MapModel
from drone_mission_planner.domain.terrain import (
    TerrainModel,
    TerrainPeak,
    flat_terrain,
    generate_mountain_terrain,
)
from drone_mission_planner.domain.wind import WindModel


class EnvironmentPanel(QScrollArea):
    environment_changed = Signal(object, object)
    import_terrain_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setFrameShape(QScrollArea.Shape.NoFrame)
        self._map_width = 1000.0
        self._map_height = 700.0
        self._updating = False

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(12)

        heading = QLabel("ENVIRONMENT")
        heading.setObjectName("SectionLabel")
        layout.addWidget(heading)

        terrain_form = QFormLayout()
        terrain_form.setHorizontalSpacing(14)
        terrain_form.setVerticalSpacing(8)
        self.base_altitude_spin = self._double_spin(-1000.0, 10000.0, " m")
        self.resolution_spin = self._double_spin(1.0, 1000.0, " m")
        self.peak_count_spin = QSpinBox()
        self.peak_count_spin.setRange(0, 8)
        self.peak_count_spin.setToolTip("Number of procedural Gaussian terrain peaks")
        terrain_form.addRow("Base altitude", self.base_altitude_spin)
        terrain_form.addRow("Terrain sample step", self.resolution_spin)
        terrain_form.addRow("Mountain peaks", self.peak_count_spin)
        layout.addLayout(terrain_form)

        self.import_terrain_button = QPushButton("Import elevation CSV...")
        self.import_terrain_button.setToolTip("Load a local CSV with x, y, elevation columns")
        layout.addWidget(self.import_terrain_button)

        self.peak_table = QTableWidget(0, 4)
        self.peak_table.setHorizontalHeaderLabels(["Center X", "Center Y", "Radius", "Height"])
        self.peak_table.verticalHeader().setVisible(False)
        self.peak_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.peak_table.setAlternatingRowColors(True)
        self.peak_table.setMinimumHeight(110)
        layout.addWidget(self.peak_table)

        wind_heading = QLabel("WIND")
        wind_heading.setObjectName("SectionLabel")
        layout.addWidget(wind_heading)

        wind_form = QFormLayout()
        wind_form.setHorizontalSpacing(14)
        wind_form.setVerticalSpacing(8)
        self.wind_enabled_checkbox = QCheckBox("Enabled")
        self.wind_direction_spin = self._double_spin(0.0, 360.0, " deg")
        self.wind_speed_spin = self._double_spin(0.0, 100.0, " m/s")
        self.wind_gust_spin = self._double_spin(0.0, 1.0, "")
        self.wind_gust_spin.setSingleStep(0.05)
        wind_form.addRow("Wind model", self.wind_enabled_checkbox)
        wind_form.addRow("Direction to", self.wind_direction_spin)
        wind_form.addRow("Speed", self.wind_speed_spin)
        wind_form.addRow("Gust factor", self.wind_gust_spin)
        layout.addLayout(wind_form)

        summary_row = QHBoxLayout()
        self.summary_label = QLabel("Flat terrain, no wind")
        self.summary_label.setWordWrap(True)
        self.summary_label.setStyleSheet("color: #93a2b8;")
        summary_row.addWidget(self.summary_label)
        layout.addLayout(summary_row)
        layout.addStretch()

        self.setWidget(container)
        self._connect_controls()

    def set_map_model(self, model: MapModel) -> None:
        self._updating = True
        self._map_width = float(model.width)
        self._map_height = float(model.height)
        self.base_altitude_spin.setValue(model.terrain.base_altitude)
        self.resolution_spin.setValue(model.terrain.resolution)
        self.peak_count_spin.setValue(len(model.terrain.peaks))
        self._set_peak_rows(model.terrain.peaks)
        self.wind_enabled_checkbox.setChecked(model.wind.enabled)
        self.wind_direction_spin.setValue(model.wind.direction_to_deg)
        self.wind_speed_spin.setValue(model.wind.speed)
        self.wind_gust_spin.setValue(model.wind.gust_factor)
        self._updating = False
        self._update_summary(model.terrain, model.wind)

    def _connect_controls(self) -> None:
        self.base_altitude_spin.valueChanged.connect(
            lambda _value: self._emit_environment_changed()
        )
        self.resolution_spin.valueChanged.connect(lambda _value: self._emit_environment_changed())
        self.peak_count_spin.valueChanged.connect(self._resize_peaks)
        self.wind_enabled_checkbox.stateChanged.connect(
            lambda _state: self._emit_environment_changed()
        )
        self.wind_direction_spin.valueChanged.connect(
            lambda _value: self._emit_environment_changed()
        )
        self.wind_speed_spin.valueChanged.connect(lambda _value: self._emit_environment_changed())
        self.wind_gust_spin.valueChanged.connect(lambda _value: self._emit_environment_changed())
        self.import_terrain_button.clicked.connect(
            lambda _checked=False: self.import_terrain_requested.emit()
        )

    def _resize_peaks(self, count: int) -> None:
        if self._updating:
            return
        peaks = self._peak_values()
        while len(peaks) < count:
            peaks.append(self._default_peak())
        self._set_peak_rows(peaks[:count])
        self._emit_environment_changed()

    def _set_peak_rows(self, peaks: list[TerrainPeak]) -> None:
        self.peak_table.setRowCount(len(peaks))
        for row, peak in enumerate(peaks):
            values = [peak.center.x, peak.center.y, peak.radius, peak.height]
            for column, value in enumerate(values):
                spin = self._peak_spin(column)
                spin.setValue(value)
                spin.valueChanged.connect(lambda _value: self._emit_environment_changed())
                self.peak_table.setCellWidget(row, column, spin)

    def _peak_values(self) -> list[TerrainPeak]:
        peaks: list[TerrainPeak] = []
        for row in range(self.peak_table.rowCount()):
            center_x = self._cell_value(row, 0)
            center_y = self._cell_value(row, 1)
            radius = self._cell_value(row, 2)
            height = self._cell_value(row, 3)
            peaks.append(TerrainPeak(Point(center_x, center_y), radius, height))
        return peaks

    def _build_models(self) -> tuple[TerrainModel, WindModel]:
        resolution = self.resolution_spin.value()
        base_altitude = self.base_altitude_spin.value()
        peaks = self._peak_values()
        terrain = (
            generate_mountain_terrain(
                width=self._map_width,
                height=self._map_height,
                resolution=resolution,
                base_altitude=base_altitude,
                peaks=peaks,
            )
            if peaks
            else flat_terrain(altitude=base_altitude, resolution=resolution)
        )
        wind = WindModel(
            direction_to_deg=self.wind_direction_spin.value(),
            speed=self.wind_speed_spin.value(),
            gust_factor=self.wind_gust_spin.value(),
            enabled=self.wind_enabled_checkbox.isChecked(),
        )
        return terrain, wind

    def _emit_environment_changed(self) -> None:
        if self._updating:
            return
        terrain, wind = self._build_models()
        self._update_summary(terrain, wind)
        self.environment_changed.emit(terrain, wind)

    def _update_summary(self, terrain: TerrainModel, wind: WindModel) -> None:
        if terrain.terrain_type == "grid" and terrain.grid_altitudes:
            terrain_text = (
                f"imported grid terrain {terrain.grid_width} x {terrain.grid_height}, "
                f"{terrain.min_altitude:.0f}-{terrain.max_altitude:.0f} m"
            )
        elif terrain.peaks:
            terrain_text = (
                f"{len(terrain.peaks)} peak terrain, "
                f"{terrain.min_altitude:.0f}-{terrain.max_altitude:.0f} m"
            )
        else:
            terrain_text = f"flat terrain at {terrain.base_altitude:.0f} m"
        wind_text = (
            f"wind {wind.speed:.1f} m/s to {wind.direction_to_deg:.0f} deg"
            if wind.enabled and wind.speed > 0
            else "no wind"
        )
        self.summary_label.setText(f"{terrain_text}; {wind_text}")

    def _default_peak(self) -> TerrainPeak:
        return TerrainPeak(
            Point(self._map_width * 0.5, self._map_height * 0.5),
            max(10.0, min(self._map_width, self._map_height) * 0.15),
            80.0,
        )

    def _peak_spin(self, column: int) -> QDoubleSpinBox:
        if column == 0:
            return self._double_spin(-1_000_000.0, 1_000_000.0, " m")
        if column == 1:
            return self._double_spin(-1_000_000.0, 1_000_000.0, " m")
        if column == 2:
            return self._double_spin(
                0.1,
                max(1.0, max(self._map_width, self._map_height) * 2.0),
                " m",
            )
        return self._double_spin(-1000.0, 10000.0, " m")

    def _cell_value(self, row: int, column: int) -> float:
        widget = self.peak_table.cellWidget(row, column)
        if isinstance(widget, QDoubleSpinBox):
            return widget.value()
        return 0.0

    @staticmethod
    def _double_spin(minimum: float, maximum: float, suffix: str) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(2)
        spin.setSingleStep(1.0)
        spin.setSuffix(suffix)
        return spin
