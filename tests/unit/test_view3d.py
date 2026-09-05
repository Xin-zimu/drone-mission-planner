from __future__ import annotations

from PySide6.QtCore import QEvent, QPointF, Qt
from PySide6.QtGui import QMouseEvent

from drone_mission_planner.app.project_service import ProjectService
from drone_mission_planner.domain.geometry import Point, Rect
from drone_mission_planner.ui.scene3d_export import Scene3D, build_scene3d
from drone_mission_planner.ui.view3d import LAYERS, ThreeDView


def _scene() -> Scene3D:
    service = ProjectService()
    service.add_base(Point(80.0, 90.0))
    drone = service.add_drone(Point(100.0, 110.0))
    drone.planned_path = [Point(100.0, 110.0), Point(420.0, 260.0)]
    service.add_obstacle(Rect(300.0, 180.0, 90.0, 70.0))
    return build_scene3d(service.project.map)


def _mouse_event(event_type: QEvent.Type, x: float, y: float) -> QMouseEvent:
    return QMouseEvent(
        event_type,
        QPointF(x, y),
        Qt.MouseButton.LeftButton,
        Qt.MouseButton.LeftButton,
        Qt.KeyboardModifier.NoModifier,
    )


def test_projection_places_map_centre_near_widget_centre(qtbot: object) -> None:
    view = ThreeDView()
    qtbot.addWidget(view)  # type: ignore[attr-defined]
    view.resize(400, 300)
    view.set_scene(_scene())
    view.apply_view_preset("top")

    centre = view.project(500.0, 350.0, 100.0)
    assert centre is not None
    assert abs(centre.x() - 200.0) < 40.0
    assert abs(centre.y() - 150.0) < 40.0


def test_view_presets_change_camera(qtbot: object) -> None:
    view = ThreeDView()
    qtbot.addWidget(view)  # type: ignore[attr-defined]
    view.set_scene(_scene())
    view.apply_view_preset("iso")
    iso_pitch = view._camera.pitch_deg
    view.apply_view_preset("side")

    assert view._camera.pitch_deg != iso_pitch
    assert view._camera.pitch_deg == 8.0


def test_layer_toggles_and_render_smoke(qtbot: object) -> None:
    view = ThreeDView()
    qtbot.addWidget(view)  # type: ignore[attr-defined]
    view.resize(360, 240)
    view.set_scene(_scene())
    view.grab()

    assert all(view.is_layer_visible(layer) for layer in LAYERS)
    view.set_layer_visible("routes", False)
    assert not view.is_layer_visible("routes")
    view.grab()

    view.set_selected_object("D-01")
    view.grab()
    assert view.selected_object == "D-01"


def test_pick_at_returns_nearest_marker(qtbot: object) -> None:
    view = ThreeDView()
    qtbot.addWidget(view)  # type: ignore[attr-defined]
    view.resize(400, 300)
    view.set_scene(_scene())
    view.apply_view_preset("top")
    view.grab()

    scene = view._scene
    assert scene is not None
    marker = scene.markers[0]
    projected = view.project(marker.x, marker.y, marker.z)
    assert projected is not None
    assert view.pick_at(projected.x(), projected.y()) == marker.object_id
    assert view.pick_at(2.0, 2.0) is None


def test_click_selects_object_through_mouse_events(qtbot: object) -> None:
    view = ThreeDView()
    qtbot.addWidget(view)  # type: ignore[attr-defined]
    view.resize(400, 300)
    view.set_scene(_scene())
    view.apply_view_preset("top")
    view.grab()

    scene = view._scene
    assert scene is not None
    marker = scene.markers[0]
    projected = view.project(marker.x, marker.y, marker.z)
    assert projected is not None
    x, y = projected.x(), projected.y()
    with qtbot.waitSignal(view.object_selected) as blocker:  # type: ignore[attr-defined]
        view.mousePressEvent(_mouse_event(QEvent.Type.MouseButtonPress, x, y))
        view.mouseReleaseEvent(_mouse_event(QEvent.Type.MouseButtonRelease, x, y))
    assert blocker.args == [marker.object_id]
    assert view.selected_object == marker.object_id


def test_live_positions_move_markers(qtbot: object) -> None:
    view = ThreeDView()
    qtbot.addWidget(view)  # type: ignore[attr-defined]
    view.set_scene(_scene())
    view.update_live_positions({"D-01": (Point(300.0, 200.0), 123.0)})

    scene = view._scene
    assert scene is not None
    assert (scene.markers[0].x, scene.markers[0].y, scene.markers[0].z) == (300.0, 200.0, 123.0)
