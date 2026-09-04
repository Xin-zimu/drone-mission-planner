from __future__ import annotations

from enum import StrEnum


class DroneStatus(StrEnum):
    IDLE = "idle"
    PLANNING = "planning"
    TAKING_OFF = "taking_off"
    FLYING = "flying"
    EXECUTING = "executing"
    RETURNING = "returning"
    CHARGING = "charging"
    FAILED = "failed"
    EMERGENCY = "emergency"
    COMPLETED = "completed"


class TaskType(StrEnum):
    WAYPOINT = "waypoint"
    INSPECTION = "inspection"
    DELIVERY = "delivery"
    AREA_SEARCH = "area_search"
    RETURN_HOME = "return_home"
    RELAY = "relay"


class TaskStatus(StrEnum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class ObstacleShape(StrEnum):
    RECTANGLE = "rectangle"
    CIRCLE = "circle"
    POLYGON = "polygon"
