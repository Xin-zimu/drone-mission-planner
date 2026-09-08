"""Per-mission actual timeline recorded by the simulation (plan §10.7).

The engine records when a mission was reached, how long it waited, when service
started and finished, and whether the finish broke its deadline. Reports and the
Gantt view read this instead of re-deriving times, so plan and actual can be
compared without either side overwriting the other.
"""

from __future__ import annotations

from dataclasses import dataclass

from drone_mission_planner.domain.enums import DeadlinePolicy, TaskStatus

TOLERANCE = 1e-9

# Business states of plan §10.7, derived from the recorded timeline.
STATE_UNASSIGNED = "unassigned"
STATE_ASSIGNED = "assigned"
STATE_EN_ROUTE = "en_route"
STATE_WAITING = "waiting"
STATE_SERVICING = "servicing"
STATE_COMPLETED = "completed"
STATE_FAILED = "failed"
STATE_CANCELLED = "cancelled"
STATE_BLOCKED = "blocked"


@dataclass(slots=True)
class TaskTimeline:
    """Actual times of one mission as the simulation observed them."""

    task_id: str
    drone_id: str | None = None
    arrived_at: float | None = None
    wait_started_at: float | None = None
    wait_ended_at: float | None = None
    service_started_at: float | None = None
    service_finished_at: float | None = None
    deadline: float | None = None
    deadline_policy: DeadlinePolicy = DeadlinePolicy.HARD
    blocked_reason: str | None = None
    status: TaskStatus = TaskStatus.PENDING

    @property
    def wait_seconds(self) -> float:
        if self.wait_started_at is None:
            return 0.0
        end = self.wait_ended_at if self.wait_ended_at is not None else self.wait_started_at
        return max(0.0, end - self.wait_started_at)

    @property
    def service_seconds(self) -> float:
        if self.service_started_at is None:
            return 0.0
        end = (
            self.service_finished_at
            if self.service_finished_at is not None
            else self.service_started_at
        )
        return max(0.0, end - self.service_started_at)

    @property
    def lateness_seconds(self) -> float:
        if self.deadline is None or self.service_finished_at is None:
            return 0.0
        return max(0.0, self.service_finished_at - self.deadline)

    @property
    def deadline_ok(self) -> bool:
        if self.deadline is None or self.service_finished_at is None:
            return True
        return self.service_finished_at <= self.deadline + TOLERANCE

    @property
    def business_status(self) -> str:
        """The §10.7 business state derived from the recorded times."""

        if self.blocked_reason is not None:
            return STATE_BLOCKED
        if self.status is TaskStatus.CANCELLED:
            return STATE_CANCELLED
        if self.status is TaskStatus.FAILED:
            return STATE_FAILED
        if self.service_finished_at is not None:
            return STATE_COMPLETED
        if self.service_started_at is not None:
            return STATE_SERVICING
        if self.wait_started_at is not None and self.wait_ended_at is None:
            return STATE_WAITING
        if self.arrived_at is not None:
            return STATE_EN_ROUTE
        if self.drone_id is not None:
            return STATE_ASSIGNED
        return STATE_UNASSIGNED

    def snapshot(self) -> TaskTimeline:
        return TaskTimeline(
            task_id=self.task_id,
            drone_id=self.drone_id,
            arrived_at=self.arrived_at,
            wait_started_at=self.wait_started_at,
            wait_ended_at=self.wait_ended_at,
            service_started_at=self.service_started_at,
            service_finished_at=self.service_finished_at,
            deadline=self.deadline,
            deadline_policy=self.deadline_policy,
            blocked_reason=self.blocked_reason,
            status=self.status,
        )
