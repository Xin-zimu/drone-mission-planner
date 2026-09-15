from __future__ import annotations

import pytest

from drone_mission_planner.domain.geometry import Point
from drone_mission_planner.domain.models import Drone, ProjectModel
from drone_mission_planner.domain.waypoint import Waypoint
from drone_mission_planner.execution.mission_compiler import compile_crazyflie_mission
from drone_mission_planner.execution.models import ExecutionFrameCalibration, ExecutionMission
from drone_mission_planner.execution.profile import CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE
from drone_mission_planner.execution.protocol import (
    ExecutionProtocolError,
    MessageTooLargeError,
    NdjsonStreamParser,
    decode_message,
    encode_message,
    hello_request,
    load_mission_request,
    mission_to_payload,
)


def test_protocol_encodes_one_json_object_per_line() -> None:
    encoded = encode_message(hello_request(request_id="r-1"))

    assert encoded.endswith(b"\n")
    decoded = decode_message(encoded)
    assert decoded["type"] == "hello"
    assert decoded["request_id"] == "r-1"
    assert decoded["protocol_version"] == "dmp-cf/1"


def test_protocol_rejects_malformed_packets() -> None:
    with pytest.raises(ExecutionProtocolError, match="valid JSON"):
        decode_message(b"{broken\n")

    with pytest.raises(ExecutionProtocolError, match="JSON object"):
        decode_message(b"[]\n")

    with pytest.raises(ExecutionProtocolError, match="type"):
        decode_message(b'{"request_id":"r-1"}\n')


def test_protocol_rejects_oversized_messages() -> None:
    with pytest.raises(MessageTooLargeError):
        encode_message({"type": "ping", "blob": "x" * 16}, max_bytes=10)

    with pytest.raises(MessageTooLargeError):
        decode_message(b'{"type":"ping","blob":"xxxxxxxxxxxxxxxx"}\n', max_bytes=10)


def test_ndjson_stream_parser_handles_fragmented_messages() -> None:
    parser = NdjsonStreamParser()

    assert parser.feed(b'{"type":"ping","request_id"') == []
    messages = parser.feed(b':"r-1"}\n{"type":"ping","request_id":"r-2"}\n')

    assert [message["request_id"] for message in messages] == ["r-1", "r-2"]


def test_load_mission_payload_contains_hash_and_waypoints() -> None:
    mission = _mission()

    request = load_mission_request(mission, request_id="load-1")
    payload = mission_to_payload(mission)

    assert request["type"] == "load_mission"
    assert request["request_id"] == "load-1"
    assert request["mission_id"] == mission.mission_id
    assert request["source_route_hash"] == mission.source_route_hash
    assert payload["source_route_hash"] == mission.source_route_hash
    assert payload["waypoints"][0]["x_m"] == 0.0
    assert decode_message(encode_message(request))["mission"]["source_route_hash"]


def _mission() -> ExecutionMission:
    project = ProjectModel(name="Protocol")
    drone = Drone(
        "D-01",
        "Alpha",
        Point(0.0, 0.0),
        waypoints=[Waypoint(0.0, 0.0, 0.5), Waypoint(0.5, 0.0, 0.5)],
    )
    project.map.drones.append(drone)
    frame = ExecutionFrameCalibration(
        mode="relative_takeoff",
        planner_origin_x_m=0.0,
        planner_origin_y_m=0.0,
        planner_origin_z_m=0.0,
        cf_origin_x_m=0.0,
        cf_origin_y_m=0.0,
        cf_origin_z_m=0.0,
        yaw_offset_rad=0.0,
        validated=True,
    )
    return compile_crazyflie_mission(
        project,
        drone,
        CRAZYFLIE_CRAZYSWARM2_SINGLE_PROFILE,
        frame,
        mission_id="mission-protocol",
        created_at_utc="2026-09-16T00:00:00Z",
    )
