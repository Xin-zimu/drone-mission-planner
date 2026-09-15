from __future__ import annotations

from PySide6.QtNetwork import QHostAddress, QTcpServer

from drone_mission_planner.execution.protocol import encode_message, ping_request
from drone_mission_planner.ui.execution_client import ExecutionClient


def test_execution_client_connects_and_receives_ndjson(qtbot: object) -> None:
    server = QTcpServer()
    assert server.listen(QHostAddress.SpecialAddress.LocalHost, 0)
    client = ExecutionClient()
    connected: list[bool] = []
    messages: list[dict[str, object]] = []
    client.connected_changed.connect(connected.append)
    client.message_received.connect(messages.append)

    client.connect_to_bridge("127.0.0.1", server.serverPort())

    qtbot.waitUntil(lambda: server.hasPendingConnections(), timeout=2000)  # type: ignore[attr-defined]
    socket = server.nextPendingConnection()
    assert socket is not None
    qtbot.waitUntil(lambda: client.is_connected, timeout=2000)  # type: ignore[attr-defined]

    socket.write(encode_message({"type": "pong", "request_id": "r-1"}))
    socket.flush()

    qtbot.waitUntil(lambda: bool(messages), timeout=2000)  # type: ignore[attr-defined]
    assert connected[-1] is True
    assert messages[0]["type"] == "pong"
    assert messages[0]["request_id"] == "r-1"

    client.send_message(ping_request(request_id="ping-1"))
    qtbot.waitUntil(lambda: socket.bytesAvailable() > 0, timeout=2000)  # type: ignore[attr-defined]
    assert b'"request_id":"ping-1"' in socket.readAll().data()
    client.disconnect_from_bridge()
    client.deleteLater()
    server.close()
