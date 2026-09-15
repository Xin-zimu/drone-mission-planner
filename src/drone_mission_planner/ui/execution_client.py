from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtNetwork import QAbstractSocket, QTcpSocket

from drone_mission_planner.execution.protocol import (
    ExecutionProtocolError,
    NdjsonStreamParser,
    encode_message,
)


class ExecutionClient(QObject):
    connected_changed = Signal(bool)
    message_received = Signal(dict)
    error_received = Signal(str)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.host = "127.0.0.1"
        self.port = 8765
        self.auto_reconnect = False
        self._manual_disconnect = False
        self._parser = NdjsonStreamParser()
        self._socket = QTcpSocket(self)
        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setInterval(1000)
        self._reconnect_timer.timeout.connect(self._try_reconnect)
        self._socket.connected.connect(lambda: self.connected_changed.emit(True))
        self._socket.disconnected.connect(self._socket_disconnected)
        self._socket.readyRead.connect(self._read_available)
        self._socket.errorOccurred.connect(self._socket_error)

    @property
    def is_connected(self) -> bool:
        return self._socket.state() == QAbstractSocket.SocketState.ConnectedState

    def connect_to_bridge(
        self,
        host: str = "127.0.0.1",
        port: int = 8765,
        *,
        auto_reconnect: bool = False,
    ) -> None:
        self.host = host
        self.port = port
        self.auto_reconnect = auto_reconnect
        self._manual_disconnect = False
        self._reconnect_timer.stop()
        self._socket.abort()
        self._socket.connectToHost(host, port)

    def disconnect_from_bridge(self) -> None:
        self._manual_disconnect = True
        self._reconnect_timer.stop()
        self._socket.disconnectFromHost()

    def send_message(self, message: Mapping[str, Any]) -> None:
        if not self.is_connected:
            raise ConnectionError("execution bridge is not connected")
        self._socket.write(encode_message(message))

    def _read_available(self) -> None:
        data = bytes(self._socket.readAll().data())
        try:
            for message in self._parser.feed(data):
                self.message_received.emit(message)
        except ExecutionProtocolError as exc:
            self.error_received.emit(str(exc))
            self._socket.abort()

    def _socket_disconnected(self) -> None:
        self.connected_changed.emit(False)
        if self.auto_reconnect and not self._manual_disconnect:
            self._reconnect_timer.start()

    def _try_reconnect(self) -> None:
        if self.is_connected:
            self._reconnect_timer.stop()
            return
        self._socket.abort()
        self._socket.connectToHost(self.host, self.port)

    def _socket_error(self, error: QAbstractSocket.SocketError) -> None:
        if error == QAbstractSocket.SocketError.RemoteHostClosedError:
            return
        self.error_received.emit(self._socket.errorString())
