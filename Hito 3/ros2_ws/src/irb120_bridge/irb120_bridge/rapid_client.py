"""
Cliente TCP del protocolo de texto de rapid/ROS_BRIDGE.mod (sin dependencias de ROS).

Ademas de los limites que aplica el controlador, el cliente repite la validacion
(defensa en profundidad): si cualquiera de las dos capas rechaza, no hay movimiento.
"""
from __future__ import annotations

import socket

# Deben coincidir con rb_LIM_LO / rb_LIM_HI / rb_MAX_STEP del modulo RAPID
LIM_LO_DEG = [-90, -40, -70, -150, -115, -300]
LIM_HI_DEG = [90, 60, 50, 150, 115, 300]
MAX_STEP_DEG = 45


class RapidBridgeError(RuntimeError):
    pass


class RapidBridgeClient:
    def __init__(self, host="192.168.125.1", port=1025, connect_timeout=5.0, move_timeout=120.0):
        self.host, self.port = host, port
        self.connect_timeout, self.move_timeout = connect_timeout, move_timeout
        self.sock: socket.socket | None = None
        self._buf = b""

    # -- conexion -----------------------------------------------------------------------------
    def connect(self):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
        self._buf = b""
        if self.request("PING", self.connect_timeout) != "PONG":
            raise RapidBridgeError("el servidor no respondio PONG")

    def close(self):
        if self.sock is not None:
            try:
                self.sock.sendall(b"BYE\n")
            except OSError:
                pass
            self.sock.close()
            self.sock = None

    # -- protocolo ----------------------------------------------------------------------------
    def request(self, msg: str, timeout: float) -> str:
        if self.sock is None:
            raise RapidBridgeError("no conectado")
        if len(msg) > 79:
            raise RapidBridgeError("mensaje > 79 caracteres (limite de string RAPID)")
        self.sock.settimeout(timeout)
        self.sock.sendall((msg + "\n").encode("ascii"))
        while b"\n" not in self._buf:
            chunk = self.sock.recv(256)
            if not chunk:
                raise RapidBridgeError("conexion cerrada por el controlador")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode("ascii", "replace").strip()

    def get_joints_deg(self) -> list[float]:
        r = self.request("GETJ", 5.0)
        if not r.startswith("JNT "):
            raise RapidBridgeError(r)
        return [float(x) for x in r.split()[1:7]]

    def set_speed(self, v_tcp_mm_s: float) -> str:
        return self.request(f"SPD {v_tcp_mm_s:.0f}", 5.0)

    @staticmethod
    def validate(target_deg, current_deg=None):
        if len(target_deg) != 6:
            return "se requieren 6 valores"
        for i, v in enumerate(target_deg):
            if not (LIM_LO_DEG[i] <= v <= LIM_HI_DEG[i]):
                return f"J{i + 1}={v:.2f} fuera de [{LIM_LO_DEG[i]}, {LIM_HI_DEG[i]}]"
            if current_deg is not None and abs(v - current_deg[i]) > MAX_STEP_DEG:
                return f"J{i + 1}: salto {abs(v - current_deg[i]):.1f} > {MAX_STEP_DEG} deg"
        return None

    def move_joints_deg(self, target_deg) -> list[float]:
        current = self.get_joints_deg()
        err = self.validate(target_deg, current)
        if err:
            raise RapidBridgeError(f"rechazado por el cliente: {err}")
        msg = "MOVJ " + " ".join(f"{v:.2f}" for v in target_deg)
        r = self.request(msg, self.move_timeout)
        if not r.startswith("DONE "):
            raise RapidBridgeError(f"rechazado por el controlador: {r}")
        return [float(x) for x in r.split()[1:7]]
