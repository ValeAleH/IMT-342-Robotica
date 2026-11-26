"""
Controlador IRC5 SIMULADO (solo para pruebas sin robot ni RobotStudio).

Emula:
  - RWS (HTTP, RobotWare 6, formato JSON _embedded/_state) en el puerto --http-port (8080)
  - el servidor de rapid/ROS_BRIDGE.mod (TCP) en el puerto --tcp-port (1025)
con un estado articular comun que se mueve a velocidad limitada.

NO es el comportamiento real del controlador: sirve para probar el software del PC
(nodos ROS 2, scripts) antes de ir al laboratorio. Registrelo asi en la bitacora.

Uso:  python tools/mock_irc5.py            ->  RWS en http://127.0.0.1:8080 , socket en 127.0.0.1:1025
"""
from __future__ import annotations

import argparse
import json
import os
import socketserver
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "kinematics"))
import irb120_model as M  # noqa: E402

LIM_LO = [-90, -40, -70, -150, -115, -300]
LIM_HI = [90, 60, 50, 150, 115, 300]
MAX_STEP, Z_MIN = 45, 100


class SimState:
    def __init__(self, speed_deg_s=20.0):
        self.q = np.array([0.0, 0.0, 0.0, 0.0, 30.0, 0.0])
        self.lock = threading.Lock()
        self.speed = speed_deg_s
        self.opmode, self.ctrlstate, self.exec = "MANR", "motoron", "running"

    def move_to(self, target):
        target = np.asarray(target, float)
        with self.lock:
            start = self.q.copy()
        dur = max(np.max(np.abs(target - start)) / self.speed, 0.05)
        t0 = time.time()
        while True:
            a = min((time.time() - t0) / dur, 1.0)
            with self.lock:
                self.q = start + a * (target - start)
            if a >= 1.0:
                break
            time.sleep(0.02)

    def joints(self):
        with self.lock:
            return self.q.copy()


ARCHIVOS = {}


def make_http(state: SimState):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/hal+json;v=2.0")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_PUT(self):   # /fileservice/$home/<archivo>
            n = int(self.headers.get("Content-Length", 0))
            ARCHIVOS[urlparse(self.path).path] = self.rfile.read(n)
            self.send_response(201)
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self):
            u = urlparse(self.path)
            if u.path.startswith("/fileservice/") and u.path in ARCHIVOS:
                body = ARCHIVOS[u.path]
                self.send_response(200)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            qs = parse_qs(u.query)
            p = u.path
            q = state.joints()
            if p.endswith("/jointtarget") or (p.endswith("/motion") and qs.get("resource") == ["jointtarget"]):
                st = {"_type": "ms-jointtarget", **{f"rax_{i + 1}": f"{q[i]:.6f}" for i in range(6)}}
            elif p.endswith("/robtarget"):
                tool = qs.get("tool", ["tool0"])[0]
                T = M.fk(np.deg2rad(q), M.T_GRIPPER1 if tool.lower() == "gripper1" else None)
                quat = M.R_to_quat_wxyz(T[:3, :3])
                st = {"_type": "ms-robtargets", "x": f"{T[0, 3] * 1000:.4f}", "y": f"{T[1, 3] * 1000:.4f}",
                      "z": f"{T[2, 3] * 1000:.4f}", **{f"q{i + 1}": f"{quat[i]:.7f}" for i in range(4)}}
            elif p == "/rw/panel/opmode":
                st = {"_type": "pnl-opmode", "opmode": state.opmode}
            elif p == "/rw/panel/ctrlstate":
                st = {"_type": "pnl-ctrlstate", "ctrlstate": state.ctrlstate}
            elif p == "/rw/rapid/execution":
                st = {"_type": "rap-execution", "ctrlexecstate": state.exec}
            else:
                return self._send({"error": "not found"}, 404)
            self._send({"_links": {"base": {"href": "mock"}}, "_embedded": {"_state": [st]}})

    return H


def make_tcp(state: SimState):
    class T(socketserver.StreamRequestHandler):
        def handle(self):
            for raw in self.rfile:
                msg = raw.decode().strip()
                if msg == "PING":
                    rep = "PONG"
                elif msg == "GETJ":
                    rep = "JNT " + " ".join(f"{v:.2f}" for v in state.joints())
                elif msg == "GETP":
                    T_ = M.fk(np.deg2rad(state.joints()), M.T_GRIPPER1)
                    qq = M.R_to_quat_wxyz(T_[:3, :3])
                    rep = "POS " + " ".join(f"{v:.1f}" for v in T_[:3, 3] * 1000) + " " + " ".join(
                        f"{v:.4f}" for v in qq)
                elif msg.startswith("SPD "):
                    try:
                        rep = "OK SPD " + str(int(min(max(float(msg[4:]), 5), 100)))
                    except ValueError:
                        rep = "ERR PARSE"
                elif msg.startswith("MOVJ "):
                    try:
                        v = [float(x) for x in msg[5:].split()]
                        assert len(v) == 6
                    except Exception:  # noqa: BLE001
                        rep = "ERR PARSE"
                    else:
                        cur = state.joints()
                        rep = None
                        for i in range(6):
                            if not LIM_LO[i] <= v[i] <= LIM_HI[i]:
                                rep = f"ERR LIMIT J{i + 1}"
                                break
                            if abs(v[i] - cur[i]) > MAX_STEP:
                                rep = f"ERR STEP J{i + 1}"
                                break
                        if rep is None:
                            z = M.fk(np.deg2rad(v), M.T_GRIPPER1)[2, 3] * 1000
                            if z < Z_MIN:
                                rep = f"ERR ZMIN {z:.1f}"
                            else:
                                state.move_to(v)
                                rep = "DONE " + " ".join(f"{x:.2f}" for x in state.joints())
                elif msg == "BYE":
                    self.wfile.write(b"BYE\n")
                    return
                else:
                    rep = "ERR UNKNOWN"
                self.wfile.write((rep + "\n").encode())

    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--tcp-port", type=int, default=1025)
    ap.add_argument("--speed", type=float, default=20.0, help="deg/s del simulador")
    a = ap.parse_args()
    st = SimState(a.speed)
    http = ThreadingHTTPServer((a.host, a.http_port), make_http(st))
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    tcp = socketserver.ThreadingTCPServer((a.host, a.tcp_port), make_tcp(st))
    threading.Thread(target=http.serve_forever, daemon=True).start()
    threading.Thread(target=tcp.serve_forever, daemon=True).start()
    print(f"mock IRC5: RWS http://{a.host}:{a.http_port}  | ROS_BRIDGE tcp {a.host}:{a.tcp_port}  (Ctrl+C para salir)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
