"""
GATE 3 - consola de comando articular por socket (SIN ROS), para la primera prueba autorizada.
Solo usar con decision GO firmada, robot en MANUAL (velocidad reducida) y supervisor presente.

Uso:  python tools/gate3_cli.py --host 192.168.125.1 --port 1025
Comandos:  getj | getp | spd <mm/s> | movj j1 j2 j3 j4 j5 j6 (deg) | salir
Cada movj muestra objetivo, salto por eje y altura prevista del TCP (modelo propio) y pide
escribir SI para confirmar. Todo se registra en results/gate3_<fecha>.log
"""
import argparse
import os
from datetime import datetime

import numpy as np

import _paths  # noqa: F401
import irb120_model as M
from rapid_client import RapidBridgeClient, RapidBridgeError

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="192.168.125.1")
ap.add_argument("--port", type=int, default=1025)
a = ap.parse_args()

log_path = os.path.join(_paths.RESULTS, f"gate3_{datetime.now():%Y%m%d_%H%M%S}.log")
logf = open(log_path, "a", encoding="utf-8")


def log(msg):
    line = f"{datetime.now().isoformat(timespec='milliseconds')} {msg}"
    print(line)
    logf.write(line + "\n")
    logf.flush()


c = RapidBridgeClient(a.host, a.port)
c.connect()
log(f"conectado a {a.host}:{a.port}")
try:
    while True:
        try:
            cmd = input("gate3> ").strip()
        except EOFError:
            break
        if not cmd:
            continue
        if cmd in ("salir", "exit", "q"):
            break
        try:
            if cmd == "getj":
                log(f"GETJ -> {c.get_joints_deg()}")
            elif cmd == "getp":
                log(f"GETP -> {c.request('GETP', 5)}")
            elif cmd.startswith("spd "):
                log(f"{cmd} -> {c.set_speed(float(cmd.split()[1]))}")
            elif cmd.startswith("movj "):
                tgt = [float(x) for x in cmd.split()[1:]]
                cur = c.get_joints_deg()
                err = c.validate(tgt, cur)
                if err:
                    log(f"RECHAZO LOCAL {tgt}: {err}")
                    continue
                z = M.fk(np.deg2rad(tgt), M.T_GRIPPER1)[2, 3] * 1000
                print(f"  actual  : {np.round(cur, 2)}\n  objetivo: {np.round(tgt, 2)}\n"
                      f"  salto   : {np.round(np.subtract(tgt, cur), 2)}\n  z TCP previsto (modelo propio): {z:.1f} mm")
                try:
                    resp = input("  Supervisor presente y zona despejada? escriba SI: ").strip()
                except EOFError:
                    resp = ""
                if resp != "SI":
                    log("movimiento cancelado por el operador")
                    continue
                log(f"MOVJ {tgt}")
                res = c.move_joints_deg(tgt)
                log(f"DONE {res}  error final por eje [deg]: {np.round(np.subtract(res, tgt), 3).tolist()}")
            else:
                print("comandos: getj | getp | spd <v> | movj j1..j6 | salir")
        except (RapidBridgeError, ValueError, OSError) as e:
            log(f"ERROR {cmd}: {e}")
finally:
    c.close()
    log("desconectado")
    logf.close()
