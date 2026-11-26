"""
GATE 1 - Comunicacion y lectura de estado SIN movimiento (Robot Web Services, solo GET).

Uso (Windows o Linux):
  python tools/gate1_leer_estado.py --url http://127.0.0.1:8080          # mock_irc5
  python tools/gate1_leer_estado.py --url http://localhost                # RobotStudio (VC)
  python tools/gate1_leer_estado.py --url http://192.168.125.1 --seg 30   # IRC5 real (con supervisor)

Prueba G1-B del protocolo: lee 'seg' segundos a 'hz' Hz y guarda:
  results/gate1_<fecha>.csv   (serie temporal: q [deg], TCP tool0 [mm], estados)
  results/gate1_<fecha>.json  (resumen: tasa real, errores, estados observados)
Criterio de aceptacion: >= 95 % de lecturas validas y q coincide con el FlexPendant (+-0.01 deg, foto).
"""
import argparse
import csv
import json
import os
import time
from datetime import datetime

import _paths  # noqa: F401
from abb_rws import RWSClient

ap = argparse.ArgumentParser()
ap.add_argument("--url", default="http://192.168.125.1")
ap.add_argument("--user", default="Default User")
ap.add_argument("--password", default="robotics")
ap.add_argument("--seg", type=float, default=10.0)
ap.add_argument("--hz", type=float, default=5.0)
a = ap.parse_args()

cli = RWSClient(a.url, a.user, a.password)
tag = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path = os.path.join(_paths.RESULTS, f"gate1_{tag}.csv")
print(f"Gate 1: leyendo {a.url} durante {a.seg:.0f} s a {a.hz:.1f} Hz (solo lectura)")

n_ok = n_tot = 0
lat = []
states = set()
first_err = None
with open(csv_path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_unix", "q1", "q2", "q3", "q4", "q5", "q6", "x_mm", "y_mm", "z_mm",
                "qw", "qx", "qy", "qz", "opmode", "ctrlstate", "execstate", "latencia_s", "errores"])
    t_end = time.time() + a.seg
    k = 0
    while time.time() < t_end:
        t0 = time.time()
        st = cli.read_all(with_status=(k % max(int(a.hz), 1) == 0))   # estados ~1 Hz
        dt = time.time() - t0
        n_tot += 1
        ok = st.joints_deg is not None
        n_ok += ok
        lat.append(dt)
        if st.errors and first_err is None:
            first_err = st.errors
        if st.opmode:
            states.add((st.opmode, st.ctrlstate, st.execstate))
        w.writerow([f"{st.stamp:.3f}", *(st.joints_deg or [""] * 6), *(st.tcp_mm or [""] * 3),
                    *(st.tcp_quat or [""] * 4), st.opmode, st.ctrlstate, st.execstate, f"{dt:.4f}",
                    json.dumps(st.errors) if st.errors else ""])
        if k % max(int(a.hz), 1) == 0:
            qtxt = " ".join(f"{v:8.3f}" for v in st.joints_deg) if ok else "SIN DATOS"
            print(f"  q[deg]= {qtxt} | {st.opmode}/{st.ctrlstate}/{st.execstate} | {dt * 1000:.0f} ms")
        k += 1
        time.sleep(max(0.0, 1.0 / a.hz - (time.time() - t0)))

lat_sorted = sorted(lat)
summary = dict(url=a.url, inicio=tag, lecturas=n_tot, validas=n_ok,
               tasa_validas=n_ok / n_tot if n_tot else 0.0,
               latencia_media_ms=1000 * sum(lat) / len(lat) if lat else None,
               latencia_p95_ms=1000 * lat_sorted[int(0.95 * (len(lat) - 1))] if lat else None,
               estados_observados=sorted(states), primer_error=first_err,
               aceptado=bool(n_tot and n_ok / n_tot >= 0.95), csv=os.path.basename(csv_path))
with open(csv_path.replace(".csv", ".json"), "w") as f:
    json.dump(summary, f, indent=2, ensure_ascii=False)
print(json.dumps(summary, indent=2, ensure_ascii=False))
