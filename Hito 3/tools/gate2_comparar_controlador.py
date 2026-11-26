"""
GATE 2 (contra el controlador) - compara el modelo PROPIO con la cinematica del IRC5.

Para cada pose: lee por RWS q (jointtarget) y el TCP de tool0 (robtarget, wobj0, Base);
calcula FK propia con esas q y reporta e_p y e_R. El robot debe estar DETENIDO en cada pose
(las dos lecturas no son simultaneas). Sin movimiento comandado desde el PC: las poses se
alcanzan con el joystick del FlexPendant (modo manual) o en RobotStudio.

Uso:
  python tools/gate2_comparar_controlador.py --url http://localhost         # RobotStudio
  python tools/gate2_comparar_controlador.py --url http://192.168.125.1     # robot real
Presione Enter en cada pose; 'q' + Enter para terminar.
Salida: results/gate2_controller_cases.csv (esquema validation_cases del protocolo).
Tolerancia: e_p <= 1.0 mm y e_R <= 0.2 deg (redondeo de RWS, calibracion nominal del controlador).
"""
import argparse
import csv
import os
from datetime import datetime

import numpy as np

import _paths  # noqa: F401
import irb120_model as M
from abb_rws import RWSClient

TOL_P, TOL_R = 1.0e-3, np.deg2rad(0.2)
ap = argparse.ArgumentParser()
ap.add_argument("--url", default="http://192.168.125.1")
ap.add_argument("--auto", type=int, default=0, help="N lecturas sin esperar Enter (pruebas)")
a = ap.parse_args()
cli = RWSClient(a.url, tool="tool0", wobj="wobj0")

out = os.path.join(_paths.RESULTS, "gate2_controller_cases.csv")
new = not os.path.exists(out)
f = open(out, "a", newline="")
w = csv.writer(f)
if new:
    w.writerow(["case_id", "case_type", "q_rad_vector", "base_frame", "tcp_frame", "model_version",
                "reference_version", "position_error_m", "orientation_error_rad", "jacobian_error_fro",
                "rank_threshold", "rank_observed", "acceptance_result", "evidence_link", "notes"])

k = 0
while True:
    if a.auto:
        if k >= a.auto:
            break
    elif input(f"[pose {k + 1}] Enter para medir, 'q' para salir: ").strip().lower() == "q":
        break
    q_deg = cli.joints_deg()
    p_mm, quat = cli.tcp()
    q = np.deg2rad(q_deg)
    T_own = M.fk(q)                                  # tool0
    T_ctrl = np.eye(4)
    T_ctrl[:3, :3] = M.quat_wxyz_to_R(quat)
    T_ctrl[:3, 3] = np.array(p_mm) / 1000.0
    ep, _ = M.pose_errors(T_ctrl, T_own)
    eR = M.rot_angle_error_robust(T_ctrl[:3, :3], T_own[:3, :3])
    ok = ep <= TOL_P and eR <= TOL_R
    cid = f"G2C_{datetime.now():%Y%m%d_%H%M%S}_{k + 1}"
    w.writerow([cid, "FK_controller", " ".join(f"{v:.6f}" for v in q), "base(wobj0)", "tool0",
                "irb120_model.py", f"IRC5 RWS {a.url}", f"{ep:.3e}", f"{eR:.3e}", "", "", "",
                "pass" if ok else "fail", "results/gate2_controller_cases.csv",
                f"q_deg={np.round(q_deg, 3).tolist()} p_ctrl_mm={np.round(p_mm, 2).tolist()} "
                f"p_own_mm={np.round(T_own[:3, 3] * 1000, 2).tolist()}"])
    f.flush()
    print(f"  q={np.round(q_deg, 2)} | p_ctrl={np.round(p_mm, 2)} mm | p_own={np.round(T_own[:3, 3] * 1000, 2)} mm"
          f" | e_p={ep * 1000:.3f} mm e_R={np.rad2deg(eR):.4f} deg -> {'PASS' if ok else 'FAIL'}")
    k += 1
f.close()
print(f"Guardado en {out}")
