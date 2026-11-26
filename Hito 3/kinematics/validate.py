"""
Protocolo de validacion cruzada FK / IK / J (Gate 2) - modelo propio vs URDF de referencia.

Ejecutar:  python kinematics/validate.py
Salidas en results/:
  validation_cases.csv       (esquema del protocolo)
  validation_details.json    (todos los numeros, para el informe)
  fig_fd_sensitivity.png     (sensibilidad de diferencias finitas al paso eps)
  fig_singular_values.png    (valores singulares cerca de singularidades)
"""
from __future__ import annotations

import csv
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import irb120_model as M  # noqa: E402
from reference_urdf import UrdfChainNumpy, load_reference  # noqa: E402

OUT = os.path.join(HERE, "..", "results")
os.makedirs(OUT, exist_ok=True)

# --------------------------------------------------------------------------- configuracion
TOL = dict(ep=1e-9, eR=1e-6, eJ=1e-9, ev=1e-9)   # modelo vs modelo (software): precision numerica
RANK_TOL = 1e-2       # umbral relativo sigma_i / sigma_max
L_CHAR = 0.5          # m, longitud caracteristica para escalar filas lineales de J
TCP_FRAME = "gripper1_tcp"
T_TCP = M.T_GRIPPER1
RNG = np.random.default_rng(42)


def deg(*v):
    return np.deg2rad(np.array(v, dtype=float))


def sha_short(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()[:10]


MODEL_VERSION = f"irb120_model.py@{sha_short(os.path.join(HERE, 'irb120_model.py'))}"
URDF_VERSION = f"irb120_3_58_gripper1.urdf@{sha_short(os.path.join(HERE, '..', 'description', 'irb120_3_58_gripper1.urdf'))}"

ref = load_reference()
ref_np = UrdfChainNumpy()
REF_VERSION = f"{ref.name}+{URDF_VERSION}"

rows, details = [], {"model_version": MODEL_VERSION, "reference_version": REF_VERSION,
                     "tolerances": TOL, "rank_tol_rel": RANK_TOL, "L_char_m": L_CHAR,
                     "tcp_frame": TCP_FRAME, "cases": {}}


def add_row(case_id, case_type, q, ep="", eR="", eJ="", rank_obs="", ok=None, notes=""):
    rows.append(dict(
        case_id=case_id, case_type=case_type,
        q_rad_vector=" ".join(f"{v:.6f}" for v in q) if q is not None else "",
        base_frame="base_link", tcp_frame=TCP_FRAME,
        model_version=MODEL_VERSION, reference_version=REF_VERSION,
        position_error_m=f"{ep:.3e}" if ep != "" else "",
        orientation_error_rad=f"{eR:.3e}" if eR != "" else "",
        jacobian_error_fro=f"{eJ:.3e}" if eJ != "" else "",
        rank_threshold=RANK_TOL if rank_obs != "" else "",
        rank_observed=rank_obs,
        acceptance_result=("pass" if ok else "fail") if ok is not None else "info",
        evidence_link="results/validation_details.json",
        notes=notes))


# --------------------------------------------------------------------------- 1. FK
fk_cases = {"FK_N1": deg(0, 0, 0, 0, 30, 0), "FK_N2": deg(30, 20, -15, 45, 60, -30),
            "FK_HOME": deg(0, 0, 0, 0, 0, 0)}
for cid, q in fk_cases.items():
    T_own = M.fk(q, T_TCP)
    T_ref = ref.fk(q, TCP_FRAME)
    ep, eR = M.pose_errors(T_ref, T_own)
    eR_rob = M.rot_angle_error_robust(T_ref[:3, :3], T_own[:3, :3])
    ok = ep <= TOL["ep"] and eR <= TOL["eR"]
    details["cases"][cid] = dict(q_deg=np.rad2deg(q).tolist(), p_own=T_own[:3, 3].tolist(),
                                 p_ref=T_ref[:3, 3].tolist(), quat_own=M.R_to_quat_wxyz(T_own[:3, :3]).tolist(),
                                 quat_ref=M.R_to_quat_wxyz(T_ref[:3, :3]).tolist(),
                                 ep=ep, eR_acos=eR, eR_atan2=eR_rob)
    add_row(cid, "FK", q, ep, eR, ok=ok, notes=f"eR(atan2)={eR_rob:.2e}")

# Monte Carlo
N = 5000
eps_p, eps_R, eps_Rr = [], [], []
for _ in range(N):
    q = RNG.uniform(M.Q_MIN, M.Q_MAX)
    T_own, T_ref = M.fk(q, T_TCP), ref.fk(q, TCP_FRAME)
    ep, eR = M.pose_errors(T_ref, T_own)
    eps_p.append(ep); eps_R.append(eR)
    eps_Rr.append(M.rot_angle_error_robust(T_ref[:3, :3], T_own[:3, :3]))
details["cases"]["FK_MC"] = dict(N=N, ep_max=max(eps_p), ep_mean=float(np.mean(eps_p)),
                                 eR_acos_max=max(eps_R), eR_atan2_max=max(eps_Rr))
add_row("FK_MC5000", "FK", None, max(eps_p), max(eps_R),
        ok=max(eps_p) <= TOL["ep"] and max(eps_R) <= TOL["eR"],
        notes=f"N={N} q uniformes en limites; max eR(atan2)={max(eps_Rr):.2e}; piso de acos ~1e-8")

# segunda referencia independiente (parser numpy) para descartar errores de la referencia
q = fk_cases["FK_N2"]
ep2, eR2 = M.pose_errors(ref_np.fk(q, TCP_FRAME), M.fk(q, T_TCP))
details["cases"]["FK_N2_vs_urdf_numpy"] = dict(ep=ep2, eR=eR2)

# --------------------------------------------------------------------------- 2. IK
def ik_case(cid, T_des, note):
    sols = M.ik(T_des, T_TCP, only_within_limits=False)
    info = []
    all_ok = True
    for s in sols:
        T_k = M.fk(s["q"], T_TCP)
        ep, eR = M.pose_errors(T_des, T_k)
        lim = M.within_limits(s["q"])
        ok = ep <= TOL["ep"] and eR <= TOL["eR"]
        all_ok &= ok
        info.append(dict(branch=s["branch"], q_deg=np.rad2deg(s["q"]).round(4).tolist(),
                         ep=ep, eR=eR, within_limits=lim))
    # referencia: IK numerica (CLIK) en Pinocchio desde varias semillas
    ref_sols = []
    if hasattr(ref, "ik_numeric"):
        seeds = [np.zeros(6), deg(0, 30, -30, 0, 60, 0), deg(180, 0, 0, 0, 30, 0),
                 deg(0, -30, -100, 90, 45, 0)]
        for sd in seeds:
            qr, res = ref.ik_numeric(T_des, TCP_FRAME, sd)
            qr_w = M.wrap_to_pi(qr)
            ep, eR = M.pose_errors(T_des, M.fk(qr_w, T_TCP))
            match = [s["branch"] for s in sols if np.allclose(M.wrap_to_pi(s["q"] - qr_w), 0, atol=1e-5)]
            ref_sols.append(dict(seed_deg=np.rad2deg(sd).tolist(), q_deg=np.rad2deg(qr_w).round(4).tolist(),
                                 residual=res, ep_via_own_fk=ep, eR_via_own_fk=eR,
                                 within_limits=M.within_limits(qr_w),
                                 matches_own_branch=match[0] if match else None))
    n_in = sum(i["within_limits"] for i in info)
    details["cases"][cid] = dict(note=note, T_des=T_des.tolist(), own_solutions=info, ref_solutions=ref_sols)
    max_ep = max(i["ep"] for i in info)
    max_eR = max(i["eR"] for i in info)
    add_row(cid, "IK", None, max_ep, max_eR, ok=all_ok and len(info) > 0,
            notes=f"{len(info)} ramas propias ({n_in} dentro de limites); "
                  f"ref CLIK: {sum(r['matches_own_branch'] is not None for r in ref_sols)}/{len(ref_sols)} "
                  f"semillas convergen a una rama propia")


q_star = deg(20, 15, -10, 30, 45, 10)
ik_case("IK_1", M.fk(q_star, T_TCP), "pose generada por FK de q*=[20,15,-10,30,45,10] deg")
T_board = np.eye(4)
T_board[:3, :3] = np.diag([-1.0, 1.0, -1.0])   # TCP apuntando hacia abajo (quat ABB [0,0,1,0])
T_board[:3, 3] = [0.35, 0.10, 0.20]
ik_case("IK_2_tablero", T_board, "TCP vertical hacia abajo en p=[0.35,0.10,0.20] m (tipo casilla de tablero)")

# --------------------------------------------------------------------------- 3. Jacobiano
def j_case(cid, q, note, typ="J"):
    J_own = M.jacobian(q, T_TCP)
    J_ref = ref.jacobian(q, TCP_FRAME)
    eJ = float(np.linalg.norm(J_own - J_ref, "fro"))
    qd = RNG.normal(size=6)
    ev = float(np.linalg.norm(J_own @ qd - J_ref @ qd))
    s_own = M.singular_values(q, T_TCP, L_CHAR)
    Js = J_ref.copy(); Js[:3] /= L_CHAR
    s_ref = np.linalg.svd(Js, compute_uv=False)
    r_own = int(np.sum(s_own > RANK_TOL * s_own[0]))
    r_ref = int(np.sum(s_ref > RANK_TOL * s_ref[0]))
    fd = {}
    for e in [1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-10]:
        fd[f"{e:.0e}"] = float(np.linalg.norm(M.jacobian_fd_linear(q, T_TCP, e) - J_own[:3], "fro"))
    ok = eJ <= TOL["eJ"] and ev <= TOL["ev"] and r_own == r_ref
    details["cases"][cid] = dict(note=note, q_deg=np.rad2deg(q).tolist(), eJ_fro=eJ, ev=ev,
                                 sigma_own=s_own.tolist(), sigma_ref=s_ref.tolist(),
                                 cond_own=float(s_own[0] / s_own[-1]) if s_own[-1] > 0 else float("inf"),
                                 rank_own=r_own, rank_ref=r_ref, fd_error_vs_eps=fd)
    add_row(cid, typ, q, eJ=eJ, rank_obs=f"{r_own}/{r_ref}", ok=ok,
            notes=f"{note}; ev={ev:.1e}; sigma_min={s_own[-1]:.2e}; rank propio/ref")
    return J_own


J_nom = j_case("J_N1", deg(10, 25, -20, 35, 50, -15), "configuracion regular")
j_case("SING_MUNECA", deg(10, 25, -20, 35, 0.5, -15), "q5=0.5 deg (ejes 4 y 6 casi colineales)", "SING")
j_case("SING_MUNECA_EXACTA", deg(10, 25, -20, 35, 0.0, -15), "q5=0 exacto", "SING")
q3_stretch = -np.rad2deg(M.PHI_FORE) + 0.5
j_case("SING_CODO", deg(0, 30, q3_stretch, 0, 45, 0), f"brazo extendido q3={q3_stretch:.2f} deg", "SING")
# hombro: centro de muneca sobre el eje 1 -> buscar q2 con q3=0
from scipy_free_root import bisect  # noqa: E402  (ver modulo auxiliar)
q2_sh = bisect(lambda a: M.fk_all(np.r_[0, a, 0, 0, 0, 0])[4][0, 3], np.deg2rad(-80), np.deg2rad(-10))
j_case("SING_HOMBRO", np.r_[0, q2_sh + np.deg2rad(0.5), 0, 0, np.deg2rad(45), 0],
       f"centro de muneca ~sobre el eje 1 (q2={np.rad2deg(q2_sh):.2f}+0.5 deg)", "SING")

# --------------------------------------------------------------------------- 4. Estatica J^T F
F = np.array([0, 0, -3.0 * 9.81, 0, 0, 0])  # carga de 3 kg en el TCP, sin momento
tau_ext = J_nom.T @ F
details["static_example"] = dict(
    q_deg=[10, 25, -20, 35, 50, -15], wrench_B=F.tolist(), tau_ext_Nm=tau_ext.round(4).tolist(),
    note="tau_ext = J^T F mapea un wrench externo estatico; NO incluye M(q)qdd + C(q,qd)qd + g(q) "
         "(el peso propio de los eslabones y la dinamica). Es solo la contribucion de F.")

# --------------------------------------------------------------------------- salida
with open(os.path.join(OUT, "validation_cases.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)
with open(os.path.join(OUT, "validation_details.json"), "w", encoding="utf-8") as f:
    json.dump(details, f, indent=2, ensure_ascii=False, default=float)

# --------------------------------------------------------------------------- figuras
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    eps_list = np.logspace(-12, -1, 45)
    qn = deg(10, 25, -20, 35, 50, -15)
    Jv = M.jacobian(qn, T_TCP)[:3]
    err = [np.linalg.norm(M.jacobian_fd_linear(qn, T_TCP, e) - Jv, "fro") for e in eps_list]
    fig, ax = plt.subplots(figsize=(6.4, 4))
    ax.loglog(eps_list, err, "o-", ms=3, color="#2a6fdb")
    ax.set_xlabel("paso ε [rad]")
    ax.set_ylabel(r"$\|J_v^{FD}(\varepsilon) - J_v\|_F$  [m/rad]")
    ax.set_title("Diferencias finitas centrales: truncamiento vs. redondeo")
    ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_fd_sensitivity.png"), dpi=160)

    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8))
    q5s = np.linspace(-30, 30, 241)
    S = np.array([M.singular_values(deg(10, 25, -20, 35, a, -15), T_TCP, L_CHAR) for a in q5s])
    axs[0].semilogy(q5s, S[:, -1], label=r"$\sigma_{min}$", color="#d9480f")
    axs[0].semilogy(q5s, S[:, 0], label=r"$\sigma_{max}$", color="#2a6fdb")
    axs[0].axhline(RANK_TOL * S[:, 0].max(), ls="--", color="gray", lw=1, label="umbral rango")
    axs[0].set_xlabel("q5 [deg]"); axs[0].set_title("Singularidad de muñeca"); axs[0].legend(fontsize=8)
    q3s = np.linspace(-105, 65, 341)
    S = np.array([M.singular_values(deg(0, 30, a, 0, 45, 0), T_TCP, L_CHAR) for a in q3s])
    axs[1].semilogy(q3s, S[:, -1], color="#d9480f", label=r"$\sigma_{min}$")
    axs[1].axvline(-np.rad2deg(M.PHI_FORE), ls=":", color="k", lw=1, label="brazo extendido")
    axs[1].set_xlabel("q3 [deg]"); axs[1].set_title("Singularidad de codo"); axs[1].legend(fontsize=8)
    for a in axs:
        a.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_singular_values.png"), dpi=160)
except ImportError:
    print("matplotlib no disponible: se omiten figuras")

# --------------------------------------------------------------------------- resumen consola
print(f"Modelo: {MODEL_VERSION}\nReferencia: {REF_VERSION}\n")
print(f"{'case_id':22s} {'tipo':5s} {'e_p [m]':>10s} {'e_R [rad]':>10s} {'e_J':>10s} {'rango':>6s}  resultado")
for r in rows:
    print(f"{r['case_id']:22s} {r['case_type']:5s} {r['position_error_m']:>10s} {r['orientation_error_rad']:>10s} "
          f"{r['jacobian_error_fro']:>10s} {str(r['rank_observed']):>6s}  {r['acceptance_result']}")
print("\ntau_ext = J^T F (3 kg en TCP) [N m]:", details["static_example"]["tau_ext_Nm"])
