"""
Graficas de la sesion de laboratorio a partir del registro del panel (results/.../panel_*.jsonl).

  python tools/graficas_laboratorio.py results/laboratorio_2026-11-21/panel_20261121_163603.jsonl

Genera en la misma carpeta:
  fig_lab_posicion_tcp.png      posicion x, y, z del TCP tras cada movimiento (medido vs objetivo) y el rechazo
  fig_lab_errores_llegada.png   error de llegada del TCP y error maximo por eje, por movimiento
  fig_lab_done_vs_rws.png       diferencia entre el valor DONE del RAPID y la lectura RWS posterior
  fig_lab_ik_mov3.png           movimiento 3 (IK): postura antes y despues, modelo propio
  resumen_movimientos.csv       tabla de los movimientos
"""
import csv
import json
import os
import sys
from datetime import datetime

import numpy as np

import _paths  # noqa: F401
from irb120_bridge import analisis as A
from irb120_bridge import irb120_model as M

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

AZUL, NARANJA = "#2a78d6", "#eb6834"          # paleta validada (2 series)
TINTA, TINTA2, REJILLA, FONDO = "#13202E", "#5F6B78", "#E3E0D8", "#FFFFFF"
plt.rcParams.update({"font.size": 11, "axes.edgecolor": TINTA2, "axes.labelcolor": TINTA, "xtick.color": TINTA2,
                     "ytick.color": TINTA2, "axes.titlecolor": TINTA, "axes.grid": True, "grid.color": REJILLA,
                     "grid.linewidth": 0.8, "axes.spines.top": False, "axes.spines.right": False,
                     "figure.facecolor": FONDO, "axes.facecolor": FONDO})

ruta = sys.argv[1]
out = os.path.dirname(os.path.abspath(ruta))
ev = [json.loads(line) for line in open(ruta, encoding="utf-8") if line.strip()]

movs, cur, ult = [], None, None
for e in ev:
    if e["evento"] == "analisis":
        ult = e
    elif e["evento"] == "envio":
        cur = dict(an=ult, t_envio=e["t"])
    elif e["evento"] == "tramo_ok" and cur:
        cur["done"] = json.loads(e["texto"].split("alcanzado: ")[1])
        cur["t_done"] = e["t"]
    elif e["evento"] == "llegada" and cur:
        cur["ll"] = e
        movs.append(cur)
        cur = None
rechazos = [e for e in ev if e["evento"] == "analisis" and not e["apto"]]

filas = []
for i, m in enumerate(movs, 1):
    a, ll = m["an"], m["ll"]
    q0, qo, qm = (np.array(a["q_actual_deg"]), np.array(a["q_objetivo_deg"]), np.array(ll["q_medido"]))
    dur = (datetime.fromisoformat(m["t_done"]) - datetime.fromisoformat(m["t_envio"])).total_seconds()
    modelo = next(c["detalle"] for c in a["chequeos"] if c["nombre"] == "Modelo vs controlador")
    rama = next((r["rama"] for r in a["ramas"] if r["elegida"]), "-")
    filas.append(dict(n=i, tipo="Pose (IK)" if a["modo"] == "pose" else "Ejes (FK)", rama=rama,
                      salto_deg=float(np.max(np.abs(qo - q0))), duracion_s=dur, ep_mm=ll["ep_mm"],
                      eR_deg=ll["eR_deg"], err_eje_max_deg=max(ll["error_eje_deg"]),
                      done_vs_rws_deg=float(np.max(np.abs(np.array(m["done"]) - qm))), modelo=modelo,
                      q0=q0, qo=qo, qm=qm))

with open(os.path.join(out, "resumen_movimientos.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["mov", "tipo", "rama_IK", "salto_max_deg", "duracion_s", "e_p_llegada_mm", "e_R_llegada_deg",
                "error_max_eje_deg", "DONE_vs_RWS_deg", "modelo_vs_controlador"])
    for r in filas:
        w.writerow([r["n"], r["tipo"], r["rama"], f"{r['salto_deg']:.1f}", f"{r['duracion_s']:.1f}",
                    f"{r['ep_mm']:.3f}", f"{r['eR_deg']:.4f}", f"{r['err_eje_max_deg']:.4f}",
                    f"{r['done_vs_rws_deg']:.3f}", r["modelo"]])

color = lambda r: NARANJA if r["tipo"].startswith("Pose") else AZUL  # noqa: E731


def camino(q0, q1, n=40):
    return np.array([A.fk_tcp(q0 + (q1 - q0) * s)[:3, 3] * 1000 for s in np.linspace(0, 1, n)])


# ---------------------------------------------------------------- 1. posicion del TCP tras cada movimiento
from matplotlib.lines import Line2D  # noqa: E402
pos = [A.fk_tcp(filas[0]["q0"])[:3, 3] * 1000] + [A.fk_tcp(r["qm"])[:3, 3] * 1000 for r in filas]
obj = [None] + [A.fk_tcp(r["qo"])[:3, 3] * 1000 for r in filas]
pos = np.array(pos)
fig, axs = plt.subplots(1, 3, figsize=(12, 3.9), dpi=200)
pasos = np.arange(len(pos))
for k, (ax, nom) in enumerate(zip(axs, ("x", "y", "z"))):
    ax.plot(pasos, pos[:, k], color=TINTA2, lw=1.5, zorder=1)
    ax.scatter([0], [pos[0, k]], s=40, color=FONDO, edgecolor=TINTA, linewidth=1.5, zorder=3)
    for r in filas:
        ax.scatter([r["n"]], [pos[r["n"], k]], s=60, color=color(r), edgecolor=FONDO, linewidth=1.5, zorder=3)
        ax.scatter([r["n"]], [obj[r["n"]][k]], s=140, facecolor="none", edgecolor=TINTA, linewidth=1, zorder=2)
    ax.set(title=f"{nom} del TCP [mm]", xticks=pasos, xticklabels=["inicio"] + [str(i) for i in pasos[1:]],
           xlabel="después del movimiento")
    ax.grid(axis="x", visible=False)
for rj in rechazos:
    zr = rj["pose_deseada"][2]
    axs[2].scatter([4.5], [zr], marker="x", s=90, color=TINTA, linewidth=2.5, zorder=3)
    axs[2].annotate("z = 40 mm pedido:\nrechazado, sin movimiento", (4.5, zr), xytext=(-150, 4),
                    textcoords="offset points", fontsize=9, color=TINTA, va="center")
axs[2].axhline(A.Z_MIN_MM_DEF, color=TINTA2, lw=1, ls="--")
axs[2].text(0.02, A.Z_MIN_MM_DEF + 14, "z mínima 100 mm", transform=axs[2].get_yaxis_transform(),
            color=TINTA2, fontsize=9)
axs[2].set_ylim(0, 660)
axs[0].legend(handles=[Line2D([], [], marker="o", ls="", color=AZUL, ms=8, label="medido · ejes (FK)"),
                       Line2D([], [], marker="o", ls="", color=NARANJA, ms=8, label="medido · pose (IK)"),
                       Line2D([], [], marker="o", ls="", mfc="none", mec=TINTA, ms=11, label="objetivo")],
              fontsize=9, frameon=False, loc="lower left")
handles, labels = axs[0].get_legend_handles_labels()
axs[0].get_legend().remove()
fig.legend(handles=[Line2D([], [], marker="o", ls="", color=AZUL, ms=8, label="medido · ejes (FK)"),
                    Line2D([], [], marker="o", ls="", color=NARANJA, ms=8, label="medido · pose (IK)"),
                    Line2D([], [], marker="o", ls="", mfc="none", mec=TINTA, ms=11, label="objetivo")],
           loc="lower center", ncol=3, fontsize=10, frameon=False)
fig.suptitle("Posición del TCP después de cada movimiento: lo medido cae dentro del círculo del objetivo (error ≤ 0,06 mm)",
             color=TINTA, fontsize=12, x=0.01, ha="left")
fig.tight_layout(rect=(0, 0.07, 1, 1))
fig.savefig(os.path.join(out, "fig_lab_posicion_tcp.png"))
plt.close(fig)

# ---------------------------------------------------------------- 2. errores de llegada
fig, axs = plt.subplots(1, 2, figsize=(11, 3.8), dpi=200)
xs = [r["n"] for r in filas]
for ax, clave, fmt, tit, ylab in ((axs[0], "ep_mm", "{:.3f}", "Error de posición del TCP al llegar", "e_p [mm]"),
                                  (axs[1], "err_eje_max_deg", "{:.3f}", "Error máximo por eje al llegar", "error [°]")):
    vals = [r[clave] for r in filas]
    ax.bar(xs, vals, color=[color(r) for r in filas], width=0.6, edgecolor=FONDO, linewidth=2)
    for x, v in zip(xs, vals):
        ax.text(x, v, fmt.format(v), ha="center", va="bottom", fontsize=9, color=TINTA)
    ax.set(title=tit, ylabel=ylab, xticks=xs, xlabel="movimiento")
    ax.set_ylim(0, max(vals) * 1.25)
    ax.grid(axis="x", visible=False)
axs[0].legend(handles=[Line2D([], [], color=AZUL, lw=6, label="Ejes (FK)"),
                       Line2D([], [], color=NARANJA, lw=6, label="Pose (IK)")], fontsize=9, frameon=False,
              loc="upper left")
fig.suptitle("Precisión de llegada medida en el robot real · tolerancia de aceptación 1 mm (≈ 20× mayor que el peor caso)",
             color=TINTA, fontsize=12, x=0.01, ha="left")
fig.tight_layout()
fig.savefig(os.path.join(out, "fig_lab_errores_llegada.png"))
plt.close(fig)

# ---------------------------------------------------------------- 3. DONE vs RWS
fig, ax = plt.subplots(figsize=(6, 3.4), dpi=200)
vals = [r["done_vs_rws_deg"] for r in filas]
ax.bar(xs, vals, color=TINTA2, width=0.6)
for x, v in zip(xs, vals):
    ax.text(x, v, f"{v:.2f}°", ha="center", va="bottom", fontsize=9, color=TINTA)
ax.set(title="DONE del RAPID vs lectura RWS ~0,2 s después", ylabel="diferencia máx. [°]", xticks=xs,
       xlabel="movimiento")
ax.set_ylim(0, max(vals) * 1.3)
ax.grid(axis="x", visible=False)
fig.tight_layout()
fig.savefig(os.path.join(out, "fig_lab_done_vs_rws.png"))
plt.close(fig)

# ---------------------------------------------------------------- 4. IK movimiento 3 (antes / despues)
r3 = next(r for r in filas if r["tipo"].startswith("Pose"))
fig = plt.figure(figsize=(6.4, 5.2), dpi=200)
ax = fig.add_subplot(111, projection="3d")
for q, c, lab, ls in ((r3["q0"], TINTA, "antes (lectura)", "-"), (r3["qm"], NARANJA, "después (lectura)", "--")):
    P = np.array([T[:3, 3] for T in M.fk_all(np.deg2rad(q), A.T_TCP)]) * 1000
    ax.plot(P[:, 0], P[:, 1], P[:, 2], color=c, lw=3, ls=ls, marker="o", ms=3, label=lab)
ax.set(xlabel="x [mm]", ylabel="y [mm]", zlabel="z [mm]", xlim=(-100, 700), ylim=(-300, 400), zlim=(0, 700))
ax.set_box_aspect((0.8, 0.7, 0.7))
ax.view_init(elev=18, azim=-60)
ax.set_title(f"Movimiento {r3['n']}: bajar 30 mm por IK (rama {r3['rama']})", color=TINTA, fontsize=11)
ax.legend(loc="upper left", fontsize=9, frameon=False)
fig.tight_layout()
fig.savefig(os.path.join(out, "fig_lab_ik_mov3.png"))
plt.close(fig)

for r in filas:
    print(f"{r['n']}  {r['tipo']:10s} {r['rama']:16s} salto {r['salto_deg']:5.1f}°  {r['duracion_s']:5.1f} s  "
          f"e_p {r['ep_mm']:.3f} mm  eje {r['err_eje_max_deg']:.4f}°  DONE-RWS {r['done_vs_rws_deg']:.3f}°")
print("figuras en", out)
