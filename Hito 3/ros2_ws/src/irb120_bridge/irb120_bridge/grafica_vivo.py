"""
Grafica en vivo de los movimientos del IRB 120 (se abre desde el panel con "Gráfica en vivo").

Tres graficas con el mismo eje de tiempo:
  1. Angulos J1..J6 leidos del robot [deg]
  2. Posicion del TCP (x, y, z) calculada con la FK propia [mm]; en linea punteada, el objetivo
  3. Distancia del TCP al objetivo [mm] en escala logaritmica, con la tolerancia de 1 mm
Las franjas grises marcan cuando el robot se estaba moviendo.

La traza se graba siempre (aunque la ventana este cerrada) y, al terminar cada movimiento,
el panel guarda automaticamente en results/vivo_<fecha>/:
  mov_<n>.png  la grafica de ese movimiento      mov_<n>.csv  las muestras de ese movimiento
"""
from __future__ import annotations

import csv
import os
import time
import tkinter as tk
from tkinter import ttk

import numpy as np

# paleta categorica validada (dataviz, modo claro); la identidad tambien va en la leyenda
# y en la etiqueta al final de cada linea
COLORES_J = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
COLORES_XYZ = COLORES_J[:3]
TINTA, TINTA_2, REJILLA, FRANJA = "#1f2328", "#5f6b78", "#e6e6e3", "#ececea"
TOL_MM = 1.0
MAX_MUESTRAS = 30000          # unos 75 min a 150 ms


class Traza:
    """Muestras (t, q, tcp) y movimientos (t_ini, t_fin, objetivo)."""

    def __init__(self):
        self.t0 = time.time()
        self.t, self.q, self.p = [], [], []
        self.movs = []                 # dict(n, t_ini, t_fin, p_obj, q_obj, ep)
        self._ultimo_stamp = None

    def agregar(self, stamp, q_deg, p_mm):
        if stamp == self._ultimo_stamp:
            return False
        self._ultimo_stamp = stamp
        self.t.append(stamp - self.t0)
        self.q.append(list(q_deg))
        self.p.append(list(p_mm))
        if len(self.t) > MAX_MUESTRAS:
            del self.t[:1000], self.q[:1000], self.p[:1000]
        return True

    def inicio_movimiento(self, q_obj, p_obj, etiqueta=None):
        n = len(self.movs) + 1
        self.movs.append(dict(n=n, t_ini=time.time() - self.t0, t_fin=None, etiqueta=etiqueta or f"mov {n}",
                              q_obj=list(q_obj), p_obj=list(p_obj), ep=None))
        return n

    def fin_movimiento(self, ep=None):
        if self.movs and self.movs[-1]["t_fin"] is None:
            self.movs[-1]["t_fin"] = time.time() - self.t0
            self.movs[-1]["ep"] = ep
            return self.movs[-1]
        return None

    def arrays(self, desde=None, hasta=None):
        t = np.asarray(self.t, float)
        q = np.asarray(self.q, float).reshape(-1, 6)
        p = np.asarray(self.p, float).reshape(-1, 3)
        m = np.ones(len(t), bool)
        if desde is not None:
            m &= t >= desde
        if hasta is not None:
            m &= t <= hasta
        return t[m], q[m], p[m]

    def objetivo_en(self, t):
        """Posicion objetivo vigente en cada instante (NaN fuera de un movimiento y su llegada)."""
        obj = np.full((len(t), 3), np.nan)
        for mv in self.movs:
            fin = (mv["t_fin"] if mv["t_fin"] is not None else np.inf) + 2.0
            m = (t >= mv["t_ini"]) & (t <= fin)
            obj[m] = mv["p_obj"]
        return obj


def _separar(etiquetas, ys, ax, frac=0.07):
    """Coloca las etiquetas del final de linea sin que se encimen (separacion minima en y)."""
    lo, hi = ax.get_ylim()
    paso = frac * (hi - lo)
    orden = np.argsort(ys)
    pos = [float(ys[i]) for i in orden]
    for k in range(1, len(pos)):
        pos[k] = max(pos[k], pos[k - 1] + paso)
    for i, y in zip(orden, pos):
        x = etiquetas[i].get_position()[0]
        etiquetas[i].set_position((x, y))


def _estilo_eje(ax, ylabel):
    ax.set_ylabel(ylabel, color=TINTA_2, fontsize=9)
    ax.grid(True, color=REJILLA, lw=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color("#c9c9c4")
    ax.tick_params(colors=TINTA_2, labelsize=8)


class Graficador:
    """Dibuja la traza en una Figure de matplotlib (ventana en vivo o PNG por movimiento)."""

    def __init__(self, fig):
        self.fig = fig
        self.ax_q, self.ax_p, self.ax_d = fig.subplots(3, 1, sharex=True,
                                                       gridspec_kw=dict(height_ratios=[1.2, 1.2, 0.9]))
        _estilo_eje(self.ax_q, "ángulos [°]")
        _estilo_eje(self.ax_p, "TCP [mm]")
        _estilo_eje(self.ax_d, "distancia al\nobjetivo [mm]")
        self.ax_d.set_xlabel("tiempo [s]", color=TINTA_2, fontsize=9)
        self.ax_d.set_yscale("log")
        self.l_q = [self.ax_q.plot([], [], color=c, lw=2, label=f"J{i + 1}")[0] for i, c in enumerate(COLORES_J)]
        self.l_p = [self.ax_p.plot([], [], color=c, lw=2, label=n)[0] for c, n in zip(COLORES_XYZ, "xyz")]
        self.l_o = [self.ax_p.plot([], [], color=c, lw=1.2, ls="--", drawstyle="steps-post")[0] for c in COLORES_XYZ]
        self.l_d = self.ax_d.plot([], [], color=COLORES_J[0], lw=2, label="|TCP − objetivo|")[0]
        self.ax_d.axhline(TOL_MM, color=TINTA_2, lw=1, ls=":")
        self.ax_d.text(1.0, TOL_MM, " tolerancia 1 mm", transform=self.ax_d.get_yaxis_transform(),
                       color=TINTA_2, fontsize=8, va="bottom", ha="right")
        self.ax_q.legend(ncol=6, fontsize=8, loc="lower left", bbox_to_anchor=(0, 1.0), frameon=False)
        self.ax_p.legend(handles=self.l_p + [self.l_o[0]], labels=["x", "y", "z", "objetivo (punteado)"],
                         ncol=4, fontsize=8, loc="lower left", bbox_to_anchor=(0, 1.0), frameon=False)
        self._etq_q = [self.ax_q.text(0, 0, f" J{i + 1}", color=TINTA, fontsize=8, va="center", clip_on=False)
                       for i in range(6)]
        self._etq_p = [self.ax_p.text(0, 0, f" {n}", color=TINTA, fontsize=8, va="center", clip_on=False)
                       for n in "xyz"]
        self._franjas, self._marcas = [], []
        self.titulo = fig.suptitle("", fontsize=10, color=TINTA, x=0.01, ha="left")
        fig.subplots_adjust(left=0.10, right=0.93, top=0.90, bottom=0.08, hspace=0.42)

    def actualizar(self, traza: Traza, desde=None, hasta=None, titulo=""):
        t, q, p = traza.arrays(desde, hasta)
        if len(t) == 0:
            return
        obj = traza.objetivo_en(t)
        for i, l in enumerate(self.l_q):
            l.set_data(t, q[:, i])
            self._etq_q[i].set_position((t[-1], q[-1, i]))
        for k in range(3):
            self.l_p[k].set_data(t, p[:, k])
            self.l_o[k].set_data(t, obj[:, k])
            self._etq_p[k].set_position((t[-1], p[-1, k]))
        d = np.linalg.norm(p - obj, axis=1)
        self.l_d.set_data(t, np.clip(d, 1e-3, None))

        for a in self._franjas + self._marcas:
            a.remove()
        self._franjas, self._marcas = [], []
        for mv in traza.movs:
            fin = mv["t_fin"] if mv["t_fin"] is not None else t[-1]
            if fin < t[0] or mv["t_ini"] > t[-1]:
                continue
            for ax in (self.ax_q, self.ax_p, self.ax_d):
                self._franjas.append(ax.axvspan(mv["t_ini"], fin, color=FRANJA, lw=0, zorder=0))
            self._marcas.append(self.ax_q.text(mv["t_ini"], 1.0, f" {mv.get('etiqueta', 'mov ' + str(mv['n']))}", color=TINTA_2, fontsize=8,
                                               transform=self.ax_q.get_xaxis_transform(), va="top"))
            if mv["ep"] is not None and not str(mv.get("etiqueta", "")).startswith("c"):
                self._marcas.append(self.ax_d.text(fin, 1.0, f" llegó a {mv['ep']:.3f} mm", color=TINTA,
                                                   fontsize=8, transform=self.ax_d.get_xaxis_transform(), va="top"))

        x0, x1 = float(t[0]), float(max(t[-1], t[0] + 1.0))
        self.ax_d.set_xlim(x0, x1 + 0.02 * (x1 - x0))
        for ax, datos in ((self.ax_q, q), (self.ax_p, np.hstack([p, obj]))):
            lo, hi = np.nanmin(datos), np.nanmax(datos)
            m = max(5.0, 0.08 * (hi - lo))
            ax.set_ylim(lo - m, hi + m)
        _separar(self._etq_q, q[-1], self.ax_q)
        _separar(self._etq_p, p[-1], self.ax_p)
        dd = d[np.isfinite(d)]
        self.ax_d.set_ylim(1e-3, max(10.0, float(dd.max()) * 2) if len(dd) else 1000)
        self.titulo.set_text(titulo)


def guardar_movimiento(traza: Traza, mv: dict, carpeta: str):
    """PNG + CSV de un movimiento (1 s antes del inicio hasta 2 s despues del fin)."""
    ep = "" if mv["ep"] is None else f" · llegada e_p = {mv['ep']:.3f} mm"
    dur = "" if mv["t_fin"] is None else f" · duración {mv['t_fin'] - mv['t_ini']:.1f} s"
    return guardar_intervalo(traza, mv["t_ini"] - 1.0, (mv["t_fin"] or mv["t_ini"]) + 2.0,
                             f"mov_{mv['n']}", f"Movimiento {mv['n']}{dur}{ep}", carpeta)


def guardar_intervalo(traza: Traza, desde, hasta, nombre, titulo, carpeta):
    """PNG + CSV de la traza entre dos instantes (s desde el inicio del panel)."""
    from matplotlib.figure import Figure
    os.makedirs(carpeta, exist_ok=True)
    fig = Figure(figsize=(9, 7), dpi=130)
    g = Graficador(fig)
    g.actualizar(traza, desde, hasta, titulo)
    base = os.path.join(carpeta, nombre)
    fig.savefig(base + ".png")
    t, q, p = traza.arrays(desde, hasta)
    obj = traza.objetivo_en(t)
    with open(base + ".csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s"] + [f"q{i + 1}_deg" for i in range(6)] + ["x_mm", "y_mm", "z_mm",
                                                                    "x_obj", "y_obj", "z_obj", "dist_mm"])
        for i in range(len(t)):
            w.writerow([f"{t[i]:.3f}"] + [f"{v:.4f}" for v in q[i]] + [f"{v:.3f}" for v in p[i]]
                       + [f"{v:.3f}" for v in obj[i]] + [f"{np.linalg.norm(p[i] - obj[i]):.4f}"])
    return base + ".png"


def guardar_ciclos(ciclos, carpeta, nombre="resumen_ciclos"):
    """Repetibilidad del trabajo en serie: error de llegada maximo y duracion de cada ciclo."""
    from matplotlib.figure import Figure
    os.makedirs(carpeta, exist_ok=True)
    with open(os.path.join(carpeta, nombre + ".csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["ciclo", "duracion_s", "ep_max_mm", "ep_medio_mm"])
        for c in ciclos:
            w.writerow([c["n"], f"{c['duracion_s']:.2f}", f"{c['ep_max_mm']:.4f}", f"{c['ep_medio_mm']:.4f}"])
    if not ciclos:
        return None
    n = [c["n"] for c in ciclos]
    fig = Figure(figsize=(8, 5.5), dpi=130)
    ax1, ax2 = fig.subplots(2, 1, sharex=True)
    _estilo_eje(ax1, "error de llegada\nmáximo [mm]")
    _estilo_eje(ax2, "duración del\nciclo [s]")
    ep = [c["ep_max_mm"] for c in ciclos]
    ax1.plot(n, ep, color=COLORES_J[0], lw=2, marker="o", ms=5)
    ax1.axhline(TOL_MM, color=TINTA_2, lw=1, ls=":")
    ax1.text(1.0, TOL_MM, " tolerancia 1 mm", transform=ax1.get_yaxis_transform(), color=TINTA_2,
             fontsize=8, va="bottom", ha="right")
    ax1.set_ylim(0, max(TOL_MM * 1.15, max(ep) * 1.2))
    d = [c["duracion_s"] for c in ciclos]
    ax2.plot(n, d, color=COLORES_J[0], lw=2, marker="o", ms=5)
    ax2.set_ylim(0, max(d) * 1.25)
    ax2.set_xlabel("ciclo", color=TINTA_2, fontsize=9)
    ax2.xaxis.get_major_locator().set_params(integer=True)
    fig.suptitle(f"Trabajo en serie · {len(ciclos)} ciclo(s) · e_p máx {max(ep):.3f} mm · "
                 f"duración media {np.mean(d):.1f} s", fontsize=10, color=TINTA, x=0.01, ha="left")
    fig.subplots_adjust(left=0.13, right=0.95, top=0.90, bottom=0.10, hspace=0.25)
    ruta = os.path.join(carpeta, nombre + ".png")
    fig.savefig(ruta)
    return ruta


class VentanaVivo:
    """Ventana Toplevel que redibuja la traza unas 5 veces por segundo."""

    VENTANAS = {"últimos 30 s": 30, "últimos 60 s": 60, "últimos 2 min": 120, "toda la sesión": None}

    def __init__(self, root, traza: Traza, carpeta: str, al_cerrar=None):
        import matplotlib
        matplotlib.use("TkAgg")
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure
        self.traza, self.carpeta, self._al_cerrar = traza, carpeta, al_cerrar
        self.top = tk.Toplevel(root)
        self.top.title("IRB 120 · Gráfica en vivo")
        ancho, alto = self.top.winfo_screenwidth(), self.top.winfo_screenheight()
        self.top.geometry(f"{min(900, ancho - 40)}x{min(700, alto - 110)}+{max(0, ancho - 920)}+0")
        barra = ttk.Frame(self.top, padding=4)
        barra.pack(fill="x")
        ttk.Label(barra, text="Mostrar:").pack(side="left")
        self.var_ventana = tk.StringVar(value="últimos 60 s")
        ttk.Combobox(barra, textvariable=self.var_ventana, values=list(self.VENTANAS), width=16,
                     state="readonly").pack(side="left", padx=4)
        self.var_pausa = tk.BooleanVar(value=False)
        ttk.Checkbutton(barra, text="Pausar", variable=self.var_pausa).pack(side="left", padx=8)
        ttk.Button(barra, text="Guardar imagen", command=self.guardar).pack(side="left")
        self.lbl = ttk.Label(barra, text="", foreground=TINTA_2)
        self.lbl.pack(side="left", padx=8)
        self.fig = Figure(figsize=(9, 6.5), dpi=100)
        self.g = Graficador(self.fig)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.top)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.top.protocol("WM_DELETE_WINDOW", self.cerrar)
        self._vivo = True
        self._tick()

    def _tick(self):
        if not self._vivo:
            return
        if not self.var_pausa.get() and self.traza.t:
            seg = self.VENTANAS.get(self.var_ventana.get())
            desde = None if seg is None else self.traza.t[-1] - seg
            mv = self.traza.movs[-1] if self.traza.movs else None
            estado = ""
            if mv is not None:
                et = mv.get("etiqueta", f"mov {mv['n']}")
                estado = f"moviendo ({et})" if mv["t_fin"] is None else f"quieto · último: {et}"
            self.g.actualizar(self.traza, desde, None, f"Lectura del robot en vivo · {estado}")
            self.canvas.draw_idle()
        self.top.after(200, self._tick)

    def guardar(self):
        os.makedirs(self.carpeta, exist_ok=True)
        ruta = os.path.join(self.carpeta, f"vivo_{time.strftime('%H%M%S')}.png")
        self.fig.savefig(ruta, dpi=130)
        self.lbl.configure(text=f"guardada: {os.path.basename(ruta)}")

    def cerrar(self):
        self._vivo = False
        self.top.destroy()
        if self._al_cerrar:
            self._al_cerrar()
