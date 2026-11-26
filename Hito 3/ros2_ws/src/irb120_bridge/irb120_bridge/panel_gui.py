"""
Panel IRB 120 (Windows / Linux) - verificar antes de mover.

  python tools/panel.py --modo simulado                       practicar sin robot
  python tools/panel.py --modo directo --robot 192.168.125.1  robot real sin ROS (RWS + socket)
  ros2 run irb120_bridge panel --modo ros                     via ROS 2 (bringup mode:=real command:=true)

Flujo: elegir objetivo (pose -> cinematica inversa, o angulos -> cinematica directa) -> ANALIZAR
-> el panel muestra las 8 ramas y 7 chequeos -> MOVER se habilita solo si todos dan OK y el robot
no se movio desde el analisis -> confirmacion -> envio por tramos -> verificacion de llegada.
Todo queda en results/panel_<fecha>.jsonl. "Abrir gráfica en vivo" muestra ángulos, TCP y distancia al
objetivo mientras el robot se mueve; cada movimiento se guarda solo en results/vivo_<fecha>/mov_<n>.png/.csv.
Pestaña "Secuencia repetitiva": lista de puntos (p. ej. recoger y dejar) que se repite en ciclo hasta
DETENER o N repeticiones; se verifica completa antes de iniciar (ver secuencia.py).

Detener el robot: soltar el habilitador, Stop o paro de emergencia. El PC NO puede frenarlo.
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import time
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import numpy as np

from . import analisis as A
from . import conexiones
from . import grafica_vivo as GV
from . import secuencia as S
from . import irb120_model as M

VERDE, ROJO, GRIS, AZUL = "#1f7a3a", "#b3261e", "#5f6b78", "#1d4f91"
REANALIZAR_DEG = 0.5
# Posicion inicial (HOME): brazo recto con la muneca a 30 grados, lejos de la singularidad q5 = 0.
# Se cambia con "Fijar HOME aqui" y se guarda en results/home.json.
HOME_DEF = [0.0, 0.0, 0.0, 0.0, 30.0, 0.0]


def _f1(v):
    t = f"{v:.1f}"
    return "0.0" if t == "-0.0" else t


def carpeta_resultados():
    """results/ del repositorio: la carpeta actual si es el repo; si no, la del codigo fuente."""
    here = os.path.dirname(os.path.abspath(__file__))
    for raiz in (os.getcwd(), os.path.abspath(os.path.join(here, "..", "..", "..", ".."))):
        if os.path.isdir(os.path.join(raiz, "tools")) and os.path.isdir(os.path.join(raiz, "kinematics")):
            destino = os.path.join(raiz, "results")
            os.makedirs(destino, exist_ok=True)
            return destino
    os.makedirs("results", exist_ok=True)
    return os.path.abspath("results")


class Panel:
    def __init__(self, root: tk.Tk, conexion, z_min_mm: float):
        self.root, self.con, self.z_min = root, conexion, z_min_mm
        self.cola = queue.Queue()
        self.analisis: A.Analisis | None = None
        self.moviendo = False
        self._llegada_pendiente = None
        self._q_dibujado = None
        self._obj_dibujado = None
        self.log_path = os.path.join(carpeta_resultados(), f"panel_{datetime.now():%Y%m%d_%H%M%S}.jsonl")
        self.carpeta_vivo = os.path.join(carpeta_resultados(), f"vivo_{datetime.now():%Y%m%d_%H%M%S}")
        self.traza = GV.Traza()     # se graba siempre, aunque la ventana de la grafica este cerrada
        self.vivo = None
        self.seq_puntos: list[S.Punto] = []
        self.seq_V: S.Verificacion | None = None
        self.ejec: S.Ejecutor | None = None
        self.home_path = os.path.join(carpeta_resultados(), "home.json")
        self.home = self._cargar_home()

        root.title(f"IRB 120 · Panel de control · modo {conexion.nombre}")
        # tamano segun la pantalla (laptops de 768 px de alto) y maximizada en Windows
        ancho, alto = root.winfo_screenwidth(), root.winfo_screenheight()
        root.geometry(f"{min(1480, ancho - 20)}x{min(940, alto - 80)}+0+0")
        try:
            root.state("zoomed")
        except tk.TclError:
            pass
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        # columna izquierda desplazable (rueda del mouse)
        cont = ttk.Frame(root)
        cont.grid(row=0, column=0, sticky="ns")
        cont.rowconfigure(0, weight=1)
        lienzo = tk.Canvas(cont, highlightthickness=0, width=700)
        barra = ttk.Scrollbar(cont, orient="vertical", command=lienzo.yview)
        lienzo.configure(yscrollcommand=barra.set)
        lienzo.grid(row=0, column=0, sticky="ns")
        barra.grid(row=0, column=1, sticky="ns")
        izq = ttk.Frame(lienzo, padding=8)
        lienzo.create_window((0, 0), window=izq, anchor="nw")
        izq.bind("<Configure>", lambda e: lienzo.configure(scrollregion=lienzo.bbox("all"),
                                                             width=izq.winfo_reqwidth()))

        def rueda(e):
            lienzo.yview_scroll(int(-e.delta / 120) if e.delta else (-1 if e.num == 4 else 1), "units")
        lienzo.bind("<Enter>", lambda e: (lienzo.bind_all("<MouseWheel>", rueda),
                                          lienzo.bind_all("<Button-4>", rueda), lienzo.bind_all("<Button-5>", rueda)))
        lienzo.bind("<Leave>", lambda e: (lienzo.unbind_all("<MouseWheel>"),
                                          lienzo.unbind_all("<Button-4>"), lienzo.unbind_all("<Button-5>")))

        # columna derecha: vista 3D, verificacion + MOVER (siempre visibles) y registro
        der = ttk.Frame(root, padding=8)
        der.grid(row=0, column=1, sticky="nsew")
        der.columnconfigure(0, weight=1)
        der.rowconfigure(0, weight=1)
        medio = ttk.Frame(der)
        medio.grid(row=1, column=0, sticky="ew")

        self._armar_estado(izq)
        self._armar_objetivo(izq)
        self._armar_ramas(izq)
        self._armar_vista(der)
        self._armar_verificacion(medio)
        self._armar_mover(medio)
        self._armar_registro(der)

        self.registrar("inicio", f"panel en modo {conexion.nombre}; z minima {z_min_mm:.0f} mm; registro {self.log_path}")
        self.root.after(200, self._ciclo)

    # ------------------------------------------------------------------ construccion
    def _armar_estado(self, p):
        f = ttk.LabelFrame(p, text="Estado del robot", padding=6)
        f.pack(fill="x")
        self.lbl_con = ttk.Label(f, text="conectando...", foreground=GRIS, font=("Segoe UI", 10, "bold"))
        self.lbl_con.grid(row=0, column=0, columnspan=6, sticky="w")
        self.lbl_q = []
        for i in range(6):
            ttk.Label(f, text=f"J{i + 1}").grid(row=1, column=i)
            l = ttk.Label(f, text="--", width=9, anchor="center", font=("Consolas", 11))
            l.grid(row=2, column=i)
            self.lbl_q.append(l)
        self.lbl_pose = ttk.Label(f, text="TCP (FK propia): --", font=("Consolas", 10))
        self.lbl_pose.grid(row=3, column=0, columnspan=6, sticky="w", pady=(6, 0))
        self.lbl_modelo = ttk.Label(f, text="Modelo vs controlador: --", font=("Consolas", 10))
        self.lbl_modelo.grid(row=4, column=0, columnspan=6, sticky="w")

    def _armar_objetivo(self, p):
        nb = ttk.Notebook(p)
        nb.pack(fill="x", pady=(8, 0))
        self.nb = nb
        # --- pose (IK)
        fp = ttk.Frame(nb, padding=6)
        nb.add(fp, text="Pose del TCP (cinemática inversa)")
        self.ent_pose = []
        for i, (nom, val) in enumerate(zip(["X mm", "Y mm", "Z mm", "Roll °", "Pitch °", "Yaw °"],
                                           [350, 100, 250, 0, 180, 0])):
            ttk.Label(fp, text=nom).grid(row=(i // 3) * 2, column=i % 3, sticky="w", padx=4)
            e = ttk.Entry(fp, width=11, font=("Consolas", 11))
            e.insert(0, str(val))
            e.grid(row=(i // 3) * 2 + 1, column=i % 3, padx=4, pady=(0, 4))
            self.ent_pose.append(e)
        b = ttk.Frame(fp)
        b.grid(row=4, column=0, columnspan=3, sticky="w", pady=(4, 0))
        ttk.Button(b, text="Copiar pose actual", command=self._copiar_pose).pack(side="left")
        ttk.Button(b, text="Gripper vertical (hacia abajo)", command=self._vertical).pack(side="left", padx=4)
        ttk.Button(b, text="ANALIZAR", command=self._analizar_pose).pack(side="left", padx=4)
        ttk.Label(fp, text="Orientación: R = Rz(yaw)·Ry(pitch)·Rx(roll). TCP = punta de Gripper1 (+167,5 mm).",
                  foreground=GRIS).grid(row=5, column=0, columnspan=3, sticky="w", pady=(4, 0))
        # --- ejes (FK)
        fe = ttk.Frame(nb, padding=6)
        nb.add(fe, text="Ángulos (cinemática directa)")
        self.ent_q = []
        ttk.Label(fe, text="paso °").grid(row=0, column=0, sticky="w")
        self.ent_paso = ttk.Entry(fe, width=5)
        self.ent_paso.insert(0, "5")
        self.ent_paso.grid(row=0, column=1, sticky="w")
        for i in range(6):
            ttk.Label(fe, text=f"J{i + 1} °").grid(row=i + 1, column=0, sticky="w")
            e = ttk.Entry(fe, width=10, font=("Consolas", 11))
            e.insert(0, "0")
            e.grid(row=i + 1, column=1, padx=4, pady=1)
            ttk.Button(fe, text="-", width=3, command=lambda k=i: self._paso(k, -1)).grid(row=i + 1, column=2)
            ttk.Button(fe, text="+", width=3, command=lambda k=i: self._paso(k, +1)).grid(row=i + 1, column=3)
            self.ent_q.append(e)
        b = ttk.Frame(fe)
        b.grid(row=7, column=0, columnspan=4, sticky="w", pady=(6, 0))
        ttk.Button(b, text="Copiar ángulos actuales", command=self._copiar_q).pack(side="left")
        ttk.Button(b, text="ANALIZAR", command=self._analizar_ejes).pack(side="left", padx=4)
        self.lbl_fk_obj = ttk.Label(fe, text="", font=("Consolas", 10))
        self.lbl_fk_obj.grid(row=8, column=0, columnspan=4, sticky="w", pady=(4, 0))
        self._armar_secuencia(nb)

    def _armar_secuencia(self, nb):
        fs = ttk.Frame(nb, padding=6)
        nb.add(fs, text="Secuencia repetitiva")
        ttk.Label(fs, text="Ciclo P1 -> P2 -> ... -> Pn -> P1 que se repite hasta DETENER (trabajo en serie).",
                  foreground=GRIS).pack(anchor="w")
        cols = ("n", "nombre", "x", "y", "z", "espera", "accion")
        t = ttk.Treeview(fs, columns=cols, show="headings", height=7, selectmode="browse")
        for c, w, tit in zip(cols, (32, 120, 60, 60, 60, 56, 120),
                             ("#", "Nombre", "X mm", "Y mm", "Z mm", "Espera s", "Acción")):
            t.heading(c, text=tit)
            t.column(c, width=w, anchor="w" if c in ("nombre", "accion") else "center")
        t.pack(fill="x", pady=(4, 0))
        self.tabla_seq = t
        f1 = ttk.Frame(fs)
        f1.pack(fill="x", pady=(4, 0))
        ttk.Button(f1, text="Recoger y dejar…", command=self._seq_plantilla).pack(side="left")
        ttk.Button(f1, text="+ Posición actual", command=self._seq_agregar_actual).pack(side="left", padx=4)
        ttk.Button(f1, text="+ Pose de la pestaña Pose", command=self._seq_agregar_pose).pack(side="left")
        f2 = ttk.Frame(fs)
        f2.pack(fill="x", pady=(4, 0))
        for txt, cmd in (("Subir", lambda: self._seq_mover_fila(-1)), ("Bajar", lambda: self._seq_mover_fila(1)),
                         ("Eliminar", self._seq_eliminar), ("Limpiar", self._seq_limpiar),
                         ("Guardar…", self._seq_guardar), ("Cargar…", self._seq_cargar)):
            ttk.Button(f2, text=txt, command=cmd).pack(side="left", padx=(0, 4))
        f3 = ttk.Frame(fs)
        f3.pack(fill="x", pady=(6, 0))
        ttk.Label(f3, text="Espera s").pack(side="left")
        self.ent_espera = ttk.Entry(f3, width=5)
        self.ent_espera.insert(0, "1.0")
        self.ent_espera.pack(side="left", padx=(2, 2))
        ttk.Button(f3, text="Aplicar al seleccionado", command=self._seq_aplicar_espera).pack(side="left")
        ttk.Label(f3, text="   Repeticiones").pack(side="left")
        self.ent_reps = ttk.Entry(f3, width=5)
        self.ent_reps.insert(0, "0")
        self.ent_reps.pack(side="left", padx=2)
        ttk.Label(f3, text="(0 = hasta detener)", foreground=GRIS).pack(side="left")
        f4 = ttk.Frame(fs)
        f4.pack(fill="x", pady=(6, 0))
        ttk.Button(f4, text="VERIFICAR SECUENCIA", command=self._seq_verificar).pack(side="left")
        self.btn_seq_iniciar = ttk.Button(f4, text="INICIAR CICLO", command=self._seq_iniciar, state="disabled")
        self.btn_seq_iniciar.pack(side="left", padx=4)
        self.btn_seq_fin_ciclo = ttk.Button(f4, text="Detener al terminar el ciclo",
                                            command=self._seq_detener_fin_ciclo, state="disabled")
        self.btn_seq_fin_ciclo.pack(side="left")
        self.var_seq_home = tk.BooleanVar(value=True)
        ttk.Checkbutton(fs, text="Al terminar, preparar el regreso a HOME (se confirma con MOVER)",
                        variable=self.var_seq_home).pack(anchor="w", pady=(4, 0))
        cols = ("tray", "salto", "zmin", "estado")
        tv = ttk.Treeview(fs, columns=cols, show="headings", height=7)
        for c, w, tit in zip(cols, (150, 70, 80, 290), ("Trayecto", "Salto °", "z mín mm", "Verificación")):
            tv.heading(c, text=tit)
            tv.column(c, width=w, anchor="w" if c in ("tray", "estado") else "center")
        tv.tag_configure("ok", foreground=VERDE)
        tv.tag_configure("no", foreground=ROJO)
        tv.pack(fill="x", pady=(6, 0))
        self.tabla_tray = tv
        self.lbl_seq_ver = ttk.Label(fs, text="Sin verificar.", foreground=GRIS, wraplength=640)
        self.lbl_seq_ver.pack(anchor="w", pady=(4, 0))
        ttk.Label(fs, text="La acción (cerrar/abrir gripper) es informativa: el gripper no está conectado "
                           "a una salida del controlador; la espera reserva su tiempo.",
                  foreground=GRIS, wraplength=640).pack(anchor="w", pady=(2, 0))

    def _armar_ramas(self, p):
        f = ttk.LabelFrame(p, text="Ramas de la cinemática inversa", padding=4)
        f.pack(fill="x", pady=(8, 0))
        cols = ("rama", "j1", "j2", "j3", "j4", "j5", "j6", "estado")
        t = ttk.Treeview(f, columns=cols, show="headings", height=8)
        for c, w in zip(cols, (118, 50, 50, 50, 50, 50, 54, 250)):
            t.heading(c, text=c.upper() if c.startswith("j") else c.capitalize())
            t.column(c, width=w, anchor="center" if c != "estado" else "w")
        t.tag_configure("elegida", background="#d8efd9")
        t.tag_configure("fuera", foreground=GRIS)
        t.pack(fill="x")
        self.tabla = t

    def _armar_verificacion(self, p):
        f = ttk.LabelFrame(p, text="Verificación antes de mover", padding=6)
        f.pack(fill="x", pady=(8, 0))
        self.lbl_chk = []
        for i in range(7):
            l = ttk.Label(f, text="", font=("Segoe UI", 10))
            l.grid(row=i, column=0, sticky="w")
            self.lbl_chk.append(l)
        self.lbl_vigencia = ttk.Label(f, text="Sin análisis: elige un objetivo y presiona ANALIZAR.",
                                      foreground=GRIS)
        self.lbl_vigencia.grid(row=7, column=0, sticky="w", pady=(4, 0))

    def _armar_mover(self, p):
        # compacto: en laptops de 768 px de alto la vista 3D necesita el espacio
        f = ttk.Frame(p, padding=(0, 6, 0, 0))
        f.pack(fill="x")
        estilo = ttk.Style()
        estilo.configure("Mover.TButton", font=("Segoe UI", 12, "bold"), padding=6)
        estilo.configure("Parar.TButton", font=("Segoe UI", 10, "bold"), foreground=ROJO, padding=4)
        fh = ttk.Frame(f)
        fh.pack(fill="x", pady=(0, 4))
        ttk.Button(fh, text="Gráfica en vivo", command=self._abrir_vivo).pack(side="left")
        ttk.Button(fh, text="IR A HOME", command=self._ir_home).pack(side="left", padx=(4, 0))
        ttk.Button(fh, text="Fijar HOME aquí", command=self._fijar_home).pack(side="left", padx=(4, 0))
        self.lbl_home = ttk.Label(fh, text="", foreground=GRIS, font=("Consolas", 9))
        self.lbl_home.pack(side="left", padx=(8, 0))
        self._mostrar_home()
        self.btn_mover = ttk.Button(f, text="MOVER", style="Mover.TButton", command=self._mover, state="disabled")
        self.btn_mover.pack(fill="x")
        self.lbl_llegada = ttk.Label(f, text="", font=("Consolas", 10))
        self.lbl_llegada.pack(anchor="w", pady=(2, 0))
        fs = ttk.Frame(f)
        fs.pack(fill="x", pady=(2, 0))
        self.btn_seq_parar = ttk.Button(fs, text="DETENER SECUENCIA", style="Parar.TButton",
                                        command=self._seq_detener_ya, state="disabled")
        self.btn_seq_parar.pack(side="right")
        self.lbl_seq = ttk.Label(fs, text="", font=("Segoe UI", 9, "bold"), wraplength=460)
        self.lbl_seq.pack(side="left", fill="x", expand=True)
        ttk.Label(f, text="Para DETENER: soltar el habilitador, Stop o paro de emergencia. "
                          "El PC no puede frenar un movimiento en curso.", foreground=ROJO,
                  wraplength=640).pack(anchor="w", pady=(2, 0))

    def _armar_vista(self, p):
        import matplotlib
        matplotlib.use("TkAgg")
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure
        self.fig = Figure(figsize=(6, 3.6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=p)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def _armar_registro(self, p):
        f = ttk.LabelFrame(p, text="Registro", padding=4)
        f.grid(row=2, column=0, sticky="nsew", pady=(6, 0))
        alto = 5 if self.root.winfo_screenheight() >= 900 else 3
        self.txt = tk.Text(f, height=alto, font=("Consolas", 9), wrap="word")
        sb = ttk.Scrollbar(f, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        self.txt.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

    # ------------------------------------------------------------------ utilidades
    def registrar(self, evento, texto, **datos):
        linea = f"{datetime.now():%H:%M:%S}  {texto}"
        self.txt.insert("end", linea + "\n")
        self.txt.see("end")
        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(dict(t=datetime.now().isoformat(timespec="milliseconds"), evento=evento,
                                    texto=texto, conexion=self.con.nombre, **datos),
                               ensure_ascii=False, default=float) + "\n")

    def _leer_numeros(self, entradas, nombre):
        try:
            return [float(e.get().replace(",", ".")) for e in entradas]
        except ValueError:
            messagebox.showerror("Dato inválido", f"Revisa los valores de {nombre}: deben ser números.")
            return None

    def _q_actual(self):
        q = self.con.estado()["q_deg"]
        if q is None:
            messagebox.showwarning("Sin estado", "Todavía no hay lectura de los ángulos del robot.")
        return q

    def _error_modelo(self, est):
        if est["q_deg"] is None or est["T_ctrl"] is None:
            return None
        T_own = M.fk(np.deg2rad(est["q_deg"]))
        return A.errores_pose(est["T_ctrl"], T_own)

    # ------------------------------------------------------------------ grafica en vivo
    def _abrir_vivo(self):
        if self.vivo is not None:
            self.vivo.top.lift()
            return
        self.vivo = GV.VentanaVivo(self.root, self.traza, self.carpeta_vivo, al_cerrar=self._vivo_cerrada)

    def _vivo_cerrada(self):
        self.vivo = None   # la ventana se cerró; la traza sigue grabando

    def _guardar_mov(self, mv):
        try:
            ruta = GV.guardar_movimiento(self.traza, mv, self.carpeta_vivo)
            self.registrar("grafica", f"gráfica del movimiento {mv['n']} guardada en {ruta}", archivo=ruta)
        except Exception as e:  # noqa: BLE001  (la grafica nunca debe tumbar el panel)
            self.registrar("error", f"no se pudo guardar la gráfica del movimiento {mv['n']}: {e}")

    def _cerrar_mov_grafica(self, ep):
        mv = self.traza.fin_movimiento(ep)
        if mv is not None:   # 2 s despues, para incluir el asentamiento en la grafica
            self.root.after(2100, lambda: self._guardar_mov(mv))

    # ------------------------------------------------------------------ HOME
    def _cargar_home(self):
        try:
            with open(self.home_path, encoding="utf-8") as f:
                q = [float(v) for v in json.load(f)["q_deg"]]
            if len(q) == 6 and A.dentro_limites(q)[0]:
                return q
        except (OSError, ValueError, KeyError, TypeError):
            pass
        return list(HOME_DEF)

    def _mostrar_home(self):
        self.lbl_home.configure(text="HOME: " + " ".join(_f1(v) for v in self.home))

    def _ir_home(self):
        """Prepara el movimiento a HOME con los mismos 7 chequeos; se ejecuta con MOVER."""
        if self.moviendo:
            return
        for e, v in zip(self.ent_q, self.home):
            e.delete(0, "end")
            e.insert(0, f"{v:.2f}")
        self.nb.select(1)
        self.registrar("home", "objetivo HOME cargado: revisa la verificación y presiona MOVER", q_home=self.home)
        self._analizar_ejes()

    def _fijar_home(self):
        q = self._q_actual()
        if q is None:
            return
        ok, motivo = A.dentro_limites(q)
        if not ok:
            messagebox.showerror("Fuera de límites", motivo)
            return
        if not messagebox.askyesno("Fijar HOME", "¿Usar la posición actual como HOME?\n"
                                   + "  ".join(_f1(v) for v in q)):
            return
        self.home = [round(v, 3) for v in q]
        with open(self.home_path, "w", encoding="utf-8") as f:
            json.dump(dict(q_deg=self.home, fijado=datetime.now().isoformat(timespec="seconds")), f)
        self._mostrar_home()
        self.registrar("home", "HOME fijado en la posición actual", q_home=self.home)

    # ------------------------------------------------------------------ secuencia repetitiva
    def _seq_refrescar(self, invalidar=True):
        self.tabla_seq.delete(*self.tabla_seq.get_children())
        for i, p in enumerate(self.seq_puntos, 1):
            x, y, z = p.xyz_mm
            self.tabla_seq.insert("", "end", values=(f"P{i}", p.nombre, f"{x:.1f}", f"{y:.1f}", f"{z:.1f}",
                                                     f"{p.espera_s:.1f}", p.accion))
        if invalidar:
            self.seq_V = None
            self.tabla_tray.delete(*self.tabla_tray.get_children())
            self.lbl_seq_ver.configure(text="Secuencia modificada: VERIFICA antes de iniciar.", foreground=GRIS)

    def _seq_sel(self):
        sel = self.tabla_seq.selection()
        return None if not sel else self.tabla_seq.index(sel[0])

    def _seq_espera(self):
        try:
            return max(0.0, float(self.ent_espera.get().replace(",", ".")))
        except ValueError:
            return 0.0

    def _seq_q_ref(self):
        return self.seq_puntos[-1].q_deg if self.seq_puntos else self.con.estado()["q_deg"]

    def _seq_agregar_actual(self):
        q = self._q_actual()
        if q is None:
            return
        ok, motivo = A.dentro_limites(q)
        if not ok:
            messagebox.showerror("Fuera de límites", motivo)
            return
        self.seq_puntos.append(S.Punto(f"P{len(self.seq_puntos) + 1}", [round(v, 4) for v in q], self._seq_espera()))
        self._seq_refrescar()

    def _seq_agregar_pose(self):
        pose = self._leer_numeros(self.ent_pose, "la pose")
        q_ref = self._seq_q_ref()
        if pose is None or q_ref is None:
            return
        try:
            p = S.punto_desde_pose(f"P{len(self.seq_puntos) + 1}", pose, q_ref, self._seq_espera(),
                                   z_min_mm=self.z_min)
        except ValueError as e:
            messagebox.showerror("Pose no válida", str(e))
            return
        self.seq_puntos.append(p)
        self._seq_refrescar()

    def _seq_plantilla(self):
        q_ref = self._q_actual()
        if q_ref is None:
            return
        top = tk.Toplevel(self.root)
        top.title("Recoger y dejar")
        top.transient(self.root)
        campos = {}
        filas = (("Recoger X mm", 400), ("Recoger Y mm", -120), ("Recoger Z mm", 150),
                 ("Dejar X mm", 400), ("Dejar Y mm", 120), ("Dejar Z mm", 150),
                 ("Altura de aproximación mm", 80), ("Giro del gripper (yaw) °", 0),
                 ("Espera al recoger/dejar s", 1.0))
        for i, (nom, val) in enumerate(filas):
            ttk.Label(top, text=nom).grid(row=i, column=0, sticky="w", padx=8, pady=2)
            e = ttk.Entry(top, width=10)
            e.insert(0, str(val))
            e.grid(row=i, column=1, padx=8, pady=2)
            campos[nom] = e
        ttk.Label(top, text=f"Gripper vertical hacia abajo. Las alturas deben ser ≥ z mínima ({self.z_min:.0f} mm).",
                  foreground=GRIS, wraplength=320).grid(row=len(filas), column=0, columnspan=2, padx=8, pady=4)

        def crear():
            v = self._leer_numeros([campos[n] for n, _ in filas], "la plantilla")
            if v is None:
                return
            try:
                pts = S.plantilla_recoger_dejar(v[0:3], v[3:6], q_ref, altura_aprox_mm=v[6],
                                                rpy=(0, 180, v[7]), espera_s=max(0.0, v[8]), z_min_mm=self.z_min)
            except ValueError as e:
                messagebox.showerror("Punto no alcanzable", str(e), parent=top)
                return
            if self.seq_puntos and not messagebox.askyesno("Reemplazar", "¿Reemplazar la secuencia actual?",
                                                           parent=top):
                return
            self.seq_puntos = pts
            self._seq_refrescar()
            self.registrar("secuencia_plantilla", f"plantilla recoger y dejar: recoger {v[0:3]}, dejar {v[3:6]}, "
                                                  f"aproximación {v[6]:.0f} mm", parametros=v)
            top.destroy()
        self._dlg_crear = crear
        ttk.Button(top, text="Crear puntos", command=crear).grid(row=len(filas) + 1, column=0, columnspan=2, pady=8)

    def _seq_mover_fila(self, d):
        i = self._seq_sel()
        if i is None or not 0 <= i + d < len(self.seq_puntos):
            return
        self.seq_puntos[i], self.seq_puntos[i + d] = self.seq_puntos[i + d], self.seq_puntos[i]
        self._seq_refrescar()
        self.tabla_seq.selection_set(self.tabla_seq.get_children()[i + d])

    def _seq_eliminar(self):
        i = self._seq_sel()
        if i is not None:
            del self.seq_puntos[i]
            self._seq_refrescar()

    def _seq_limpiar(self):
        if self.seq_puntos and messagebox.askyesno("Limpiar", "¿Borrar todos los puntos de la secuencia?"):
            self.seq_puntos = []
            self._seq_refrescar()

    def _seq_aplicar_espera(self):
        i = self._seq_sel()
        if i is not None:
            self.seq_puntos[i].espera_s = self._seq_espera()
            self._seq_refrescar()

    def _seq_guardar(self):
        if not self.seq_puntos:
            return
        ruta = filedialog.asksaveasfilename(initialdir=carpeta_resultados(), defaultextension=".json",
                                            filetypes=[("Secuencia", "*.json")], initialfile="secuencia.json")
        if ruta:
            S.guardar(self.seq_puntos, ruta)
            self.registrar("secuencia_guardada", f"secuencia guardada en {ruta}")

    def _seq_cargar(self):
        ruta = filedialog.askopenfilename(initialdir=carpeta_resultados(), filetypes=[("Secuencia", "*.json")])
        if not ruta:
            return
        try:
            self.seq_puntos = S.cargar(ruta)
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("No se pudo cargar", str(e))
            return
        self._seq_refrescar()
        self.registrar("secuencia_cargada", f"secuencia cargada de {ruta} ({len(self.seq_puntos)} puntos)")

    def _seq_verificar(self):
        q = self._q_actual()
        if q is None:
            return
        est = self.con.estado()
        V = S.verificar(self.seq_puntos, q, self.z_min, self._error_modelo(est), self.con.listo())
        self.seq_V = V
        tv = self.tabla_tray
        tv.delete(*tv.get_children())
        for t in V.trayectos:
            an = t.analisis
            salto = float(np.max(np.abs(np.subtract(an.q_objetivo_deg, an.q_actual_deg))))
            if t.ok:
                texto = "OK" + (" (ya está ahí)" if t.quieto else f" · {len(an.tramos)} tramo(s)")
            else:
                texto = "NO · " + "; ".join(t.fallas)
            tv.insert("", "end", values=(t.nombre, f"{salto:.1f}", f"{an.z_min_trayecto_mm:.0f}", texto),
                      tags=("ok" if t.ok else "no",))
        if V.ok:
            self.lbl_seq_ver.configure(text=f"Secuencia APTA: {len(self.seq_puntos)} puntos, {len(V.trayectos)} "
                                            "trayectos verificados. Revisa la vista 3D y presiona INICIAR CICLO.",
                                       foreground=VERDE)
        else:
            self.lbl_seq_ver.configure(text="Secuencia NO apta: " + ("; ".join(V.problemas) or
                                                                     "corrige los trayectos en rojo."),
                                       foreground=ROJO)
        self.registrar("secuencia_verificada", f"secuencia {'APTA' if V.ok else 'NO apta'} "
                                               f"({len(self.seq_puntos)} puntos)",
                       puntos=[dict(nombre=p.nombre, q=p.q_deg, espera_s=p.espera_s, accion=p.accion)
                               for p in self.seq_puntos],
                       trayectos=[dict(nombre=t.nombre, ok=t.ok, fallas=t.fallas) for t in V.trayectos])

    def _seq_lista_para_iniciar(self, q, ok_listo):
        V = self.seq_V
        return bool(V and V.ok and V.firma == S.firma(self.seq_puntos) and q is not None and ok_listo
                    and not self.moviendo and self.ejec is None
                    and float(np.max(np.abs(np.subtract(q, V.q_actual_deg)))) <= REANALIZAR_DEG)

    def _seq_iniciar(self):
        try:
            reps = max(0, int(float(self.ent_reps.get())))
        except ValueError:
            messagebox.showerror("Dato inválido", "Repeticiones debe ser un número entero (0 = hasta detener).")
            return
        texto = (f"Modo: {self.con.nombre}\n{len(self.seq_puntos)} puntos · "
                 f"{'hasta que presiones DETENER' if reps == 0 else f'{reps} repetición(es)'}\n\n")
        texto += ("¿Supervisor presente, zona despejada y operador con la mano en el habilitador o en el paro "
                  "de emergencia durante TODA la secuencia?" if self.con.nombre != "simulado"
                  else "Robot simulado: no hay movimiento físico.")
        if not messagebox.askyesno("Iniciar trabajo en serie", texto, icon="warning"):
            return
        self.analisis = None
        self.moviendo = True

        def enviar(tramos):
            self.con.enviar(tramos, lambda *x: None, lambda ok, msg: self.cola.put(("seq_fin", (ok, msg))))

        def al_iniciar(etq, q_dest):
            self.traza.inicio_movimiento(q_dest, A.pose_desde_T(A.fk_tcp(q_dest))[:3], etiqueta=etq)

        self.ejec = S.Ejecutor(self.seq_puntos, self.seq_V, reps, enviar, self.registrar,
                               al_iniciar_tramo=al_iniciar, al_llegar=lambda ep: self.traza.fin_movimiento(ep),
                               al_cerrar_ciclo=self._seq_ciclo_cerrado)
        self.registrar("secuencia_inicio", f"trabajo en serie iniciado: {len(self.seq_puntos)} puntos, "
                                           f"repeticiones {reps or 'hasta detener'}")

    def _seq_ciclo_cerrado(self, datos):
        t_fin = time.time() - self.traza.t0
        n = datos["n"]
        try:
            GV.guardar_ciclos(self.ejec.ciclos, self.carpeta_vivo)
            if n <= 3 or n % 10 == 0:   # evidencia sin llenar el disco en series largas
                GV.guardar_intervalo(self.traza, t_fin - datos["duracion_s"] - 0.5, t_fin + 0.5, f"ciclo_{n}",
                                     f"Ciclo {n} · {datos['duracion_s']:.1f} s · e_p máx {datos['ep_max_mm']:.3f} mm",
                                     self.carpeta_vivo)
        except Exception as e:  # noqa: BLE001
            self.registrar("error", f"no se pudo guardar la gráfica del ciclo {n}: {e}")

    def _seq_detener_fin_ciclo(self):
        if self.ejec is not None:
            self.ejec.detener_fin_ciclo = True
            self.registrar("secuencia_detener", "el operador pidió detener al terminar el ciclo")

    def _seq_detener_ya(self):
        if self.ejec is not None:
            self.ejec.detener_ya = True
            self.registrar("secuencia_detener", "el operador pidió DETENER (al terminar el tramo en curso)")

    def _seq_terminada(self):
        ej = self.ejec
        self.ejec = None
        self.moviendo = False
        self.seq_V = None     # la posicion cambio: hay que verificar de nuevo
        self.lbl_seq.configure(text=f"Secuencia {'detenida' if ej.estado == 'detenido' else 'DETENIDA POR ERROR'}: "
                                    f"{ej.motivo} · {ej.ciclo} ciclo(s) completos",
                               foreground=GRIS if ej.estado == "detenido" else ROJO)
        self.lbl_seq_ver.configure(text="Secuencia terminada: VERIFICA de nuevo antes de iniciar otra vez.",
                                   foreground=GRIS)
        try:
            ruta = GV.guardar_ciclos(ej.ciclos, self.carpeta_vivo)
            if ruta:
                self.registrar("grafica", f"resumen de ciclos guardado en {ruta}", archivo=ruta)
        except Exception as e:  # noqa: BLE001
            self.registrar("error", f"no se pudo guardar el resumen de ciclos: {e}")
        if ej.estado == "detenido" and self.var_seq_home.get():
            self.root.after(300, self._ir_home)
        if ej.estado == "error":
            messagebox.showwarning("Secuencia detenida", ej.motivo)

    # ------------------------------------------------------------------ acciones
    def _copiar_pose(self):
        q = self._q_actual()
        if q is None:
            return
        for e, v in zip(self.ent_pose, A.pose_desde_T(A.fk_tcp(q))):
            e.delete(0, "end")
            e.insert(0, f"{v:.2f}")

    def _vertical(self):
        for e, v in zip(self.ent_pose[3:], (0, 180, 0)):
            e.delete(0, "end")
            e.insert(0, str(v))

    def _copiar_q(self):
        q = self._q_actual()
        if q is None:
            return
        for e, v in zip(self.ent_q, q):
            e.delete(0, "end")
            e.insert(0, f"{v:.2f}")

    def _paso(self, k, signo):
        try:
            paso = float(self.ent_paso.get().replace(",", "."))
            v = float(self.ent_q[k].get().replace(",", "."))
        except ValueError:
            return
        self.ent_q[k].delete(0, "end")
        self.ent_q[k].insert(0, f"{v + signo * paso:.2f}")

    def _analizar_pose(self):
        pose = self._leer_numeros(self.ent_pose, "la pose")
        q = self._q_actual()
        if pose is None or q is None:
            return
        self._analizar(q, pose=pose)

    def _analizar_ejes(self):
        qo = self._leer_numeros(self.ent_q, "los ángulos")
        q = self._q_actual()
        if qo is None or q is None:
            return
        self._analizar(q, q_objetivo_deg=qo)
        p = A.pose_desde_T(A.fk_tcp(qo))
        self.lbl_fk_obj.configure(text="FK del objetivo: X {:.1f} Y {:.1f} Z {:.1f} mm | R {:.1f} P {:.1f} Y {:.1f} °".format(*p))

    def _analizar(self, q, **kw):
        est = self.con.estado()
        self.analisis = A.analizar(q, z_min_mm=self.z_min, error_modelo=self._error_modelo(est),
                                   listo=self.con.listo(), **kw)
        a = self.analisis
        self.tabla.delete(*self.tabla.get_children())
        for r in a.ramas:
            estado = "ELEGIDA" if r.elegida else ("válida" if r.en_limites else r.motivo)
            self.tabla.insert("", "end", values=(r.nombre, *[_f1(v) for v in r.q_deg], estado),
                              tags=("elegida",) if r.elegida else (() if r.en_limites else ("fuera",)))
        for l, c in zip(self.lbl_chk, a.chequeos + [None] * 7):
            if c is None:
                l.configure(text="")
            else:
                l.configure(text=f"{'OK ' if c.ok else 'NO '}   {c.nombre}: {c.detalle}",
                            foreground=VERDE if c.ok else ROJO)
        self.lbl_vigencia.configure(
            text="Apto para mover: revisa el objetivo en la vista 3D (naranja) antes de confirmar."
            if a.apto else "No apto: corrige los puntos en rojo y vuelve a ANALIZAR.",
            foreground=VERDE if a.apto else ROJO)
        self.registrar("analisis", f"análisis {a.modo}: {'APTO' if a.apto else 'NO APTO'} "
                                   f"({sum(c.ok for c in a.chequeos)}/{len(a.chequeos)} OK)", **a.resumen())
        self._obj_dibujado = None   # forzar redibujo

    def _mover(self):
        a = self.analisis
        if a is None or not a.apto or self.moviendo:
            return
        salto = float(np.max(np.abs(np.subtract(a.q_objetivo_deg, a.q_actual_deg))))
        texto = (f"Modo: {self.con.nombre}\nObjetivo (°): {np.round(a.q_objetivo_deg, 2).tolist()}\n"
                 f"Tramos: {len(a.tramos)} · salto máximo {salto:.1f}° · z mínima {a.z_min_trayecto_mm:.0f} mm\n\n")
        if self.con.nombre != "simulado":
            texto += ("¿Supervisor presente, zona despejada y operador con la mano en el habilitador "
                      "o en el paro de emergencia?")
        else:
            texto += "Robot simulado: no hay movimiento físico."
        if not messagebox.askyesno("Confirmar movimiento", texto, icon="warning"):
            self.registrar("cancelado", "movimiento cancelado por el operador")
            return
        self.moviendo = True
        self.btn_mover.configure(state="disabled")
        self.traza.inicio_movimiento(a.q_objetivo_deg, A.pose_desde_T(A.fk_tcp(a.q_objetivo_deg))[:3])
        self.registrar("envio", f"enviando {len(a.tramos)} tramo(s) a {np.round(a.q_objetivo_deg, 2).tolist()}",
                       tramos=a.tramos)
        self.con.enviar(a.tramos,
                        lambda *x: self.cola.put(("avance", x)),
                        lambda ok, msg: self.cola.put(("fin", (ok, msg))))

    def _fin_movimiento(self, ok, msg):
        if ok:
            # la verificacion de llegada usa una lectura POSTERIOR al fin del movimiento
            self._llegada_pendiente = time.time()
            self.lbl_llegada.configure(text="Movimiento terminado: verificando llegada...", foreground=GRIS)
            return
        self.moviendo = False
        self.lbl_llegada.configure(text=f"Movimiento no completado: {msg}", foreground=ROJO)
        self.registrar("error", f"movimiento no completado: {msg}")
        self._cerrar_mov_grafica(None)
        self._cerrar_analisis()

    def _verificar_llegada(self, est):
        a = self.analisis
        self._llegada_pendiente = None
        self.moviendo = False
        err_q = np.abs(np.subtract(est["q_deg"], a.q_objetivo_deg))
        ep, eR = A.errores_pose(a.T_deseada, A.fk_tcp(est["q_deg"]))
        texto = f"Llegada: error máx. por eje {err_q.max():.3f}° · TCP e_p {ep:.3f} mm · e_R {eR:.3f}°"
        self.lbl_llegada.configure(text=texto, foreground=VERDE if ep <= 1.0 else ROJO)
        self.registrar("llegada", texto, q_medido=est["q_deg"], error_eje_deg=err_q.tolist(), ep_mm=ep, eR_deg=eR)
        self._cerrar_mov_grafica(ep)
        self._cerrar_analisis()

    def _cerrar_analisis(self):
        self.analisis = None   # obligar a analizar de nuevo desde la nueva postura
        self.lbl_vigencia.configure(text="Movimiento terminado: ANALIZA de nuevo antes del siguiente.",
                                    foreground=GRIS)

    # ------------------------------------------------------------------ ciclo
    def _ciclo(self):
        while not self.cola.empty():
            tipo, datos = self.cola.get()
            if tipo == "avance":
                i, n, q = datos[:3]
                if len(datos) == 3:
                    self.registrar("tramo", f"tramo {i}/{n} -> {np.round(q, 2).tolist()}")
                else:
                    self.registrar("tramo_ok", f"tramo {i}/{n} alcanzado: {np.round(datos[3], 2).tolist()}")
            elif tipo == "fin":
                self._fin_movimiento(*datos)
            elif tipo == "seq_fin" and self.ejec is not None:
                self.ejec.fin(*datos)

        est = self.con.estado()
        if (self._llegada_pendiente is not None and est["q_deg"] is not None and self.analisis is not None
                and est["stamp"] > self._llegada_pendiente + 0.05):
            self._verificar_llegada(est)
        ok_listo, motivo = self.con.listo()
        if est["conectado"]:
            self.lbl_con.configure(text=f"Conectado ({self.con.nombre}) · {motivo}",
                                   foreground=VERDE if ok_listo else AZUL)
        else:
            self.lbl_con.configure(text=f"Sin conexión ({self.con.nombre}): {est['error'] or 'esperando datos'}",
                                   foreground=ROJO)
        q = est["q_deg"]
        if q is not None:
            for l, v in zip(self.lbl_q, q):
                l.configure(text=f"{v:8.2f}")
            p = A.pose_desde_T(A.fk_tcp(q))
            self.traza.agregar(est["stamp"], q, p[:3])
            self.lbl_pose.configure(text="TCP (FK propia): X {:.1f}  Y {:.1f}  Z {:.1f} mm | R {:.1f}  P {:.1f}  Y {:.1f} °".format(*p))
        em = self._error_modelo(est)
        if em is not None:
            ok_m = em[0] <= A.TOL_MODELO_MM and em[1] <= A.TOL_MODELO_DEG
            self.lbl_modelo.configure(text=f"Modelo vs controlador (tool0): e_p {em[0]:.3f} mm · e_R {em[1]:.3f}° "
                                           f"{'OK' if ok_m else 'REVISAR'}", foreground=VERDE if ok_m else ROJO)

        # MOVER solo si el analisis es apto, vigente (robot quieto desde entonces) y el robot esta listo
        a = self.analisis
        vigente = (a is not None and q is not None and
                   float(np.max(np.abs(np.subtract(q, a.q_actual_deg)))) <= REANALIZAR_DEG)
        if a is not None and not vigente and not self.moviendo:
            self.lbl_vigencia.configure(text="El robot se movió después del análisis: vuelve a ANALIZAR.",
                                        foreground=ROJO)
        habilitar = bool(a and a.apto and vigente and ok_listo and not self.moviendo)
        self.btn_mover.configure(state="normal" if habilitar else "disabled")

        # secuencia repetitiva
        if self.ejec is not None:
            self.ejec.tick(est, (ok_listo, motivo), em)
            if self.ejec.activo:
                self.lbl_seq.configure(text=self.ejec.descripcion(), foreground=AZUL)
            else:
                self._seq_terminada()
        en_curso = self.ejec is not None
        self.btn_seq_iniciar.configure(state="normal" if self._seq_lista_para_iniciar(q, ok_listo) else "disabled")
        self.btn_seq_fin_ciclo.configure(state="normal" if en_curso else "disabled")
        self.btn_seq_parar.configure(state="normal" if en_curso else "disabled")

        self._dibujar(q)
        self.root.after(150, self._ciclo)

    def _dibujar(self, q):
        obj = None if self.analisis is None else self.analisis.q_objetivo_deg
        clave_q = None if q is None else tuple(np.round(q, 1))
        clave_o = None if obj is None else tuple(np.round(obj, 1))
        clave_o = (clave_o, S.firma(self.seq_puntos) if self.seq_puntos else None)
        if clave_q == self._q_dibujado and clave_o == self._obj_dibujado:
            return
        self._q_dibujado, self._obj_dibujado = clave_q, clave_o
        ax = self.ax
        ax.cla()
        xs = np.array([-0.3, 0.6])
        X, Y = np.meshgrid(xs, np.array([-0.45, 0.45]))
        ax.plot_surface(X, Y, np.zeros_like(X), alpha=0.12, color="#888888")
        ax.plot_surface(X, Y, np.full_like(X, self.z_min / 1000.0), alpha=0.10, color="#d9541e")

        def esqueleto(qq, **st):
            Ts = M.fk_all(np.deg2rad(qq), A.T_TCP)
            P = np.array([T[:3, 3] for T in Ts])
            ax.plot(P[:, 0], P[:, 1], P[:, 2], **st)
            return Ts[-1]

        if q is not None:
            T = esqueleto(q, color="#13202e", lw=4, marker="o", ms=4, label="robot (lectura)")
            ax.scatter(*T[:3, 3], color="#13202e", s=30)
        if obj is not None:
            T = esqueleto(obj, color="#e0662f", lw=2.5, ls="--", marker="o", ms=3, label="objetivo")
            for k, c in enumerate(("r", "g", "b")):
                ax.quiver(*T[:3, 3], *(T[:3, k] * 0.06), color=c, lw=1.5)
        if self.seq_puntos:
            P = np.array([p.xyz_mm for p in self.seq_puntos + self.seq_puntos[:1]]) / 1000.0
            ax.plot(P[:, 0], P[:, 1], P[:, 2], color="#2a78d6", lw=1.2, ls=":", marker="s", ms=4,
                    label="secuencia")
            for i, pt in enumerate(P[:-1], 1):
                ax.text(*pt, f" P{i}", fontsize=7, color="#1f2328")
        ax.set_xlim(-0.3, 0.6)
        ax.set_ylim(-0.45, 0.45)
        ax.set_zlim(0, 0.8)
        ax.set_box_aspect((0.9, 0.9, 0.8))
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.set_title(f"Vista 3D (modelo propio) · plano naranja = z mínima {self.z_min:.0f} mm", fontsize=10)
        ax.legend(loc="upper left", fontsize=8)
        self.canvas.draw_idle()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Panel IRB 120: verificar antes de mover")
    ap.add_argument("--modo", choices=["simulado", "directo", "ros"], default="simulado")
    ap.add_argument("--robot", default="192.168.125.1", help="IP del controlador (modo directo)")
    ap.add_argument("--rws", default=None, help="URL RWS (por defecto http://<robot>)")
    ap.add_argument("--puerto", type=int, default=1025)
    ap.add_argument("--velocidad", type=float, default=25.0, help="mm/s (modo directo)")
    ap.add_argument("--z-min", type=float, default=A.Z_MIN_MM_DEF, help="altura minima del TCP en mm")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])
    if args.modo == "ros":
        try:
            import rclpy  # noqa: F401
            args_ros = rclpy.utilities.remove_ros_args(sys.argv)
            args = ap.parse_args(args_ros[1:] if argv is None else argv)
        except ImportError:
            print("modo ros requiere ROS 2 (rclpy)")
            return 1
    con = conexiones.crear(args.modo, **({} if args.modo != "directo" else dict(
        rws_url=args.rws or f"http://{args.robot}", host=args.robot, port=args.puerto,
        velocidad_mm_s=args.velocidad)))
    con.iniciar()
    root = tk.Tk()
    Panel(root, con, args.z_min)
    try:
        root.mainloop()
    finally:
        con.cerrar()
    return 0


if __name__ == "__main__":
    sys.exit(main())
