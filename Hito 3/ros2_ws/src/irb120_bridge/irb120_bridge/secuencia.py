"""
Secuencia repetitiva (trabajo en serie): una lista de puntos que el robot recorre en ciclo
P1 -> P2 -> ... -> Pn -> P1 -> ... hasta que el operador la detiene o se cumplen N repeticiones.

Misma regla que el panel: la PC verifica TODO antes de mover.
  * Cada punto se guarda como angulos articulares (la rama de IK se elige UNA vez, al crear el punto,
    asi el robot repite exactamente la misma postura en todos los ciclos).
  * VERIFICAR analiza cada trayecto: entrada (posicion actual -> P1), P1->P2, ..., Pn->P1 (cierre),
    con los mismos chequeos del panel (limites, altura en todo el trayecto, tramos <= 40 deg,
    modelo vs controlador, robot listo).
  * Durante la ejecucion, antes de CADA envio se vuelve a comprobar: robot listo, modelo vs
    controlador y que el robot este realmente en el punto anterior. Al llegar a cada punto se
    verifica la llegada con una lectura posterior al fin del movimiento (e_p <= 1 mm).
    Cualquier falla detiene la secuencia.
  * Detener: "al terminar el ciclo" (queda en P1) o "ya" (queda en el punto al que esta yendo).
    El PC no puede frenar un tramo en curso: para eso estan el habilitador, Stop y el paro de emergencia.

Sin GUI y sin ROS: se prueba sola (test/test_secuencia.py).
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field

import numpy as np

from . import analisis as A

TOL_LLEGADA_MM = 1.0
TOL_LLEGADA_DEG = 0.2
TOL_POSICION_INICIAL_DEG = 0.5


@dataclass
class Punto:
    nombre: str
    q_deg: list
    espera_s: float = 0.0
    accion: str = ""                 # texto informativo (p. ej. "cerrar gripper"); no se ejecuta

    @property
    def xyz_mm(self):
        return A.pose_desde_T(A.fk_tcp(self.q_deg))[:3]


def punto_desde_pose(nombre, pose, q_ref, espera_s=0.0, accion="", z_min_mm=A.Z_MIN_MM_DEF):
    """IK propia: elige la rama valida mas cercana a q_ref. Lanza ValueError con el motivo."""
    an = A.analizar(q_ref, pose=pose, z_min_mm=z_min_mm, error_modelo=(0.0, 0.0))
    if an.q_objetivo_deg is None:
        motivo = next((c.detalle for c in an.chequeos if not c.ok), "sin solucion")
        raise ValueError(f"{nombre}: {motivo}")
    return Punto(nombre, [round(v, 4) for v in an.q_objetivo_deg], float(espera_s), accion)


def plantilla_recoger_dejar(recoger_xyz, dejar_xyz, q_ref, altura_aprox_mm=80.0, rpy=(0, 180, 0),
                            espera_s=1.0, z_min_mm=A.Z_MIN_MM_DEF):
    """Ciclo clasico de pick & place con aproximacion vertical (6 puntos)."""
    rx, ry, rz = recoger_xyz
    dx, dy, dz = dejar_xyz
    h = float(altura_aprox_mm)
    pasos = [("sobre recoger", (rx, ry, rz + h), 0.0, ""),
             ("recoger", (rx, ry, rz), espera_s, "cerrar gripper"),
             ("subir con pieza", (rx, ry, rz + h), 0.0, ""),
             ("sobre dejar", (dx, dy, dz + h), 0.0, ""),
             ("dejar", (dx, dy, dz), espera_s, "abrir gripper"),
             ("subir sin pieza", (dx, dy, dz + h), 0.0, "")]
    puntos, q = [], list(q_ref)
    for nombre, xyz, espera, accion in pasos:
        p = punto_desde_pose(nombre, [*xyz, *rpy], q, espera, accion, z_min_mm)
        puntos.append(p)
        q = p.q_deg
    return puntos


def guardar(puntos, ruta):
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(dict(formato="irb120_secuencia_v1", puntos=[asdict(p) for p in puntos]), f,
                  ensure_ascii=False, indent=2)


def cargar(ruta):
    with open(ruta, encoding="utf-8") as f:
        datos = json.load(f)
    return [Punto(**p) for p in datos["puntos"]]


# ------------------------------------------------------------------------- verificacion
@dataclass
class Trayecto:
    nombre: str                      # "entrada -> P1", "P1 -> P2", ..., "P6 -> P1 (cierre)"
    destino: int                     # indice del punto destino
    analisis: A.Analisis
    quieto: bool                     # origen y destino iguales: no hay que mover

    @property
    def ok(self):
        return all(c.ok or (self.quieto and c.nombre == "Tramos") for c in self.analisis.chequeos)

    @property
    def fallas(self):
        return [f"{c.nombre}: {c.detalle}" for c in self.analisis.chequeos
                if not c.ok and not (self.quieto and c.nombre == "Tramos")]


@dataclass
class Verificacion:
    trayectos: list = field(default_factory=list)
    problemas: list = field(default_factory=list)
    q_actual_deg: list | None = None
    firma: str = ""

    @property
    def ok(self):
        return not self.problemas and bool(self.trayectos) and all(t.ok for t in self.trayectos)


def firma(puntos):
    return json.dumps([asdict(p) for p in puntos], sort_keys=True)


def verificar(puntos, q_actual_deg, z_min_mm=A.Z_MIN_MM_DEF, error_modelo=None, listo=(True, "")):
    V = Verificacion(q_actual_deg=list(q_actual_deg), firma=firma(puntos))
    if len(puntos) < 2:
        V.problemas.append("la secuencia necesita al menos 2 puntos")
        return V
    n = len(puntos)
    pares = [("entrada -> P1", list(q_actual_deg), 0)]
    pares += [(f"P{i} -> P{i + 1}", puntos[i - 1].q_deg, i) for i in range(1, n)]
    pares.append((f"P{n} -> P1 (cierre)", puntos[-1].q_deg, 0))
    for nombre, q0, j in pares:
        an = A.analizar(q0, q_objetivo_deg=puntos[j].q_deg, z_min_mm=z_min_mm,
                        error_modelo=error_modelo, listo=listo)
        quieto = float(np.max(np.abs(np.subtract(q0, puntos[j].q_deg)))) <= 0.01
        V.trayectos.append(Trayecto(nombre, j, an, quieto))
    return V


# ------------------------------------------------------------------------- ejecucion
class Ejecutor:
    """
    Maquina de estados que el panel avanza con tick() cada ~150 ms.
      enviar(tramos)           manda los tramos al robot (asincrono); el panel llama fin(ok, msg)
      registrar(evento, texto, **datos)
      al_iniciar_tramo(nombre, q_destino) / al_llegar(e_p) / al_cerrar_ciclo(dict)   (graficas)
    """

    def __init__(self, puntos, verificacion, repeticiones, enviar, registrar,
                 al_iniciar_tramo=None, al_llegar=None, al_cerrar_ciclo=None):
        self.puntos, self.V, self.reps = puntos, verificacion, int(repeticiones)
        self._enviar, self._reg = enviar, registrar
        self._ini_tramo = al_iniciar_tramo or (lambda *a: None)
        self._llegar = al_llegar or (lambda *a: None)
        self._cerrar_ciclo = al_cerrar_ciclo or (lambda *a: None)
        self.estado = "por_enviar"         # por_enviar | moviendo | asentando | esperando | detenido | error
        self.idx = 0                       # punto destino actual
        self.ciclo = 0                     # ciclos completos
        self.entrada = True                # primer trayecto: posicion actual -> P1
        self.detener_fin_ciclo = False
        self.detener_ya = False
        self.motivo = ""
        self._t_fin = self._t_espera = None
        self._t_ciclo = None
        self._ep_ciclo = []
        self.ciclos = []                   # dict(n, duracion_s, ep_max_mm, ep_medio_mm)
        self._q_origen = list(verificacion.q_actual_deg)

    # -- informacion para la interfaz
    @property
    def activo(self):
        return self.estado not in ("detenido", "error")

    def descripcion(self):
        p = self.puntos[self.idx]
        base = {"por_enviar": "preparando", "moviendo": "yendo a", "asentando": "verificando llegada a",
                "esperando": f"esperando {p.espera_s:.1f} s en", "detenido": "detenida en",
                "error": "DETENIDA POR ERROR en"}[self.estado]
        texto = f"Ciclo {self.ciclo + 1}{'' if self.reps == 0 else f' de {self.reps}'} · {base} P{self.idx + 1} ({p.nombre})"
        if p.accion and self.estado == "esperando":
            texto += f" · {p.accion}"
        if self.ciclos:
            u = self.ciclos[-1]
            texto += f" · último ciclo {u['duracion_s']:.1f} s, e_p máx {u['ep_max_mm']:.3f} mm"
        return texto

    def _trayecto(self):
        if self.entrada:
            return self.V.trayectos[0]
        return self.V.trayectos[self.idx if self.idx > 0 else len(self.puntos)]

    def _parar(self, estado, motivo):
        self.estado, self.motivo = estado, motivo
        self._reg("secuencia_fin" if estado == "detenido" else "secuencia_error",
                  f"secuencia {'detenida' if estado == 'detenido' else 'DETENIDA POR ERROR'}: {motivo} "
                  f"({self.ciclo} ciclo(s) completos)", ciclos=self.ciclos)

    def fin(self, ok, msg):
        if self.estado != "moviendo":
            return
        if not ok:
            self._parar("error", f"movimiento no completado: {msg}")
            return
        self.estado, self._t_fin = "asentando", time.time()

    def tick(self, est, listo, error_modelo, ahora=None):
        ahora = time.time() if ahora is None else ahora
        q = est.get("q_deg")
        if self.estado == "por_enviar":
            if self.detener_ya:
                self._parar("detenido", "detenida por el operador")
                return
            ok_l, motivo_l = listo
            if not ok_l:
                self._parar("error", f"robot no listo: {motivo_l}")
                return
            if error_modelo is None or error_modelo[0] > A.TOL_MODELO_MM or error_modelo[1] > A.TOL_MODELO_DEG:
                self._parar("error", f"modelo vs controlador fuera de tolerancia: {error_modelo}")
                return
            if q is None or np.max(np.abs(np.subtract(q, self._q_origen))) > TOL_POSICION_INICIAL_DEG:
                self._parar("error", "el robot no está donde la secuencia lo esperaba (¿se movió a mano?)")
                return
            tr = self._trayecto()
            destino = self.puntos[self.idx]
            if tr.quieto:
                self.estado, self._t_fin = "asentando", ahora - 1.0
            else:
                if self._t_ciclo is None and not self.entrada:
                    self._t_ciclo = ahora
                self.estado = "moviendo"
                self._ini_tramo(f"c{self.ciclo + 1}·P{self.idx + 1}", destino.q_deg)
                self._reg("secuencia_tramo", f"ciclo {self.ciclo + 1}: {tr.nombre} ({destino.nombre})",
                          ciclo=self.ciclo + 1, punto=self.idx + 1, q_destino=destino.q_deg)
                self._enviar(tr.analisis.tramos)
        elif self.estado == "asentando":
            if q is None or est.get("stamp", 0) <= self._t_fin + 0.05:
                return
            destino = self.puntos[self.idx]
            err_q = float(np.max(np.abs(np.subtract(q, destino.q_deg))))
            ep, eR = A.errores_pose(A.fk_tcp(destino.q_deg), A.fk_tcp(q))
            self._llegar(ep)
            if ep > TOL_LLEGADA_MM or err_q > TOL_LLEGADA_DEG:
                self._parar("error", f"llegada a P{self.idx + 1} fuera de tolerancia: e_p {ep:.3f} mm, "
                                     f"error por eje {err_q:.3f}°")
                return
            if not self.entrada:
                self._ep_ciclo.append(ep)
            self._reg("secuencia_llegada", f"P{self.idx + 1} ({destino.nombre}): e_p {ep:.3f} mm"
                      + (f" · {destino.accion}" if destino.accion else ""),
                      ciclo=self.ciclo + 1, punto=self.idx + 1, ep_mm=ep, eR_deg=eR, q_medido=q)
            self._q_origen = list(q)
            self.estado, self._t_espera = "esperando", ahora + destino.espera_s
        elif self.estado == "esperando":
            if ahora < self._t_espera:
                return
            if self.entrada:
                self.entrada = False
                self._t_ciclo = None
            elif self.idx == 0:            # volvio a P1: ciclo completo
                self.ciclo += 1
                t0 = self._t_ciclo if self._t_ciclo is not None else ahora
                datos = dict(n=self.ciclo, duracion_s=ahora - t0,
                             ep_max_mm=max(self._ep_ciclo) if self._ep_ciclo else 0.0,
                             ep_medio_mm=float(np.mean(self._ep_ciclo)) if self._ep_ciclo else 0.0)
                self.ciclos.append(datos)
                self._reg("ciclo", f"ciclo {self.ciclo} completo en {datos['duracion_s']:.1f} s · "
                                   f"e_p máx {datos['ep_max_mm']:.3f} mm", **datos)
                self._cerrar_ciclo(datos)
                self._ep_ciclo, self._t_ciclo = [], ahora
                if self.detener_fin_ciclo:
                    self._parar("detenido", "detenida al terminar el ciclo (robot en P1)")
                    return
                if self.reps and self.ciclo >= self.reps:
                    self._parar("detenido", f"se completaron las {self.reps} repeticiones (robot en P1)")
                    return
            self.idx = (self.idx + 1) % len(self.puntos)
            self.estado = "por_enviar"
