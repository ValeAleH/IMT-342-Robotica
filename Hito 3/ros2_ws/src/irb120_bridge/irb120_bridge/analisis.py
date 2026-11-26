"""
Verificacion de un objetivo ANTES de mover (lo que pide el docente: calcular en la PC si la
posicion es posible y recien entonces ofrecer el movimiento). Sin GUI y sin ROS: se prueba sola.

Chequeos, en orden (todos deben dar OK para habilitar MOVER):
  1. Pose alcanzable          la IK analitica propia encuentra al menos una solucion (modo pose)
  2. Limites reducidos        existe una rama dentro de los limites del puente (los mismos del RAPID)
  3. IK -> FK                 FK(q_elegida) reproduce la pose pedida (e_p <= 0.01 mm, e_R <= 0.01 deg)
  4. Altura en el trayecto    el TCP no baja de z_min en NINGUN punto del trayecto articular
  5. Tramos                   el movimiento se parte en tramos de <= 40 deg por eje (RAPID acepta 45)
  6. Modelo vs controlador    FK propia de tool0 vs pose informada por el IRC5 (<= 1 mm, <= 0.2 deg)
  7. Robot listo              conectado, motores encendidos, RAPID corriendo y sin otro movimiento

Unidades de la interfaz: mm y grados. Internamente: m y rad (irb120_model).
Referencia de diseno: el panel del repositorio Marcelo767n/robotica-IMT342 (rama proyectoHito3y4)
verifica IK->FK y modelo vs controlador antes de permitir el envio; aqui se implementa de forma
independiente sobre el modelo propio de este proyecto.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import irb120_model as M
from .rapid_client import LIM_HI_DEG, LIM_LO_DEG, MAX_STEP_DEG

TOL_IK_MM, TOL_IK_DEG = 0.01, 0.01
TOL_MODELO_MM, TOL_MODELO_DEG = 1.0, 0.2
TRAMO_MAX_DEG = 40.0          # margen respecto de MAX_STEP_DEG (45) del RAPID
Z_MIN_MM_DEF = 100.0          # igual que rb_Z_MIN del RAPID (revisar contra la mesa)
MUESTRAS_TRAYECTO = 60
T_TCP = M.T_GRIPPER1


# ------------------------------------------------------------------------- conversiones
def T_desde_pose(pose_mm_deg) -> np.ndarray:
    """[x, y, z (mm), roll, pitch, yaw (deg)] -> T 4x4 (m)."""
    x, y, z, r, p, yw = pose_mm_deg
    T = np.eye(4)
    T[:3, :3] = M.rpy_to_R(*np.deg2rad([r, p, yw]))
    T[:3, 3] = np.array([x, y, z]) / 1000.0
    return T


def pose_desde_T(T) -> list:
    r, p, yw = np.rad2deg(M.R_to_rpy(T[:3, :3]))
    return [*(T[:3, 3] * 1000.0).tolist(), float(r), float(p), float(yw)]


def fk_tcp(q_deg) -> np.ndarray:
    return M.fk(np.deg2rad(q_deg), T_TCP)


def errores_pose(T_ref, T_otro):
    """(e_p en mm, e_R en deg)."""
    ep, _ = M.pose_errors(T_ref, T_otro)
    eR = M.rot_angle_error_robust(T_ref[:3, :3], T_otro[:3, :3])
    return ep * 1000.0, math.degrees(eR)


def dentro_limites(q_deg):
    for i, v in enumerate(q_deg):
        if not (LIM_LO_DEG[i] <= v <= LIM_HI_DEG[i]):
            return False, f"J{i + 1} = {v:.1f} deg fuera de [{LIM_LO_DEG[i]}, {LIM_HI_DEG[i]}]"
    return True, ""


# ------------------------------------------------------------------------- resultados
@dataclass
class Rama:
    nombre: str
    q_deg: list
    en_limites: bool
    motivo: str
    salto_max_deg: float
    ep_mm: float
    eR_deg: float
    elegida: bool = False


@dataclass
class Chequeo:
    nombre: str
    ok: bool
    detalle: str


@dataclass
class Analisis:
    modo: str                         # 'pose' | 'ejes'
    q_actual_deg: list
    q_objetivo_deg: list | None
    T_deseada: np.ndarray | None
    ramas: list = field(default_factory=list)
    tramos: list = field(default_factory=list)       # lista de q (deg) a enviar, en orden
    z_min_trayecto_mm: float | None = None
    chequeos: list = field(default_factory=list)

    @property
    def apto(self) -> bool:
        return bool(self.chequeos) and all(c.ok for c in self.chequeos)

    def resumen(self) -> dict:
        return dict(modo=self.modo, apto=self.apto, q_actual_deg=self.q_actual_deg,
                    q_objetivo_deg=self.q_objetivo_deg,
                    pose_deseada=None if self.T_deseada is None else pose_desde_T(self.T_deseada),
                    tramos=len(self.tramos), z_min_trayecto_mm=self.z_min_trayecto_mm,
                    ramas=[dict(rama=r.nombre, q=[round(v, 3) for v in r.q_deg], en_limites=r.en_limites,
                                elegida=r.elegida) for r in self.ramas],
                    chequeos=[dict(nombre=c.nombre, ok=c.ok, detalle=c.detalle) for c in self.chequeos])


# ------------------------------------------------------------------------- planificacion
def planificar_tramos(q_ini, q_fin, tramo_max=TRAMO_MAX_DEG):
    q_ini, q_fin = np.asarray(q_ini, float), np.asarray(q_fin, float)
    n = max(1, int(math.ceil(np.max(np.abs(q_fin - q_ini)) / tramo_max - 1e-9)))
    return [(q_ini + (q_fin - q_ini) * k / n).tolist() for k in range(1, n + 1)]


def z_minima_trayecto(q_ini, q_fin, n=MUESTRAS_TRAYECTO) -> float:
    """Altura minima del TCP (mm) a lo largo de la interpolacion articular lineal (MoveAbsJ)."""
    q_ini, q_fin = np.asarray(q_ini, float), np.asarray(q_fin, float)
    return min(fk_tcp(q_ini + (q_fin - q_ini) * s)[2, 3] * 1000.0 for s in np.linspace(0, 1, n))


# ------------------------------------------------------------------------- analisis
def _ramas_ik(T_deseada, q_actual_deg):
    sols = M.ik(T_deseada, T_TCP, only_within_limits=False, q6_alternatives=False)
    ramas = []
    for s in sols:
        base = np.rad2deg(s["q"])
        # el eje 6 admite varias vueltas: de q6 y q6 +- 360 se toma la valida mas cercana a la actual
        variantes = []
        for k in (0, -1, 1):
            q = base.copy()
            q[5] += 360.0 * k
            variantes.append(q.tolist())
        variantes.sort(key=lambda q: (not dentro_limites(q)[0], abs(q[5] - q_actual_deg[5])))
        q = variantes[0]
        en, motivo = dentro_limites(q)
        ep, eR = errores_pose(T_deseada, fk_tcp(q))
        salto = float(np.max(np.abs(np.subtract(q, q_actual_deg))))
        ramas.append(Rama(s["branch"], q, en, motivo, salto, ep, eR))
    # orden: primero las validas, luego por cercania (norma ponderada: ejes de muneca pesan menos)
    w = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.3])
    ramas.sort(key=lambda r: (not r.en_limites,
                              float(np.linalg.norm(w * np.subtract(r.q_deg, q_actual_deg)))))
    return ramas


def analizar(q_actual_deg, *, pose=None, q_objetivo_deg=None, z_min_mm=Z_MIN_MM_DEF,
             error_modelo=None, listo=(True, "")) -> Analisis:
    """
    pose:            [x, y, z mm, roll, pitch, yaw deg] del TCP de Gripper1 (modo pose, usa IK)
    q_objetivo_deg:  seis angulos (modo ejes)
    error_modelo:    (e_p mm, e_R deg) FK propia vs controlador, o None si no hay dato
    listo:           (bool, motivo) estado del robot / conexion
    """
    if (pose is None) == (q_objetivo_deg is None):
        raise ValueError("indicar pose o q_objetivo_deg (uno solo)")
    q_actual_deg = [float(v) for v in q_actual_deg]
    A = Analisis(modo="pose" if pose is not None else "ejes", q_actual_deg=q_actual_deg,
                 q_objetivo_deg=None, T_deseada=None)

    if pose is not None:
        A.T_deseada = T_desde_pose(pose)
        A.ramas = _ramas_ik(A.T_deseada, q_actual_deg)
        A.chequeos.append(Chequeo("Pose alcanzable", bool(A.ramas),
                                  f"{len(A.ramas)} soluciones de IK" if A.ramas
                                  else "la IK no tiene solucion: fuera del alcance del brazo"))
        validas = [r for r in A.ramas if r.en_limites]
        if validas:
            validas[0].elegida = True
            A.q_objetivo_deg = validas[0].q_deg
        A.chequeos.append(Chequeo("Rama dentro de limites", bool(validas),
                                  f"elegida: {validas[0].nombre} ({len(validas)} validas)" if validas
                                  else ("ninguna rama cabe en los limites reducidos" if A.ramas else "sin ramas")))
        if validas:
            r = validas[0]
            ok = r.ep_mm <= TOL_IK_MM and r.eR_deg <= TOL_IK_DEG
            A.chequeos.append(Chequeo("IK -> FK", bool(ok), f"e_p = {r.ep_mm:.2e} mm, e_R = {r.eR_deg:.2e} deg"))
        else:
            A.chequeos.append(Chequeo("IK -> FK", False, "sin rama elegida"))
    else:
        q = [float(v) for v in q_objetivo_deg]
        en, motivo = dentro_limites(q)
        A.q_objetivo_deg = q
        A.T_deseada = fk_tcp(q)
        A.chequeos.append(Chequeo("Limites reducidos", en, motivo or "los 6 ejes dentro de los limites"))

    if A.q_objetivo_deg is not None:
        A.z_min_trayecto_mm = z_minima_trayecto(q_actual_deg, A.q_objetivo_deg)
        ok_z = A.z_min_trayecto_mm >= z_min_mm
        A.chequeos.append(Chequeo("Altura en el trayecto", bool(ok_z),
                                  f"z minima {A.z_min_trayecto_mm:.1f} mm (limite {z_min_mm:.0f} mm)"))
        A.tramos = planificar_tramos(q_actual_deg, A.q_objetivo_deg)
        salto = float(np.max(np.abs(np.subtract(A.q_objetivo_deg, q_actual_deg))))
        A.chequeos.append(Chequeo("Tramos", salto > 0.01,
                                  f"{len(A.tramos)} tramo(s), salto maximo {salto:.1f} deg"
                                  if salto > 0.01 else "el robot ya esta en el objetivo"))
    else:
        A.chequeos.append(Chequeo("Altura en el trayecto", False, "sin objetivo articular"))

    if error_modelo is None:
        A.chequeos.append(Chequeo("Modelo vs controlador", False, "sin pose del controlador"))
    else:
        ep, eR = error_modelo
        A.chequeos.append(Chequeo("Modelo vs controlador", bool(ep <= TOL_MODELO_MM and eR <= TOL_MODELO_DEG),
                                  f"e_p = {ep:.3f} mm, e_R = {eR:.3f} deg"))
    A.chequeos.append(Chequeo("Robot listo", bool(listo[0]), listo[1] or "listo"))
    return A
