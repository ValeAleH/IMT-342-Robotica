"""
Modelo cinematico PROPIO del ABB IRB 120-3/0.6 (Hito 3, IMT-342).

Convenciones (deben coincidir con model_manifest.yaml):
  - Marco base {B}: base del robot (= base_link del URDF de ROS-Industrial = wobj0/base del IRC5).
  - Marco de brida: tool0 (convencion ABB: z sale de la brida, x hacia abajo en la pose de calibracion).
  - TCP opcional: transformacion fija tool0 -> TCP (p.ej. Gripper1: [0, 0, 0.1675] m).
  - Orden articular: joint_1 ... joint_6 (= rax_1 ... rax_6 del controlador).
  - Unidades: rad y m. (El controlador entrega grados y mm.)
  - Jacobiano geometrico: twist ordenado [v; w] (lineal arriba), expresado en {B},
    velocidad lineal del punto TCP.

Parametros DH estandar (Denavit-Hartenberg clasico, A_i = Rz(th) Tz(d) Tx(a) Rx(alpha)),
derivados de las cotas del IRB 120 (d1=290, a2=270, a3=70, d4=302, d6=72 mm):

  i |  a_i [m] | alpha_i | d_i [m] | theta_i
  1 |  0       |  -pi/2  |  0.290  | q1
  2 |  0.270   |   0     |  0      | q2 - pi/2
  3 |  0.070   |  -pi/2  |  0      | q3
  4 |  0       |  +pi/2  |  0.302  | q4
  5 |  0       |  -pi/2  |  0      | q5
  6 |  0       |   0     |  0.072  | q6 + pi

Solo depende de numpy, para que corra igual en Windows.
"""
from __future__ import annotations

import numpy as np

# ----------------------------------------------------------------------------- parametros
D1, A2, A3, D4, D6 = 0.290, 0.270, 0.070, 0.302, 0.072
DH_A = np.array([0.0, A2, A3, 0.0, 0.0, 0.0])
DH_ALPHA = np.array([-np.pi / 2, 0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, 0.0])
DH_D = np.array([D1, 0.0, 0.0, D4, 0.0, D6])
DH_OFFSET = np.array([0.0, -np.pi / 2, 0.0, 0.0, 0.0, np.pi])

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

# Limites articulares del IRB 120 (especificacion de producto; coinciden con el URDF de ROS-I)
Q_MIN = np.deg2rad([-165.0, -110.0, -110.0, -160.0, -120.0, -400.0])
Q_MAX = np.deg2rad([165.0, 110.0, 70.0, 160.0, 120.0, 400.0])

# Geometria del codo (para la IK): el antebrazo tiene un offset a3 antes de la muneca
L_FORE = float(np.hypot(A3, D4))           # distancia codo -> centro de muneca
PHI_FORE = float(np.arctan2(D4, A3))       # angulo del antebrazo respecto de x3

T_GRIPPER1 = np.eye(4)
T_GRIPPER1[2, 3] = 0.1675  # tooldata Gripper1.tframe.trans = [0,0,167.5] mm (rot. por verificar)


# ----------------------------------------------------------------------------- utilidades
def dh_matrix(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def wrap_to_pi(x):
    return (np.asarray(x) + np.pi) % (2 * np.pi) - np.pi


def rot_angle_error(R_ref: np.ndarray, R_own: np.ndarray) -> float:
    """e_R = acos(clip((tr(R_ref^T R_own) - 1)/2, -1, 1))  [rad]  (protocolo, ec. 1)."""
    Re = R_ref.T @ R_own
    u = np.clip((np.trace(Re) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(u))


def rot_angle_error_robust(R_ref: np.ndarray, R_own: np.ndarray) -> float:
    """Mismo angulo que rot_angle_error pero via atan2(|vee(Re-Re^T)|/2, (tr-1)/2).
    acos() pierde resolucion cerca de 0 (piso ~ sqrt(2*eps) ~ 2e-8 rad); atan2 no."""
    Re = R_ref.T @ R_own
    v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]]) / 2.0
    return float(np.arctan2(np.linalg.norm(v), (np.trace(Re) - 1.0) / 2.0))


def rpy_to_R(roll, pitch, yaw) -> np.ndarray:
    """R = Rz(yaw) Ry(pitch) Rx(roll), angulos en rad (convencion ZYX, la de ROS)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def R_to_rpy(R: np.ndarray):
    """Inversa de rpy_to_R -> (roll, pitch, yaw) en rad. En pitch = +-90 deg fija roll = 0."""
    sp = -R[2, 0]
    if abs(sp) >= 1.0 - 1e-12:
        pitch = np.sign(sp) * np.pi / 2
        return 0.0, float(pitch), float(np.arctan2(-R[0, 1], R[1, 1]))
    return (float(np.arctan2(R[2, 1], R[2, 2])), float(np.arcsin(np.clip(sp, -1, 1))),
            float(np.arctan2(R[1, 0], R[0, 0])))


def quat_wxyz_to_R(q) -> np.ndarray:
    """Cuaternion ABB [q1,q2,q3,q4] = [w,x,y,z] -> matriz de rotacion."""
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q if q[0] >= 0 else -q


def within_limits(q, q_min=Q_MIN, q_max=Q_MAX, tol=1e-9) -> bool:
    q = np.asarray(q)
    return bool(np.all(q >= q_min - tol) and np.all(q <= q_max + tol))


# ----------------------------------------------------------------------------- FK
def fk_all(q, T_tcp: np.ndarray | None = None):
    """Devuelve la lista [T_0^0, T_0^1, ..., T_0^6, T_0^TCP] (7 u 8 matrices 4x4)."""
    q = np.asarray(q, dtype=float)
    Ts = [np.eye(4)]
    T = np.eye(4)
    for i in range(6):
        T = T @ dh_matrix(DH_A[i], DH_ALPHA[i], DH_D[i], q[i] + DH_OFFSET[i])
        Ts.append(T.copy())
    if T_tcp is not None:
        Ts.append(T @ T_tcp)
    return Ts


def fk(q, T_tcp: np.ndarray | None = None) -> np.ndarray:
    """^B T_TCP(q). Sin T_tcp devuelve tool0 (brida, convencion ABB)."""
    return fk_all(q, T_tcp)[-1]


# ----------------------------------------------------------------------------- Jacobiano
def jacobian(q, T_tcp: np.ndarray | None = None) -> np.ndarray:
    """Jacobiano geometrico 6x6 en {B}, punto TCP, twist [v; w] (protocolo, ec. 2)."""
    Ts = fk_all(q, T_tcp)
    p_tcp = Ts[-1][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = Ts[i][:3, 2]          # ^B z_{i-1}
        o = Ts[i][:3, 3]          # ^B o_{i-1}
        J[:3, i] = np.cross(z, p_tcp - o)
        J[3:, i] = z
    return J


def jacobian_fd_linear(q, T_tcp=None, eps: float = 1e-6) -> np.ndarray:
    """J_v por diferencias finitas centrales: (p(q+e_i eps) - p(q-e_i eps)) / 2eps (ec. 3)."""
    q = np.asarray(q, dtype=float)
    Jv = np.zeros((3, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        Jv[:, i] = (fk(q + dq, T_tcp)[:3, 3] - fk(q - dq, T_tcp)[:3, 3]) / (2 * eps)
    return Jv


def singular_values(q, T_tcp=None, scale_length: float = 1.0) -> np.ndarray:
    """Valores singulares de J. scale_length escala filas lineales para hacerlas comparables."""
    J = jacobian(q, T_tcp)
    J = J.copy()
    J[:3, :] /= scale_length
    return np.linalg.svd(J, compute_uv=False)


def rank_numeric(q, T_tcp=None, tol: float = 1e-3, scale_length: float = 1.0) -> int:
    s = singular_values(q, T_tcp, scale_length)
    return int(np.sum(s > tol * s[0]))


# ----------------------------------------------------------------------------- IK analitica
def ik(T_des: np.ndarray, T_tcp: np.ndarray | None = None, only_within_limits: bool = True,
       q6_alternatives: bool = False):
    """
    IK analitica cerrada (desacople de muneca esferica). Devuelve hasta 8 ramas:
      hombro {frente, atras} x codo {arriba, abajo} x muneca {no-flip, flip}.
    Cada solucion se devuelve como np.array (6,) en rad, envuelta a [-pi, pi) salvo q6,
    que opcionalmente incluye q6 +/- 2pi (el eje 6 del IRB120 admite +/-400 deg).
    """
    T_des = np.asarray(T_des, dtype=float)
    T_flange = T_des @ np.linalg.inv(T_tcp) if T_tcp is not None else T_des
    R = T_flange[:3, :3]
    p = T_flange[:3, 3]
    # centro de muneca: retroceder d6 a lo largo del eje de aproximacion (z de tool0)
    pw = p - D6 * R[:, 2]

    sols = []
    r_xy = np.hypot(pw[0], pw[1])
    th1_a = np.arctan2(pw[1], pw[0]) if r_xy > 1e-12 else 0.0
    for shoulder, th1 in (("front", th1_a), ("back", th1_a + np.pi)):
        u = r_xy if shoulder == "front" else -r_xy     # coordenada radial en el plano del brazo
        w = pw[2] - D1                                 # altura respecto del eje 2
        Dsq = u * u + w * w
        c = (Dsq - A2 ** 2 - L_FORE ** 2) / (2 * A2 * L_FORE)
        if abs(c) > 1.0 + 1e-12:
            continue                                   # fuera del alcance
        c = np.clip(c, -1.0, 1.0)
        for elbow, sgn in (("up", 1.0), ("down", -1.0)):
            th3 = sgn * np.arccos(c) - PHI_FORE
            # vector desde eje 2 al centro de muneca con q2 = 0 (plano u-w, giro horario = +q)
            s3, c3 = np.sin(th3), np.cos(th3)
            vu = D4 * c3 + A3 * s3
            vw = A2 - D4 * s3 + A3 * c3
            th2 = np.arctan2(u, w) - np.arctan2(vu, vw)
            q123 = np.array([th1, th2, th3])
            R03 = fk_all(np.r_[q123, 0, 0, 0])[3][:3, :3]
            R36 = R03.T @ R
            # R36 = Rz(q4) Ry(-q5) Rz(q6 + pi)
            c5 = np.clip(R36[2, 2], -1.0, 1.0)
            s5_abs = np.sqrt(max(0.0, 1.0 - c5 * c5))
            for wrist, s5 in (("noflip", s5_abs), ("flip", -s5_abs)):
                if s5_abs < 1e-9:
                    # singularidad de muneca: q4 y q6 acoplados, se fija q4 = 0
                    th5 = 0.0 if c5 > 0 else np.pi
                    th4 = 0.0
                    # con sb=0: R36 = Rz(q4 + q6 + pi) (o con signo si c5<0)
                    th6p = np.arctan2(R36[1, 0], R36[0, 0])
                    th6 = th6p - np.pi - th4
                    if wrist == "flip":
                        continue
                else:
                    th5 = np.arctan2(s5, c5)
                    th4 = np.arctan2(R36[1, 2] / (-s5), R36[0, 2] / (-s5))
                    th6p = np.arctan2(-R36[2, 1] / s5, R36[2, 0] / s5)
                    th6 = th6p - np.pi
                q = wrap_to_pi(np.array([th1, th2, th3, th4, th5, th6]))
                cands = [q]
                if q6_alternatives:
                    for k in (-1, 1):
                        q2 = q.copy()
                        q2[5] += 2 * np.pi * k
                        cands.append(q2)
                for qq in cands:
                    if only_within_limits and not within_limits(qq):
                        continue
                    sols.append({"q": qq, "branch": f"{shoulder}/{elbow}/{wrist}"})
    return sols


def pose_errors(T_ref: np.ndarray, T_own: np.ndarray):
    """(e_p [m], e_R [rad]) segun el protocolo de validacion."""
    ep = float(np.linalg.norm(T_own[:3, 3] - T_ref[:3, 3]))
    eR = rot_angle_error(T_ref[:3, :3], T_own[:3, :3])
    return ep, eR


if __name__ == "__main__":
    np.set_printoptions(precision=5, suppress=True)
    T = fk(np.zeros(6))
    print("FK(q=0) tool0:\n", T)
    print("Esperado (home ABB): p = [0.374, 0, 0.630] m, q = [0.7071, 0, 0.7071, 0]")
    print("quat:", R_to_quat_wxyz(T[:3, :3]))
