"""
Modelo de REFERENCIA a partir del URDF oficial de ROS-Industrial (abb_irb120_support).

Dos implementaciones, ninguna usa los parametros DH del modelo propio:
  1) Pinocchio (si esta instalado: `pip install pin`) -> referencia de terceros.
  2) Parser URDF minimo en numpy (respaldo para Windows sin Pinocchio): recorre la cadena
     base_link -> ... -> frame aplicando origin(xyz, rpy) y rotacion sobre 'axis'.

Ambas devuelven ^B T_frame(q) y el Jacobiano geometrico [v; w] en {B} para el punto 'frame'.
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "..", "description", "irb120_3_58_gripper1.urdf")


def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle_R(axis, th):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


class UrdfChainNumpy:
    """Cadena serie leida del URDF (implementacion independiente, solo numpy)."""

    name = "urdf-numpy"

    def __init__(self, urdf_path: str = URDF_PATH, root: str = "base_link"):
        tree = ET.parse(urdf_path)
        self.joints = {}
        for j in tree.getroot().findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [1, 0, 0]
            self.joints[j.find("child").get("link")] = dict(
                name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                T=np.block([[_rpy_to_R(*rpy), xyz[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]),
                axis=np.array(axis))
        self.root = root

    def _chain(self, frame):
        chain, link = [], frame
        while link != self.root:
            j = self.joints[link]
            chain.append(j)
            link = j["parent"]
        return chain[::-1]

    def fk(self, q, frame="tool0"):
        T = np.eye(4)
        qi = iter(np.asarray(q, float))
        self._axes = []
        for j in self._chain(frame):
            T = T @ j["T"]
            if j["type"] in ("revolute", "continuous"):
                self._axes.append((T[:3, 3].copy(), T[:3, :3] @ j["axis"]))
                Rj = np.eye(4)
                Rj[:3, :3] = _axis_angle_R(j["axis"], next(qi))
                T = T @ Rj
        return T

    def jacobian(self, q, frame="tool0"):
        T = self.fk(q, frame)
        p = T[:3, 3]
        J = np.zeros((6, len(self._axes)))
        for i, (o, z) in enumerate(self._axes):
            J[:3, i] = np.cross(z, p - o)
            J[3:, i] = z
        return J


class UrdfPinocchio:
    """Referencia de terceros: Pinocchio cargando el mismo URDF."""

    def __init__(self, urdf_path: str = URDF_PATH):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.name = f"pinocchio-{pin.__version__}"

    def fk(self, q, frame="tool0"):
        pin = self.pin
        pin.framesForwardKinematics(self.model, self.data, np.asarray(q, float))
        return self.data.oMf[self.model.getFrameId(frame)].homogeneous.copy()

    def jacobian(self, q, frame="tool0"):
        pin = self.pin
        return pin.computeFrameJacobian(self.model, self.data, np.asarray(q, float),
                                        self.model.getFrameId(frame), pin.LOCAL_WORLD_ALIGNED).copy()

    def ik_numeric(self, T_des, frame, q0, iters=500, damp=1e-6, tol=1e-12):
        """IK numerica de referencia (CLIK con minimos cuadrados amortiguados) desde una semilla."""
        pin = self.pin
        fid = self.model.getFrameId(frame)
        oMdes = pin.SE3(T_des[:3, :3], T_des[:3, 3])
        q = np.asarray(q0, float).copy()
        for _ in range(iters):
            pin.framesForwardKinematics(self.model, self.data, q)
            iMd = self.data.oMf[fid].actInv(oMdes)
            err = pin.log(iMd).vector
            if np.linalg.norm(err) < tol:
                break
            J = pin.computeFrameJacobian(self.model, self.data, q, fid, pin.LOCAL)
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            v = -J.T @ np.linalg.solve(J @ J.T + damp * np.eye(6), err)
            q = pin.integrate(self.model, q, v)
        return q, float(np.linalg.norm(err))


def load_reference():
    try:
        return UrdfPinocchio()
    except Exception:  # Pinocchio no disponible (p.ej. Windows sin conda)
        return UrdfChainNumpy()
