"""
Cliente minimo de Robot Web Services (RobotWare 6, RWS 1.0) - SOLO LECTURA.
Sin dependencias de ROS: se usa desde los nodos ROS 2 y desde los scripts de Windows.

Endpoints usados (todos GET):
  /rw/motionsystem/mechunits/ROB_1/jointtarget        -> rax_1..rax_6 [deg]
  /rw/motionsystem/mechunits/ROB_1/robtarget?tool=..&wobj=..&coordinate=Base -> x,y,z [mm], q1..q4
  /rw/panel/opmode     -> AUTO | MANR | MANF
  /rw/panel/ctrlstate  -> motoron | motoroff | guardstop | emergencystop | ...
  /rw/rapid/execution  -> ctrlexecstate: running | stopped
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import requests
from requests.auth import HTTPDigestAuth


@dataclass
class RobotState:
    stamp: float
    joints_deg: list[float] | None = None
    tcp_mm: list[float] | None = None        # x, y, z
    tcp_quat: list[float] | None = None      # q1..q4 = w, x, y, z
    opmode: str | None = None
    ctrlstate: str | None = None
    execstate: str | None = None
    errors: dict = field(default_factory=dict)


class RWSClient:
    def __init__(self, base_url="http://192.168.125.1", user="Default User", password="robotics",
                 mechunit="ROB_1", tool="tool0", wobj="wobj0", timeout=2.0):
        self.base = base_url.rstrip("/")
        self.s = requests.Session()
        self.s.auth = HTTPDigestAuth(user, password)
        self.mechunit, self.tool, self.wobj, self.timeout = mechunit, tool, wobj, timeout

    # -- bajo nivel ---------------------------------------------------------------------------
    def _state(self, path: str) -> dict:
        sep = "&" if "?" in path else "?"
        r = self.s.get(f"{self.base}{path}{sep}json=1", timeout=self.timeout)
        r.raise_for_status()
        js = r.json()
        states = js.get("_embedded", {}).get("_state", [])
        if not states:
            raise ValueError(f"respuesta sin _state en {path}: {str(js)[:200]}")
        return states[0]

    @staticmethod
    def _pick(d: dict, keys) -> list[float]:
        return [float(d[k]) for k in keys]

    # -- lecturas -----------------------------------------------------------------------------
    def joints_deg(self) -> list[float]:
        try:
            st = self._state(f"/rw/motionsystem/mechunits/{self.mechunit}/jointtarget")
        except Exception:
            st = self._state("/rw/rapid/tasks/T_ROB1/motion?resource=jointtarget")
        for keys in ([f"rax_{i}" for i in range(1, 7)], [f"j{i}" for i in range(1, 7)]):
            if all(k in st for k in keys):
                return self._pick(st, keys)
        raise KeyError(f"no se encontraron claves de ejes en {list(st)}")

    def tcp(self):
        path = (f"/rw/motionsystem/mechunits/{self.mechunit}/robtarget"
                f"?tool={self.tool}&wobj={self.wobj}&coordinate=Base")
        st = self._state(path)
        return self._pick(st, ["x", "y", "z"]), self._pick(st, ["q1", "q2", "q3", "q4"])

    def opmode(self) -> str:
        return self._state("/rw/panel/opmode").get("opmode")

    def ctrlstate(self) -> str:
        return self._state("/rw/panel/ctrlstate").get("ctrlstate")

    def execstate(self) -> str:
        return self._state("/rw/rapid/execution").get("ctrlexecstate")

    def read_all(self, with_status=True) -> RobotState:
        st = RobotState(stamp=time.time())
        try:
            st.joints_deg = self.joints_deg()
        except Exception as e:  # noqa: BLE001
            st.errors["joints"] = repr(e)
        try:
            st.tcp_mm, st.tcp_quat = self.tcp()
        except Exception as e:  # noqa: BLE001
            st.errors["tcp"] = repr(e)
        if with_status:
            for name in ("opmode", "ctrlstate", "execstate"):
                try:
                    setattr(st, name, getattr(self, name)())
                except Exception as e:  # noqa: BLE001
                    st.errors[name] = repr(e)
        return st
