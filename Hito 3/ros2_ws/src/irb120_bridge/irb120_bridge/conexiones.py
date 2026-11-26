"""
Conexiones del panel. Las tres tienen la misma interfaz:

  iniciar()                      arranca la lectura de estado en segundo plano
  estado() -> dict               q_deg, T_ctrl (tool0 segun el controlador, m), opmode, ctrlstate,
                                 execstate, conectado, error, ocupado
  listo() -> (bool, motivo)      si se puede enviar un movimiento ahora
  enviar(tramos, al_avanzar, al_terminar)   envia los tramos (deg) uno por uno en otro hilo
  cerrar()

  Simulada  robot ficticio dentro del panel (sin red) para practicar
  Directa   RWS (estado) + socket a ROS_BRIDGE.mod (comando), sin ROS: igual que tools/gate3_cli.py
  Ros2      se suscribe a /joint_states y publica /irb120/joint_goal: requiere
            `ros2 launch irb120_bridge bringup.launch.py mode:=real command:=true`
"""
from __future__ import annotations

import json
import threading
import time

import numpy as np

from . import irb120_model as M


def _vacio():
    return dict(q_deg=None, T_ctrl=None, opmode=None, ctrlstate=None, execstate=None,
                conectado=False, error=None, ocupado=False, stamp=0.0)


class _Base:
    nombre = "base"

    def __init__(self):
        self._st = _vacio()
        self._lock = threading.Lock()
        self._fin = threading.Event()

    def estado(self) -> dict:
        with self._lock:
            return dict(self._st)

    def _set(self, **kw):
        with self._lock:
            self._st.update(kw)

    def listo(self):
        s = self.estado()
        if not s["conectado"] or s["q_deg"] is None:
            return False, "sin lectura del robot"
        if s["ocupado"]:
            return False, "hay un movimiento en curso"
        if s["ctrlstate"] not in (None, "motoron"):
            return False, f"motores: {s['ctrlstate']}"
        if s["execstate"] not in (None, "running"):
            return False, f"RAPID: {s['execstate']} (iniciar ROS_BRIDGE con PP en rb_main)"
        return True, f"{s['opmode'] or ''} {s['ctrlstate'] or ''} {s['execstate'] or ''}".strip()

    def enviar(self, tramos, al_avanzar, al_terminar):
        ok, motivo = self.listo()
        if not ok:
            al_terminar(False, motivo)
            return
        self._set(ocupado=True)

        def trabajo():
            try:
                for i, q in enumerate(tramos, 1):
                    al_avanzar(i, len(tramos), q)
                    alcanzado = self._mover(q)
                    al_avanzar(i, len(tramos), q, alcanzado)
                al_terminar(True, "llegada confirmada")
            except Exception as e:  # noqa: BLE001  (se informa al panel y al registro)
                al_terminar(False, str(e))
            finally:
                self._set(ocupado=False)

        threading.Thread(target=trabajo, daemon=True).start()

    def cerrar(self):
        self._fin.set()


# ------------------------------------------------------------------------- simulada
class Simulada(_Base):
    nombre = "simulado"

    def __init__(self, velocidad_deg_s=30.0, q_inicial=(0, 0, 0, 0, 30, 0)):
        super().__init__()
        self.v = velocidad_deg_s
        self._q = np.array(q_inicial, float)

    def iniciar(self):
        self._publicar()
        threading.Thread(target=self._latido, daemon=True).start()

    def _latido(self):   # lectura periodica, como haria RWS
        while not self._fin.wait(0.1):
            self._publicar()

    def _publicar(self):
        T = M.fk(np.deg2rad(self._q))
        self._set(q_deg=self._q.tolist(), T_ctrl=T, opmode="SIM", ctrlstate="motoron",
                  execstate="running", conectado=True, stamp=time.time())

    def _mover(self, q):
        q = np.asarray(q, float)
        q0 = self._q.copy()
        dur = max(float(np.max(np.abs(q - q0))) / self.v, 0.05)
        t0 = time.time()
        while not self._fin.is_set():
            a = min((time.time() - t0) / dur, 1.0)
            self._q = q0 + a * (q - q0)
            self._publicar()
            if a >= 1.0:
                break
            time.sleep(0.03)
        return self._q.tolist()


# ------------------------------------------------------------------------- directa (sin ROS)
class Directa(_Base):
    nombre = "directo"

    def __init__(self, rws_url="http://192.168.125.1", host="192.168.125.1", port=1025,
                 velocidad_mm_s=25.0, periodo=0.2):
        super().__init__()
        from .abb_rws import RWSClient
        from .rapid_client import RapidBridgeClient
        self.rws = RWSClient(rws_url, tool="tool0", wobj="wobj0")
        self.cli = RapidBridgeClient(host, port)
        self.velocidad = velocidad_mm_s
        self.periodo = periodo

    def iniciar(self):
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        k = 0
        while not self._fin.is_set():
            try:
                q = self.rws.joints_deg()
                p, quat = self.rws.tcp()
                T = np.eye(4)
                T[:3, :3] = M.quat_wxyz_to_R(quat)
                T[:3, 3] = np.array(p) / 1000.0
                upd = dict(q_deg=q, T_ctrl=T, conectado=True, error=None, stamp=time.time())
                if k % 5 == 0:
                    upd.update(opmode=self.rws.opmode(), ctrlstate=self.rws.ctrlstate(),
                               execstate=self.rws.execstate())
                self._set(**upd)
            except Exception as e:  # noqa: BLE001
                self._set(conectado=False, error=f"RWS: {e}")
            k += 1
            self._fin.wait(self.periodo)

    def _mover(self, q):
        if self.cli.sock is None:
            self.cli.connect()
            self.cli.set_speed(self.velocidad)
        try:
            return self.cli.move_joints_deg(q)
        except OSError:
            self.cli.close()
            raise

    def cerrar(self):
        super().cerrar()
        self.cli.close()


# ------------------------------------------------------------------------- ROS 2
class Ros2(_Base):
    nombre = "ROS 2"
    JOINTS = [f"joint_{i}" for i in range(1, 7)]

    def __init__(self, timeout_mov=120.0):
        super().__init__()
        self.timeout_mov = timeout_mov
        self._res = None
        self._res_evt = threading.Event()

    def iniciar(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = rclpy.create_node("panel_irb120")
        self._JointState = JointState
        self.pub = self.node.create_publisher(JointState, "/irb120/joint_goal", 10)
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(PoseStamped, "/irb120/tcp_pose_controller", self._on_pose, 10)
        self.node.create_subscription(String, "/irb120/controller_status", self._on_status, 10)
        self.node.create_subscription(String, "/irb120/command_result", self._on_result, 10)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self._hilo_spin = threading.Thread(target=self._spin, daemon=True)
        self._hilo_spin.start()
        threading.Thread(target=self._vigilar, daemon=True).start()

    def _spin(self):
        while not self._fin.is_set() and self._rclpy.ok():
            self.ex.spin_once(timeout_sec=0.1)

    def _vigilar(self):
        while not self._fin.is_set():
            if time.time() - self.estado()["stamp"] > 2.0:
                self._set(conectado=False, error="sin /joint_states (lanzar bringup mode:=real)")
            self._fin.wait(0.5)

    def _on_js(self, msg):
        try:
            q = [float(np.rad2deg(msg.position[list(msg.name).index(j)])) for j in self.JOINTS]
        except ValueError:
            return
        self._set(q_deg=q, conectado=True, error=None, stamp=time.time())

    def _on_pose(self, msg):
        o, p = msg.pose.orientation, msg.pose.position
        T = np.eye(4)
        T[:3, :3] = M.quat_wxyz_to_R([o.w, o.x, o.y, o.z])
        T[:3, 3] = [p.x, p.y, p.z]
        self._set(T_ctrl=T)

    def _on_status(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self._set(opmode=d.get("opmode"), ctrlstate=d.get("ctrlstate"), execstate=d.get("execstate"))

    def _on_result(self, msg):
        self._res = msg.data
        self._res_evt.set()

    def listo(self):
        ok, motivo = super().listo()
        if ok and self.pub.get_subscription_count() == 0:
            return False, "no hay socket_command_node (bringup con command:=true)"
        return ok, motivo

    def _mover(self, q):
        self._res_evt.clear()
        m = self._JointState()
        m.name = self.JOINTS
        m.position = [float(np.deg2rad(v)) for v in q]
        self.pub.publish(m)
        if not self._res_evt.wait(self.timeout_mov):
            raise TimeoutError("sin respuesta de socket_command_node")
        d = json.loads(self._res)
        if not d.get("ok"):
            raise RuntimeError(d.get("error") or "rechazado")
        return d.get("alcanzado_deg")

    def cerrar(self):
        super().cerrar()
        self._hilo_spin.join(timeout=2.0)       # no destruir el nodo mientras el executor gira
        try:
            self.ex.shutdown()
            self.node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


def crear(modo: str, **kw) -> _Base:
    if modo == "simulado":
        return Simulada()
    if modo == "directo":
        return Directa(**kw)
    if modo == "ros":
        return Ros2()
    raise ValueError(f"modo desconocido: {modo}")
