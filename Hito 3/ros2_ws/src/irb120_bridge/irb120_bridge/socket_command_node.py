"""
socket_command_node - Gate 3 en ROS 2: comando articular al IRC5 via rapid/ROS_BRIDGE.mod.

Suscribe:  /irb120/joint_goal       sensor_msgs/JointState (position en rad, orden joint_1..joint_6;
                                     si trae 'name', se reordena por nombre)
Publica:   /irb120/command_result   std_msgs/String (JSON: ok, objetivo, alcanzado, error, duracion)
Parametros: robot_host, robot_port, speed_mm_s

Un solo comando a la vez: si llega otro mientras el robot se mueve, se rechaza (no se encola).
"""
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .rapid_client import RapidBridgeClient, RapidBridgeError

JOINTS = [f"joint_{i}" for i in range(1, 7)]


class SocketCommandNode(Node):
    def __init__(self):
        super().__init__("socket_command_node")
        self.declare_parameter("robot_host", "192.168.125.1")
        self.declare_parameter("robot_port", 1025)
        self.declare_parameter("speed_mm_s", 50.0)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.client = RapidBridgeClient(gp("robot_host"), int(gp("robot_port")))
        self.speed = float(gp("speed_mm_s"))
        self.busy = threading.Lock()
        self.pub = self.create_publisher(String, "/irb120/command_result", 10)
        self.create_subscription(JointState, "/irb120/joint_goal", self.on_goal, 10)
        self.get_logger().info(f"esperando objetivos en /irb120/joint_goal -> {gp('robot_host')}:{gp('robot_port')}")

    def _ensure_connected(self):
        if self.client.sock is None:
            self.client.connect()
            self.get_logger().info("conectado al servidor RAPID: " + self.client.set_speed(self.speed))

    def on_goal(self, msg: JointState):
        if msg.name:
            try:
                q = [msg.position[list(msg.name).index(j)] for j in JOINTS]
            except ValueError:
                return self._report(False, None, None, "faltan articulaciones en 'name'", 0.0)
        else:
            q = list(msg.position)
        if len(q) != 6:
            return self._report(False, None, None, "se requieren 6 posiciones", 0.0)
        if not self.busy.acquire(blocking=False):
            return self._report(False, q, None, "ocupado: comando anterior en ejecucion", 0.0)
        threading.Thread(target=self._execute, args=([math.degrees(v) for v in q],), daemon=True).start()

    def _execute(self, target_deg):
        t0 = time.time()
        try:
            self._ensure_connected()
            self.get_logger().info(f"MOVJ {[round(v, 2) for v in target_deg]} deg")
            reached = self.client.move_joints_deg(target_deg)
            self._report(True, target_deg, reached, None, time.time() - t0)
        except (RapidBridgeError, OSError) as e:
            if isinstance(e, OSError):
                self.client.close()
            self._report(False, target_deg, None, str(e), time.time() - t0)
        finally:
            self.busy.release()

    def _report(self, ok, target, reached, err, dur):
        d = dict(ok=ok, objetivo_deg=target, alcanzado_deg=reached, error=err, duracion_s=round(dur, 3))
        self.pub.publish(String(data=json.dumps(d)))
        if ok:
            self.get_logger().info(json.dumps(d))
        else:
            self.get_logger().error(json.dumps(d))

    def destroy_node(self):
        self.client.close()
        super().destroy_node()


def main():
    rclpy.init()
    n = SocketCommandNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
