"""
rws_state_node - Gate 1 en ROS 2: estado del IRC5 por Robot Web Services (solo lectura).

Publica:
  /joint_states                  sensor_msgs/JointState   (joint_1..joint_6, rad)
  /irb120/tcp_pose_controller    geometry_msgs/PoseStamped (tool0 segun el controlador, frame base_link, m)
  /irb120/controller_status      std_msgs/String (JSON: opmode, ctrlstate, execstate, errores) ~1 Hz
Parametros: rws_url, user, password, rate_hz, publish_tcp
"""
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .abb_rws import RWSClient

JOINTS = [f"joint_{i}" for i in range(1, 7)]


class RwsStateNode(Node):
    def __init__(self):
        super().__init__("rws_state_node")
        self.declare_parameter("rws_url", "http://192.168.125.1")
        self.declare_parameter("user", "Default User")
        self.declare_parameter("password", "robotics")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("publish_tcp", True)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cli = RWSClient(gp("rws_url"), gp("user"), gp("password"))
        self.period = 1.0 / float(gp("rate_hz"))
        self.publish_tcp = bool(gp("publish_tcp"))

        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_tcp = self.create_publisher(PoseStamped, "/irb120/tcp_pose_controller", 10)
        self.pub_status = self.create_publisher(String, "/irb120/controller_status", 10)
        self.n_ok = self.n_err = 0
        self._stop = False
        threading.Thread(target=self._loop, daemon=True).start()
        self.get_logger().info(f"leyendo RWS en {gp('rws_url')} a {gp('rate_hz')} Hz (solo GET)")

    def _loop(self):
        k = 0
        per_status = max(int(round(1.0 / self.period)), 1)
        while rclpy.ok() and not self._stop:
            t0 = time.time()
            with_status = (k % per_status == 0)
            try:
                q = self.cli.joints_deg()
                now = self.get_clock().now().to_msg()
                js = JointState()
                js.header.stamp = now
                js.name = JOINTS
                js.position = [math.radians(v) for v in q]
                self.pub_js.publish(js)
                if self.publish_tcp:
                    p, quat = self.cli.tcp()
                    ps = PoseStamped()
                    ps.header.stamp = now
                    ps.header.frame_id = "base_link"
                    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = [v / 1000.0 for v in p]
                    (ps.pose.orientation.w, ps.pose.orientation.x,
                     ps.pose.orientation.y, ps.pose.orientation.z) = quat
                    self.pub_tcp.publish(ps)
                self.n_ok += 1
            except Exception as e:  # noqa: BLE001
                self.n_err += 1
                self.get_logger().warn(f"lectura RWS fallida: {e}", throttle_duration_sec=2.0)
            if with_status:
                st = {"ok": self.n_ok, "errores": self.n_err}
                for name in ("opmode", "ctrlstate", "execstate"):
                    try:
                        st[name] = getattr(self.cli, name)()
                    except Exception as e:  # noqa: BLE001
                        st[name] = f"ERROR {e}"
                self.pub_status.publish(String(data=json.dumps(st)))
            k += 1
            time.sleep(max(0.0, self.period - (time.time() - t0)))

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main():
    rclpy.init()
    n = RwsStateNode()
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
