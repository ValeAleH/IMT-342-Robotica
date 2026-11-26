"""
send_goal - envia UN objetivo al socket_command_node y espera el resultado.

  ros2 run irb120_bridge send_goal --joints 10 10 -10 0 40 0          # grados
  ros2 run irb120_bridge send_goal --xyz 0.35 0.10 0.25 [--yaw 0]     # m, TCP Gripper1 vertical hacia abajo

Con --xyz se usa la IK ANALITICA PROPIA (8 ramas), se descartan las ramas fuera de los limites
reducidos del puente y se elige la mas cercana a la configuracion actual (/joint_states).
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from . import irb120_model as M
from .rapid_client import LIM_HI_DEG, LIM_LO_DEG

JOINTS = [f"joint_{i}" for i in range(1, 7)]


def choose_ik(xyz, yaw, q_now):
    T = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    T[:3, :3] = np.array([[-c, s, 0], [s, c, 0], [0, 0, -1.0]])  # z_TCP hacia abajo, giro yaw
    T[:3, 3] = xyz
    lo, hi = np.deg2rad(LIM_LO_DEG), np.deg2rad(LIM_HI_DEG)
    sols = [s for s in M.ik(T, M.T_GRIPPER1, only_within_limits=False, q6_alternatives=True)
            if M.within_limits(s["q"], lo, hi)]
    if not sols:
        return None, None
    best = min(sols, key=lambda s: np.linalg.norm(s["q"] - q_now))
    return best["q"], best["branch"]


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--joints", nargs=6, type=float, metavar="J")
    g.add_argument("--xyz", nargs=3, type=float, metavar="M")
    ap.add_argument("--yaw", type=float, default=0.0, help="rad, giro del TCP sobre la vertical")
    ap.add_argument("--timeout", type=float, default=120.0)
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node("send_goal")
    state = {"q": None, "res": None}
    node.create_subscription(JointState, "/joint_states", lambda m: state.__setitem__(
        "q", np.array([m.position[list(m.name).index(j)] for j in JOINTS])), 10)
    node.create_subscription(String, "/irb120/command_result", lambda m: state.__setitem__("res", m.data), 10)
    pub = node.create_publisher(JointState, "/irb120/joint_goal", 10)

    t0 = time.time()
    while state["q"] is None and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if a.joints:
        q = np.deg2rad(a.joints)
        info = "objetivo articular"
    else:
        if state["q"] is None:
            node.get_logger().error("sin /joint_states: no se puede elegir rama de IK")
            return 1
        q, branch = choose_ik(np.array(a.xyz), a.yaw, state["q"])
        if q is None:
            node.get_logger().error("ninguna rama de IK dentro de los limites reducidos")
            return 1
        info = f"IK propia, rama {branch}"
    node.get_logger().info(f"{info}: q = {np.round(np.rad2deg(q), 2).tolist()} deg")

    while pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    msg = JointState()
    msg.name = JOINTS
    msg.position = [float(v) for v in q]
    pub.publish(msg)
    t1 = time.time()
    while state["res"] is None and time.time() - t1 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(json.dumps(json.loads(state["res"]), indent=2, ensure_ascii=False) if state["res"] else "sin respuesta")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
