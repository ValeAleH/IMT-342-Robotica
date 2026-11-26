"""
fk_monitor_node - Gate 2 en linea: compara el modelo PROPIO con la pose que reporta el IRC5.

Suscribe:  /joint_states, /irb120/tcp_pose_controller (tool0 del controlador)
Publica:   /irb120/tcp_pose_own   geometry_msgs/PoseStamped (FK propia, tool0)
           /irb120/fk_error       std_msgs/Float64MultiArray [e_p (m), e_R (rad)]
Valido con el robot DETENIDO (las lecturas de q y de pose no son simultaneas).
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import numpy as np

from . import irb120_model as M

JOINTS = [f"joint_{i}" for i in range(1, 7)]


class FkMonitor(Node):
    def __init__(self):
        super().__init__("fk_monitor_node")
        self.q = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(PoseStamped, "/irb120/tcp_pose_controller", self.on_pose, 10)
        self.pub_own = self.create_publisher(PoseStamped, "/irb120/tcp_pose_own", 10)
        self.pub_err = self.create_publisher(Float64MultiArray, "/irb120/fk_error", 10)

    def on_js(self, msg):
        try:
            self.q = np.array([msg.position[list(msg.name).index(j)] for j in JOINTS])
        except ValueError:
            pass
        if self.q is None:
            return
        T = M.fk(self.q)
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = "base_link"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = T[:3, 3].tolist()
        w, x, y, z = M.R_to_quat_wxyz(T[:3, :3]).tolist()
        ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = w, x, y, z
        self.pub_own.publish(ps)

    def on_pose(self, msg):
        if self.q is None:
            return
        o, p = msg.pose.orientation, msg.pose.position
        T_ctrl = np.eye(4)
        T_ctrl[:3, :3] = M.quat_wxyz_to_R([o.w, o.x, o.y, o.z])
        T_ctrl[:3, 3] = [p.x, p.y, p.z]
        T_own = M.fk(self.q)
        ep = float(np.linalg.norm(T_own[:3, 3] - T_ctrl[:3, 3]))
        eR = M.rot_angle_error_robust(T_ctrl[:3, :3], T_own[:3, :3])
        self.pub_err.publish(Float64MultiArray(data=[ep, eR]))
        self.get_logger().info(f"e_p = {ep * 1000:.3f} mm   e_R = {np.degrees(eR):.4f} deg",
                               throttle_duration_sec=2.0)


def main():
    rclpy.init()
    n = FkMonitor()
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
