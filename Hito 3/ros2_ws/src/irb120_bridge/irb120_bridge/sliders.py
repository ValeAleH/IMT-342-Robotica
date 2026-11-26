"""
sliders - barras para mover el modelo en RViz SIN robot (reemplaza a joint_state_publisher_gui,
que no viene en la distribucion binaria de ROS 2 Humble para Windows).

Publica /joint_states (joint_1..joint_6, rad) a 20 Hz y muestra la pose del TCP con la FK propia.
    ros2 run irb120_bridge sliders          (lo lanza bringup.launch.py mode:=gui)
"""
import math
import threading
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

from . import analisis as A
from . import irb120_model as M

JOINTS = [f"joint_{i}" for i in range(1, 7)]


def main():
    rclpy.init()
    node = rclpy.create_node("irb120_sliders")
    pub = node.create_publisher(JointState, "/joint_states", 10)
    q = [0.0, 0.0, 0.0, 0.0, 30.0, 0.0]
    fin = threading.Event()

    def publicar():
        while not fin.is_set() and rclpy.ok():
            m = JointState()
            m.header.stamp = node.get_clock().now().to_msg()
            m.name = JOINTS
            m.position = [math.radians(v) for v in q]
            pub.publish(m)
            fin.wait(0.05)

    root = tk.Tk()
    root.title("IRB 120 · barras (sin robot)")
    vars_ = []
    lo, hi = np.rad2deg(M.Q_MIN), np.rad2deg(M.Q_MAX)
    lbl = ttk.Label(root, text="", font=("Consolas", 10))

    def actualizar(*_):
        for i, v in enumerate(vars_):
            q[i] = float(v.get())
        p = A.pose_desde_T(A.fk_tcp(q))
        lbl.configure(text="TCP (FK propia): X {:.1f}  Y {:.1f}  Z {:.1f} mm\nR {:.1f}  P {:.1f}  Y {:.1f} °".format(*p))

    for i in range(6):
        ttk.Label(root, text=f"J{i + 1} [°]").grid(row=i, column=0, padx=6, sticky="w")
        v = tk.DoubleVar(value=q[i])
        s = tk.Scale(root, variable=v, from_=float(lo[i]), to=float(hi[i]), orient="horizontal",
                     length=420, resolution=0.5, command=actualizar)
        s.grid(row=i, column=1, padx=6)
        vars_.append(v)

    def cero():
        for v, val in zip(vars_, [0, 0, 0, 0, 30, 0]):
            v.set(val)
        actualizar()

    ttk.Button(root, text="Pose inicial", command=cero).grid(row=6, column=0, pady=6)
    lbl.grid(row=6, column=1, sticky="w", pady=6)
    actualizar()
    threading.Thread(target=publicar, daemon=True).start()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        fin.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
