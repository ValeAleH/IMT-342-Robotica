"""Agrega al sys.path los modulos del repositorio (modelo propio, clientes y panel sin ROS)."""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PKG_PARENT = os.path.join(ROOT, "ros2_ws", "src", "irb120_bridge")
for p in (os.path.join(ROOT, "kinematics"), os.path.join(PKG_PARENT, "irb120_bridge"), PKG_PARENT):
    if p not in sys.path:
        sys.path.insert(0, p)
RESULTS = os.path.join(ROOT, "results")
os.makedirs(RESULTS, exist_ok=True)
