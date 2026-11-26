"""
Lanzador del panel grafico sin necesidad de compilar el paquete ROS 2 (Windows o Linux).

  python tools\\panel.py                                     robot simulado (practicar)
  python tools\\panel.py --modo directo --robot 127.0.0.1 --rws http://127.0.0.1:8080   (mock_irc5)
  python tools\\panel.py --modo directo --robot 192.168.125.1                         (robot real)
  python tools\\panel.py --modo ros       (con el bringup de ROS 2 corriendo)
"""
import sys

import _paths  # noqa: F401
from irb120_bridge.panel_gui import main

if __name__ == "__main__":
    sys.exit(main())
