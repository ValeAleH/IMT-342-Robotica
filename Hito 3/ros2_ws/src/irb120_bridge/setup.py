import os
from glob import glob

from setuptools import setup

package_name = "irb120_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "meshes", "visual"), glob("meshes/visual/*.stl")),
        (os.path.join("share", package_name, "meshes", "collision"), glob("meshes/collision/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Valeria Alejandra Hoyos Tovar",
    description="Puente ROS 2 - ABB IRC5 (IRB 120)",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "rws_state_node = irb120_bridge.rws_state_node:main",
            "socket_command_node = irb120_bridge.socket_command_node:main",
            "fk_monitor_node = irb120_bridge.fk_monitor_node:main",
            "send_goal = irb120_bridge.send_goal:main",
            "panel = irb120_bridge.panel_gui:main",
            "sliders = irb120_bridge.sliders:main",
        ],
    },
)
