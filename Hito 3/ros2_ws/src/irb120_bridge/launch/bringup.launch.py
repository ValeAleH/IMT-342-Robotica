"""
bringup.launch.py

  mode:=gui    solo modelo + RViz + barras propias (sin robot)               [simulacion]
  mode:=real   RWS (estado) + monitor FK + RViz; comando solo si command:=true  [Gate 1 / Gate 3]

Ejemplos:
  ros2 launch irb120_bridge bringup.launch.py mode:=gui
  ros2 launch irb120_bridge bringup.launch.py mode:=real rws_url:=http://127.0.0.1:8080 robot_host:=127.0.0.1   # mock_irc5
  ros2 launch irb120_bridge bringup.launch.py mode:=real                                                       # IRC5 Gate 1 (lectura)
  ros2 launch irb120_bridge bringup.launch.py mode:=real command:=true                                         # Gate 3 (solo con GO)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration as LC
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("irb120_bridge")
    with open(os.path.join(share, "urdf", "irb120_3_58_gripper1.urdf")) as f:
        urdf = f.read()
    rviz_cfg = os.path.join(share, "rviz", "irb120.rviz")
    real = LaunchConfigurationEquals("mode", "real")
    gui = LaunchConfigurationEquals("mode", "gui")

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="gui", choices=["gui", "real"]),
        DeclareLaunchArgument("rws_url", default_value="http://192.168.125.1"),
        DeclareLaunchArgument("robot_host", default_value="192.168.125.1"),
        DeclareLaunchArgument("robot_port", default_value="1025"),
        DeclareLaunchArgument("command", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        # barras propias (joint_state_publisher_gui no viene en ROS 2 Humble para Windows)
        Node(package="irb120_bridge", executable="sliders", condition=gui),
        Node(package="irb120_bridge", executable="rws_state_node", condition=real,
             parameters=[{"rws_url": LC("rws_url"), "rate_hz": 10.0}]),
        Node(package="irb120_bridge", executable="fk_monitor_node", condition=real),
        Node(package="irb120_bridge", executable="socket_command_node",
             condition=IfCondition(PythonExpression(["'", LC("mode"), "' == 'real' and '", LC("command"), "' == 'true'"])),
             parameters=[{"robot_host": LC("robot_host"),
                          "robot_port": PythonExpression([LC("robot_port")]),
                          "speed_mm_s": 50.0}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], condition=IfCondition(LC("rviz"))),
    ])
