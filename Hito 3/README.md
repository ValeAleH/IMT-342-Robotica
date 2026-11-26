# Hito 3 (ECI-3/4) — Integración ROS 2 ↔ ABB IRB 120 / IRC5

IMT-342 · Valeria Alejandra Hoyos Tovar

**Novedad: panel gráfico para Windows** (`python tools\panel.py`): calcula en la PC si la posición
es posible (IK/FK propias, límites, altura en todo el trayecto, modelo vs. controlador) y recién
entonces habilita MOVER. Guía: [`docs/panel.md`](docs/panel.md).

Ruta implementada (**Ruta B**): **estado** por Robot Web Services (HTTP, solo lectura) y **comando
articular punto a punto** por socket TCP hacia un módulo RAPID propio (`rapid/ROS_BRIDGE.mod`). Hay
nodos ROS 2 para cada parte y un monitor que compara en línea el **modelo cinemático propio** con
la pose que reporta el controlador.

```
hito3_irb120/
├── model_manifest.yaml          Gate 0: versiones, opciones, marcos y convenciones (null = no verificado)
├── kinematics/                  Gate 2: modelo propio y validación cruzada
│   ├── irb120_model.py          FK (DH), IK analítica (8 ramas), Jacobiano geométrico, SVD
│   ├── reference_urdf.py        referencia: URDF ROS-Industrial vía Pinocchio o parser numpy propio
│   └── validate.py              protocolo FK/IK/J → results/validation_cases.csv + figuras
├── description/                 URDF oficial (ROS-I abb_irb120_support) + TCP gripper1_tcp, mallas
├── rapid/ROS_BRIDGE.mod         servidor TCP en el IRC5 (límites reducidos, salto máx., z mínima)
├── tools/                       scripts SIN ROS (corren en Windows)
│   ├── gate1_leer_estado.py     Gate 1: lectura RWS, guarda CSV/JSON de evidencia
│   ├── gate2_comparar_controlador.py   Gate 2 vs IRC5 (robot detenido en poses de jog)
│   ├── gate3_cli.py             Gate 3: consola de comando con confirmación y log
│   ├── panel.py                 PANEL GRÁFICO: verificar antes de mover (simulado / directo / ROS 2)
│   └── mock_irc5.py             controlador SIMULADO (RWS + socket) para probar sin robot
├── ros2_ws/src/irb120_bridge/   paquete ROS 2 (Humble/Jazzy)
│   ├── rws_state_node           /joint_states, /irb120/tcp_pose_controller, /irb120/controller_status
│   ├── socket_command_node      /irb120/joint_goal → MOVJ → /irb120/command_result
│   ├── fk_monitor_node          /irb120/fk_error = [e_p, e_R] modelo propio vs controlador
│   ├── send_goal                CLI: --joints (deg) o --xyz (m, usa la IK propia)
│   ├── panel                    panel gráfico (analisis.py + conexiones.py + panel_gui.py)
│   └── launch/bringup.launch.py mode:=gui | real, command:=true|false
├── docs/                        guía Windows/WSL2, protocolo de laboratorio, bitácora
└── results/                     evidencias generadas (CSV, JSON, PNG, logs)
```

## Inicio rápido en Windows (ROS 2 Humble nativo, sin Linux)

Usar una ruta corta y sin espacios ni tildes, p. ej. `C:\robotica\hito3_irb120`.

```bat
call C:\dev\ros2_humble\local_setup.bat
cd C:\robotica\hito3_irb120
python -m pip install numpy requests matplotlib
python kinematics\validate.py                  # Gate 2 offline (modelo propio vs URDF)
python tools\panel.py                          # panel gráfico con robot simulado
python tools\mock_irc5.py                      # terminal 1: controlador simulado
python tools\gate1_leer_estado.py --url http://127.0.0.1:8080 --seg 10     # terminal 2
python tools\gate3_cli.py --host 127.0.0.1 --port 1025                    # terminal 2
```

## ROS 2 (WSL2 Ubuntu 22.04 + Humble), ver `docs/guia_windows_wsl2.md`

```bash
cd ros2_ws && colcon build && source install/setup.bash
ros2 launch irb120_bridge bringup.launch.py mode:=gui                                   # solo modelo
python3 ../tools/mock_irc5.py &                                                         # simulador
ros2 launch irb120_bridge bringup.launch.py mode:=real command:=true \
     rws_url:=http://127.0.0.1:8080 robot_host:=127.0.0.1
ros2 run irb120_bridge send_goal --joints 10 10 -10 0 40 0
ros2 run irb120_bridge send_goal --xyz 0.35 0.10 0.25
```

Robot real: `rws_url:=http://192.168.125.1 robot_host:=192.168.125.1`; `command:=true` **solo con
decisión GO** (ver `docs/protocolo_laboratorio.md`).

## Seguridad

Software ≠ seguridad. Las capas de este repositorio (límites reducidos, salto máximo de 45°,
z mínima del TCP, velocidad 50 mm/s con VelSet 50 %, confirmación manual) son **adicionales** al
E-stop, al modo manual con dispositivo de habilitación y a la presencia del supervisor. No se mueve
el robot físico sin el checklist GO/NO-GO firmado.

## Referencias

- Panel de verificación antes de mover: idea tomada como referencia del trabajo de Marcelo767n,
  [robotica-IMT342](https://github.com/Marcelo767n/robotica-IMT342), rama `proyectoHito3y4`
  (commit 194c077). Implementación independiente; ver `docs/panel.md`.

## Procedencia

URDF y mallas: ROS-Industrial `abb` (licencia en `description/`), commit en `description/SOURCE_COMMIT.txt`.
