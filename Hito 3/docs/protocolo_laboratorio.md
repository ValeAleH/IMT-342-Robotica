# Protocolo de sesión de laboratorio — Gate 1, Gate 2 (controlador) y Gate 3

Imprimir y llevar. Cada prueba genera una entrada de bitácora (plantilla al final de `bitacora.md`).
Responsable: Valeria Alejandra Hoyos Tovar · Supervisor: ____________________ · Fecha: __________

## 0. Antes de ir (en casa, sin robot)
- [ ] `python kinematics/validate.py` → todos `pass` (evidencia: results/validation_cases.csv)
- [ ] Prueba completa con `tools/mock_irc5.py` (Gate 1, 2 y 3 simulados) y, si hay WSL, con ROS 2
- [ ] `ROS_BRIDGE.mod` cargado y **compilado sin errores en RobotStudio**; probado con `gate3_cli.py`
      contra el controlador virtual (`rb_SERVER_IP := "127.0.0.1"`)
- [ ] Repositorio con commit etiquetado; anotar el hash en la bitácora
- [ ] Llevar: USB (backup), cable de red, laptop cargada, este protocolo impreso

## 1. Checklist GO / NO-GO

| # | Condición | Gate 1 (lectura) | Gate 3 (movimiento) | Estado |
|---|---|---|---|---|
| 1 | Supervisor autorizado presente, nombrado y de acuerdo con el plan | requerido | requerido | sí / no |
| 2 | E-stop del FlexPendant y del gabinete probados y rearmados | recomendado | requerido | sí / no |
| 3 | Modo **Manual velocidad reducida** (MANR); Auto prohibido | — | requerido | sí / no |
| 4 | Zona despejada: nadie dentro de 0,8 m del robot salvo el operador | — | requerido | sí / no |
| 5 | Operador con dispositivo de habilitación y acceso inmediato al E-stop | — | requerido | sí / no |
| 6 | Backup del sistema realizado **antes** de cargar cualquier módulo | — | requerido | sí / no |
| 7 | `ROS_BRIDGE.mod` probado en RobotStudio (evidencia) | — | requerido | sí / no |
| 8 | `rb_Z_MIN` revisado contra la altura real de la mesa/tablero | — | requerido | sí / no |
| 9 | Override de velocidad del FlexPendant ≤ 25 % para la primera prueba | — | requerido | sí / no |
| 10 | Gate 1 completado y estado leído coincide con el FlexPendant | — | requerido | sí / no |

**Regla:** cualquier "no" en una fila requerida → **NO-GO** para ese gate; se continúa solo con
lectura/auditoría. Decisión: GO / NO-GO · Firma supervisor: __________ · Hora: ______

## 2. Gate 1 — comunicación y lectura sin movimiento

| ID | Prueba | Cómo | Criterio de aceptación | Evidencia |
|---|---|---|---|---|
| G1-0 | Backup del sistema | FlexPendant: ABB → Backup and Restore → Backup Current System → USB | carpeta de backup creada | carpeta + foto |
| G1-0b | Versión de RobotWare | System Info → Software resources, o `BACKINFO` del backup | versión anotada en `model_manifest.yaml` | foto |
| G1-A | Conectividad IP | `ipconfig` + `ping -n 20 192.168.125.1` | 0 % pérdida | captura |
| G1-B | Lectura RWS | `python tools/gate1_leer_estado.py --url http://192.168.125.1 --seg 30` | ≥ 95 % lecturas válidas | `results/gate1_*.csv/json` |
| G1-C | Coherencia con el FlexPendant | Jogging → ventana de posición (ejes, deg) vs. salida del script | diferencia ≤ 0,01° | foto + CSV |
| G1-D | Estado en ROS 2 | `bringup.launch.py mode:=real` + operador mueve con joystick | RViz sigue al robot; `/joint_states` ≈ 10 Hz | video + `ros2 bag record /joint_states /irb120/fk_error /irb120/tcp_pose_controller` |

Si G1-B falla con *connection refused* en el puerto de servicio: registrar el error exacto (hipótesis H1
refutada para ese puerto). Alternativas, en orden: (a) puerto WAN con IP asignada por el supervisor;
(b) lectura por socket (`GETJ`) ejecutando `ROS_BRIDGE` sin enviar `MOVJ` (requiere autorización de
ejecución de programa).

## 3. Gate 2 — modelo propio vs. controlador (sin movimiento comandado desde el PC)
El operador lleva el robot con el joystick a cada pose y lo **detiene**; luego Enter en
`python tools/gate2_comparar_controlador.py --url http://192.168.125.1`.

| Pose | Descripción (aprox., deg) | Criterio |
|---|---|---|
| P1 | calibración / home [0, 0, 0, 0, 30, 0] | e_p ≤ 1 mm, e_R ≤ 0,2° |
| P2 | regular [30, 20, −15, 45, 60, −30] | ídem |
| P3 | regular [−20, 10, 20, −30, 45, 60] | ídem |
| P4 | cerca de singularidad de muñeca, q5 ≈ 5° | ídem (FK no se degrada en singularidad) |
| P5 | sobre el tablero, herramienta vertical | ídem |

## 4. Gate 3 — comando y resultado observado (SOLO con GO)
| ID | Prueba | Cómo | Criterio | Evidencia |
|---|---|---|---|---|
| G3-A | Servidor RAPID | Cargar `ROS_BRIDGE.mod` en T_ROB1 (no borrar Module1); `rb_SERVER_IP := "192.168.125.1"`; PP to Routine `rb_main`; MANR; Play con habilitación | FlexPendant muestra "esperando cliente" | foto |
| G3-B | Ping y lectura | `gate3_cli.py`: `getj`, `getp` | PONG, valores = FlexPendant | log |
| G3-C | Primer movimiento | `spd 25`; `movj` = actual con J1 + 5° | DONE; error final ≤ 0,05°; movimiento observado = esperado | log + video |
| G3-D | Rechazos | `movj` con J1 = 120 (límite) y con salto de 60° | ERR, **sin movimiento** | log + video |
| G3-E | Interrupción | soltar la habilitación durante un `movj` | robot se detiene; se registra el comportamiento del nodo | video + log |
| G3-F | Desde ROS 2 | `bringup.launch.py mode:=real command:=true`; `send_goal --joints …` (paso pequeño) y `send_goal --xyz x y 0.25` | `ok: true`; `/irb120/fk_error` ≤ 1 mm al detenerse | bag + video |

Al terminar: detener el programa, devolver el PP a `main` si el laboratorio lo pide, registrar el estado final.

## 5. Decisión de contingencia (cierre W13)
- **Ruta A física**: Gate 1 y 3 completos → W14–W15 tarea repetible sobre posiciones del tablero.
- **Ruta B interfaz sin movimiento**: Gate 1 completo, Gate 3 NO-GO → se demuestra el comando en
  RobotStudio/mock y la lectura real en ROS 2.
- **Ruta C bloqueo externo**: sin acceso/red → evidencia documental + simulación, bloqueo registrado.
