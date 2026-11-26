# Bitácora técnica — Hito 3 (ECI-3/4)

Equipo: Valeria Alejandra Hoyos Tovar (responsable única). Ciclo: hipótesis → prueba → resultado
observado → interpretación → siguiente acción.

---
**ID:** BIT-001 · **fecha/hora:** 2026-11-10 · **responsables:** V. Hoyos
**hipótesis:** el robot del laboratorio es un IRB 120 con controlador IRC5 de la generación RobotWare 6.
**objetivo de la prueba:** identificar hardware, red y antecedentes (auditoría Gate 0).
**software/hardware y versiones:** placa del robot, gabinete, FlexPendant.
**modo:** auditoría
**prueba ejecutada:** fotos de la placa (IRB 120-3/0.6, serie 120-101516, fab. 2018-01-10), del gabinete
(30 kg) y de la computadora principal (X2 Service, LAN1–3, WAN, AXC); lectura del programa del ajedrez en
el FlexPendant.
**resultado observable:** PC conectado a X2 Service (controlador 192.168.125.1). Module1 abre un socket
cliente a 192.168.125.203. Robot en Auto, motores apagados.
**evidencia:** fotos 1–4 (placa, FlexPendant, puertos, gabinete).
**estado funcional:** partially_verified
**limitaciones:** versión de RobotWare no leída en esta sesión (confirmada después: RobotWare 6.12, ver BIT-008); no consta que el ajedrez funcionara con esta red.
**próxima acción:** leer las opciones del controlador.

---
**ID:** BIT-002 · **fecha/hora:** 2026-11-1_ (completar) · **responsables:** V. Hoyos
**hipótesis:** el controlador tiene opciones para comunicarse con un PC (PC Interface) y quizá EGM.
**modo:** auditoría
**prueba ejecutada:** System Info → Control module → Options; Program Data → tooldata.
**resultado observable:** 616-1 PC Interface, 623-1 Multitasking, 689-1 EGM, 628-1 Sensor Interface,
UDPUC Driver, entre otras. Módulos EGM_ROS y Module1 en T_ROB1. Gripper1 con tframe.trans [0,0,167.5] mm
(módulo CalibData). Modo Manual, Guard Stop.
**evidencia:** fotos 5–6.
**estado funcional:** functional (opciones verificadas por pantalla)
**hipótesis descartadas:** "sin EGM": descartada. "abb_driver imposible por falta de Multitasking": descartada.
**limitaciones:** rotación y carga de Gripper1 no visibles; contenido de EGM_ROS desconocido.
**próxima acción:** elegir la arquitectura; backup para recuperar módulos.

---
**ID:** BIT-003 · **fecha/hora:** 2026-11-16 · **responsables:** V. Hoyos
**hipótesis:** el modelo DH propio reproduce el URDF de ROS-Industrial (mismos marcos, orden, signos y offsets).
**software/hardware y versiones:** Python 3, numpy; referencia Pinocchio 4.1.0 + URDF commit 45f4769 (ROS-I abb).
**modo:** simulación / validación offline
**condiciones / frames:** base_link, tool0 y gripper1_tcp (+167,5 mm en z de tool0); rad, m.
**prueba ejecutada:** `python kinematics/validate.py`
**resultado observable:** FK: e_p máx. 4,6e−16 m (5000 configuraciones), e_R ≤ 3,7e−8 rad (piso de acos; con
atan2 ≈ 1e−15). IK: 8 ramas, todas reproducen la pose (e_p ≤ 3e−16 m). Jacobiano: e_J ≤ 4e−16. Rango 6 en
la configuración regular y 5 en las tres singularidades. Diferencias finitas: paso óptimo ≈ 1e−5 rad.
**evidencia:** results/validation_cases.csv, validation_details.json, fig_*.png
**estado funcional:** functional (modelo vs. URDF)
**limitaciones:** el URDF también es un modelo nominal; falta la comparación contra el controlador real (Gate 2 en el lab).
**interpretación:** la IK numérica de referencia (CLIK) converge, según la semilla, a ramas fuera de los
límites (IK_1 semilla 4 → front/down/flip con q2 = 87°). Eso justifica enumerar las ramas analíticamente y filtrar por límites.
**próxima acción:** integrar la comunicación.

---
**ID:** BIT-004 · **fecha/hora:** 2026-11-16 · **responsables:** V. Hoyos
**hipótesis:** la ruta "RWS para el estado + socket RAPID para el comando" se integra con ROS 2 sin add-ins del controlador.
**software/hardware y versiones:** ROS 2 (Jazzy en pruebas, objetivo Humble), irb120_bridge 0.1.0, mock_irc5.
**modo:** simulación (controlador simulado, NO es el IRC5)
**prueba ejecutada:** launch mode:=real contra mock; send_goal --joints, --xyz y comandos inválidos.
**resultado observable:** /joint_states a 9,98 Hz; fk_error ≈ 6e−8 m; MOVJ ok; IK propia → TCP en
(0,35; 0,10; 0,25) m confirmado por TF de robot_state_publisher; rechazos por límite, salto y z mínima.
**evidencia:** logs de la sesión, results/gate1_*.json
**estado funcional:** partially_verified (solo simulado)
**fallo encontrado y corregido:** socket_command_node se caía al reportar un rechazo (rclpy no permite cambiar
la severidad del log en la misma llamada) y el resultado no se publicaba. Corregido y re-probado.
**próxima acción:** compilar ROS_BRIDGE.mod en RobotStudio; sesión de laboratorio (Gate 1).

---
**ID:** BIT-005 · **fecha/hora:** 2026-11-19 · **responsables:** V. Hoyos
**hipótesis:** se puede cumplir el requisito docente ("calcular en la PC si la posición es posible y
recién entonces dar la opción de mover") con una interfaz gráfica en Windows sobre el modelo propio.
**software/hardware y versiones:** Python + tkinter + matplotlib; panel en `irb120_bridge` (analisis.py,
conexiones.py, panel_gui.py); pruebas con ROS 2 Jazzy (Linux) y mock_irc5.
**modo:** simulación (robot simulado y controlador simulado, NO es el IRC5)
**prueba ejecutada:** test_analisis.py (8 pruebas) y recorridos completos del panel en los modos simulado,
directo (mock_irc5) y ROS 2 (bringup + mock).
**resultado observable:** 8/8 pruebas OK. Pose alcanzable → MOVER → llegada con e_p 0,000 mm. Pose a 40 mm
rechazada por altura. Salto de 70° partido en 2 tramos. Caso de trayecto con extremos a ~645 mm que baja
a 269 mm en el medio: rechazado con z mínima de 300 mm.
**evidencia:** docs/img/panel_*.png, results/panel_*.jsonl, docs/panel.md
**estado funcional:** partially_verified (solo simulado)
**fallos encontrados y corregidos:** (1) el registro JSON fallaba por un campo `modo` repetido;
(2) la verificación de llegada usaba una lectura RWS anterior al fin del movimiento (falso error de 4,9°);
ahora espera una lectura posterior; (3) cierre abrupto en modo ROS 2 al destruir el nodo mientras el
executor giraba; corregido.
**referencia:** idea de verificar antes de enviar tomada del panel de Marcelo767n (robotica-IMT342,
rama proyectoHito3y4, commit 194c077); implementación propia e independiente.
**próxima acción:** probar el panel en Windows nativo (Humble) con mock_irc5; luego en el laboratorio
en modo directo o ROS 2 con el robot real, siguiendo el checklist GO/NO-GO.

---
## BIT-006 · Sesión con el robot real: Gates 1 a 3
**ID:** BIT-006 · **fecha/hora:** 2026-11-21, 16:36 (registro del panel) · **responsables:** V. Hoyos
(supervisión: docente de la materia)
**hipótesis:** H1 (RWS accesible por el puerto de servicio X2) y H4 (signos y ceros del modelo coinciden
con el controlador).
**modo:** comunicación sin movimiento → movimiento autorizado (GO del docente).
**software/hardware:** IRB 120 + IRC5 Compact; PC Windows + ROS 2 Humble nativo; adaptador USB-Ethernet
al puerto X2 (PC 192.168.125.1); módulo `ROS_BRIDGE.mod` con `rb_tool` (TCP +167,5 mm en z).
**prueba ejecutada:** (1) lectura RWS de ejes y pose; (2) comparación FK propia vs. controlador;
(3) subida del módulo por red (`tools/subir_modulo.py`), carga desde el FlexPendant, PP to Routine →
rb_main, Running con el dispositivo de habilitación; (4) 6 movimientos desde el panel (2 por ejes,
4 por pose con IK) y 1 objetivo a z = 40 mm.
**resultado observable:** tool0 0,000 mm; TCP del gripper 0,115 mm / 0,0054° en
q = [20,73; 19,52; −17,7; −1,31; 0,12; 106,42]°. Error de llegada de 0,002 a 0,056 mm, error máximo por
eje de 0,011°. El objetivo a 40 mm se rechazó sin mover (ninguna rama dentro de límites). DONE vs.
lectura RWS posterior: hasta 0,14°.
**evidencia:** `results/laboratorio_2026-11-21/` (log JSONL del panel, CSV resumen, 4 figuras de
`tools/graficas_laboratorio.py`); capturas de consola entregadas al docente.
**estado funcional:** functional (Gates 1, 2 y 3 en el robot real).
**fallos encontrados y corregidos:** (1) `Gripper1` no existía en el controlador (error en la
línea 137) → `rb_tool` local en el módulo; (2) PING sin respuesta con el programa en Stopped → mantener
el dispositivo de habilitación y el programa en Running; (3) sin USB → subida por el servicio de
archivos de RWS.
**prueba de seguridad (movimiento 5):** durante el movimiento 5 se soltó a propósito el dispositivo de
habilitación. El robot se detuvo y, al volver a habilitar, completó el movimiento: 40,3 s en total
(contra 1,6–5,7 s de los demás), llegada a 0,056 mm. Confirma que la detención física depende del
controlador y no del PC.
**autorización:** GO verbal del docente presente durante toda la sesión. El checklist GO/NO-GO de
`docs/protocolo_laboratorio.md` **no se llenó por escrito** en esta sesión; queda como acción
correctiva para las siguientes (ver BIT-008).
**limitaciones:** la masa del gripper es una estimación (0,5 kg).
**próxima acción:** aplicar la misma IK y verificación a la tarea del tablero (Hito 4); evaluar EGM.

---
## BIT-007 · Secuencia repetitiva (trabajo en serie)
**ID:** BIT-007 · **fecha/hora:** 2026-11-24 · **responsables:** V. Hoyos
**requisito:** el programa debe repetir un movimiento (p. ej. recoger y trasladar) hasta que el
operador lo detenga, como en una línea de producción.
**hipótesis:** la misma verificación previa del panel, aplicada a cada trayecto del ciclo (incluido
el cierre Pn -> P1), basta para repetir el ciclo de forma segura; los puntos guardados como ángulos
garantizan la misma postura en cada ciclo.
**modo:** simulación (robot simulado del panel) y comunicación por socket/RWS contra `mock_irc5`.
**prueba ejecutada:** plantilla recoger y dejar de 6 puntos; 2 y 3 repeticiones; detener al
terminar el ciclo; detención si el robot se movió después de verificar; punto bajo z mínima.
**resultado observable:** 14/14 pruebas automáticas OK. Con mock_irc5: 2 ciclos de 8,6 s, e_p máx.
de 0,064 mm, y el robot termina en P1.
**evidencia:** `secuencia.py`, `test/test_secuencia.py`, `results/vivo_<fecha>/resumen_ciclos.png`.
**estado funcional:** partially_verified (falta la prueba en el robot real).
**limitaciones:** el gripper no está accionado (la acción es informativa); cada punto es un punto
de parada (zona fine), así que el ciclo no es continuo.
**próxima acción:** probar 3 ciclos en el robot real con supervisor y velocidad baja; confirmar la
señal del gripper para agregar `SetDO` al módulo RAPID.

---
## BIT-008 · Cierre de pendientes del Gate 0 y posición HOME
**ID:** BIT-008 · **fecha/hora:** 2026-11-24 · **responsables:** V. Hoyos
**resultado:** (1) RobotWare **6.12**, confirmado en el FlexPendant (System Info). Confirma H6 y la
compatibilidad con RWS 1.0 que usa `abb_rws.py`; `model_manifest.yaml` actualizado. Gate 0: verified.
(2) El movimiento 5 del 21-11 fue la prueba de soltar el dispositivo de habilitación (BIT-006).
(3) Botón **IR A HOME** en el panel: carga la posición inicial (por defecto [0, 0, 0, 0, 30, 0]°)
como objetivo, la analiza con los mismos 7 chequeos y se ejecuta con MOVER. "Fijar HOME aquí" guarda
otra postura en `results/home.json`. La secuencia repetitiva puede preparar el regreso a HOME al
terminar. Probado con el robot simulado.
**aclaración:** "ir a HOME" no es calibrar. El IRB 120 conserva su posición con resolvers y contadores
de vueltas alimentados por batería; la calibración (Calibration > Update revolution counters) solo se
hace en el FlexPendant si el controlador lo pide (por ejemplo, batería agotada) y no se hace desde el PC.
**acción correctiva:** llenar y firmar el checklist GO/NO-GO de `docs/protocolo_laboratorio.md`
antes de cada sesión con movimiento (supervisor, E-stop probado, modo, zona, habilitación, override).
**estado funcional:** Gate 0 verified; HOME partially_verified (falta probarlo en el robot real).

---
## Plantilla (para cada prueba del laboratorio)
```
ID:
fecha/hora:
equipo y responsables:
hipotesis:
objetivo de la prueba:
software/hardware y versiones:            (commit del repo, RobotWare, ROS)
modo: simulacion / auditoria / comunicacion sin movimiento / movimiento autorizado
condiciones / frames / parametros:
prueba ejecutada:
resultado observable:
evidencia (log, commit, foto permitida, archivo):
estado funcional: functional / partially_verified / blocked / not verified
limitaciones / hipotesis descartadas:
proxima accion y responsable:
revision docente/laboratorio requerida:
```
