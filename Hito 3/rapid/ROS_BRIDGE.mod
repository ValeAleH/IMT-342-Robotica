MODULE ROS_BRIDGE
  !**********************************************************************
  ! IMT-342 Hito 3 - Servidor TCP para comandos articulares desde ROS 2.
  !
  ! El robot es SERVIDOR; el nodo ROS 2 (socket_command_node) es cliente.
  ! Protocolo de texto, un comando por mensaje, respuesta terminada en LF:
  !   PING                      -> PONG
  !   GETJ                      -> JNT j1 j2 j3 j4 j5 j6        [deg]
  !   GETP                      -> POS x y z q1 q2 q3 q4        [mm, cuaternion] (rb_tool, wobj0)
  !   SPD v                     -> OK SPD v                     (v_tcp mm/s, max rb_V_MAX)
  !   MOVJ j1 j2 j3 j4 j5 j6    -> DONE j1..j6  |  ERR <motivo>
  !   BYE                       -> BYE (cierra la conexion)
  !
  ! Seguridad (capa de software, NO reemplaza E-stop, modo manual ni supervisor):
  !   - limites articulares REDUCIDOS (rb_LIM_LO / rb_LIM_HI)
  !   - salto maximo por comando rb_MAX_STEP (deg)
  !   - altura minima del TCP (rb_tool) sobre la base rb_Z_MIN (mm)
  !   - velocidad limitada (rb_V_MAX) y VelSet al 50 %
  ! Ejecutar con: PP to Routine -> rb_main. Probar primero en RobotStudio.
  !**********************************************************************

  ! --- configuracion (revisar antes de cada sesion) ---
  ! Robot real por puerto de servicio: "192.168.125.1". Controlador virtual: "127.0.0.1".
  LOCAL CONST string rb_SERVER_IP := "192.168.125.1";
  LOCAL CONST num rb_PORT := 1025;
  LOCAL CONST num rb_LIM_LO{6} := [-90, -40, -70, -150, -115, -300];
  LOCAL CONST num rb_LIM_HI{6} := [ 90,  60,  50,  150,  115,  300];
  LOCAL CONST num rb_MAX_STEP := 45;
  LOCAL CONST num rb_Z_MIN := 100;
  LOCAL CONST num rb_V_MAX := 100;
  ! Herramienta propia del modulo (no depende de Gripper1/CalibData de otros programas).
  ! TCP = Gripper1 del laboratorio: 167.5 mm en z de tool0. Carga ESTIMADA (0.5 kg): confirmar con el docente.
  LOCAL PERS tooldata rb_tool := [TRUE, [[0, 0, 167.5], [1, 0, 0, 0]], [0.5, [0, 0, 60], [1, 0, 0, 0], 0, 0, 0]];

  LOCAL VAR socketdev rb_srv;
  LOCAL VAR socketdev rb_cli;
  LOCAL VAR speeddata rb_speed := [50, 30, 5000, 1000];
  LOCAL VAR string rb_rx := "";
  LOCAL VAR string rb_tx := "";

  PROC rb_main()
    VelSet 50, 250;
    SocketClose rb_cli;
    SocketClose rb_srv;
    SocketCreate rb_srv;
    SocketBind rb_srv, rb_SERVER_IP, rb_PORT;
    SocketListen rb_srv;
    WHILE TRUE DO
      TPWrite "ROS_BRIDGE: esperando cliente en puerto " + NumToStr(rb_PORT, 0);
      SocketAccept rb_srv, rb_cli \Time:=WAIT_MAX;
      TPWrite "ROS_BRIDGE: cliente conectado";
      rb_serve;
      SocketClose rb_cli;
      TPWrite "ROS_BRIDGE: cliente desconectado";
    ENDWHILE
  ENDPROC

  LOCAL PROC rb_serve()
    VAR string cmd;
    VAR num v{6};
    VAR num sp;
    VAR bool fin := FALSE;

    WHILE NOT fin DO
      SocketReceive rb_cli \Str:=rb_rx \Time:=WAIT_MAX;
      rb_rx := rb_strip(rb_rx);
      IF StrLen(rb_rx) >= 4 THEN
        cmd := StrPart(rb_rx, 1, 4);
      ELSE
        cmd := rb_rx;
      ENDIF

      TEST cmd
      CASE "PING":
        rb_tx := "PONG";
      CASE "GETJ":
        rb_tx := "JNT " + rb_joints_str(CJointT());
      CASE "GETP":
        rb_tx := rb_pose_str();
      CASE "SPD ":
        IF StrLen(rb_rx) < 5 THEN
          rb_tx := "ERR PARSE";
        ELSEIF rb_parse(StrPart(rb_rx, 5, StrLen(rb_rx) - 4), 1, v) THEN
          sp := v{1};
          IF sp < 5 sp := 5;
          IF sp > rb_V_MAX sp := rb_V_MAX;
          rb_speed.v_tcp := sp;
          rb_tx := "OK SPD " + NumToStr(sp, 0);
        ELSE
          rb_tx := "ERR PARSE";
        ENDIF
      CASE "MOVJ":
        IF StrLen(rb_rx) < 6 THEN
          rb_tx := "ERR PARSE";
        ELSEIF NOT rb_parse(StrPart(rb_rx, 6, StrLen(rb_rx) - 5), 6, v) THEN
          rb_tx := "ERR PARSE";
        ELSE
          rb_tx := rb_check_and_move(v);
        ENDIF
      CASE "BYE":
        rb_tx := "BYE";
        fin := TRUE;
      DEFAULT:
        rb_tx := "ERR UNKNOWN";
      ENDTEST
      SocketSend rb_cli \Str:=rb_tx + "\0A";
    ENDWHILE

  ERROR
    IF ERRNO = ERR_SOCK_CLOSED THEN
      RETURN;
    ENDIF
  ENDPROC

  ! Valida limites, salto y altura; si todo es valido mueve y devuelve DONE.
  LOCAL FUNC string rb_check_and_move(num v{*})
    VAR jointtarget jt;
    VAR jointtarget jnow;
    VAR robtarget pt;
    VAR num i;
    VAR num cur{6};

    jnow := CJointT();
    cur{1} := jnow.robax.rax_1; cur{2} := jnow.robax.rax_2; cur{3} := jnow.robax.rax_3;
    cur{4} := jnow.robax.rax_4; cur{5} := jnow.robax.rax_5; cur{6} := jnow.robax.rax_6;

    FOR i FROM 1 TO 6 DO
      IF v{i} < rb_LIM_LO{i} OR v{i} > rb_LIM_HI{i} THEN
        RETURN "ERR LIMIT J" + NumToStr(i, 0);
      ENDIF
      IF Abs(v{i} - cur{i}) > rb_MAX_STEP THEN
        RETURN "ERR STEP J" + NumToStr(i, 0);
      ENDIF
    ENDFOR

    jt := jnow;
    jt.robax.rax_1 := v{1}; jt.robax.rax_2 := v{2}; jt.robax.rax_3 := v{3};
    jt.robax.rax_4 := v{4}; jt.robax.rax_5 := v{5}; jt.robax.rax_6 := v{6};

    pt := CalcRobT(jt, rb_tool \WObj:=wobj0);
    IF pt.trans.z < rb_Z_MIN THEN
      RETURN "ERR ZMIN " + NumToStr(pt.trans.z, 1);
    ENDIF

    MoveAbsJ jt, rb_speed, fine, rb_tool \WObj:=wobj0;
    RETURN "DONE " + rb_joints_str(CJointT());
  ENDFUNC

  LOCAL FUNC string rb_joints_str(jointtarget j)
    RETURN NumToStr(j.robax.rax_1, 2) + " " + NumToStr(j.robax.rax_2, 2) + " "
         + NumToStr(j.robax.rax_3, 2) + " " + NumToStr(j.robax.rax_4, 2) + " "
         + NumToStr(j.robax.rax_5, 2) + " " + NumToStr(j.robax.rax_6, 2);
  ENDFUNC

  LOCAL FUNC string rb_pose_str()
    VAR robtarget p;
    p := CRobT(\Tool:=rb_tool \WObj:=wobj0);
    RETURN "POS " + NumToStr(p.trans.x, 1) + " " + NumToStr(p.trans.y, 1) + " "
         + NumToStr(p.trans.z, 1) + " " + NumToStr(p.rot.q1, 4) + " "
         + NumToStr(p.rot.q2, 4) + " " + NumToStr(p.rot.q3, 4) + " " + NumToStr(p.rot.q4, 4);
  ENDFUNC

  ! Quita CR/LF finales.
  LOCAL FUNC string rb_strip(string s)
    VAR num p;
    p := StrFind(s, 1, "\0D\0A");
    IF p <= StrLen(s) THEN
      RETURN StrPart(s, 1, p - 1);
    ENDIF
    RETURN s;
  ENDFUNC

  ! Lee n numeros separados por un espacio. Devuelve FALSE si falta alguno.
  LOCAL FUNC bool rb_parse(string s, num n, INOUT num v{*})
    VAR num pos := 1;
    VAR num sp;
    VAR num len;
    VAR num i;
    VAR string tok;

    len := StrLen(s);
    FOR i FROM 1 TO n DO
      IF pos > len RETURN FALSE;
      sp := StrFind(s, pos, " ");
      tok := StrPart(s, pos, sp - pos);
      IF NOT StrToVal(tok, v{i}) RETURN FALSE;
      pos := sp + 1;
    ENDFOR
    RETURN TRUE;
  ENDFUNC
ENDMODULE
