"""Pruebas de la secuencia repetitiva con el robot simulado (sin GUI ni ROS)."""
import time

import numpy as np
import pytest

from irb120_bridge import analisis as A
from irb120_bridge import conexiones
from irb120_bridge import secuencia as S

Q0 = [0, 0, 0, 0, 30, 0]


def _plantilla():
    return S.plantilla_recoger_dejar((400, -100, 150), (350, 150, 150), Q0, altura_aprox_mm=80, espera_s=0.1)


def test_plantilla_recoger_dejar():
    pts = _plantilla()
    assert [p.nombre for p in pts][:2] == ["sobre recoger", "recoger"]
    assert len(pts) == 6
    for p, z in zip(pts, (230, 150, 230, 230, 150, 230)):
        assert abs(p.xyz_mm[2] - z) < 0.01   # q redondeado a 1e-4 deg
        assert A.dentro_limites(p.q_deg)[0]


def test_punto_inalcanzable():
    with pytest.raises(ValueError):
        S.punto_desde_pose("lejos", [900, 0, 300, 0, 180, 0], Q0)


def test_verificacion_detecta_punto_bajo():
    pts = _plantilla()
    assert S.verificar(pts, Q0, error_modelo=(0, 0)).ok
    pts[1] = S.punto_desde_pose("muy bajo", [400, -100, 60, 0, 180, 0], pts[0].q_deg, z_min_mm=0)
    V = S.verificar(pts, Q0, z_min_mm=100, error_modelo=(0, 0))
    assert not V.ok
    assert any("Altura" in f for t in V.trayectos for f in t.fallas)


def _correr(con, ej, hasta, limite_s=60):
    t0 = time.time()
    while ej.activo and time.time() - t0 < limite_s:
        est = con.estado()
        em = A.errores_pose(est["T_ctrl"], __import__("irb120_bridge.irb120_model", fromlist=["fk"]).fk(
            np.deg2rad(est["q_deg"])))
        ej.tick(est, con.listo(), em)
        if hasta(ej):
            ej.detener_fin_ciclo = True
        time.sleep(0.02)


@pytest.fixture
def robot():
    con = conexiones.Simulada(velocidad_deg_s=400.0, q_inicial=Q0)
    con.iniciar()
    yield con
    con.cerrar()


def test_repite_n_ciclos_y_termina_en_p1(robot):
    pts = _plantilla()
    V = S.verificar(pts, Q0, error_modelo=(0, 0))
    eventos = []
    ej = S.Ejecutor(pts, V, 3, enviar=lambda tr: robot.enviar(tr, lambda *a: None, lambda ok, m: ej.fin(ok, m)),
                    registrar=lambda ev, txt, **d: eventos.append(ev))
    _correr(robot, ej, hasta=lambda e: False)
    assert ej.estado == "detenido" and ej.ciclo == 3
    assert np.max(np.abs(np.subtract(robot.estado()["q_deg"], pts[0].q_deg))) < 1e-6
    assert eventos.count("ciclo") == 3
    assert all(c["ep_max_mm"] < 1e-6 for c in ej.ciclos)


def test_detener_al_terminar_ciclo(robot):
    pts = _plantilla()
    V = S.verificar(pts, Q0, error_modelo=(0, 0))
    ej = S.Ejecutor(pts, V, 0, enviar=lambda tr: robot.enviar(tr, lambda *a: None, lambda ok, m: ej.fin(ok, m)),
                    registrar=lambda *a, **k: None)
    _correr(robot, ej, hasta=lambda e: e.ciclo >= 2 and e.idx == 3)
    assert ej.estado == "detenido" and ej.ciclo == 3   # termino el ciclo en curso y paro en P1


def test_se_detiene_si_alguien_mueve_el_robot(robot):
    pts = _plantilla()
    V = S.verificar(pts, Q0, error_modelo=(0, 0))
    robot._q = np.array(Q0, float) + 5.0      # alguien lo movio despues de verificar
    ej = S.Ejecutor(pts, V, 0, enviar=lambda tr: pytest.fail("no debia enviar"),
                    registrar=lambda *a, **k: None)
    time.sleep(0.15)
    _correr(robot, ej, hasta=lambda e: False, limite_s=2)
    assert ej.estado == "error" and "no está donde" in ej.motivo
