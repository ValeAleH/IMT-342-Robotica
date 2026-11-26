"""Pruebas de la verificacion antes de mover (sin GUI, sin ROS, sin robot).

    python -m pytest ros2_ws/src/irb120_bridge/test      (o: python ros2_ws/src/irb120_bridge/test/test_analisis.py)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from irb120_bridge import analisis as A  # noqa: E402
from irb120_bridge import conexiones  # noqa: E402

Q0 = [0, 0, 0, 0, 30, 0]
OK_MODELO = (0.0, 0.0)


def test_pose_alcanzable_apta():
    a = A.analizar(Q0, pose=[350, 100, 250, 0, 180, 0], error_modelo=OK_MODELO)
    assert a.apto
    assert len(a.ramas) == 8
    elegida = [r for r in a.ramas if r.elegida]
    assert len(elegida) == 1 and elegida[0].en_limites
    ep, eR = A.errores_pose(a.T_deseada, A.fk_tcp(a.q_objetivo_deg))
    assert ep < 1e-6 and eR < 1e-6


def test_fuera_de_alcance():
    a = A.analizar(Q0, pose=[900, 0, 200, 0, 180, 0], error_modelo=OK_MODELO)
    assert not a.apto
    assert not a.chequeos[0].ok and a.chequeos[0].nombre == "Pose alcanzable"


def test_demasiado_bajo():
    a = A.analizar(Q0, pose=[350, 100, 40, 0, 180, 0], error_modelo=OK_MODELO)
    assert not a.apto
    assert [c.nombre for c in a.chequeos if not c.ok] == ["Altura en el trayecto"]


def test_trayecto_que_baja_aunque_el_final_este_alto():
    # ambos extremos a ~645 mm, pero la interpolacion articular baja a ~269 mm a mitad de camino:
    # con z_min = 300 mm el destino "parece" seguro y aun asi debe rechazarse
    q_ini = [70, 18, 15, -146, 113, -86]
    q_fin = [-20, 45, -48, 149, 20, -98]
    assert A.fk_tcp(q_ini)[2, 3] * 1000 > 600 and A.fk_tcp(q_fin)[2, 3] * 1000 > 600
    a = A.analizar(q_ini, q_objetivo_deg=q_fin, z_min_mm=300.0, error_modelo=OK_MODELO)
    assert not a.apto
    assert [c.nombre for c in a.chequeos if not c.ok] == ["Altura en el trayecto"]
    assert a.z_min_trayecto_mm < 300.0


def test_limites_y_tramos():
    a = A.analizar(Q0, q_objetivo_deg=[80, 0, 0, 0, 30, 0], error_modelo=OK_MODELO)
    assert a.apto and len(a.tramos) == 2
    assert np.allclose(a.tramos[-1], [80, 0, 0, 0, 30, 0])
    pasos = np.abs(np.diff(np.vstack([Q0, *a.tramos]), axis=0))
    assert pasos.max() <= A.TRAMO_MAX_DEG + 1e-9
    b = A.analizar(Q0, q_objetivo_deg=[120, 0, 0, 0, 30, 0], error_modelo=OK_MODELO)
    assert not b.apto


def test_modelo_y_robot_bloquean():
    a = A.analizar(Q0, q_objetivo_deg=[10, 0, 0, 0, 30, 0], error_modelo=(5.0, 0.0))
    assert not a.apto
    b = A.analizar(Q0, q_objetivo_deg=[10, 0, 0, 0, 30, 0], error_modelo=None)
    assert not b.apto
    c = A.analizar(Q0, q_objetivo_deg=[10, 0, 0, 0, 30, 0], error_modelo=OK_MODELO,
                   listo=(False, "motores apagados"))
    assert not c.apto


def test_rpy_ida_y_vuelta():
    rng = np.random.default_rng(3)
    for _ in range(200):
        pose = [*rng.uniform(-500, 500, 3), *rng.uniform(-179, 179, 3)]
        pose[4] = rng.uniform(-89, 89)
        T = A.T_desde_pose(pose)
        assert np.allclose(A.T_desde_pose(A.pose_desde_T(T)), T, atol=1e-9)


def test_simulada_mueve_por_tramos():
    import threading
    s = conexiones.Simulada(velocidad_deg_s=400.0)
    s.iniciar()
    fin = threading.Event()
    res = {}
    s.enviar([[20, 0, 0, 0, 30, 0], [40, 0, 0, 0, 30, 0]], lambda *x: None,
             lambda ok, msg: (res.update(ok=ok, msg=msg), fin.set()))
    assert fin.wait(5)
    assert res["ok"] and np.allclose(s.estado()["q_deg"], [40, 0, 0, 0, 30, 0])
    s.cerrar()


if __name__ == "__main__":
    n = 0
    for nombre, f in list(globals().items()):
        if nombre.startswith("test_") and callable(f):
            f()
            n += 1
            print("ok", nombre)
    print(f"{n} pruebas OK")
