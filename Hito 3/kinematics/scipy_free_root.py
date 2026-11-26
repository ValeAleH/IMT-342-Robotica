"""Biseccion simple (evita depender de scipy)."""


def bisect(f, a, b, tol=1e-12, it=200):
    fa = f(a)
    for _ in range(it):
        m = 0.5 * (a + b)
        fm = f(m)
        if abs(b - a) < tol:
            break
        if (fa < 0) == (fm < 0):
            a, fa = m, fm
        else:
            b = m
    return 0.5 * (a + b)
