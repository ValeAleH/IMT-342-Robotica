"""
Sube un modulo RAPID a la carpeta HOME del controlador por Robot Web Services (sin USB).
No carga ni ejecuta nada: despues se carga desde el FlexPendant (Load Module -> HOME).

Uso:
  python tools/subir_modulo.py                                   (sube rapid/ROS_BRIDGE.mod a 192.168.125.1)
  python tools/subir_modulo.py --url http://127.0.0.1:8080       (mock_irc5)
"""
import argparse
import os

import requests
from requests.auth import HTTPDigestAuth

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ap = argparse.ArgumentParser()
ap.add_argument("--url", default="http://192.168.125.1")
ap.add_argument("--archivo", default=os.path.join(RAIZ, "rapid", "ROS_BRIDGE.mod"))
ap.add_argument("--user", default="Default User")
ap.add_argument("--password", default="robotics")
a = ap.parse_args()

datos = open(a.archivo, "rb").read()
nombre = os.path.basename(a.archivo)
destino = f"{a.url.rstrip('/')}/fileservice/$home/{nombre}"
s = requests.Session()
s.auth = HTTPDigestAuth(a.user, a.password)

print(f"Subiendo {nombre} ({len(datos)} bytes) a {destino}")
r = s.put(destino, data=datos, headers={"Content-Type": "text/plain"}, timeout=10)
print(f"  respuesta: {r.status_code} {r.reason}")
if r.status_code not in (200, 201, 204):
    print("  ERROR: el controlador no acepto el archivo.", r.text[:300])
    raise SystemExit(1)

r = s.get(destino, timeout=10)          # verificacion: volver a descargarlo y comparar
igual = r.status_code == 200 and r.content.replace(b"\r\n", b"\n") == datos.replace(b"\r\n", b"\n")
print(f"  verificacion (descarga y compara): {'OK, el archivo esta en HOME' if igual else 'NO coincide'}")
if igual:
    print("\nAhora en el FlexPendant: Program Editor -> Modules -> File -> Load Module -> carpeta HOME -> "
          f"{nombre} -> OK")
