# Guía de entorno en Windows

> **Opción usada en este proyecto: ROS 2 Humble nativo en Windows** (sección 0). WSL2 (sección 3)
> queda como alternativa.

## 0. ROS 2 Humble nativo en Windows
1. Ruta de trabajo corta, sin espacios ni tildes: `C:\robotica\hito3_irb120`.
2. Consola *x64 Native Tools Command Prompt for VS 2019* y, en cada consola nueva:
   ```bat
   call C:\dev\ros2_humble\local_setup.bat
   cd C:\robotica\hito3_irb120
   ```
   El aviso de RTI Connext es normal (DDS opcional; se usa Fast DDS).
3. Dependencias en el Python de ROS: `python -m pip install numpy requests matplotlib`
4. Compilar (sin `--symlink-install`):
   ```bat
   cd ros2_ws
   colcon build --merge-install
   call install\local_setup.bat
   ```
5. Firewall: permitir Python y RViz en **redes privadas**; marcar como privada la red del adaptador
   USB-Ethernet que va al robot.
6. Panel: `python tools\panel.py` (simulado) · guía en `docs/panel.md`.

## 1. Python en Windows (lo mínimo, sin Linux)
1. Instalar Python 3.10+ (python.org, marcar "Add to PATH").
2. `pip install numpy requests matplotlib`
3. Opcional: Pinocchio como referencia de terceros (`pip install pin`; si falla en Windows, `validate.py`
   usa automáticamente el parser URDF propio, que da los mismos resultados).

## 2. RobotStudio (controlador virtual)
1. Descargar RobotStudio desde abb.com (prueba de 30 días; consultar con el docente la licencia académica).
2. Crear una estación con un IRB 120-3/0.58 y un controlador virtual (RobotWare 6).
   Ideal: **crear el controlador desde el backup del robot real** (mismas opciones y módulos).
3. Si se crea desde cero, activar opciones: 616-1 PC Interface, 623-1 Multitasking, 689-1 EGM.
4. Cargar `rapid/ROS_BRIDGE.mod` en T_ROB1 con `rb_SERVER_IP := "127.0.0.1"` y una tooldata `Gripper1`
   (tframe [0,0,167.5]). Verificar que compile (sin errores en la ventana de salida).
5. RWS del controlador virtual: `http://localhost` (puerto 80; si está ocupado RobotStudio usa otro,
   ver la ventana de salida). Probar: `python tools\gate1_leer_estado.py --url http://localhost`.

## 3. ROS 2 en WSL2 (Ubuntu 22.04 + Humble)
PowerShell (administrador):
```powershell
wsl --install -d Ubuntu-22.04
```
Red: para que WSL vea la misma red que Windows (necesario para hablar con el IRC5), en Windows 11
22H2 o superior crear `C:\Users\<usuario>\.wslconfig`:
```ini
[wsl2]
networkingMode=mirrored
```
y reiniciar con `wsl --shutdown`. (Con la red NAT por defecto, las conexiones salientes TCP/HTTP desde WSL
al robot también funcionan, que es lo que usa esta ruta; EGM/UDP entrante es lo que requiere mirrored.)

Dentro de Ubuntu (instalación oficial de ROS 2 Humble):
```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools \
     ros-humble-joint-state-publisher-gui python3-requests python3-numpy
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```
Compilar el paquete (copiar el repositorio dentro de WSL, p.ej. `~/hito3_irb120`):
```bash
cd ~/hito3_irb120/ros2_ws && colcon build && source install/setup.bash
ros2 launch irb120_bridge bringup.launch.py mode:=gui      # debe abrir RViz con el IRB120 y sliders
```
Si la instalación oficial cambió, seguir docs.ros.org (Humble → Installation → Ubuntu deb packages).

## 4. Red con el robot real (puerto de servicio X2)
- Controlador: `192.168.125.1`. El PC recibe IP por DHCP del puerto de servicio (o fijar 192.168.125.x/24).
- Desactivar Wi-Fi o asegurarse de que no use la misma subred.
- Firewall de Windows: permitir Python/WSL en redes privadas.
- Prueba: `ping 192.168.125.1` desde Windows y desde WSL.
