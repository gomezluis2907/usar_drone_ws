# SAR Drone Simulation (ROS 2 Jazzy + Gazebo Harmonic)

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF6F00?logo=gazebo&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-Thermal_CV-5C3EE8?logo=opencv&logoColor=white)
![License](https://img.shields.io/badge/License-Apache_2.0-green)

Este proyecto presenta una simulación de un dron de búsqueda y rescate en **ROS 2 Jazzy** y **Gazebo Harmonic**. El dron opera en una zona urbana afectada por un terremoto. La aeronave utiliza una cámara térmica y visión computacional para detectar personas entre los escombros. El piloto controla el vuelo en tiempo real desde el teclado mediante comandos `/cmd_vel`.

---

## Características principales

- **El escenario de terremoto** incluye cuatro edificios colapsados, losas inclinadas, escombros y un helipuerto central de despegue.
- **Las víctimas simuladas** consisten en cinco personas en postura tendida y sentada con temperatura corporal realista (35 °C a 37 °C).
- **Los distractores térmicos** incluyen un auto volcado con motor tibio (~28 °C) y un foco de calor (~75 °C) para probar el filtro de detección.
- **El dron cuadricóptero** opera con control de velocidad `/cmd_vel`, cámara térmica de 16 bits (`L16`) y cámara RGB de referencia.
- **El detector térmico** procesa las imágenes con OpenCV, dibuja recuadros sobre cada persona y separa las anomalías de fuego o calor extremo.
- **El nodo de teleoperación** permite volar con el teclado, mantener *hover* automático y leer telemetría básica en la terminal.

---

## Estructura del repositorio

```text
sar_drone_ws/
├── .gitignore
├── README.md
└── src/
    └── sar_drone_sim/
        ├── config/
        │   └── ros_gz_bridge.yaml
        ├── launch/
        │   └── sim.launch.py
        ├── models/
        │   ├── sar_drone/
        │   ├── victim_lying/
        │   └── victim_sitting/
        ├── sar_drone_sim/
        │   ├── keyboard_teleop.py
        │   └── thermal_detector.py
        └── worlds/
            └── earthquake_disaster.sdf
```

---

## Requisitos del sistema

El entorno necesita las siguientes herramientas instaladas en **Ubuntu 24.04**:

- **ROS 2 Jazzy** con `colcon`.
- **Gazebo Harmonic** (`ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-ros-gz-sim`).
- **Python 3** con `ros-jazzy-cv-bridge`, `python3-opencv` y `python3-numpy`.

---

## Instalación y compilación

El usuario debe clonar este repositorio y compilar el workspace con `colcon`:

```bash
git clone <URL_DE_TU_REPOSITORIO> sar_drone_ws
cd sar_drone_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## Ejecución de la simulación

La simulación utiliza dos terminales independientes.

### 1. Lanzamiento de Gazebo y visión térmica

La primera terminal inicia el mundo en Gazebo, el puente con ROS 2 y el detector térmico:

```bash
source /opt/ros/jazzy/setup.bash
source ~/sar_drone_ws/install/setup.bash
ros2 launch sar_drone_sim sim.launch.py
```

### 2. Control del dron por teclado

La segunda terminal inicia el nodo de teleoperación manual:

```bash
source /opt/ros/jazzy/setup.bash
source ~/sar_drone_ws/install/setup.bash
ros2 run sar_drone_sim keyboard_teleop
```

---

## Controles de vuelo

La siguiente tabla resume las teclas disponibles en `keyboard_teleop`:

| Tecla | Acción |
| :--- | :--- |
| `T` | El dron realiza un despegue suave automático (~2 s). |
| `W` / `S` | El dron avanza o retrocede. |
| `A` / `D` | El dron se desplaza a la izquierda o a la derecha. |
| `R` / `F` o Flechas `↑` / `↓` | El dron sube o baja. |
| `Q` / `E` o Flechas `←` / `→` | El dron gira sobre su eje (Yaw). |
| `Espacio` | El dron se detiene en *hover* inmediato. |
| `1` / `2` / `3` o `+` / `-` | El sistema cambia el modo de velocidad (Lento, Normal, Rápido). |
| `X` o `Ctrl+C` | El nodo detiene el dron y cierra la teleoperación. |

---

## Tópicos principales de ROS 2

La arquitectura desacopla el control, los sensores y la detección mediante tópicos estándar:

| Tópico | Tipo | Descripción |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Este tópico recibe los comandos de velocidad del dron. |
| `/sar_drone/odom` | `nav_msgs/msg/Odometry` | Este tópico publica la posición y orientación 3D del dron. |
| `/sar_drone/thermal/image_raw` | `sensor_msgs/msg/Image` | Este tópico entrega la imagen térmica cruda (`mono16`). |
| `/sar_drone/rgb/image_raw` | `sensor_msgs/msg/Image` | Este tópico entrega la imagen RGB de referencia. |
| `/sar_drone/thermal/annotated` | `sensor_msgs/msg/Image` | Este tópico publica el mapa térmico con detecciones dibujadas. |
| `/sar_drone/detections` | `std_msgs/msg/String` | Este tópico publica el conteo, cajas y temperaturas en formato JSON. |
| `/sar_drone/victim_detected` | `std_msgs/msg/Bool` | Este tópico indica si la cámara detecta al menos una persona. |

---

## Trabajo futuro

- **La integración con PX4** reemplazará el controlador `/cmd_vel` por un piloto automático completo mediante Micro XRCE-DDS o MAVROS.
- **La navegación autónoma** permitirá patrullar los escombros y registrar las coordenadas GPS de cada persona detectada.

---

## Licencia

Este proyecto se distribuye bajo la licencia **Apache 2.0**.
