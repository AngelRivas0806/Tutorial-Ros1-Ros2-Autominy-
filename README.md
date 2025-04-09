# Tutorial para la Migración del Proyecto AutoMiny de ROS Noetic a ROS 2 Humble

Este repositorio contiene el código migrado y documentación necesaria para controlar un carrito AutoMiny utilizando **ROS 2 Humble** en **Ubuntu 22.04**.

> **Nota:** Este tutorial documenta el proceso de migración del ecosistema AutoMiny, originalmente desarrollado en ROS1 (Noetic), a ROS2 (Humble).

---

## 📦 Contenidos

- [Preparación del entorno](#preparación-del-entorno)
- [¿Qué es AutoMiny?](#qué-es-autominy)
- [Diferencias clave entre ROS1 y ROS2](#diferencias-ros1-vs-ros2)
- [Pasos de migración](#pasos-de-migración)
- [Migración por paquetes](#migración-por-paquetes)
- [Instalación del proyecto AutoMiny](#instalación-del-proyecto-autominy)
- [Simulación y pruebas](#simulación-y-pruebas)
- [Estado actual del proyecto](#estado-actual-del-proyecto)
- [Referencias](#referencias)

---

## 🛠️ Preparación del Entorno

- Ubuntu 22.04 LTS
- Instalar [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Crear y configurar el workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🤖 ¿Qué es AutoMiny?

AutoMiny es una plataforma educativa de robótica diseñada para facilitar el aprendizaje y la investigación en conducción autónoma a pequeña escala. Su ecosistema incluye:

- Un vehículo autónomo físico con hardware abierto.
- Paquetes ROS1 para:
  - Control remoto.
  - Planificación de trayectorias.
  - Procesamiento de sensores (cámaras, LiDAR, IMU).
  - Odometría.
  - Visualización en RViz.
- Control mediante joystick o nodos remotos.

---

## 🔄 Diferencias ROS1 vs ROS2

- Comunicación basada en DDS (Data Distribution Service).
- Launch files en Python (`.launch.py`) en lugar de XML.
- Compilación con `colcon` en lugar de `catkin_make`.
- Uso de `rclcpp` y `rclpy` como nuevos APIs.
- Herramientas modernas (`ros2 topic`, `ros2 node`, `ros2 interface`).
- Mejor soporte para sistemas distribuidos y robótica en tiempo real.

---

## 🔧 Pasos de Migración

1. Análisis del código ROS1.
2. Conversión de mensajes y servicios personalizados a `.msg` / `.srv` compatibles con ROS2.
3. Migración de nodos C++ y/o Python a `rclcpp` / `rclpy`.
4. Adaptación de `CMakeLists.txt` y `package.xml` al formato `ament`.
5. Migración de launch files a formato `.launch.py`.
6. Pruebas unitarias y validación por nodo.
7. Integración con herramientas ROS2 (RViz2, Gazebo, etc.).
8. Validación en entorno físico (carrito AutoMiny).

---

## 📦 Migración por Paquetes

| Paquete                | Estado       | Notas                                       |
|------------------------|--------------|---------------------------------------------|
| `remote_control`       | ✅ Migrado   | Nodo en C++ con SDL para gamepad            |
| `lidar_pose_estimation`| 🔄 En proceso| Uso de RPLIDAR A2                           |
| `autominy_msgs`        | ✅ Migrado   | Mensajes personalizados                     |
| `localization`         | 🕒 Pendiente | Usa odometría y sensores                    |
| `planning`             | 🕒 Pendiente | Algoritmos de trayectoria                   |
| `simulation`           | 🕒 Pendiente | Integración con Gazebo / RViz               |
| `unity_bridge`         | ✅ (pruebas) | Comunicación con Unity y ROS2               |

> Puedes actualizar esta tabla conforme avances con cada paquete.

---

## 📥 Instalación del Proyecto AutoMiny

Para detalles sobre hardware, sensores y montaje del carrito, consulta la [documentación oficial de AutoMiny](https://autominy.github.io/AutoMiny/docs/installation/).

Para clonar los paquetes:

```bash
cd ~/ros2_ws/src
git clone https://github.com/tu-usuario/autominy_ros2.git
cd ..
colcon build
source install/setup.bash
```

> Asegúrate de haber migrado correctamente los mensajes (`autominy_msgs`) antes de compilar otros paquetes dependientes.

---

## 🧪 Simulación y Pruebas

- ✔️ Visualización en RViz2 usando URDF y `tf`.
- 🕒 Simulación con Gazebo (en progreso).
- ✅ Control con joystick (migrado desde `remote_control`).
- 🔄 Integración opcional con Unity para entornos 3D.

---

## 🚧 Estado Actual del Proyecto

- 🔧 Migración en curso.
- ✔️ Paquetes esenciales ya funcionan en ROS2.
- 🧪 Pruebas en hardware real comenzarán tras validación en simulación.
- 🎯 Meta: Plataforma educativa y de investigación totalmente funcional con ROS2.

---

## 🔗 Referencias

- [ROS 2 Migration Guide](https://docs.ros.org/en/foxy/How-To-Guides/Contributing/Migration-Guide.html)
- [AutoMiny GitHub](https://github.com/AutoMiny)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS Index](https://index.ros.org/)
