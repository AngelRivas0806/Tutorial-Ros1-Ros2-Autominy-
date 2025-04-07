# Tutorial para migración del proyecto Autominy de ROS noetic a ROS humble

Este repositorio contiene el código para controlar un carrito AutoMiny con ROS2.

---
## Preparación del Entorno

- Instala ROS2 Humble
- Ubuntu 22

---
## Migración a ROS2
AutoMiny es una plataforma educativa de robótica diseñada para facilitar el aprendizaje y la investigación en conducción autónoma a pequeña escala. Originalmente desarrollado para ROS1 (Robot Operating System), AutoMiny ofrece un sistema modular y flexible que incluye:

    - Un vehículo autónomo físico con hardware abierto.

    - Paquetes ROS1 para control remoto, planificación de trayectorias, sensores, odometría, y visualización en RViz.

    - Soporte para múltiples sensores como cámaras Intel RealSense, sensores LiDAR (como RPLIDAR A2), y unidades inerciales (IMU).

    - Control mediante joystick o comandos desde un nodo remoto. 
Actualmente, el proyecto se encuentra en proceso de migración a ROS2 (Robot Operating System 2), con el objetivo de aprovechar las nuevas capacidades del sistema operativo robótico moderno, como:

    - Comunicación basada en DDS.

    - Mejor soporte para múltiples nodos distribuidos.

    - Herramientas actualizadas para simulación, visualización y desarrollo.

La migración incluye la conversión de nodos a ROS2, actualización de launch files a formato Python, integración con colcon, y validación tanto en simulación como en el vehículo físico.

---
## ¿Cómo instalar el proyecto autominy?
Visitar [Autominy](https://autominy.github.io/AutoMiny/docs/installation/)


##
