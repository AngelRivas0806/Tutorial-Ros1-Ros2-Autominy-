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

## Preparación del Entorno

- Ubuntu 22.04 LTS
- Instalar [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Crear un workspace ROS2:  
  ```bash
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws
  colcon build
  source install/setup.bash
