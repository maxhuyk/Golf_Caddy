# Documentación Técnica: Sistema de Seguimiento Autónomo UWB
## Proyecto: Smart Golf Caddy 

Este repositorio constituye la entrega formal de la ingeniería completa del sistema de transporte autónomo con seguimiento de precisión. El desarrollo integra algoritmos de localización en tiempo real, control de potencia de motores y diseño industrial de gabinetes.

### 🎯 Alcance del Proyecto
El sistema permite que una unidad motriz autónoma mantenga una distancia y ángulo constante respecto a un usuario portador de un TAG activo, utilizando tecnología de radiofrecuencia de banda ultra ancha (UWB).

### 🏗 Arquitectura del Ecosistema
La información está segmentada en tres áreas críticas de producción:

1.  **[Hardware](./Hardware):** Documentación para la fabricación de las 4 unidades de control electrónico (PCBA).
2.  **[Software](./Software):** Firmware desarrollado bajo estándares de alta disponibilidad para la gestión de sensores y actuadores.
3.  **[Gabinete](./Gabinete):** Ingeniería mecánica y archivos de manufactura aditiva para housings y estaciones de carga.

### 🛡 Core Tecnológico
* **Localización:** Módulos UWB **BU03** (Chip Decawave Serie DW).
* **Procesamiento:** Arquitectura ESP32 con soporte para multitarea.
* **Control:** Algoritmos de control PID aplicados a tracción diferencial.
* **Protocolo:** Comunicación redundante para minimización de latencia en la trilateración.
