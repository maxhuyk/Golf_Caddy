# Especificaciones de Hardware y Manufactura

El sistema electrónico se compone de cuatro PCBs independientes diseñadas para operar en entornos exteriores y soportar vibraciones mecánicas.

## 🗂 Unidades de Producción (PCBA)
Cada subdirectorio en `/Produccion` contiene el paquete completo de fabricación:
* **Gerber Files:** Archivos RS-274X para fabricación de circuitos impresos.
* **BOM (Bill of Materials):** Listado exhaustivo de componentes con especificación de encapsulado y valores técnicos.
* **Pick & Place:** Archivos de coordenadas XY para montaje automatizado de componentes SMD.

## 📋 Descripción de Placas
1.  **PCB1 - Cerebro:** Unidad central de procesamiento y conectividad.
2.  **PCB2 - Motores:** Etapa de potencia y drivers de corriente para tracción.
3.  **PCB3 - Tag:** Electrónica miniaturizada para el dispositivo remoto.
4.  **PCB4 - Sensores:** Módulos auxiliares de detección UWB periférica.

> **Nota de Producción:** Todos los archivos están validados para manufactura industrial en procesos de soldadura por reflujo.
