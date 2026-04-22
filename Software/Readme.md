# Especificaciones de Software y Firmware

El desarrollo se basa en el framework de Arduino sobre el entorno de gestión **PlatformIO**, garantizando la portabilidad y el control de versiones de las librerías.

## 🛠 Entorno de Desarrollo
* **IDE:** Visual Studio Code.
* **Core:** ESP32 Dev Module.
* **Dependencias Críticas:** * `DW3000`: Driver optimizado para el transceptor BU03 ( Es una modificacion de la libreria original para incorporar el uso del multiplexor).
    * `Adafruit BusIO`: Gestión de buses I2C/SPI.
    * `ArduinoJson`: Serialización de datos de telemetría.

## 📦 Módulos de Firmware
* **CarroClean:** Gestión del lazo de control de los motores, procesamiento de señales UWB y lógica  perimetral.
* **TAGClean:** Gestión de energía y beaconing activo para la localización del usuario.

## ⚙️ Procedimiento de Compilación
1. Abrir el workspace `codigo_final.code-workspace`.
2. Seleccionar el entorno correspondiente (`Carro` o `Tag`).
3. Ejecutar `Build` y `Upload` vía UART.
