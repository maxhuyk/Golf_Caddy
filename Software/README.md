# Proyecto Carro Autónomo con ESP32 y UWB

Este proyecto contiene el código para un sistema de carrito autónomo que utiliza tecnología Ultra-Wideband (UWB) para posicionamiento y navegación.

## Estructura del Proyecto

- **Carro/**: Código para el ESP32 del carrito (receptor/controlador principal)
- **TAG/**: Código para el ESP32 del tag/beacon (transmisor de posición)  
- **raspberry/**: Código Python para la Raspberry Pi (procesamiento y control)

## Cómo cargar el código en los ESP32

### Requisitos previos
- **PlatformIO IDE** (extensión de VS Code) o **PlatformIO Core**
- **Cable USB** para conectar el ESP32
- **Drivers USB-Serial** para tu ESP32 instalados

### Pasos para cargar el proyecto:

1. **Conecta tu ESP32** al puerto USB de tu computadora

2. **Identifica el puerto COM**:
   - **Windows**: Abre el Administrador de dispositivos  Puertos (COM y LPT) 
   - Anota el número del puerto (ej: COM3, COM4, etc.)
   - **Linux/Mac**: Ejecuta `ls /dev/tty*` en terminal

3. **Configura el puerto en PlatformIO**:
   - Abre el archivo `platformio.ini` en la carpeta correspondiente (Carro/ o TAG/)
   - Busca la línea que dice `upload_port = ` 
   - **Modifica el puerto COM** al correcto:
     ```ini
     upload_port = COM3    ; Cambia COM3 por tu puerto
     monitor_port = COM3   ; Cambia COM3 por tu puerto
     ```

4. **Compila y sube el código**:
   - Abre VS Code en la carpeta del proyecto (Carro/ o TAG/)
   - Presiona `Ctrl+Shift+P`  "PlatformIO: Upload"
   - O usa el botón "Upload" en la barra inferior de VS Code
   - O en terminal: `pio run --target upload`

### Configuración típica platformio.ini:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
upload_port = COM3        ;  CAMBIAR AQUÍ
monitor_port = COM3       ;  CAMBIAR AQUÍ  
monitor_speed = 115200
lib_deps = 
    SPI
    Wire
```

### Problemas comunes:

- **"Port not found"**: Verifica que el ESP32 esté conectado y el puerto COM sea correcto
- **"Permission denied"**: En Linux, agrega tu usuario al grupo dialout: `sudo usermod -a -G dialout $USER`
- **Error de compilación**: Asegúrate de tener las librerías necesarias instaladas

### Monitorear la salida serial:
```bash
pio device monitor --port COM3 --baud 115200
```

## Uso del sistema:

1. Carga el código del **TAG** en un ESP32
2. Carga el código del **Carro** en otro ESP32  
3. Ejecuta el código de **raspberry** en la Raspberry Pi
4. El sistema iniciará automáticamente la comunicación UWB y el control del carro

Listo para navegar! 