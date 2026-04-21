#include <Arduino.h>
#include "DW3000.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <ArduinoOTA.h>
#include <esp_sleep.h>
#include <Adafruit_NeoPixel.h>
#include <esp_wifi.h> // Ponelo arriba de todo

// ================== Guía de Estado de LEDs ==================
// El TAG utiliza 8 LEDs WS2812B para indicar su estado de forma visual.
//
// LED 1: Encendido/Actividad General.
//   - Se enciende en ROJO al inicio del arranque.
//   - Permanece en ROJO si el setup se completa correctamente.
//
// Barra de Progreso (Arranque):
//   - Durante el `setup`, los LEDs se encienden en secuencia (1->0->7->6->5->4->3->2)
//     en color ROJO para mostrar el progreso del arranque.
//
// LED 3: Modo de Operación.
//   - APAGADO: Modo OFF (0).
//   - VERDE:   Modo TRACKING (1).
//   - ROJO:    Modo PAUSE (2).
//   - AZUL:    Modo MANUAL (3).
//
// LED 5: Estado del Cargador.
//   - ROJO:          Batería cargando.
//   - APAGADO:       Batería completamente cargada.
//   - AZUL PARPADEANTE: Fallo en la carga.
//
// LED 0: Error de Conexión UWB.
//   - Se enciende en AMARILLO si el TAG está en modo TRACKING pero no puede
//     detectar la comunicación con el ancla (el carro).
//
// ================== Mapeo hardware actualizado v4 ==================
// Joystick X: GPIO39 (ADC1, input only)
// Joystick Y: GPIO35 (ADC1, input only)
// LED estado: GPIO5 (8x WS2812B LEDs)
// Botón modo (OK): GPIO25
// Pin stat (estado del cargador de batería): GPIO14

#define PIN_JOY_X       39
#define PIN_JOY_Y       35
#define PIN_LED         5
#define PIN_BOTON_MODO  25
#define PIN_STAT        14

#define LED_COUNT 8
Adafruit_NeoPixel strip(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);

// Mantener compatibilidad con joystick (RIGHT=Y, DOWN=X)
#define BTN_RIGHT PIN_JOY_Y
#define BTN_DOWN  PIN_JOY_X
#define BTN_OK    PIN_BOTON_MODO

// Índices de botones (simplificado: solo OK es digital)
#define BTN_IDX_OK 0


// ================== Config batería ==================
#define BATTERY_PIN 36
#define R1 10000.0
#define R2 33000.0
#define VOLTAGE_DIVIDER_FACTOR ((R1 + R2) / R2)
#define ADC_RESOLUTION 4095.0
#define ADC_REFERENCE_VOLTAGE 3.3
#define BATTERY_SAMPLES 10

// ================== Estado cargador ==================
enum ChargerStatus {
    CHARGER_UNKNOWN,
    CHARGER_CHARGING,      // LOW
    CHARGER_COMPLETE,      // HIGH
    CHARGER_FAULT          // BLINKING
};
volatile ChargerStatus currentChargerStatus = CHARGER_UNKNOWN;

// ================== Estado analógico ==================
static int analogRightRaw = 0; // Y
static int analogDownRaw  = 0; // X
static float analogRightVolt = 0.0f;
static float analogDownVolt  = 0.0f;
static unsigned long lastAnalogSampleMs = 0;
static const unsigned long ANALOG_SAMPLE_INTERVAL_MS = 25; // ~40 Hz
static float analogRightNorm = 0.0f;
static float analogDownNorm  = 0.0f;

// MAC Address de Broadcast (envía a cualquier ESP32 escuchando)
static uint8_t carroMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ================== Namespace ESP-NOW ==================
namespace EspNowComm {
    typedef struct {
        uint8_t  version = 2;
        uint16_t batteryVoltage_mV;
        uint8_t  modo;
        uint16_t ax_raw;
        uint16_t ay_raw;
        uint32_t timestamp;
    } __attribute__((packed)) EspNowData;

    volatile EspNowData toSend = {};
    volatile unsigned long lastSendMs = 0;
    volatile unsigned long totalPacketsSent = 0;
}

// ================== Modos de sistema ==================
enum SystemMode {
    MODE_OFF = 0,
    MODE_TRACKING = 1,
    MODE_PAUSE = 2,
    MODE_MANUAL = 3,
};
volatile SystemMode currentMode = MODE_OFF;
volatile uint8_t joystickValue = 0; // legacy (se mantiene para compatibilidad logs)

// Protección de cambio de modo
static unsigned long lastModeChangeTime = 0;
static const unsigned long MODE_CHANGE_GUARD_MS = 300;

// ================== Estado botones ==================
struct ButtonState {
    bool raw = false;
    bool stable = false;
    bool previousStable = false;
    unsigned long lastChange = 0;
    bool rising = false;
    bool falling = false;
    unsigned long pressStart = 0;
    bool longPressFired = false;
};

// (ButtonState y BTN_IDX_* ya definidos arriba)

const unsigned long LONG_PRESS_TIME = 1500; // 1.5s mejora reconocimiento
const unsigned long DEBOUNCE_TIME   = 40;   // 40ms debounce (modo)
// Long press especial para habilitar OTA (10s)
const unsigned long OTA_LONG_PRESS_MS = 10000;

// Activación de modo MANUAL por joystick (ay < umbral por cierto tiempo)
// Nota: ay corresponde a analogRightRaw (RIGHT = Y)
const int JOY_MANUAL_THRESHOLD = 1200;                 // umbral crudo ADC para activar
const unsigned long JOY_MANUAL_HOLD_MS = LONG_PRESS_TIME; // tiempo de sostén (por defecto 1500ms)

// (Definiciones analógicas ya declaradas arriba; se elimina duplicado)

// Variables compartidas entre cores (thread-safe)
volatile bool dw3000_data_ready = false;
volatile unsigned long total_cycles = 0;
volatile unsigned long successful_cycles = 0;
volatile unsigned long timeout_resets = 0;
volatile bool carro_detected = false;

// Semáforo para sincronización entre cores
SemaphoreHandle_t dw3000Semaphore = NULL;

// Handle de la tarea DW3000
TaskHandle_t dw3000TaskHandle = NULL;


uint8_t motorL = 127; // 0..255 (127 = neutro)
uint8_t motorR = 127; // 0..255 (127 = neutro)

// Estado OTA / ESP-NOW
static bool otaModeActive = false;
static bool espNowActive = true; // Mientras true, se envían paquetes ESP-NOW

// Credenciales WiFi para OTA (REEMPLAZAR por las reales)
const char* OTA_SSID = "CLAROWIFI";
const char* OTA_PASS = "11557788";

// Forward declaration
void enterOtaMode();



// ESP-NOW Callbacks
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //Serial.print("[ESP-NOW] Envío ");
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ÉXITO" : "FALLO");
    if (status == ESP_NOW_SEND_SUCCESS) {
        EspNowComm::totalPacketsSent++;
        EspNowComm::lastSendMs = millis();
    }
}

// Función para inicializar ESP-NOW
bool initESPNOW() {
    if (!espNowActive) return false; // No inicializar si se ha deshabilitado por OTA
    // Configurar WiFi en modo Station
    WiFi.mode(WIFI_STA);
    
    WiFi.disconnect(); // Asegurar que no esté conectado a ninguna red
    esp_wifi_set_max_tx_power(80); // Establece la potencia al máximo (~20dBm) (80 * 0.25 = 20)

    // Mostrar MAC Address del TAG
    Serial.print("[ESP-NOW] TAG MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Inicializar ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Error inicializando ESP-NOW");
        return false;
    }

    // Registrar callback de envío
    esp_now_register_send_cb(onDataSent);

    // Agregar peer (Carro)
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, carroMacAddress, 6);
    peerInfo.channel = 0;  // Canal automático
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    // Intentar agregar peer con reintentos
    esp_err_t addStatus = esp_now_add_peer(&peerInfo);
    if (addStatus != ESP_OK) {
        Serial.printf("[ESP-NOW] Error agregando peer: %d\n", addStatus);
        return false;
    }

    Serial.println("[ESP-NOW] Inicializado correctamente");
    Serial.printf("[ESP-NOW] Peer agregado: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  carroMacAddress[0], carroMacAddress[1], carroMacAddress[2],
                  carroMacAddress[3], carroMacAddress[4], carroMacAddress[5]);
    return true;
}

// ============== Definiciones de botones ==============
ButtonState buttons[1]; // Solo un botón digital (OK)

// Función para leer estado de botones (no bloqueante)
void readButtons() {
    unsigned long now = millis();
    
    // Leer solo el botón OK
    bool r = !digitalRead(PIN_BOTON_MODO); // Activo en LOW

    if (r != buttons[BTN_IDX_OK].raw) {
        buttons[BTN_IDX_OK].raw = r;
        buttons[BTN_IDX_OK].lastChange = now;
    }

    if ((now - buttons[BTN_IDX_OK].lastChange) >= DEBOUNCE_TIME && buttons[BTN_IDX_OK].stable != buttons[BTN_IDX_OK].raw) {
        buttons[BTN_IDX_OK].previousStable = buttons[BTN_IDX_OK].stable;
        buttons[BTN_IDX_OK].stable = buttons[BTN_IDX_OK].raw;
        buttons[BTN_IDX_OK].rising = (buttons[BTN_IDX_OK].stable && !buttons[BTN_IDX_OK].previousStable);
        buttons[BTN_IDX_OK].falling = (!buttons[BTN_IDX_OK].stable && buttons[BTN_IDX_OK].previousStable);

        if (buttons[BTN_IDX_OK].rising) {
            buttons[BTN_IDX_OK].pressStart = now;
            buttons[BTN_IDX_OK].longPressFired = false;
        }
    } else {
        buttons[BTN_IDX_OK].rising = false;
        buttons[BTN_IDX_OK].falling = false;
    }

    if (buttons[BTN_IDX_OK].stable && !buttons[BTN_IDX_OK].longPressFired && (now - buttons[BTN_IDX_OK].pressStart >= LONG_PRESS_TIME)) {
        buttons[BTN_IDX_OK].longPressFired = true;
    }
}

// Lectura analógica de los nuevos ejes (RIGHT=Y, DOWN=X)
void readAnalogAxes() {
    unsigned long now = millis();
    if (now - lastAnalogSampleMs < ANALOG_SAMPLE_INTERVAL_MS) return;
    lastAnalogSampleMs = now;

    analogRightRaw = analogRead(BTN_RIGHT); // ADC1 -> fiable con WiFi
    analogDownRaw  = analogRead(BTN_DOWN);  // ADC2 -> puede dar 0 con WiFi activo

    analogRightVolt = (analogRightRaw / 4095.0f) * 3.3f;
    analogDownVolt  = (analogDownRaw  / 4095.0f) * 3.3f;

    analogRightNorm = constrain(analogRightVolt / 3.3f, 0.0f, 1.0f);
    analogDownNorm  = constrain(analogDownVolt  / 3.3f, 0.0f, 1.0f);
}



// Función para procesar cambios de modo
void processModeChanges() {
    unsigned long now = millis();
    if (now - lastModeChangeTime < MODE_CHANGE_GUARD_MS) return;

    auto resetButtonsAfterModeChange = [&](){
        lastModeChangeTime = now;
        buttons[BTN_IDX_OK].previousStable = buttons[BTN_IDX_OK].stable;
        buttons[BTN_IDX_OK].rising = false;
        buttons[BTN_IDX_OK].falling = false;
        buttons[BTN_IDX_OK].longPressFired = false;
        buttons[BTN_IDX_OK].pressStart = now;
        buttons[BTN_IDX_OK].lastChange = now;
    };

    // Joystick Y sostenido bajo umbral -> MANUAL (modo 3)
    // Requiere mantener ay (analogRightRaw) < JOY_MANUAL_THRESHOLD durante JOY_MANUAL_HOLD_MS
    {
        static unsigned long joyManualSince = 0;
        if (analogRightRaw < JOY_MANUAL_THRESHOLD) {
            if (joyManualSince == 0) joyManualSince = now;
            if ((now - joyManualSince) >= JOY_MANUAL_HOLD_MS && currentMode != MODE_MANUAL) {
                currentMode = MODE_MANUAL;
                Serial.println("[MODE] Cambiado a MANUAL (por joystick Y)");
                resetButtonsAfterModeChange();
                // evitar re-entrada inmediata si se mantiene el joystick presionado
                joyManualSince = now;
                return;
            }
        } else {
            joyManualSince = 0;
        }
    }

    // OK long press: OFF -> TRACKING, others -> OFF
    if (buttons[BTN_IDX_OK].longPressFired) {
        buttons[BTN_IDX_OK].longPressFired = false;
        if (currentMode == MODE_OFF) {
            currentMode = MODE_TRACKING;
            Serial.println("[MODE] Cambiado a SEGUIMIENTO");
        } else {
            currentMode = MODE_OFF;
            Serial.println("[MODE] Cambiado a APAGADO");
        }
        resetButtonsAfterModeChange();
        return;
    }

    // OK short press: TRACKING <-> PAUSE
    if (buttons[BTN_IDX_OK].falling) {
        // Consideramos short press si no hubo longPressFired en este ciclo
        if (currentMode == MODE_TRACKING) {
            currentMode = MODE_PAUSE;
            Serial.println("[MODE] Cambiado a PAUSA (OK corto)");
            resetButtonsAfterModeChange();
            return;
        } else if (currentMode == MODE_PAUSE) {
            currentMode = MODE_TRACKING;
            Serial.println("[MODE] Cambiado a SEGUIMIENTO (OK corto)");
            resetButtonsAfterModeChange();
            return;
        }
    }
}

// Función para calcular valor de joystick (solo modo MANUAL)
void calculateJoystickValue() {
    // Reset por defecto
    joystickValue = 0;
    motorL = motorR = 127;

    // Solo calcular joystick en modo MANUAL
    if (currentMode != MODE_MANUAL) { return; }

    // Usar valores analógicos normalizados para determinar la dirección
    bool up = (analogRightNorm > 0.8); // Y-axis
    bool down = (analogRightNorm < 0.2); // Y-axis
    bool left = (analogDownNorm < 0.2); // X-axis
    bool right = (analogDownNorm > 0.8); // X-axis

    // Invalidar combinaciones opuestas verticales/horizontales
    if ((up && down) || (left && right)) {
        joystickValue = 0;
        return;
    }

    // Mapeo a valor de joystick (0-7)
    if (up && !left && !right) {
        joystickValue = 1; // adelante
    } else if (up && left) {
        joystickValue = 2; // adelante-izquierda
    } else if (!up && !down && left) {
        joystickValue = 3; // izquierda
    } else if (down && left) {
        joystickValue = 4; // atrás-izquierda
    } else if (down && !left && !right) {
        joystickValue = 5; // atrás
    } else if (down && right) {
        joystickValue = 6; // atrás-derecha
    } else if (!up && !down && right) {
        joystickValue = 7; // derecha
    } else if (up && right) {
        joystickValue = 1; // adelante (fallback para adelante-derecha)
    } else {
        joystickValue = 0; // sin movimiento
    }
}

DW3000Class dw(4, 32, 34); // CS=4, RST=32, IRQ=34

// Tarea para Core 0 - Manejo del DW3000
void dw3000Task(void* parameter) {
    // Variables locales para la tarea DW3000 (Core 0)
    int frame_buffer = 0;
    int rx_status;
    int tx_status;
    int curr_stage = 0;
    int t_roundB = 0;
    int t_replyB = 0;
    long long rx = 0;
    long long tx = 0;
    unsigned long last_anchor_communication = 0;
    unsigned long communication_timeout = 1200; // Aumentado para reducir falsos timeouts
    bool local_carro_detected = false;
    unsigned long local_total_cycles = 0;
    unsigned long local_successful_cycles = 0;
    unsigned long local_timeout_resets = 0;
    unsigned long last_reset_time = 0;        // Último softReset real
    int consecutive_timeouts = 0;             // Timeouts seguidos desde última detección válida
    int consecutive_soft_resets = 0;          // Conteo de softResets para backoff

    Serial.println("[DW3000-Core0] Iniciando tarea DW3000...");

    // Inicialización DW3000 en Core 0
    dw.begin();
    dw.hardReset();
    delay(300);

    if(!dw.checkSPI()) {
        Serial.println("[ERROR] Could not establish SPI Connection to DW3000!");
        vTaskDelete(NULL);
        return;
    }

    while (!dw.checkForIDLE()) {
        Serial.println("[ERROR] IDLE1 FAILED");
        delay(500);
    }

    dw.softReset();
    delay(300);

    if (!dw.checkForIDLE()) {
        Serial.println("[ERROR] IDLE2 FAILED");
        vTaskDelete(NULL);
        return;
    }

    dw.init();
    delay(50);
    dw.setupGPIO();
    delay(30);
    dw.configureAsTX();
    delay(30);
    dw.clearSystemStatus();
    delay(20);
    dw.standardRX();

    last_anchor_communication = millis();
    local_carro_detected = false;

    Serial.println("[DW3000-Core0] DW3000 inicializado - Esperando detección del CARRO..."); 

    while (true) {
        // AUTO-DETECCIÓN DEL CARRO
        if (millis() - last_anchor_communication > communication_timeout) {
            // Timeout de comunicación
            consecutive_timeouts++;
            // Solo considerar "desconectado" si antes estaba detectado
            if (local_carro_detected) {
                if (consecutive_timeouts <= 2) {
                    // Reintento ligero: no soft reset, sólo limpiar y volver a RX
                    if ((consecutive_timeouts % 1) == 1) {
                        Serial.println("[DW3000-Core0] Timeout leve -> reintento sin softReset");
                    }
                    dw.clearSystemStatus();
                    dw.standardRX();
                    curr_stage = 0;
                } else {
                    // Soft reset escalonado con backoff mínimo 3s
                    unsigned long nowMs = millis();
                    if (nowMs - last_reset_time > 3000) {
                        Serial.printf("[DW3000-Core0] Timeout %d consecutivo -> softReset\n", consecutive_timeouts);
                        local_carro_detected = false;
                        local_timeout_resets++;
                        dw.softReset();
                        delay(120);
                        dw.init();
                        delay(25);
                        dw.setupGPIO();
                        delay(8);
                        dw.configureAsTX();
                        delay(8);
                        dw.clearSystemStatus();
                        delay(5);
                        dw.standardRX();
                        curr_stage = 0;
                        last_reset_time = nowMs;
                        consecutive_soft_resets++;
                        consecutive_timeouts = 0; // reiniciar contador tras softReset
                        // Ajustar timeout dinámicamente si muchos resets
                        if (consecutive_soft_resets >= 3 && communication_timeout < 2000) {
                            communication_timeout += 300; // ampliar ventana
                            Serial.printf("[DW3000-Core0] Aumentando communication_timeout a %lu ms\n", communication_timeout);
                        }
                        if (xSemaphoreTake(dw3000Semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                            carro_detected = false;
                            timeout_resets = local_timeout_resets;
                            xSemaphoreGive(dw3000Semaphore);
                        }
                    } else {
                        // Demasiado pronto para otro softReset: sólo reintento ligero
                        dw.clearSystemStatus();
                        dw.standardRX();
                        curr_stage = 0;
                    }
                }
            } else {
                // No detectado aún: sólo seguir escuchando sin resets agresivos
                dw.standardRX();
                curr_stage = 0;
            }
            last_anchor_communication = millis();
        }

        // Máquina de estados DW3000
        switch (curr_stage) {
            case 0: { // Esperando ping inicial del carro
                if (rx_status = dw.receivedFrameSucc()) {
                    dw.clearSystemStatus();
                    if (rx_status == 1) {
                        if (dw.ds_isErrorFrame()) {
                            curr_stage = 0;
                            dw.standardRX();
                        } else if (dw.ds_getStage() != 1) {
                            dw.ds_sendErrorFrame();
                            dw.standardRX();
                            curr_stage = 0;
                        } else {
                            // ÉXITO: Comunicación detectada con CARRO
                            if (!local_carro_detected) {
                                Serial.println("[DW3000-Core0] CARRO detectado!");
                                local_carro_detected = true;
                                consecutive_timeouts = 0; 
                                consecutive_soft_resets = 0;
                            }
                            last_anchor_communication = millis();
                            curr_stage = 1;
                            local_total_cycles++;
                        }
                    } else {
                        dw.clearSystemStatus();
                    }
                } else {
                    // Si no recibimos nada, ES AQUÍ donde dejamos descansar al procesador 1ms
                    vTaskDelay(pdMS_TO_TICKS(1)); 
                }
                break;
            }

            case 1:  // Recibimos el Ping, enviamos la Respuesta rápida.
                dw.ds_sendFrame(2);
                delayMicroseconds(500); // 500 microsegundos en vez de 5 milisegundos
                rx = dw.readRXTimestamp();
                delayMicroseconds(200); // 200 microsegundos en vez de 2 milisegundos
                tx = dw.readTXTimestamp();
                t_replyB = tx - rx;
                curr_stage = 2;
                break;

            case 2:  // Esperando el segundo Ping del carro (NO USAMOS DELAY AQUÍ)
                if (rx_status = dw.receivedFrameSucc()) {
                    dw.clearSystemStatus();
                    if (rx_status == 1) {
                        if (dw.ds_isErrorFrame()) {
                            curr_stage = 0;
                            dw.standardRX();
                        } else if (dw.ds_getStage() != 3) {
                            dw.ds_sendErrorFrame();
                            dw.standardRX();
                            curr_stage = 0;
                        } else {
                            last_anchor_communication = millis();
                            curr_stage = 3;
                        }
                    } else {
                        dw.clearSystemStatus();
                        curr_stage = 0;
                        dw.standardRX();
                    }
                }
                // Si aún no llega, hacemos un mini-descanso pero NO dormimos el core
                else {
                    taskYIELD(); // Permite que otras tareas vitales operen, sin forzar 1ms de latencia
                }
                break;

            case 3:  // Recibimos el 2do Ping. Enviamos la Información final de tiempo.
                rx = dw.readRXTimestamp();
                delayMicroseconds(200); 
                t_roundB = rx - tx;
                dw.ds_sendRTInfo(t_roundB, t_replyB);
                delayMicroseconds(500); 

                // ÉXITO: Ciclo completado
                local_successful_cycles++;
                last_anchor_communication = millis();
                curr_stage = 0;

                // Actualizar variables compartidas thread-safe
                if (xSemaphoreTake(dw3000Semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                    carro_detected = local_carro_detected;
                    total_cycles = local_total_cycles;
                    successful_cycles = local_successful_cycles;
                    timeout_resets = local_timeout_resets;
                    dw3000_data_ready = true;
                    xSemaphoreGive(dw3000Semaphore);
                }
                break;

            default:
                curr_stage = 0;
                dw.standardRX();
                break;
        }

        // ELIMINA EL vTaskDelay(pdMS_TO_TICKS(1)) QUE ESTABA AQUÍ ABAJO
    }
}

// Función para leer el voltaje de la batería LiPo
float readBatteryVoltage() {
    long sum = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
        sum += analogRead(BATTERY_PIN);
        delayMicroseconds(100);
    }
    float average = (float)sum / BATTERY_SAMPLES;
    float voltage_at_pin = (average / ADC_RESOLUTION) * ADC_REFERENCE_VOLTAGE;
    float battery_voltage = voltage_at_pin * VOLTAGE_DIVIDER_FACTOR;
    return battery_voltage;
}

String getBatteryStatus(float voltage) {
    if (voltage > 4.1) return "FULL";
    else if (voltage > 3.8) return "GOOD";
    else if (voltage > 3.6) return "MID";
    else if (voltage > 3.3) return "LOW";
    else return "CRITICAL";
}

int getBatteryPercentage(float voltage) {
    if (voltage >= 4.1) return 100;
    else if (voltage >= 3.9) return 80;
    else if (voltage >= 3.8) return 60;
    else if (voltage >= 3.7) return 40;
    else if (voltage >= 3.6) return 20;
    else if (voltage >= 3.3) return 10;
    else return 0;
}

void setup() {
    // Iniciar NeoPixel primero para feedback visual inmediato
    strip.begin();
    strip.setBrightness(50);
    strip.clear();
    strip.setPixelColor(1, strip.Color(255, 0, 0)); // LED 1 se enciende al inicio absoluto
    strip.show();
    
    Serial.begin(500000);
    delay(50); // Pequeño delay para que se vea el primer LED
    
    strip.setPixelColor(0, strip.Color(255, 0, 0)); // LED 0
    strip.show();

    // Configurar pines
    pinMode(BATTERY_PIN, INPUT);
    pinMode(PIN_STAT, INPUT_PULLUP);
    pinMode(BTN_RIGHT, INPUT); // Joystick Y
    pinMode(BTN_DOWN, INPUT);  // Joystick X
    pinMode(BTN_OK, INPUT_PULLUP);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    delay(100);
    strip.setPixelColor(7, strip.Color(255, 0, 0)); // LED 7
    strip.show();

    float initial_voltage = readBatteryVoltage();
    int battery_percentage = getBatteryPercentage(initial_voltage);
    Serial.printf("[BATTERY] Voltaje inicial: %.2fV (%s - %d%%)\n",
                  initial_voltage, getBatteryStatus(initial_voltage).c_str(), battery_percentage);
    delay(100);
    strip.setPixelColor(6, strip.Color(255, 0, 0)); // LED 6
    strip.show();

    Serial.println("[TAG] Inicializando sistema dual-core...");

    if (!initESPNOW()) {
        Serial.println("[TAG] ERROR: No se pudo inicializar ESP-NOW");
    }
    delay(100);
    strip.setPixelColor(5, strip.Color(255, 0, 0)); // LED 5
    strip.show();

    EspNowComm::toSend.version = 2;
    EspNowComm::toSend.batteryVoltage_mV = 0;
    EspNowComm::toSend.modo = 0;
    EspNowComm::toSend.ax_raw = 0;
    EspNowComm::toSend.ay_raw = 0;
    EspNowComm::toSend.timestamp = 0;

    dw3000Semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(dw3000Semaphore);
    delay(100);
    strip.setPixelColor(4, strip.Color(255, 0, 0)); // LED 4
    strip.show();

    BaseType_t result = xTaskCreatePinnedToCore(
        dw3000Task, "DW3000_Task", 4096, NULL, 2, &dw3000TaskHandle, 0);

    if (result == pdPASS) {
        Serial.println("[TAG] Tarea DW3000 creada en Core 0");
    } else {
        Serial.printf("[TAG] ERROR: No se pudo crear tarea DW3000 (error %d)\n", result);    
    }
    delay(100);
    strip.setPixelColor(3, strip.Color(255, 0, 0)); // LED 3
    strip.show();

    Serial.println("[TAG] Setup completado");
    delay(100);
    strip.setPixelColor(2, strip.Color(255, 0, 0)); // LED 2
    strip.show();
    
    // Al final del setup, dejar solo el LED 1 encendido
    delay(500); // Pausa para ver la barra completa
    strip.clear();
    strip.setPixelColor(1, strip.Color(255, 0, 0)); // Solo LED 1 queda encendido
    strip.show();
}
// Entrar en modo OTA (one-way hasta reinicio)
void enterOtaMode() {
    if (otaModeActive) return;
    Serial.println("\n[OTA] Activando modo OTA (deshabilitando ESP-NOW)...");
    otaModeActive = true;
    espNowActive = false;

    // Deshabilitar ESP-NOW si estaba activo
    esp_now_deinit();
    Serial.println("[OTA] ESP-NOW deshabilitado");

    // Mantener WiFi en modo STA y conectar a la red
    WiFi.mode(WIFI_STA);
    WiFi.begin(OTA_SSID, OTA_PASS);
    Serial.printf("[OTA] Conectando a WiFi SSID='%s' ...\n", OTA_SSID);

    unsigned long startConnect = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startConnect < 10000) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[OTA] WiFi conectado. IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[OTA] No se pudo conectar a WiFi (continuará intentando en background)");
    }

    // Configurar eventos OTA
    ArduinoOTA.onStart([](){
        Serial.println("[OTA] Inicio de actualización");
    });
    ArduinoOTA.onEnd([](){
        Serial.println("\n[OTA] Actualización completada");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total){
        static unsigned int lastPct = 0;
        unsigned int pct = (progress * 100) / total;
        if (pct != lastPct) {
            Serial.printf("[OTA] Progreso: %u%%\n", pct);
            lastPct = pct;
        }
    });
    ArduinoOTA.onError([](ota_error_t error){
        Serial.printf("[OTA] Error %u: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.setHostname("TAG-UWB-V4");
    ArduinoOTA.begin();
    Serial.println("[OTA] Servidor OTA listo. Mantén este modo hasta completar la actualización.");

    // Encender LED fijo para indicar modo OTA
    strip.fill(strip.Color(0, 0, 255)); // Azul
    strip.show();
}
void updateChargerStatus() {
    static unsigned long lastCheck = 0;
    unsigned long now = millis();
    if (now - lastCheck < 200) return; // Muestrear cada 200ms
    lastCheck = now;

    static int lastState = HIGH;
    static unsigned long lastStateChangeTime = 0;
    static bool isBlinking = false;
    static int blinkCount = 0;
    static ChargerStatus detectedStatus = CHARGER_UNKNOWN;

    int currentState = digitalRead(PIN_STAT);

    if (currentState != lastState) {
        unsigned long duration = now - lastStateChangeTime;
        // Un parpadeo de 1Hz tiene un semi-periodo de 500ms. Damos un margen.
        if (duration > 400 && duration < 600) {
            blinkCount++;
        } else {
            blinkCount = 0; // El tiempo no coincide, reseteamos el contador de parpadeo
        }
        lastStateChangeTime = now;
        lastState = currentState;
        // Consideramos que parpadea si vemos al menos 2 cambios de estado con el timing correcto
        isBlinking = (blinkCount >= 2); 
    }

    // Si el estado ha sido estable por un tiempo, determinamos que no está parpadeando
    if (now - lastStateChangeTime > 1200) {
        isBlinking = false;
        blinkCount = 0;
        if (currentState == LOW) {
            detectedStatus = CHARGER_CHARGING;
        } else { // HIGH (debido al PULLUP)
            detectedStatus = CHARGER_COMPLETE;
        }
    } else if (isBlinking) {
        detectedStatus = CHARGER_FAULT;
    }
    
    if (currentChargerStatus != detectedStatus) {
        currentChargerStatus = detectedStatus;
        Serial.printf("[CHARGER] Nuevo estado: %d\n", currentChargerStatus);
    }
}

void loop() {
    // === CORE 1 - Tareas de monitoreo y control ===
    
    // Lecturas de hardware
    readAnalogAxes();
    readButtons();
    updateChargerStatus();

    // Detección de pulsación larga (10s) para entrar a OTA
    {
        static unsigned long otaPressStart = 0;
        static bool otaPressing = false;
        if (buttons[BTN_IDX_OK].rising) {
            otaPressStart = millis();
            otaPressing = true;
        }
        if (!buttons[BTN_IDX_OK].stable) {
            otaPressing = false;
        }
        if (!otaModeActive && otaPressing && (millis() - otaPressStart >= OTA_LONG_PRESS_MS)) {
            enterOtaMode();
        }
    }

    if (otaModeActive) {
        ArduinoOTA.handle();
        delay(10);
        return;
    }

    processModeChanges();
    calculateJoystickValue();

    unsigned long currentTime = millis();

    // Envío de datos por ESP-NOW a 20Hz
    static unsigned long lastESPNowSend = 0;
    if (currentTime - lastESPNowSend >= 50) {
        if (espNowActive) {
            EspNowComm::toSend.batteryVoltage_mV = (uint16_t)(readBatteryVoltage() * 1000.0);
            EspNowComm::toSend.modo = (uint8_t)currentMode;
            EspNowComm::toSend.ax_raw = (uint16_t)analogDownRaw;
            EspNowComm::toSend.ay_raw = (uint16_t)analogRightRaw;
            EspNowComm::toSend.timestamp = currentTime;
            
            esp_now_send(carroMacAddress, (uint8_t*)&EspNowComm::toSend, sizeof(EspNowComm::EspNowData));
        }
        lastESPNowSend = currentTime;
    }

    // Actualización de LEDs (y logs en el futuro) a una frecuencia más baja
    static unsigned long lastSlowUpdate = 0;
    if (currentTime - lastSlowUpdate > 250) { // 4Hz
        strip.clear(); // Limpiar todos los LEDs antes de re-dibujar

        // --- Lógica de LEDs individuales ---

        // LED 1: Actividad (ya se dejó encendido en setup)
        strip.setPixelColor(1, strip.Color(255, 0, 0));

        // LED 3: Modo de Operación
        switch(currentMode) {
            case MODE_TRACKING: strip.setPixelColor(3, strip.Color(0, 255, 0)); break; // Verde
            case MODE_PAUSE:    strip.setPixelColor(3, strip.Color(255, 0, 0)); break; // Rojo
            case MODE_MANUAL:   strip.setPixelColor(3, strip.Color(0, 0, 255)); break; // Azul
            case MODE_OFF:      // Apagado
            default:            strip.setPixelColor(3, strip.Color(0, 0, 0)); break;
        }

        // LED 5: Estado del Cargador
        switch(currentChargerStatus) {
            case CHARGER_CHARGING:
                strip.setPixelColor(5, strip.Color(255, 0, 0)); // Rojo
                break;
            case CHARGER_FAULT:
                // Parpadeo Azul
                if ((currentTime / 500) % 2 == 0) {
                    strip.setPixelColor(5, strip.Color(0, 0, 255)); // Azul
                } else {
                    strip.setPixelColor(5, strip.Color(0, 0, 0)); // Negro
                }
                break;
            case CHARGER_COMPLETE:
            case CHARGER_UNKNOWN:
            default:
                strip.setPixelColor(5, strip.Color(0, 0, 0)); // Apagado
                break;
        }

        // LED 0: Error UWB
        bool local_carro_detected = false;
        if (xSemaphoreTake(dw3000Semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            local_carro_detected = carro_detected;
            xSemaphoreGive(dw3000Semaphore);
        }
        if (currentMode == MODE_TRACKING && !local_carro_detected) {
            strip.setPixelColor(0, strip.Color(255, 255, 0)); // Amarillo
        }

        strip.show();
        lastSlowUpdate = currentTime;
    }

    // Estadísticas y logs (cada 15 segundos)
    static unsigned long lastStats = 0;
    if (currentTime - lastStats >= 15000) {
        float voltage = readBatteryVoltage();
        const char* chargerStates[] = {"?", "Cargando", "Completa", "Fallo"};

        Serial.printf("[STATUS] Bat: %.2fV (%d%%) | Modo: %d | Cargador: %s | UWB: %s\n",
                      voltage, getBatteryPercentage(voltage), currentMode, chargerStates[currentChargerStatus], carro_detected ? "OK" : "Error");
        
        lastStats = currentTime;
    }

    delay(5); // Pequeño delay para estabilidad
}