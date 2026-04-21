#include <Arduino.h>
#include <Wire.h>
#include "DW3000.h"
#include "UWBCore.h"
#include <esp_now.h>
#include <WiFi.h>
#include "MotorController.h"
#include "control.h"
#include "config.h"
#include "BLEManager.h"


// === CONFIGURACIÓN BATERÍA CARRO ===
#define CAR_BATTERY_ADC_PIN 34
#define CAR_BATTERY_DIVIDER ((51.0f + 15.0f) / 15.0f) // Factor de escala 4.4
#define ADC_REF_VOLT 3.3f
#define ADC_MAX 4095

uint16_t readCarBatteryMilliVolts() {
    int sum = 0;
    for(int i = 0; i < 4; i++) { sum += analogRead(CAR_BATTERY_ADC_PIN); }
    float rawAvg = (float)sum / 4.0f;
    float vAdc = (rawAvg * ADC_REF_VOLT) / ADC_MAX;
    return (uint16_t)round(vAdc * CAR_BATTERY_DIVIDER * 1000.0f);
}
// ===================================

// Namespace para encapsular estado de comunicación ESP-NOW
namespace EspNowComm {
    typedef struct {
        uint8_t  version = 2;
        uint16_t batteryVoltage_mV;
        uint8_t  modo;
        uint16_t ax_raw;
        uint16_t ay_raw;
        uint32_t timestamp;
    } __attribute__((packed)) EspNowData;

    volatile EspNowData receivedData = {};
    volatile bool newDataReceived = false;
    volatile unsigned long lastDataReceived = 0;
    volatile unsigned long totalPacketsReceived = 0;
    volatile int16_t manualVelL = 0;
    volatile int16_t manualVelR = 0;
}

int current_motor_L = 0;
int current_motor_R = 0;
// Variables globales de estado
CarroData sharedCarroData = {0};
SemaphoreHandle_t dataMutex = NULL;

// Configuración del Timer de Control
const unsigned long CONTROL_PERIOD_MS = 33; // 33ms ≈ 30.3 Hz
unsigned long lastControlExecutionTime = 0;

// Declaraciones de funciones
void printControlStatus();
void motorControlCallback(float velocidad_izq, float velocidad_der);
bool initESPNOW();

// Callback ESP-NOW
void onDataReceived(const uint8_t * mac, const uint8_t *incomingData, int len) {
    if (len != sizeof(EspNowComm::EspNowData)) return;
    memcpy((void*)&EspNowComm::receivedData, incomingData, sizeof(EspNowComm::EspNowData));
    
    if (EspNowComm::receivedData.version != 2) return;

    EspNowComm::newDataReceived = true;
    EspNowComm::lastDataReceived = millis();
    EspNowComm::totalPacketsReceived++;

    // Mapeo de ejes a velocidades (Lógica Proporcional)
    constexpr int RAW_MIN = 600;
    constexpr int RAW_MAX = 3400;
    constexpr int RAW_CENTER = 2000;
    constexpr int RAW_DEAD = 200;

    auto mapAxis = [](int raw){
        if (raw < RAW_CENTER - RAW_DEAD) {
            float norm = (float)(raw - (RAW_CENTER - RAW_DEAD)) / (float)(RAW_MIN - (RAW_CENTER - RAW_DEAD));
            return -constrain(norm, 0.0f, 1.0f);
        } else if (raw > RAW_CENTER + RAW_DEAD) {
            float norm = (float)(raw - (RAW_CENTER + RAW_DEAD)) / (float)(RAW_MAX - (RAW_CENTER + RAW_DEAD));
            return constrain(norm, 0.0f, 1.0f);
        }
        return 0.0f;
    };

    float xNorm = mapAxis(EspNowComm::receivedData.ax_raw); 
    float yNorm = mapAxis(EspNowComm::receivedData.ay_raw); 

    const float maxManual = (float)VELOCIDAD_MANUAL;
    float vBase = yNorm * maxManual;
    float vTurn = xNorm * maxManual;

    EspNowComm::manualVelL = (int16_t)constrain(vBase - vTurn, -G_VELOCIDAD_MAXIMA, G_VELOCIDAD_MAXIMA);
    EspNowComm::manualVelR = (int16_t)constrain(vBase + vTurn, -G_VELOCIDAD_MAXIMA, G_VELOCIDAD_MAXIMA);
}

void setup() {
    Serial.begin(500000);
    delay(1000);

    // Inicializamos el pin de la batería
    analogReadResolution(12);
    pinMode(CAR_BATTERY_ADC_PIN, INPUT);

    Wire.begin(5, 4);
    Wire.setClock(500000);
    Wire.setTimeout(50); // Si el I2C se traba, se libera a los 50ms evitando que el ESP32 colapse

    if (!initESPNOW()) return;
    if (!DW3000Class::initMCP23008()) return;

    // Inicializar UWB en Core 0
    UWBCore_setup();
    UWBCore_startTask();
    
    // Motores
    setupMotorController();
    setupMotorControlDirect();
    
    // Mutex y Control
    dataMutex = xSemaphoreCreateMutex();
    control_init();
    
    // Iniciar el Servidor Bluetooth
    BLEManager_init();
    
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.println("[SYSTEM] Ready - Control Loop set to 30Hz");
}

void loop() {
    // 1. ADQUISICIÓN ASÍNCRONA
    UWBCore_update();

    // 2. CONTROL DETERMINISTA (30Hz)
    unsigned long currentTime = millis();
    if (currentTime - lastControlExecutionTime >= CONTROL_PERIOD_MS) {
        lastControlExecutionTime = currentTime;

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            
            // --- ACTUALIZAR VALIDEZ DE COMUNICACIÓN (SIEMPRE) ---
            // Esto asegura que el reporte de "Tag Valid" refleje la realidad del ESP-NOW
            bool tagCommValid = (currentTime - EspNowComm::lastDataReceived) < 500;
            sharedCarroData.data_valid = tagCommValid;

            // --- ACTUALIZAR MODO, JOYSTICK Y BATERÍAS ---
            // 1. Batería del TAG (Viene por ESP-NOW) -> La guardamos en control_data[0]
            sharedCarroData.control_data[0] = EspNowComm::receivedData.batteryVoltage_mV;
            sharedCarroData.control_data[1] = EspNowComm::receivedData.modo;

            // 2. Batería del CARRO (La leemos localmente) -> La guardamos en power_data[0]
            sharedCarroData.power_data[0] = readCarBatteryMilliVolts();

            // 3. Joystick
            sharedCarroData.buttons_data[2] = EspNowComm::manualVelL;
            sharedCarroData.buttons_data[3] = EspNowComm::manualVelR;

            // --- ACTUALIZAR DISTANCIAS UWB ---
            float distances[NUM_ANCHORS];
            bool anchor_status[NUM_ANCHORS];
            UWBCore_getDistances(distances);
            UWBCore_getAnchorStatus(anchor_status);

            for (int i = 0; i < 3; i++) {
                sharedCarroData.distancias[i] = anchor_status[i] ? (distances[i] * 10.0) : -1.0;
            }

            // --- EJECUTAR CONTROL ---
            control_main(&sharedCarroData, motorControlCallback, [](){ 
                current_motor_L = 0; 
                current_motor_R = 0; 
                motor_detener(); 
            });

            // --- ENVIAR TELEMETRÍA SIEMPRE (Incluso detenido) ---
            BLEManager_update(&sharedCarroData, current_motor_L, current_motor_R);

            xSemaphoreGive(dataMutex);
        }
    }

    // 3. DIAGNÓSTICO
    printControlStatus();
}

void motorControlCallback(float velocidad_izq, float velocidad_der) {
    // 🛠️ Multiplicamos la velocidad que pide el código por el Trim configurado
    current_motor_L = (int)(velocidad_izq * G_TRIM_L);
    current_motor_R = (int)(velocidad_der * G_TRIM_R);
    
    motor_enviar_pwm(current_motor_L, current_motor_R);
}

bool initESPNOW() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(onDataReceived);
    return true;
}

void printControlStatus() {
    static unsigned long lastDisplay = 0;
    if (millis() - lastDisplay >= 10000) {
        lastDisplay = millis();
        unsigned long uwbCount = UWBCore_getMeasurementCount();
        Serial.printf("\n--- SYSTEM STATUS (30Hz Loop) ---\n");
        Serial.printf("UWB Measurements: %lu\n", uwbCount);
        Serial.printf("Free Heap: %u bytes\n", esp_get_free_heap_size());
        
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            Serial.printf("Current Mode: %.0f | Tag Valid: %s\n", 
                sharedCarroData.control_data[1], sharedCarroData.data_valid ? "YES" : "NO");
            xSemaphoreGive(dataMutex);
        }
        Serial.println("---------------------------------");
    }
}