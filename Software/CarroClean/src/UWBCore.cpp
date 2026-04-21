#include "UWBCore.h"
#include "DW3000.h"
#include <Arduino.h>
#include <math.h>

// Configuración de pines del multiplexor MCP23008
#define ANCHOR1_CS 0     // GP0 del MCP23008
#define ANCHOR1_RST 4    // GP4 del MCP23008
#define ANCHOR1_IRQ 35   // GPIO35 del ESP32 (directo)

#define ANCHOR2_CS 1     // GP1 del MCP23008
#define ANCHOR2_RST 5    // GP5 del MCP23008
#define ANCHOR2_IRQ 27   // GPIO27 del ESP32 (directo)

#define ANCHOR3_CS 2     // GP2 del MCP23008
#define ANCHOR3_RST 6    // GP6 del MCP23008
#define ANCHOR3_IRQ 13   // GPIO13 del ESP32 (directo)

 

// Instancias DW3000
static DW3000Class anchor1(ANCHOR1_CS, ANCHOR1_RST, ANCHOR1_IRQ);
static DW3000Class anchor2(ANCHOR2_CS, ANCHOR2_RST, ANCHOR2_IRQ);
static DW3000Class anchor3(ANCHOR3_CS, ANCHOR3_RST, ANCHOR3_IRQ);

// Variables compartidas entre cores
volatile UWBRawData g_uwb_raw_data = {NAN, NAN, NAN, false, false, false, 0};
volatile bool g_uwb_data_ready = false;
volatile unsigned long g_uwb_measurement_count = 0;
volatile unsigned long g_uwb_last_measurement_time = 0;


// Semáforo binario para acceso thread-safe entre cores
static SemaphoreHandle_t uwbSemaphore = NULL;

// Handle de la tarea
static TaskHandle_t uwbTaskHandle = NULL;

// Contadores de fallo por anchor (opcional: monitoreo)
static uint16_t failCountA1 = 0;
static uint16_t failCountA2 = 0;
static uint16_t failCountA3 = 0;

// Estado y recuperación por anchor
struct AnchorState {
    uint16_t consecTimeouts = 0;     // timeouts consecutivos (posible falta de TAG)
    uint16_t consecErrors = 0;       // errores/frame inválido
    uint16_t consecNotIdle = 0;      // veces seguidas sin IDLE (sospecha trabado)
    unsigned long lastSuccessMs = 0; // última medición válida
    uint8_t softResets = 0;          // conteo de soft resets
    uint8_t hardResets = 0;          // conteo de hard resets
};

static AnchorState a1State, a2State, a3State;

static void anchor_soft_reinit(DW3000Class& D) {
    // Re-configuración mínima sin reiniciar toda la interfaz
    D.softReset();
    delay(200);
    D.init();
    D.setupGPIO();
    D.configureAsTX();
    D.clearSystemStatus();
}

static void try_anchor_recovery(DW3000Class& D, AnchorState& S, const char* name) {
    // Solo considerar "trabado" si el dispositivo no está en IDLE
    if (!D.checkForIDLE()) {
        S.consecNotIdle++;
        if (S.consecNotIdle >= 2) {
            Serial.printf("[UWBCore][RECOVERY] %s no IDLE (%u). Soft reinit...\n", name, S.consecNotIdle);
            anchor_soft_reinit(D);
            S.softResets++;
            S.consecNotIdle = 0; // reset counter tras intento
            delay(100);
            // Si sigue no IDLE después de soft, intentar hard reset (de forma conservadora)
            if (!D.checkForIDLE()) {
                Serial.printf("[UWBCore][RECOVERY] %s sigue no IDLE. Hard reset...\n", name);
                D.hardReset();
                // Esperar IDLE con backoff
                unsigned long t0 = millis();
                while (!D.checkForIDLE() && (millis() - t0) < 1000) {
                    delay(50);
                }
                D.softReset();
                delay(200);
                D.init();
                D.setupGPIO();
                D.configureAsTX();
                D.clearSystemStatus();
                S.hardResets++;
            }
        }
    } else {
        // Si está IDLE, no asumir trabado por falta de respuesta del TAG
        S.consecNotIdle = 0;
    }
}


// Variables globales para almacenar las últimas distancias medidas
static float g_distances[NUM_ANCHORS] = {NAN, NAN, NAN};
static bool g_anchor_status[NUM_ANCHORS] = {false, false, false};


// Ejecuta un ciclo completo de Double-Sided Ranging según el .ino
// Devuelve distancia en cm o NAN si falla (con timeout y manejo de errores)
static float dsr_once(DW3000Class& D, uint16_t& failCount, unsigned long rxTimeoutMs = 5) {
    int stage = 0;
    long long tx = 0, rx = 0;
    int t_roundA = 0, t_replyA = 0;
    int clock_offset = 0;

    // stage 0: iniciar
    D.ds_sendFrame(1);
    tx = D.readTXTimestamp();
    stage = 1;

    // stage 1: esperar primera respuesta
    {
        unsigned long t0 = millis();
        while (true) {
            int rx_status = D.receivedFrameSucc();
            if (rx_status) {
                D.clearSystemStatus();
                if (rx_status == 1) {
                    if (D.ds_isErrorFrame()) {
                        if (failCount < 0xFFFF) failCount++;
                        return NAN; //no vuelve a stage 0
                    } else if (D.ds_getStage() != 2) {
                        D.ds_sendErrorFrame();
                        if (failCount < 0xFFFF) failCount++;
                        return NAN;//no vuelve a stage 0
                    }
                    break; // ok, pasar a stage 2
                } else {
                    if (failCount < 0xFFFF) failCount++;
                    return NAN;
                }
            }
            if (millis() - t0 > rxTimeoutMs) {
                D.clearSystemStatus();
                if (failCount < 0xFFFF) failCount++;
                return NAN;
            }
            // pequeña espera para no saturar bus
            delayMicroseconds(200);
        }
    }

    // stage 2: enviar segundo ranging
    rx = D.readRXTimestamp();
    D.ds_sendFrame(3);
    t_roundA = (int)(rx - tx);
    tx = D.readTXTimestamp();
    t_replyA = (int)(tx - rx);
    stage = 3;

    // stage 3: esperar segunda respuesta
    {
        unsigned long t0 = millis();
        while (true) {
            int rx_status = D.receivedFrameSucc();
            if (rx_status) {
                D.clearSystemStatus();
                if (rx_status == 1) {
                    if (D.ds_isErrorFrame()) {
                        if (failCount < 0xFFFF) failCount++;
                        return NAN;//no vuelve a stage 0
                    }
                    clock_offset = D.getRawClockOffset();
                    break; // ok, pasar a stage 4
                } else {
                    if (failCount < 0xFFFF) failCount++;
                    D.clearSystemStatus();
                    return NAN;
                }
            }
            if (millis() - t0 > rxTimeoutMs) {
                D.clearSystemStatus();
                if (failCount < 0xFFFF) failCount++;
                return NAN;
            }
            delayMicroseconds(200);
        }
    }

    // stage 4: calcular
    int t_roundB = D.read(0x12, 0x04);
    int t_replyB = D.read(0x12, 0x08);
    int ranging_time = D.ds_processRTInfo(t_roundA, t_replyA, t_roundB, t_replyB, clock_offset);
    float distance = D.convertToCM(ranging_time);
    if (distance < 0 || distance > 100000) distance = NAN;
    return distance;
}

void UWBCore_setup() {
    // Crear semáforo binario para sincronización entre cores
    uwbSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(uwbSemaphore); // Inicializar como disponible
    
    Serial.println("[UWBCore] Inicializando anchors con timings para multiplexor MCP23008...");
    
    // TIMING CRÍTICO: El MCP23008 introduce ~2-5ms de latencia en I2C
    // Necesitamos delays más largos entre operaciones SPI consecutivas
    
    // Inicializar Anchor 1
    Serial.println("[UWBCore] Inicializando Anchor 1...");
    anchor1.begin();
    //delay(50);  // Estabilización multiplexor
    anchor1.hardReset();
    //delay(500); // CRÍTICO: 500ms para reset a través de multiplexor
    while (!anchor1.checkForIDLE()) {
        delay(200); // Mayor tiempo entre chequeos
    }
    anchor1.softReset();
    delay(200); // CRÍTICO: 200ms después de soft reset
    anchor1.init();
    //delay(100); // Estabilización post-init
    anchor1.setupGPIO();
    //delay(50);  // Estabilización GPIO
    anchor1.configureAsTX();
    //delay(50);  // Estabilización TX config
    anchor1.clearSystemStatus();
    //delay(100); // Pausa antes del siguiente anchor

    // Inicializar Anchor 2
    Serial.println("[UWBCore] Inicializando Anchor 2...");
    anchor2.begin();
    //delay(50);  // Estabilización multiplexor
    anchor2.hardReset();
    //delay(500); // CRÍTICO: 500ms para reset a través de multiplexor
    while (!anchor2.checkForIDLE()) {
        delay(200); // Mayor tiempo entre chequeos
    }
    anchor2.softReset();
    delay(200); // CRÍTICO: 200ms después de soft reset
    anchor2.init();
    //delay(100); // Estabilización post-init
    anchor2.setupGPIO();
    //delay(50);  // Estabilización GPIO
    anchor2.configureAsTX();
    //delay(50);  // Estabilización TX config
    anchor2.clearSystemStatus();
    //delay(100); // Pausa antes del siguiente anchor

    // Inicializar Anchor 3
    Serial.println("[UWBCore] Inicializando Anchor 3...");
    anchor3.begin();
    //delay(50);  // Estabilización multiplexor
    anchor3.hardReset();
    //delay(500); // CRÍTICO: 500ms para reset a través de multiplexor
    while (!anchor3.checkForIDLE()) {
        delay(200); // Mayor tiempo entre chequeos
    }
    anchor3.softReset();
    delay(200); // CRÍTICO: 200ms después de soft reset
    anchor3.init();
    //delay(100); // Estabilización post-init
    anchor3.setupGPIO();
    //delay(50);  // Estabilización GPIO
    anchor3.configureAsTX();
    //delay(50);  // Estabilización TX config
    anchor3.clearSystemStatus();
    //delay(100); // Estabilización final

    Serial.println("[UWBCore] Todos los anchors inicializados con timings optimizados");
}

void UWBCore_task(void* parameter) {
    
    // Inicializar tiempos de último éxito para no disparar recovery al inicio
    unsigned long now0 = millis();
    a1State.lastSuccessMs = now0;
    a2State.lastSuccessMs = now0;
    a3State.lastSuccessMs = now0;

    while (true) {
        // Delays entre anchors y ritmo global
        const int inter_anchor_delay_us = 1000; // 15ms entre anchors (MCP23008 necesita holgura)
        const int cycle_delay_ms = 1;           
        /*
        Con esta configuracion de delays donde en total suman 45ms se obtiene una frecuencia de 9Hz 
        si se eliminan los delays se podria obtener una frecuencia maxima de 15Hz 
        probar eliminar los delays
        */
        // ANCHOR 1: verificación y posible recovery si está no IDLE
        try_anchor_recovery(anchor1, a1State, "A1");
        // ciclo completo
        float d1 = dsr_once(anchor1, failCountA1);
        if (!isnan(d1)) {
            a1State.lastSuccessMs = millis();
            a1State.consecTimeouts = 0;
            a1State.consecErrors = 0;
        } else {
            // Diferenciar falta de TAG (IDLE) de trabado (no IDLE)
            if (!anchor1.checkForIDLE()) {
                a1State.consecNotIdle++;
            }
            a1State.consecTimeouts++;
        }
        delayMicroseconds(inter_anchor_delay_us);

        // ANCHOR 2
        try_anchor_recovery(anchor2, a2State, "A2");
        float d2 = dsr_once(anchor2, failCountA2);
        if (!isnan(d2)) {
            a2State.lastSuccessMs = millis();
            a2State.consecTimeouts = 0;
            a2State.consecErrors = 0;
        } else {
            if (!anchor2.checkForIDLE()) {
                a2State.consecNotIdle++;
            }
            a2State.consecTimeouts++;
        }
        delayMicroseconds(inter_anchor_delay_us);

        // ANCHOR 3
        try_anchor_recovery(anchor3, a3State, "A3");
        float d3 = dsr_once(anchor3, failCountA3);
        if (!isnan(d3)) {
            a3State.lastSuccessMs = millis();
            a3State.consecTimeouts = 0;
            a3State.consecErrors = 0;
        } else {
            if (!anchor3.checkForIDLE()) {
                a3State.consecNotIdle++;
            }
            a3State.consecTimeouts++;
        }

        // Recovery adicional por ausencia prolongada de éxitos + no IDLE repetido
        unsigned long nowMs2 = millis();
        if ((nowMs2 - a1State.lastSuccessMs) > 10000 && a1State.consecNotIdle >= 2) {
            Serial.println("[UWBCore][RECOVERY] A1 sin éxitos >10s y no IDLE: reinit");
            anchor_soft_reinit(anchor1);
            a1State.softResets++;
            a1State.consecNotIdle = 0;
        }
        if ((nowMs2 - a2State.lastSuccessMs) > 10000 && a2State.consecNotIdle >= 2) {
            Serial.println("[UWBCore][RECOVERY] A2 sin éxitos >10s y no IDLE: reinit");
            anchor_soft_reinit(anchor2);
            a2State.softResets++;
            a2State.consecNotIdle = 0;
        }
        if ((nowMs2 - a3State.lastSuccessMs) > 10000 && a3State.consecNotIdle >= 2) {
            Serial.println("[UWBCore][RECOVERY] A3 sin éxitos >10s y no IDLE: reinit");
            anchor_soft_reinit(anchor3);
            a3State.softResets++;
            a3State.consecNotIdle = 0;
        }

        // Publicar resultados en la estructura compartida (thread-safe)
        bool v1 = !isnan(d1);
        bool v2 = !isnan(d2);
        bool v3 = !isnan(d3);
        unsigned long nowMs = millis();
    if (xSemaphoreTake(uwbSemaphore, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_uwb_raw_data.distance1 = d1;
            g_uwb_raw_data.distance2 = d2;
            g_uwb_raw_data.distance3 = d3;
            g_uwb_raw_data.valid1 = v1;
            g_uwb_raw_data.valid2 = v2;
            g_uwb_raw_data.valid3 = v3;
            g_uwb_raw_data.timestamp = nowMs;
            g_uwb_data_ready = true;
            g_uwb_measurement_count++;
            g_uwb_last_measurement_time = nowMs;
            xSemaphoreGive(uwbSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(cycle_delay_ms));
    }
}

void UWBCore_startTask() {
    Serial.println("[UWBCore] Iniciando tarea UWB en Core 0...");
    
    // Crear tarea en Core 0 con alta prioridad
    BaseType_t result = xTaskCreatePinnedToCore(
        UWBCore_task,        // Función de la tarea
        "UWB_Core_Task",     // Nombre de la tarea
        4096,                // Tamaño del stack
        NULL,                // Parámetros
        2,                   // Prioridad (alta)
        &uwbTaskHandle,      // Handle de la tarea
        0                    // Core 0
    );
    
    if (result == pdPASS) {
        Serial.println("[UWBCore] Tarea UWB creada exitosamente en Core 0");
    } else {
        Serial.printf("[UWBCore] ERROR: No se pudo crear la tarea UWB (error %d)\n", result);
    }
}

bool UWBCore_getRawData(UWBRawData& data) {
    if (xSemaphoreTake(uwbSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
        bool is_new = g_uwb_data_ready; // Guardamos si la data es nueva
        if (is_new) {
            data.distance1 = g_uwb_raw_data.distance1;
            data.distance2 = g_uwb_raw_data.distance2;
            data.distance3 = g_uwb_raw_data.distance3;
            data.valid1 = g_uwb_raw_data.valid1;
            data.valid2 = g_uwb_raw_data.valid2;
            data.valid3 = g_uwb_raw_data.valid3;
            data.timestamp = g_uwb_raw_data.timestamp;
            g_uwb_data_ready = false; 
        }
        xSemaphoreGive(uwbSemaphore);
        return is_new; // ¡VITAL! Solo devuelve true si consumió data fresca
    }
    return false;
}

bool UWBCore_update() {
    // WATCHDOG: Si el Core 0 se cuelga por más de 500ms, matamos los sensores
    if (millis() - g_uwb_last_measurement_time > 500) {
        g_anchor_status[0] = false;
        g_anchor_status[1] = false;
        g_anchor_status[2] = false;
    }

    UWBRawData rawData = {NAN, NAN, NAN, false, false, false, 0};
    
    if (UWBCore_getRawData(rawData)) {
        g_distances[0] = rawData.distance1;
        g_distances[1] = rawData.distance2;
        g_distances[2] = rawData.distance3;
        g_anchor_status[0] = rawData.valid1;
        g_anchor_status[1] = rawData.valid2;
        g_anchor_status[2] = rawData.valid3;
        return true;
    }
    return false; 
}

void UWBCore_getDistances(float distances[NUM_ANCHORS]) {
    for (int i = 0; i < NUM_ANCHORS; i++) {
        distances[i] = g_distances[i];
    }
}

void UWBCore_getAnchorStatus(bool status[NUM_ANCHORS]) {
    for (int i = 0; i < NUM_ANCHORS; i++) {
        status[i] = g_anchor_status[i];
    }
}

unsigned long UWBCore_getMeasurementCount() {
    // Thread-safe access al contador
    unsigned long count = 0;
    if (xSemaphoreTake(uwbSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
        count = g_uwb_measurement_count;
        xSemaphoreGive(uwbSemaphore);
    }
    return count;
}