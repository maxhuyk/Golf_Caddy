#include "BLEManager.h"
#include "config.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>


extern float dbg_angulo;
extern float dbg_distancia;
extern float dbg_pid_dist;
extern float dbg_pid_ang;

// UUIDs de la App
#define SERVICE_UUID               "12345678-1234-1234-1234-123456789abc"
#define CHAR_TELEMETRY_UUID        "12345678-1234-1234-1234-123456789abd"
#define CHAR_STATUS_UUID           "12345678-1234-1234-1234-123456789abe"
#define CHAR_ERRORS_UUID           "12345678-1234-1234-1234-123456789abf"
#define CHAR_CONFIG_READ_UUID      "12345678-1234-1234-1234-123456789ac0"
#define CHAR_CONFIG_WRITE_UUID     "12345678-1234-1234-1234-123456789ac1"

#pragma pack(push, 1)
struct CarTelemetry {
    uint32_t timestamp;
    uint16_t carro_battery_mV;
    int16_t motor_left_speed;
    int16_t motor_right_speed;
    uint16_t tag_battery_mV;
    uint8_t control_mode;
    bool espnow_connected;
    uint32_t espnow_packets;
    float uwb_distance[3];
    bool uwb_valid[3];
    uint8_t system_errors;
    uint32_t uptime_ms;
    // --- NUEVOS DATOS PARA LA APP ---
    float tag_angulo;
    float tag_distancia;
    float pid_potencia_avance;
    float pid_potencia_giro;
};
#pragma pack(pop)

static CarTelemetry currentTelemetry;
static BLEServer* pServer = NULL;
static BLECharacteristic* pTelemetryChar = NULL;
static BLECharacteristic* pConfigReadChar = NULL;

static bool deviceConnected = false;
static uint32_t lastTelemetryUpdate = 0;

// ✅ INSTANCIA GLOBAL DE PREFERENCES
static Preferences prefs;

// Joystick override BLE
static int ble_joy_l = 0;
static int ble_joy_r = 0;
static unsigned long last_ble_joy_ms = 0;

// Variables dinámicas 
float G_PID_KP = 0.3;                       //250
float G_PID_KI = 0.0;                       //0
float G_PID_KD = 3.0;                       //320
float G_PID_DISTANCIA_KP = 0.07;            //70
float G_PID_DISTANCIA_KI = 0.00;           //50
float G_PID_DISTANCIA_KD = 0.1;            //700
float G_UMBRAL_MINIMO = 0.0;
float G_UMBRAL_MAXIMO = 0.0;
float G_VELOCIDAD_MAXIMA = 100;
float G_DISTANCIA_OBJETIVO = 2000.0;      //3000  
float G_PID_SALIDA_MAX = 100.0;
float G_ACELERACION_MAXIMA = 100.0;
float G_DESACELERACION_MAXIMA = 100.0;    //2  
float G_TRIM_L = 0.98;                    //1
float G_TRIM_R = 1.0;                    //0.93
float G_FILTRO_MODO = 1.0;     // Arranca en el Modo 1 (El tuyo actual)
float G_KALMAN_Q_NUEVO = 0.5;  // Más alto = reacciona más rápido a tus movimientos
float G_KALMAN_R_NUEVO = 0.5;  // Más bajo = confía más en el UWB crudo
float G_PID_KP_MIN = 0.30;              //250
float G_ANTI_TROMPO = 0.0;              //1
float G_FRENADO_SEGURO = 0.0;   
float G_DEADZONE_PWM = 140.0; 
float G_POT_PWM_MIN = 30.0;             //0
float G_CADDIE_INTELIGENTE = 0.0;       // 1
float G_DIST_INTERACCION = 1800.0;      //2000
float G_MODO_GEOMETRIA = 1.0; // 1 = Nuevo 2D con Desempate, 0 = Clásico 3D
float G_STATIC_KP = 400.0;             //300
float G_STATIC_KI = 0.0;              //150
float G_STATIC_KD = 50.0;             //350  
float G_ANGULO_ARRANQUE = 35.0;
float G_DEADZONE_CADDIE = 2.0;       //10
float G_ANTI_WINDUP_DIST = 250.0;
float G_FILTRO_PICO_EN = 1.0;  // Prendido por defecto
float G_FILTRO_ANG_PICO = 8.0;     
float G_MAX_DELTA = 6.0;           //6 grados máximo entre frames (30Hz)
float G_HISTERESIS_Y = 120.0;           // Candado para el andar recto de V2
float G_SELECTOR_ZONA_FILTRO = 2.0;     // 0=Todas, 1=Mayor a Objetivo, 2=Mayor a Caddie


struct ParamMap { 
    const char* name; 
    float* ptr; 
    float minVal; 
    float maxVal; 
    const char* description; 
    float escala;
};

// --- LA HOJA DE TRUCOS DIRECTO EN TU APP ---
static ParamMap params[] = {
    // --- PARÁMETROS DE PID (Escala x1000) ---
    {"PID_KP", &G_PID_KP, 0.0, 5000.0, "Giro: (estatico)+Mide rapido, -Perezoso (x1000)", 1000.0},
    {"PID_KP_MIN", &G_PID_KP_MIN, 0.0, 5000.0, "Kp Mínimo (en Velocidad)(x1000)", 1000.0},
    {"PID_KD", &G_PID_KD, 0.0, 5000.0, "Giro: +Frena clavado, -Hace ZigZag (x1000)", 1000.0},
    {"PID_KI", &G_PID_KI, 0.0, 5000.0, "Giro: +Vence friccion (usar poco) (x1000)", 1000.0},
    {"STAT_KP", &G_STATIC_KP, 0.0, 5000.0, "Avance: +Acelera facil, -Lento (x1000)", 1000.0},
    {"STAT_KD", &G_STATIC_KD, 0.0, 5000.0, "Avance: +Frena antes, -Te choca (x1000)", 1000.0},
    {"STAT_KI", &G_STATIC_KI, 0.0, 5000.0, "Avance: +Fuerza subida colinas (x1000)", 1000.0},
    {"DIST_KP", &G_PID_DISTANCIA_KP, 0.0, 5000.0, "Avance: +Acelera facil, -Lento (x1000)", 1000.0},
    {"DIST_KD", &G_PID_DISTANCIA_KD, 0.0, 5000.0, "Avance: +Frena antes, -Te choca (x1000)", 1000.0},
    {"DIST_KI", &G_PID_DISTANCIA_KI, 0.0, 5000.0, "Avance: +Fuerza subida colinas (x1000)", 1000.0},
    

    // --- EL RESTO DE PARÁMETROS (Escala x1) ---
    {"DEAD_CAD", &G_DEADZONE_CADDIE, 0.0, 90.0, "Tolerancia Caddie (grados)", 1.0},
    {"WIND_DIST", &G_ANTI_WINDUP_DIST, 50.0, 1000.0, "Distancia escape atasco (mm)", 1.0},
    {"UMBRAL_MIN", &G_UMBRAL_MINIMO, 0.0, 30.0, "Zona Muerta Cerca (+Ignora, -Tiembla)", 1.0},
    {"UMBRAL_MAX", &G_UMBRAL_MAXIMO, 0.0, 30.0, "Zona Muerta Lejos (grados)", 1.0},
    {"ANG_ARRAN", &G_ANGULO_ARRANQUE, 0.0, 180.0, "Angulo Arranque", 1.0},
    {"VEL_MAX", &G_VELOCIDAD_MAXIMA, 0.0, 100.0, "Velocidad Tope (%)", 1.0},
    {"DIST_OBJ", &G_DISTANCIA_OBJETIVO, 500.0, 10000.0, "Distancia de Sombra (mm)", 1.0},
    {"PID_SAL_MAX", &G_PID_SALIDA_MAX, 0.0, 100.0, "Poder Max de Giro (%)", 1.0},
    {"ACEL_MAX", &G_ACELERACION_MAXIMA, 0.0, 100.0, "Arranque: +Pique, -Suave", 1.0},
    {"FRENO_MAX", &G_DESACELERACION_MAXIMA, 0.0, 100.0, "Frenado: +Clavado, -Suave", 1.0},
    {"TRIM_L", &G_TRIM_L, 0.5, 1.0, "Fuerza Motor Izq (1.0 = 100%)", 1.0},
    {"TRIM_R", &G_TRIM_R, 0.5, 1.0, "Fuerza Motor Der (1.0 = 100%)", 1.0},
    {"FILTRO_MODO", &G_FILTRO_MODO, 0.0, 2.0, "0=Crudo, 1=Clasico, 2=Rapido", 1.0},
    {"KALMAN_Q2", &G_KALMAN_Q_NUEVO, 0.001, 5.0, "Modo2: Confianza al mov.", 1.0},
    {"KALMAN_R2", &G_KALMAN_R_NUEVO, 0.001, 10.0, "Modo2: Ruido Sensor", 1.0},
    {"ANTI_TROMPO", &G_ANTI_TROMPO, 0.0, 1.0, "1=Cepo al giro, 0=Licuadora", 1.0},
    {"FRENADO_SEG", &G_FRENADO_SEGURO, 0.0, 1.0, "1=Frena ya, 0=Sigue 1 seg", 1.0},
    {"DEADZONE", &G_DEADZONE_PWM, 0.0, 255.0, "Fuerza Arranque Motores (0-255)", 1.0},
    {"POT_MIN", &G_POT_PWM_MIN, 0.0, 100.0, "Minimo PWM Potenciometro (%)", 1.0},
    {"MODO_CADDIE", &G_CADDIE_INTELIGENTE, 0.0, 1.0, "1=Protege palos, 0=Gira libre", 1.0},
    {"DIST_CADDIE", &G_DIST_INTERACCION, 500.0, 4000.0, "Distancia de bloqueo giro (mm)", 1.0},
    {"MODO_GEO", &G_MODO_GEOMETRIA, 0.0, 1.0, "1=2D+Desempate, 0=3D Clasico", 1.0},
    {"FILT_PICO_EN", &G_FILTRO_PICO_EN, 0.0, 1.0, "1=ON Filtro Picos", 1.0},
    {"FILT_PICO", &G_FILTRO_ANG_PICO, 0.0, 60.0, "Pico Maximo (grados)", 1.0},
    {"HIST_Y", &G_HISTERESIS_Y, 0.0, 300.0, "Candado recto (0=Giro en el lugar)", 1.0},
    {"ZONA_FILTRO", &G_SELECTOR_ZONA_FILTRO, 0.0, 2.0, "0=Todas, 1=Lejos, 2=Fuera Caddie", 1.0},
    
};
static const int numParams = sizeof(params)/sizeof(params[0]);

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; Serial.println("[BLE] Celular Conectado"); }
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; pServer->startAdvertising(); Serial.println("[BLE] Celular Desconectado"); }
};

// Función auxiliar para limpiar basura de Android
void limpiarString(std::string &s) {
    while(!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '\0')) s.pop_back();
    while(!s.empty() && (s.front() == '\n' || s.front() == '\r' || s.front() == ' ' || s.front() == '\0')) s.erase(0, 1);
}

class ConfigCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        std::string data = pChar->getValue();
        limpiarString(data); 

        if(data.empty()) return;

        // --- ARREGLO DEL COMANDO LIST ---
        if(data == "LIST") {
            std::string response = "PARAMS:"; // Usamos std::string (es más estable en BLE)
            for(int i=0; i<numParams; i++) {
                response += params[i].name;
                if(i < numParams - 1) response += ",";
            }
            pConfigReadChar->setValue(response);
            pConfigReadChar->notify(); // <-- ¡VITAL! Le avisa a la App que ya puede leer
            Serial.println("[BLE] Lista de parámetros enviada a la App.");
            return;
        }

        size_t colon = data.find(':');
        if(colon == std::string::npos) return;
        
        std::string cmd = data.substr(0, colon);
        std::string param = data.substr(colon+1);
        limpiarString(cmd);
        limpiarString(param);

        if (cmd == "GET") {
            bool found = false;
            for(int i=0; i<numParams; i++) {
                if(param == params[i].name) {
                    char msg[128];
                    // Multiplicamos por la escala solo para mostrarlo en el celular
                    float valor_mostrar = *(params[i].ptr) * params[i].escala;

                    snprintf(msg, sizeof(msg), "%s:%g (%g-%g) %s", params[i].name, valor_mostrar, params[i].minVal, params[i].maxVal, params[i].description);
                    pConfigReadChar->setValue(msg);
                    pConfigReadChar->notify(); // <-- ¡Avisar!
                    found = true;
                    break;
                }
            }
            if(!found) {
                pConfigReadChar->setValue("ERROR: Parametro no encontrado");
                pConfigReadChar->notify();
            }
            return;
        }

        if(cmd == "CMD" && param.find("JOY") == 0) {
            int l, r;
            if(sscanf(param.c_str(), "JOY:%d:%d", &l, &r) == 2) {
                ble_joy_l = l; ble_joy_r = r;
                last_ble_joy_ms = millis();
                pConfigReadChar->setValue("OK:JOY");
                pConfigReadChar->notify(); // <-- ¡Avisar!
            }
            return;
        }

        if(cmd == "SET") {
            size_t c2 = param.find(':');
            if(c2 == std::string::npos) {
                pConfigReadChar->setValue("ERROR: Formato SET invalido");
                pConfigReadChar->notify();
                return;
            }
            
            std::string key = param.substr(0, c2);
            std::string valStr = param.substr(c2+1);
            
            limpiarString(key);
            limpiarString(valStr);

            for (char& c : valStr) {
                if (c == ',') c = '.';
            }

            float val = atof(valStr.c_str()); 

            bool found = false;
            for(int i=0; i<numParams; i++) {
                if(key == params[i].name) {
                    if(val < params[i].minVal) val = params[i].minVal;
                    if(val > params[i].maxVal) val = params[i].maxVal;

                    // ¡MAGIA! Dividimos el valor por la escala antes de inyectarlo en la RAM y en la Flash
                    float valor_interno = val / params[i].escala;

                    *(params[i].ptr) = valor_interno;
                    prefs.putFloat(key.c_str(), valor_interno); // Guardamos el 0.015 real, no el 15.
                    
                    char msg[64]; 
                    snprintf(msg, sizeof(msg), "OK:%s=%g", key.c_str(), val);
                    pConfigReadChar->setValue(msg);
                    pConfigReadChar->notify(); // <-- ¡Avisar!
                    Serial.printf("[BLE] Guardado -> %s\n", msg);
                    found = true;
                    break;
                }
            }
            if (!found) {
                char errMsg[64];
                snprintf(errMsg, sizeof(errMsg), "ERROR: '%s' no existe", key.c_str());
                pConfigReadChar->setValue(errMsg);
                pConfigReadChar->notify(); // <-- ¡Avisar!
            }
        }
    }
};

void BLEManager_init() {
    prefs.begin("car_cfg", false);
    
    for(int i=0; i<numParams; i++) {
        if(prefs.isKey(params[i].name)) {
            *(params[i].ptr) = prefs.getFloat(params[i].name);
        }
    }

    BLEDevice::init("Carro_Golf_UWB");
    
    // --- MAGIA PARA TEXTOS LARGOS ---
    // Obligamos al Bluetooth a permitir mensajes de hasta 512 bytes 
    // en lugar de los 20 por defecto.
    BLEDevice::setMTU(512); 
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pTelemetryChar = pService->createCharacteristic(CHAR_TELEMETRY_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pTelemetryChar->addDescriptor(new BLE2902());

    // --- ARREGLO DE NOTIFICACIONES ---
    // Le sumamos el PROPERTY_NOTIFY y el Descriptor BLE2902 para que el celular reciba alertas
    pConfigReadChar = pService->createCharacteristic(CHAR_CONFIG_READ_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pConfigReadChar->addDescriptor(new BLE2902()); 

    BLECharacteristic *pConfigWriteChar = pService->createCharacteristic(CHAR_CONFIG_WRITE_UUID, BLECharacteristic::PROPERTY_WRITE);
    pConfigWriteChar->setCallbacks(new ConfigCallbacks());

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Servidor iniciado, Memoria cargada y MTU ampliado...");
}

void BLEManager_update(CarroData* data, int vel_L, int vel_R) {
    if (!deviceConnected) return;

    if (millis() - lastTelemetryUpdate > 200) { 
        currentTelemetry.timestamp = millis();
        currentTelemetry.motor_left_speed = vel_L;
        currentTelemetry.motor_right_speed = vel_R;
        currentTelemetry.control_mode = data->control_data[1];
        currentTelemetry.espnow_connected = data->data_valid;
        
        currentTelemetry.tag_battery_mV = (uint16_t)data->control_data[0];
        currentTelemetry.carro_battery_mV = (uint16_t)data->power_data[0];

        for(int i=0; i<3; i++) {
            currentTelemetry.uwb_distance[i] = data->distancias[i];
            currentTelemetry.uwb_valid[i] = (data->distancias[i] > 0);
        }
        currentTelemetry.tag_angulo = dbg_angulo;
        currentTelemetry.tag_distancia = dbg_distancia;
        currentTelemetry.pid_potencia_avance = dbg_pid_dist;
        currentTelemetry.pid_potencia_giro = dbg_pid_ang;

        pTelemetryChar->setValue((uint8_t*)&currentTelemetry, sizeof(CarTelemetry));
        pTelemetryChar->notify();
        lastTelemetryUpdate = millis();
    }
}

bool BLEManager_isManualOverride(int* out_L, int* out_R) {
    if (millis() - last_ble_joy_ms < 500) { 
        *out_L = ble_joy_l;
        *out_R = ble_joy_r;
        return true;
    }
    return false;
}