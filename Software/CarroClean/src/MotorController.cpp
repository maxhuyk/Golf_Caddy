#include "MotorController.h"
#include <ArduinoJson.h>
#include "config.h"

#define ENABLE_MOTORS 14  
#define PWML1 26
#define PWML2 25  
#define PWMR1 33
#define PWMR2 32
#define CURRR 39  

#define PWM_FREQ 20000  
#define PWM_RES 8       
#define CH_L1 0
#define CH_L2 1
#define CH_R1 2
#define CH_R2 3

static bool motorsEnabled = false;
static bool pwm_initialized = false;
static unsigned long lastCommandTime = 0;
static const unsigned long COMMAND_TIMEOUT = 2000; 
static bool directControlMode = false; 

// --- ZONA MUERTA ELÉCTRICA --- 
// 100 sobre 255 = ~39% de potencia mínima. Si tus motores aún no giran en 1%, súbelo a 75.
//const int DEADZONE_PWM = 100; 

void disableMotors() {
    digitalWrite(ENABLE_MOTORS, LOW);
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, 0);
    ledcWrite(CH_R1, 0);
    ledcWrite(CH_R2, 0);
}

void setMotorL(int pct, bool dir) {
    int duty = map(constrain(pct, 0, 100), 0, 100, 0, 255);
    if (duty > 0) {
        // Mapeo lineal: El 1% empieza en la Zona Muerta, superando la fricción.
        duty = map(duty, 1, 255, (int)G_DEADZONE_PWM, 255); 
        digitalWrite(ENABLE_MOTORS, HIGH);
    }
    
    if (dir) {
        ledcWrite(CH_L1, duty);
        ledcWrite(CH_L2, 0);
    } else {
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, duty);
    }
}

void setMotorR(int pct, bool dir) {
    int duty = map(constrain(pct, 0, 100), 0, 100, 0, 255);
    if (duty > 0) {
        duty = map(duty, 1, 255, (int)G_DEADZONE_PWM, 255);
        digitalWrite(ENABLE_MOTORS, HIGH);
    }
    
    if (dir) {
        ledcWrite(CH_R1, duty);
        ledcWrite(CH_R2, 0);
    } else {
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, duty);
    }
}

void setupMotorPWM() {
    pinMode(ENABLE_MOTORS, OUTPUT);
    digitalWrite(ENABLE_MOTORS, LOW);
    if (!pwm_initialized) {
        ledcSetup(CH_L1, PWM_FREQ, PWM_RES);
        ledcSetup(CH_L2, PWM_FREQ, PWM_RES);
        ledcSetup(CH_R1, PWM_FREQ, PWM_RES);
        ledcSetup(CH_R2, PWM_FREQ, PWM_RES);
        ledcAttachPin(PWML1, CH_L1);
        ledcAttachPin(PWML2, CH_L2);
        ledcAttachPin(PWMR1, CH_R1);
        ledcAttachPin(PWMR2, CH_R2);
        pwm_initialized = true;
    }
}

void motor_enviar_pwm(int izq, int der) {
    lastCommandTime = millis();
    int L_pct = constrain(izq, -100, 100);
    int R_pct = constrain(der, -100, 100);

    setMotorL(abs(L_pct), L_pct >= 0);
    setMotorR(abs(R_pct), R_pct >= 0);

    motorsEnabled = (L_pct != 0 || R_pct != 0);
    
    static int lastL = 0, lastR = 0;
    if (abs(L_pct - lastL) > 5 || abs(R_pct - lastR) > 5) {
        if (motorsEnabled) Serial.printf("[MOTORES] L:%d%% R:%d%%\n", L_pct, R_pct);
        lastL = L_pct; lastR = R_pct;
    }
}

void motor_detener() { disableMotors(); motorsEnabled = false; }
void setupMotorController() { setupMotorPWM(); motor_detener(); }
void setupMotorControlDirect() { directControlMode = true; }