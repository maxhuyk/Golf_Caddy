#ifndef CONFIG_H
#define CONFIG_H

// #################################################################
// CONFIGURACIÓN DE SENSORES Y FILTROS (Fijos)
// #################################################################
#define N_SENSORES 3
#define MEDIA_MOVIL_VENTANA 3

#define KALMAN_Q 1e-2
#define KALMAN_R 0.1

#define MAX_DELTA_POS 100   // Delta máximo mm entre muestras UWB
extern float G_MAX_DELTA;

// #################################################################
// RECUPERACIÓN RÁPIDA TRAS PÉRDIDA DE SEÑAL
// #################################################################
#define RECOVERY_GRACE_MS 700
#define RECOVERY_K_CYCLES 20
#define RECOVERY_P_INIT 1e6
#define RECOVERY_Q_TEMP 1.0f
#define RECOVERY_DELTA_POS 1000

// =========================================================
// VARIABLES DINÁMICAS (Modificables en vivo por Bluetooth)
// =========================================================
extern float G_PID_KP;
extern float G_PID_KI;
extern float G_PID_KD;
extern float G_STATIC_KP;
extern float G_STATIC_KI;
extern float G_STATIC_KD;
extern float G_PID_DISTANCIA_KP;
extern float G_PID_DISTANCIA_KI;
extern float G_PID_DISTANCIA_KD;
extern float G_UMBRAL_MINIMO;
extern float G_UMBRAL_MAXIMO;
extern float G_VELOCIDAD_MAXIMA;
extern float G_DISTANCIA_OBJETIVO;
extern float G_PID_SALIDA_MAX;
extern float G_ACELERACION_MAXIMA;
extern float G_DESACELERACION_MAXIMA;
extern float G_TRIM_L;
extern float G_TRIM_R;
extern float G_FILTRO_MODO;
extern float G_KALMAN_Q_NUEVO;
extern float G_KALMAN_R_NUEVO;
extern float G_PID_KP_MIN;
extern float G_ANTI_TROMPO;
extern float G_FRENADO_SEGURO;
extern float G_DEADZONE_PWM;
extern float G_POT_PWM_MIN;
extern float G_CADDIE_INTELIGENTE;
extern float G_DIST_INTERACCION;
extern float G_MODO_GEOMETRIA;
extern float G_ANGULO_ARRANQUE;
extern float G_DEADZONE_CADDIE;
extern float G_ANTI_WINDUP_DIST;
extern float G_FILTRO_PICO_EN;
extern float G_FILTRO_ANG_PICO;
extern float G_HISTERESIS_Y;
extern float G_SELECTOR_ZONA_FILTRO;

// El truco de magia (255 -> 100%)
#define VELOCIDAD_MAXIMA ((G_VELOCIDAD_MAXIMA / 255.0) * 100.0) 

// Mapeos
#define PID_KP G_PID_KP
#define PID_KI G_PID_KI
#define PID_KD G_PID_KD
#define PID_DISTANCIA_KP G_PID_DISTANCIA_KP
#define PID_DISTANCIA_KI G_PID_DISTANCIA_KI
#define PID_DISTANCIA_KD G_PID_DISTANCIA_KD
#define UMBRAL_MINIMO G_UMBRAL_MINIMO
#define UMBRAL_MAXIMO G_UMBRAL_MAXIMO
#define DISTANCIA_OBJETIVO G_DISTANCIA_OBJETIVO
#define PID_SALIDA_MAX G_PID_SALIDA_MAX
#define ACELERACION_MAXIMA G_ACELERACION_MAXIMA
#define DESACELERACION_MAXIMA G_DESACELERACION_MAXIMA

// #################################################################
// PARÁMETROS PID ESTÁTICOS
// #################################################################
#define PID_SETPOINT 0.0
#define PID_ALPHA 0.5       
#define INTEGRAL_MAX 20.0   
#define PID_DISTANCIA_ALPHA 0.5
#define PID_DISTANCIA_INTEGRAL_MAX 80.0  
#define DISTANCIA_UMBRAL_MIN 1500.0   
#define DISTANCIA_UMBRAL_MAX 10000.0  

// #################################################################
// CONTROL DE VELOCIDAD Y ACELERACIÓN ESTÁTICAS
// #################################################################
//#define ACELERACION_MAXIMA 6   // Incremento de PWM por frame (30Hz * 4 = 120 PWM/seg)
//#define DESACELERACION_MAXIMA 8 // Frenado más rápido que aceleración para seguridad

// #################################################################
// CONTROL MANUAL Y FALLBACK
// #################################################################
#define VELOCIDAD_MANUAL 80
#define DISTANCIA_FALLBACK 2500
#define VELOCIDAD_FALLBACK 15
#define TIMEOUT_FALLBACK 1.0

// Posiciones de sensores (Se definen en main.cpp o calculos.cpp)
extern const float SENSOR_POSICIONES[3][3];

#endif // CONFIG_H