#include "config.h"
#include "calculos.h"
#include "MotorController.h"
#include "control.h"
#include <Arduino.h>
#include <cmath>

// Variables globales para monitoreo Bluetooth
float dbg_angulo = 0;
float dbg_distancia = 0;
float dbg_pid_dist = 0;
float dbg_pid_ang = 0;


// === VARIABLES DEL POTENCIÓMETRO (MODO EMERGENCIA) ===
static bool emergency_active = false;
const int PIN_POTENCIOMETRO = 36;
const int POT_THRESHOLD_ON = 900;
const int POT_THRESHOLD_OFF = 700;
const int POT_DEADZONE = 0;
// =====================================================


static bool control_initialized = false;
static PIDController pid_angular;
static PIDController pid_distancia;
static float distancias_previas[3] = {0, 0, 0};
static bool distancias_previas_valid = false;
static float angulo_anterior = 0;
static float velocidad_actual = 0;
static uint8_t modo = 0;
static unsigned long ultimo_frame_valido_ms = 0;
static unsigned long last_log_time = 0;
const unsigned long LOG_INTERVAL_MS = 200; 
const float PWM_MINIMO_MOVIMIENTO = 25.0; 

static bool signal_lost = false;
static unsigned long signal_lost_time = 0;
static float last_valid_distance = 0;
static float last_valid_correction = 0;
static bool in_recovery = false;
static unsigned long recovery_start_ms = 0;
static int recovery_cycles = 0;
static float saved_normal_Q = KALMAN_Q;

void control_init() {
    setupMotorControlDirect();
    setupMotorControlDirect();
    
    // Inicializar lectura del potenciómetro
    pinMode(PIN_POTENCIOMETRO, INPUT);
    media_movil_init(3, MEDIA_MOVIL_VENTANA);
    kalman_init(3, KALMAN_Q, KALMAN_R);
    pid_init(&pid_angular, 1.2, 0.01, 0.3, 0.0, PID_ALPHA, PID_SALIDA_MAX, INTEGRAL_MAX);
    pid_init(&pid_distancia, PID_DISTANCIA_KP, PID_DISTANCIA_KI, PID_DISTANCIA_KD, 
             DISTANCIA_OBJETIVO, PID_DISTANCIA_ALPHA, 
             VELOCIDAD_MAXIMA, PID_DISTANCIA_INTEGRAL_MAX);
    
    distancias_previas_valid = false;
    velocidad_actual = 0;
    control_initialized = true;
    Serial.println("[CONTROL] Sistema a 30Hz Anti-Pánico inicializado");
}

void control_main(CarroData* data, PWMCallback enviar_pwm, StopCallback detener) {
    if (!control_initialized || data == nullptr) return;

    unsigned long current_time = millis();

    // === 1. GAIN SCHEDULING (KP ADAPTATIVO) ===
    // Calculamos qué tan rápido va el carro respecto al máximo (0.0 a 1.0)
    float v_ref = (G_VELOCIDAD_MAXIMA > 0) ? G_VELOCIDAD_MAXIMA : 255.0f;
    float factor_v = abs(velocidad_actual) / v_ref;
    factor_v = constrain(factor_v, 0.0f, 1.0f);

    // Interpolación: A más velocidad, menos Kp.
    float kp_dinamico = G_PID_KP - (factor_v * (G_PID_KP - G_PID_KP_MIN));
    
    // Aplicamos los valores actualizados al PID
    pid_angular.kp = max(G_PID_KP_MIN, kp_dinamico); // Seguridad para no bajar del mínimo
    pid_angular.ki = G_PID_KI;
    pid_angular.kd = G_PID_KD;
    pid_angular.salida_maxima = G_PID_SALIDA_MAX;

    // Sincronización PID Distancia
    pid_distancia.kp = G_PID_DISTANCIA_KP;
    pid_distancia.ki = G_PID_DISTANCIA_KI;
    pid_distancia.kd = G_PID_DISTANCIA_KD;
    pid_distancia.setpoint = G_DISTANCIA_OBJETIVO;
    pid_distancia.salida_maxima = G_VELOCIDAD_MAXIMA;
    // === 2. MODO EMERGENCIA (POTENCIÓMETRO) ===
    int raw_pot = analogRead(PIN_POTENCIOMETRO);
    if (!emergency_active) {
        if (raw_pot >= POT_THRESHOLD_ON + POT_DEADZONE) emergency_active = true;
    } else {
        if (raw_pot < POT_THRESHOLD_OFF) {
            emergency_active = false;
            detener();
            velocidad_actual = 0;
        }
    }

    // 2. Control de Potencia
    if (emergency_active) {
        // ZONA MUERTA MECÁNICA: 200 puntos de margen después del "click" 
        // para absorber el ruido y el inicio del recorrido de la pista de carbón.
        int inicio_giro = POT_THRESHOLD_ON + 200; // 1100
        
        int pwm_pot = 0; // Guillotina: Por defecto es 0 absoluto.
        
        // Si giraste la perilla más allá de la zona muerta, calculamos la potencia
        if (raw_pot > inicio_giro) {
            int clamped = constrain(raw_pot, inicio_giro, 4095);
            pwm_pot = map(clamped, inicio_giro, 4095, (int)G_POT_PWM_MIN, (int)G_VELOCIDAD_MAXIMA);
        }
        
        // Enviamos la orden (negativa porque es reversa de emergencia)
        enviar_pwm(-pwm_pot, -pwm_pot);
        return; 
    }
        
    // ##############################################################

    modo = (uint8_t)data->control_data[1];
    int manual_L = (int)data->buttons_data[2];
    int manual_R = (int)data->buttons_data[3];

    bool esp_now_valid = data->data_valid;
    int anchors_perdidos = 0;

    // --- PASO 1: PRE-PROCESAMIENTO (Tolerancia a pérdida de frames) ---
    for(int i = 0; i < 3; i++) {
        if(isnan(data->distancias[i]) || data->distancias[i] <= 0) {
            anchors_perdidos++;
            if (distancias_previas_valid) {
                // Si falla un sensor, usamos el último valor para no romper la matemática
                data->distancias[i] = distancias_previas[i]; 
            }
        }
    }

    // --- NUEVO SISTEMA ANTI MICRO-CORTES ---
    if (anchors_perdidos ==0 && esp_now_valid) {
        // Si vemos al menos 1 ancla y el control remoto funciona, actualizamos el cronómetro
        ultimo_frame_valido_ms = current_time; 
    }

    bool signal_valid = true;
    // Solo entramos en pánico real si llevamos más de 1000ms (1 segundo) completamente a ciegas
    if (current_time - ultimo_frame_valido_ms > 1000) {
        signal_valid = false;
    }

    if (!signal_valid) {
        if (!signal_lost) {
            signal_lost = true;
            signal_lost_time = current_time;
            pid_distancia.integral = 0;
            pid_angular.integral = 0;
        }

        if (last_valid_distance < DISTANCIA_FALLBACK) {
            detener();
            velocidad_actual = 0;
        } else {
            // === LÓGICA SELECCIONABLE CON LA VARIABLE ===
            if (G_FRENADO_SEGURO >= 0.5) {
                // MODO 1: NUEVO (Enderezado rápido y frenado suave)
                last_valid_correction = last_valid_correction * 0.5; 
                velocidad_actual = suavizar_velocidad_avanzada(velocidad_actual, 0.0, ACELERACION_MAXIMA, DESACELERACION_MAXIMA * 1.5);

                if (abs(velocidad_actual) < 5.0 && abs(last_valid_correction) < 5.0) {
                    detener();
                    velocidad_actual = 0;
                    last_valid_correction = 0;
                } else {
                    auto v_err = calcular_velocidades_diferenciales(velocidad_actual, last_valid_correction, G_VELOCIDAD_MAXIMA);
                    enviar_pwm(v_err.vel_izq, v_err.vel_der);
                }
            } else {
                // MODO 0: VIEJO (Mantiene el giro ciego por 1 segundo)
                unsigned long elapsed = current_time - signal_lost_time;
                if (elapsed < (TIMEOUT_FALLBACK * 1000) && velocidad_actual > 5) {
                    auto v_err = calcular_velocidades_diferenciales(VELOCIDAD_FALLBACK, last_valid_correction, G_VELOCIDAD_MAXIMA);
                    enviar_pwm(v_err.vel_izq, v_err.vel_der);
                } else {
                    detener();
                    velocidad_actual = 0;
                }
            }
        }

        if (current_time - last_log_time >= LOG_INTERVAL_MS) {
            last_log_time = current_time;
            Serial.printf("[FAILSAFE] ESP-NOW:%s | Anchors_Perdidos:%d | V_Act:%.0f\n", 
                          esp_now_valid ? "OK" : "LOSS", anchors_perdidos, velocidad_actual);
        }
        return; 
    }

    // --- PASO 2: RECUPERACIÓN ---
    if (signal_lost) {
        signal_lost = false;
        in_recovery = true;
        recovery_start_ms = current_time;
        recovery_cycles = 0;
        saved_normal_Q = kalman_get_Q();
        kalman_set_Q(RECOVERY_Q_TEMP);
        kalman_force_state(data->distancias, RECOVERY_P_INIT);
        media_movil_init(3, MEDIA_MOVIL_VENTANA);
        
        // --- MAGIA ANTI-LATIGAZO ---
        // Forzamos al PID a olvidar la última posición para que no calcule 
        // una velocidad falsa al reconectar.
        pid_distancia.medida_anterior = -999999.0;
        pid_angular.medida_anterior = -999999.0;
    }

    // --- PASO 3: FILTRADO ---
    float distancias_proc[3];
    for(int i=0; i<3; i++) distancias_proc[i] = data->distancias[i];

    // TU LÓGICA ORIGINAL INTACTA: Limitador de saltos y memoria
    if (distancias_previas_valid && !in_recovery) {
        limitar_variacion(distancias_proc, distancias_previas, distancias_proc, 3, MAX_DELTA_POS);
    }
    for(int i=0; i<3; i++) distancias_previas[i] = distancias_proc[i];
    distancias_previas_valid = true;

    float dist_media[3], dist_kalman[3];

    // ACÁ DIVIDIMOS: ¿Estamos en pánico o estamos en control?
    if (in_recovery) {
        // MODO EMERGENCIA: Todo clásico y seguro (Tu lógica intacta)
        media_movil_actualizar(distancias_proc, dist_media);
        kalman_filtrar(dist_media, dist_kalman);
        
        // TU SALIDA DE EMERGENCIA INTACTA
        if (++recovery_cycles > RECOVERY_K_CYCLES) {
            kalman_set_Q(saved_normal_Q);
            in_recovery = false;
        }
    } else {
        // MODO NORMAL: Conectado a la App
        if (G_FILTRO_MODO >= 1.5) {
            // MODO 2: Depredador Baja Latencia (Solo Kalman tuneable por App)
            kalman_set_Q(G_KALMAN_Q_NUEVO);
            kalman_set_R(G_KALMAN_R_NUEVO);
            // Salteamos la media móvil para que la reacción sea instantánea
            kalman_filtrar(distancias_proc, dist_kalman); 
        } 
        else if (G_FILTRO_MODO >= 0.5) {
            // MODO 1: Clásico Conservador (El tuyo original)
            media_movil_actualizar(distancias_proc, dist_media);
            kalman_set_Q(KALMAN_Q); // Valores fijos de config.h
            kalman_set_R(KALMAN_R);
            kalman_filtrar(dist_media, dist_kalman);
        } 
        else {
            // MODO 0: Pura Adrenalina (Crudo, sin filtros)
            for(int i=0; i<3; i++) dist_kalman[i] = distancias_proc[i];
        }
    }

    

    // --- PASO 4: V4 - LO MEJOR DE V1 Y V2 (Sin agregados extraños) ---
    static float pos_tag[3] = {0, 0, 0}; 
    static bool just_recovered = true; 
    float angulo_relativo = angulo_anterior; 

    // Memorias del Filtro y Estado
    static float ultimo_valido_pico = 0;
    static int picos_ignorados = 0;
    static bool estoy_atras = false;

    if (anchors_perdidos == 0) {
        float angulo_actual = 0;
        float dist_frontal = (dist_kalman[0] + dist_kalman[1]) / 2.0;

        // ===============================================================
        // 1. CANDADO DEL EJE Y (Dinámico V1 vs V2)
        // ===============================================================
        float histeresis_actual = 0.0;
        
        // Si estamos lejos (ej. mayor a DISTANCIA_OBJETIVO), aplicamos la estabilidad de la V2
        // G_HISTERESIS_Y viene de la app (ej: 100.0 a 150.0 mm máximo, ya que tu sensor está a 230mm)
        if (dist_frontal > DISTANCIA_OBJETIVO) {
            histeresis_actual = G_HISTERESIS_Y; 
        } else {
            // Si estamos cerca, la histéresis es 0 para tener los reflejos instantáneos de la V1
            histeresis_actual = 0.0; 
        }

        // Lógica de desempate con el margen dinámico
        if (!estoy_atras && dist_kalman[2] < (dist_frontal - histeresis_actual)) {
            estoy_atras = true;
        } else if (estoy_atras && dist_kalman[2] > (dist_frontal - histeresis_actual + 50.0)) { 
            // El +50.0 es un pequeño margen de salida para evitar que oscile justo en la línea
            estoy_atras = false;
        }

        // ===============================================================
        // 2. CÁLCULO GEOMÉTRICO (Crudo, sin Mediana ni EMA)
        // ===============================================================
        if (G_MODO_GEOMETRIA >= 0.5) {
            // === MODO 1: 2D ROBUSTA ===
            float dL = dist_kalman[0];
            float dR = dist_kalman[1];
            float x_estable = (pow(dR, 2) - pow(dL, 2)) / 1120.0;
            float y_2d = dist_frontal;
            
            if (estoy_atras) y_2d = -abs(y_2d);
            else y_2d = abs(y_2d);

            if (abs(y_2d) < 1.0) y_2d = (estoy_atras) ? -1.0 : 1.0;
            angulo_actual = atan2(x_estable, y_2d) * 180.0 / M_PI;

        } else {
            // === MODO 0: TRILATERACIÓN 3D CLÁSICA ===
            trilateracion_3d(SENSOR_POSICIONES, dist_kalman, pos_tag);
            
            if (estoy_atras) pos_tag[1] = -abs(pos_tag[1]);
            else pos_tag[1] = abs(pos_tag[1]);

            if (abs(pos_tag[1]) < 1.0) pos_tag[1] = (estoy_atras) ? -1.0 : 1.0;
            angulo_actual = angulo_direccion_xy(pos_tag);
        }
        
        // ===============================================================
        // 3. RECUPERACIÓN POST-CORTE
        // ===============================================================
        if (just_recovered) {
            angulo_anterior = angulo_actual; 
            ultimo_valido_pico = angulo_actual; 
            just_recovered = false;
        }

        // ===============================================================
        // 4. FILTRO DE PICOS (Corregido y con Selector de Zonas)
        // ===============================================================
        float angulo_procesado = angulo_actual;
        bool aplicar_filtro = false;

        if (G_FILTRO_PICO_EN >= 0.5) {
            // G_SELECTOR_ZONA_FILTRO viene de la app:
            // 0 = Todas las zonas
            // 1 = Mayor a Zona Objetivo
            // 2 = Mayor a Zona Caddie (Interacción)
            if (G_SELECTOR_ZONA_FILTRO == 0) {
                aplicar_filtro = true;
            } else if (G_SELECTOR_ZONA_FILTRO == 1 && dist_frontal > DISTANCIA_OBJETIVO) {
                aplicar_filtro = true;
            } else if (G_SELECTOR_ZONA_FILTRO == 2 && dist_frontal > G_DIST_INTERACCION) {
                aplicar_filtro = true;
            }
        }

        if (aplicar_filtro) {
            // Cálculo real del salto angular (la corrección que faltaba en V3)
            float angulo_continuo = desenrollar_angulo(ultimo_valido_pico, angulo_actual);
            float delta_pico = angulo_continuo - ultimo_valido_pico; 
            
            if (abs(delta_pico) > G_FILTRO_ANG_PICO && picos_ignorados < 30) {
                angulo_procesado = ultimo_valido_pico; // Rechazamos el ruido
                picos_ignorados++;
            } else {
                ultimo_valido_pico = angulo_actual; // Lectura válida
                picos_ignorados = 0;
            }
        } else {
            // Si estamos fuera de la zona seleccionada, pasa directo (reflejos de V1)
            ultimo_valido_pico = angulo_actual;
            picos_ignorados = 0;
        }

        // ===============================================================
        // 5. LIMITADOR FINAL
        // ===============================================================
        angulo_relativo = limitar_cambio(angulo_anterior, angulo_procesado, G_MAX_DELTA);
        
    } else {
        just_recovered = true; 
    }
    
    angulo_anterior = angulo_relativo;
    
// --- PASO 5: CONTROL DE DISTANCIA Y "POINT & SHOOT" INTELIGENTE ---
    static float distancia_ref_integral = last_valid_distance; // Movido arriba para scope global

    if (modo != 1) {
        pid_distancia.integral = 0;
        pid_angular.integral = 0;
        // SOLUCIÓN 2 (Anti-Windup Sync): Mantenemos la memoria fresca en manual
        distancia_ref_integral = (dist_kalman[0] + dist_kalman[1]) / 2.0; 
    }

    float distancia_al_tag = (dist_kalman[0] + dist_kalman[1]) / 2.0;
    if (isnan(distancia_al_tag) || distancia_al_tag > 15000.0) {
        distancia_al_tag = last_valid_distance;
    }

    // ==== ANTI-WINDUP POR AVANCE EFECTIVO ====
    if (distancia_al_tag > distancia_ref_integral) {
        distancia_ref_integral = distancia_al_tag;
    } else if ((distancia_ref_integral - distancia_al_tag) > G_ANTI_WINDUP_DIST) {
        pid_distancia.integral = 0; 
        distancia_ref_integral = distancia_al_tag; 
    }
    // =========================================

    float valor_pid_dist = -pid_update(&pid_distancia, distancia_al_tag, false);
    float velocidad_objetivo = 0;

    static bool en_movimiento = false;
    
    if (velocidad_actual < 2.0) {
        en_movimiento = false;
    }

    if (distancia_al_tag > G_DISTANCIA_OBJETIVO) {
        // POINT & SHOOT
        if (!en_movimiento && abs(angulo_relativo) > G_ANGULO_ARRANQUE) { 
            velocidad_objetivo = 0; 
            pid_distancia.integral = 0; 
        } else {
            velocidad_objetivo = max(0.0f, valor_pid_dist); 
            if (velocidad_objetivo > 2.0) {
                en_movimiento = true; 
            }
        }
    } else {
        // ZONA OBJETIVO
        velocidad_objetivo = 0; 
        pid_distancia.integral = 0; 
        en_movimiento = false; 
        distancia_ref_integral = distancia_al_tag; 
    }

    velocidad_actual = suavizar_velocidad_avanzada(velocidad_actual, velocidad_objetivo, G_ACELERACION_MAXIMA, G_DESACELERACION_MAXIMA);


    // --- PASO 6: CONTROL ANGULAR (Dual PID Basado en Física + Modo Caddie) ---
    static bool usando_pid_dinamico = true;

    // SOLUCIÓN 1: El PID ahora elige su fuerza según la VELOCIDAD, no la distancia.
    if (abs(velocidad_actual) > 2.0) {
        // EL CARRO ESTÁ RODANDO -> MODO DINÁMICO
        if (!usando_pid_dinamico) {
            pid_angular.integral = 0; 
            usando_pid_dinamico = true;
        }
        // No pisamos el KP para no matar el Gain Scheduling del Paso 1
        pid_angular.ki = G_PID_KI;
        pid_angular.kd = G_PID_KD;
    } else {
        // EL CARRO ESTÁ DETENIDO -> MODO ESTÁTICO (Fuerza bruta para pivotar en el pasto)
        if (usando_pid_dinamico) {
            pid_angular.integral = 0; 
            usando_pid_dinamico = false;
        }
        pid_angular.kp = G_STATIC_KP;
        pid_angular.ki = G_STATIC_KI;
        pid_angular.kd = G_STATIC_KD;
    }

    // (SOLUCIÓN 3: El código de "Cruce por cero" fue eliminado aquí para evitar tartamudeos)

    float umbral_dyn = calcular_umbral_dinamico(distancia_al_tag, UMBRAL_MINIMO, UMBRAL_MAXIMO, DISTANCIA_UMBRAL_MIN, DISTANCIA_UMBRAL_MAX);
    float correccion_teorica = pid_update(&pid_angular, angulo_relativo, true);
    float correccion = 0; 

    if (G_CADDIE_INTELIGENTE >= 0.5) {
        // === MODO CADDIE ACTIVADO ===
        if (distancia_al_tag <= G_DISTANCIA_OBJETIVO) {
            if (abs(velocidad_actual) < 2.0) {
                if (distancia_al_tag < G_DIST_INTERACCION) {
                    correccion = 0;
                    pid_angular.integral = 0; 
                } else {
                    if (abs(angulo_relativo) > G_DEADZONE_CADDIE) {
                        correccion = correccion_teorica; 
                    } else {
                        pid_angular.integral = 0; // Este es el reseteo seguro que reemplaza al cruce por cero
                    }
                }
            } else {
                pid_angular.integral = 0; 
            }
        } else {
            if (abs(angulo_relativo) > umbral_dyn) {
                correccion = correccion_teorica; 
            } else {
                pid_angular.integral = 0; 
            }
        }
    } else {
        // === MODO CADDIE DESACTIVADO ===
        if (distancia_al_tag <= G_DISTANCIA_OBJETIVO) {
            correccion = 0;
            pid_angular.integral = 0;
        } else {
            if (abs(angulo_relativo) > umbral_dyn) correccion = correccion_teorica; 
            else pid_angular.integral = 0; 
        }
    }   
    // === PASO 6.5: EL CEPO ANTI-TROMPO ===
    if (G_ANTI_TROMPO >= 0.5) {
        // Si el carro casi no tiene velocidad frontal (está pivoteando)
        if (abs(velocidad_actual) < 15.0) { 
            // Capamos la fuerza física de giro para que no patine en el pasto
            correccion = constrain(correccion, -80.0, 80.0);
        }
    }
    // --- PASO 7: SALIDA A MOTORES ---
    if (modo == 1) { 
        last_valid_distance = distancia_al_tag; 
        last_valid_correction = correccion;
        // Nos movemos si hay empuje hacia adelante (>2) O si hay giro activo
        if (velocidad_actual > 2 || abs(correccion) > 0) {
            auto v = calcular_velocidades_diferenciales(velocidad_actual, correccion, G_VELOCIDAD_MAXIMA);
            enviar_pwm(v.vel_izq, v.vel_der);
        } else {
            detener();
        }
    } 
    else if (modo == 3) { 
        enviar_pwm(manual_L, manual_R);
    } 
    else {
        detener();
        velocidad_actual = 0;
    }

    // --- TELEMETRÍA ---
    if (current_time - last_log_time >= LOG_INTERVAL_MS) {
        last_log_time = current_time;
        // Agregamos el Kp actual al log para que veas como cambia en tiempo real
        Serial.printf("[30Hz] V:%.0f | Kp:%.2f | Dist:%.0f | Ang:%.1f\n", 
                      velocidad_actual, pid_angular.kp, distancia_al_tag, angulo_relativo);
    }
    // --- EXPORTAR TELEMETRÍA PARA BLUETOOTH ---
    dbg_angulo = angulo_relativo;
    dbg_distancia = distancia_al_tag;
    dbg_pid_dist = velocidad_actual; // Potencia frontal (Avance)
    dbg_pid_ang = correccion;        // Diferencia de potencia (Giro)
}