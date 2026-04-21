#include "calculos.h"
#include <Arduino.h>
#include <math.h>

// =================================================================
// 1. VARIABLES ESTÁTICAS DE ESTADO
// =================================================================

// Media Móvil
static float buffers[3][10];
static int buffer_indices[3] = {0, 0, 0};
static int buffer_counts[3] = {0, 0, 0};
static int ventana_size = 5;
static int num_sensores = 3;

// Filtro Kalman
static float kalman_x[3] = {0.0, 0.0, 0.0};
static float kalman_P[3] = {1.0, 1.0, 1.0};
static float kalman_Q = 0.01;
static float kalman_R = 1.0;
static int kalman_num_sensores = 3;

// =================================================================
// 2. FUNCIONES AUXILIARES Y VECTORIALES
// =================================================================

float vector_norm(float* v, int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) sum += v[i] * v[i];
    return sqrt(sum);
}

void vector_subtract(float* a, float* b, float* result, int size) {
    for (int i = 0; i < size; i++) result[i] = a[i] - b[i];
}

void vector_add(float* a, float* b, float* result, int size) {
    for (int i = 0; i < size; i++) result[i] = a[i] + b[i];
}

void vector_scale(float* v, float scale, float* result, int size) {
    for (int i = 0; i < size; i++) result[i] = v[i] * scale;
}

float vector_dot(float* a, float* b, int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) sum += a[i] * b[i];
    return sum;
}

void vector_cross_3d(float* a, float* b, float* result) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

float clip(float value, float min_val, float max_val) {
    return constrain(value, min_val, max_val);
}

float desenrollar_angulo(float previo, float actual) {
    float delta = actual - previo;
    while (delta > 180) delta -= 360;
    while (delta < -180) delta += 360;
    return delta; // Retorna solo la diferencia más corta
}

float limitar_cambio(float previo, float actual, float max_delta) {
    float delta = desenrollar_angulo(previo, actual);
    delta = constrain(delta, -max_delta, max_delta);
    float nuevo = previo + delta;
    // Forzamos a que el resultado final siempre viva entre -180 y 180
    while (nuevo > 180) nuevo -= 360;
    while (nuevo < -180) nuevo += 360;
    return nuevo;
}

void limitar_variacion(float* dist_actual, float* dist_anterior, float* resultado, int n_sensores, float max_delta) {
    for (int i = 0; i < n_sensores; i++) {
        float delta = dist_actual[i] - dist_anterior[i];
        resultado[i] = dist_anterior[i] + constrain(delta, -max_delta, max_delta);
    }
}

// =================================================================
// 3. SISTEMAS DE FILTRADO
// =================================================================

void media_movil_init(int n_sensores, int ventana) {
    num_sensores = n_sensores;
    ventana_size = ventana;
    for (int i = 0; i < n_sensores; i++) {
        buffer_indices[i] = 0;
        buffer_counts[i] = 0;
        for (int j = 0; j < ventana; j++) buffers[i][j] = 0.0;
    }
}

void media_movil_actualizar(float* nuevos_valores, float* resultado) {
    for (int i = 0; i < num_sensores; i++) {
        buffers[i][buffer_indices[i]] = nuevos_valores[i];
        buffer_indices[i] = (buffer_indices[i] + 1) % ventana_size;
        if (buffer_counts[i] < ventana_size) buffer_counts[i]++;
        
        float suma = 0.0;
        for (int j = 0; j < buffer_counts[i]; j++) suma += buffers[i][j];
        resultado[i] = suma / buffer_counts[i];
    }
}

void kalman_init(int n_sensores, float Q, float R) {
    kalman_num_sensores = n_sensores;
    kalman_Q = Q;
    kalman_R = R;
    for (int i = 0; i < n_sensores; i++) {
        kalman_x[i] = 0.0;
        kalman_P[i] = 1.0;
    }
}

void kalman_filtrar(float* mediciones, float* resultado) {
    for (int i = 0; i < kalman_num_sensores; i++) {
        kalman_P[i] += kalman_Q;
        float K = kalman_P[i] / (kalman_P[i] + kalman_R);
        kalman_x[i] = kalman_x[i] + K * (mediciones[i] - kalman_x[i]);
        kalman_P[i] = (1 - K) * kalman_P[i];
        resultado[i] = kalman_x[i];
    }
}

void kalman_force_state(float* mediciones, float P_init) {
    for (int i = 0; i < kalman_num_sensores; i++) {
        kalman_x[i] = mediciones[i];
        kalman_P[i] = P_init;
    }
}

void kalman_set_Q(float Q) { kalman_Q = Q; }
void kalman_set_R(float R) { kalman_R = R; }
float kalman_get_Q() { return kalman_Q; }
float kalman_get_R() { return kalman_R; }

// =================================================================
// 3.5. SELECTOR DE FILTROS DINÁMICO (Pruebas A/B)
// =================================================================

void aplicar_filtros_inteligentes(float* mediciones_crudas, float* mediciones_anteriores, float* resultado, int modo, float q_orig, float r_orig, float q_nuevo, float r_nuevo, float max_delta) {
    if (modo == 0) {
        // MODO 0: PURA ADRENALINA (Directo del sensor a la matemática, sin filtros)
        for (int i = 0; i < num_sensores; i++) {
            resultado[i] = mediciones_crudas[i];
        }
    } 
    else if (modo == 1) {
        // MODO 1: CLÁSICO CONSERVADOR (Limitador + Media Móvil + Kalman)
        float temp_sin_saltos[3];
        float temp_media[3];
        
        limitar_variacion(mediciones_crudas, mediciones_anteriores, temp_sin_saltos, num_sensores, max_delta);
        media_movil_actualizar(temp_sin_saltos, temp_media);
        
        kalman_set_Q(q_orig);
        kalman_set_R(r_orig);
        kalman_filtrar(temp_media, resultado);
    } 
    else if (modo == 2) {
        // MODO 2: DEPREDADOR BAJA LATENCIA (Solo Kalman Agresivo tuneable)
        kalman_set_Q(q_nuevo);
        kalman_set_R(r_nuevo);
        kalman_filtrar(mediciones_crudas, resultado);
    } 
    else {
        // Fallback por seguridad
        for (int i = 0; i < num_sensores; i++) {
            resultado[i] = mediciones_crudas[i];
        }
    }
}

// =================================================================
// 4. CONTROLADOR PID (Optimizado para 30Hz sin latigazos)
// =================================================================

void pid_init(PIDController* pid, float kp, float ki, float kd, float setpoint, float alpha, float salida_maxima, float integral_maxima) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->alpha = alpha;
    pid->error_anterior = 0.0;
    pid->medida_anterior = -999999.0;
    pid->derivada_anterior = 0.0; // Inicializamos la memoria de la derivada
    pid->integral = 0.0;
    pid->salida_maxima = salida_maxima;
    pid->integral_maxima = integral_maxima;
    pid->ultima_salida = 0.0;
}

float pid_update(PIDController* pid, float medida_actual, bool es_angulo) {
    // 1. Proporcional (Con manejo de círculo si es ángulo)
    float error_actual;
    if (es_angulo) {
        error_actual = desenrollar_angulo(medida_actual, pid->setpoint);
    } else {
        error_actual = pid->setpoint - medida_actual;
    }
    
    float error_filtrado = pid->alpha * error_actual + (1 - pid->alpha) * pid->error_anterior;
    
    // 2. Derivada (Evita el latigazo inicial y protegida contra cero-movimiento)
    float dif_medicion = 0.0;
    
    if (pid->medida_anterior <= -999998.0) {
        pid->medida_anterior = medida_actual; // Primer frame
    } else {
        // Solo calculamos derivada si el sensor detectó un cambio real
        if (abs(medida_actual - pid->medida_anterior) >= 0.0001) {
            if (es_angulo) {
                dif_medicion = -desenrollar_angulo(pid->medida_anterior, medida_actual);
            } else {
                dif_medicion = -(medida_actual - pid->medida_anterior);
            }
        }
    }
    
    float derivada_filtrada = pid->alpha * dif_medicion + (1 - pid->alpha) * pid->derivada_anterior;
    
    // 3. Integral (AHORA SÍ ACUMULA AUNQUE EL CARRO ESTÉ TRABADO)
    pid->integral += error_filtrado;
    if (pid->integral_maxima != 0.0) {
        pid->integral = constrain(pid->integral, -pid->integral_maxima, pid->integral_maxima);
    }
    
    // 4. Salida
    float salida = (pid->kp * error_filtrado) + (pid->ki * pid->integral) + (pid->kd * derivada_filtrada);
    if (pid->salida_maxima != 0.0) {
        salida = constrain(salida, -pid->salida_maxima, pid->salida_maxima);
    }
    
    // 5. Memorias
    pid->error_anterior = error_filtrado;
    pid->medida_anterior = medida_actual;
    pid->derivada_anterior = derivada_filtrada;
    pid->ultima_salida = salida;
    
    return salida;
}

// =================================================================
// 5. GEOMETRÍA Y NAVEGACIÓN
// =================================================================

void trilateracion_3d(const float sensor_posiciones[3][3], float* distancias, float* resultado) {
    float P1[3] = {sensor_posiciones[0][0], sensor_posiciones[0][1], sensor_posiciones[0][2]};
    float P2[3] = {sensor_posiciones[1][0], sensor_posiciones[1][1], sensor_posiciones[1][2]};
    float P3[3] = {sensor_posiciones[2][0], sensor_posiciones[2][1], sensor_posiciones[2][2]};
    
    float r1 = distancias[0];
    float r2 = distancias[1];
    float r3 = distancias[2];

    float p2_p1[3];
    vector_subtract(P2, P1, p2_p1, 3);
    float d = vector_norm(p2_p1, 3);
    float ex[3];
    vector_scale(p2_p1, 1.0/d, ex, 3);

    float p3_p1[3];
    vector_subtract(P3, P1, p3_p1, 3);
    float i = vector_dot(ex, p3_p1, 3);
    
    float ey_dir[3];
    for(int k=0; k<3; k++) ey_dir[k] = p3_p1[k] - (i * ex[k]);
    float j = vector_norm(ey_dir, 3);
    
    // PARCHE NINJA: Evitar división por cero si j es demasiado pequeño
    if (j < 0.0001) j = 0.0001; 
    
    float ey[3];
    vector_scale(ey_dir, 1.0/j, ey, 3);

    float ez[3];
    vector_cross_3d(ex, ey, ez);

    float x = (r1*r1 - r2*r2 + d*d) / (2 * d);
    float y = (r1*r1 - r3*r3 + i*i + j*j) / (2 * j) - (i / j) * x;
    float z2 = r1*r1 - x*x - y*y;
    float z = (z2 < 0) ? 0.0 : sqrt(z2);

    for(int k=0; k<3; k++) {
        resultado[k] = P1[k] + (x * ex[k]) + (y * ey[k]) + (z * ez[k]);
    }
}

float angulo_direccion_xy(float* pos_tag) {
    return atan2(pos_tag[0], pos_tag[1]) * 180.0 / M_PI;
}

bool debe_corregir(float angulo, float umbral) { return abs(angulo) > umbral; }

float calcular_umbral_dinamico(float distancia_mm, float umbral_min, float umbral_max, float dist_min, float dist_max) {
    if (distancia_mm <= dist_min) return umbral_min;
    if (distancia_mm >= dist_max) return umbral_max;
    float factor = (distancia_mm - dist_min) / (dist_max - dist_min);
    return umbral_min + factor * (umbral_max - umbral_min);
}

// =================================================================
// 6. CINEMÁTICA Y SALIDA
// =================================================================

float suavizar_velocidad_avanzada(float v_actual, float v_obj, float acel_max, float desacel_max) {
    // Acelerando hacia adelante
    if (v_obj > v_actual && v_actual >= 0) {
        return min(v_obj, v_actual + acel_max);
    }
    // Frenando hacia adelante
    else if (v_obj < v_actual && v_actual >= 0) {
        return max(v_obj, v_actual - desacel_max);
    }
    // Acelerando en reversa (v_obj es más negativo que v_actual)
    else if (v_obj < v_actual && v_actual <= 0) {
        return max(v_obj, v_actual - acel_max);
    }
    // Frenando en reversa (v_obj es menos negativo que v_actual)
    else if (v_obj > v_actual && v_actual <= 0) {
        return min(v_obj, v_actual + desacel_max);
    }
    return v_actual;
}

VelocidadesDiferenciales calcular_velocidades_diferenciales(float v_lineal, float correccion_angular, float max_v) {
    VelocidadesDiferenciales resultado;
    
    // Suma y resta directa de la corrección del PID
    float vel_izq_float = v_lineal - correccion_angular;
    float vel_der_float = v_lineal + correccion_angular;
    
    // AHORA PERMITIMOS VALORES NEGATIVOS (-max_v) PARA GIRAR SOBRE SU EJE
    resultado.vel_izq = (int)clip(vel_izq_float, -max_v, max_v);
    resultado.vel_der = (int)clip(vel_der_float, -max_v, max_v);
    return resultado;
}