#ifndef CALCULOS_H
#define CALCULOS_H

// Funciones para media móvil
void media_movil_init(int n_sensores, int ventana);
void media_movil_actualizar(float* nuevos_valores, float* resultado);

// Funciones para filtro Kalman
void kalman_init(int n_sensores, float Q, float R);
void kalman_filtrar(float* mediciones, float* resultado);
void kalman_force_state(float* mediciones, float P_init);
void kalman_set_Q(float Q);
float kalman_get_Q();

// Estructura para controlador PID
typedef struct {
    float kp, ki, kd;
    float setpoint;
    float alpha;
    float error_anterior;
    float medida_anterior;
    float derivada_anterior; // <-- NUEVA LÍNEA (Obligatoria para evitar latigazos)
    float integral;
    float salida_maxima;
    float integral_maxima;
    float ultima_salida;
} PIDController;

// Funciones para controlador PID
void pid_init(PIDController* pid, float kp, float ki, float kd, float setpoint, float alpha, float salida_maxima, float integral_maxima);
float pid_update(PIDController* pid, float medida_actual, bool es_angulo = false);

// Función para limitar variación de distancias
void limitar_variacion(float* dist_actual, float* dist_anterior, float* resultado, int n_sensores, float max_delta);

// Función para trilateración 3D
void trilateracion_3d(const float sensor_posiciones[3][3], float* distancias, float* resultado);

// Funciones para cálculo y manejo de ángulos
float angulo_direccion_xy(float* pos_tag);
float desenrollar_angulo(float previo, float actual);
float limitar_cambio(float previo, float actual, float max_delta);

// Función de validación/umbral
bool debe_corregir(float angulo, float umbral);

// Función para calcular umbral dinámico basado en distancia
float calcular_umbral_dinamico(float distancia_mm, float umbral_min, float umbral_max, 
                               float dist_min, float dist_max);


// Nueva función con aceleración y desaceleración separadas
float suavizar_velocidad_avanzada(float velocidad_actual, float velocidad_objetivo, 
                                  float aceleracion_maxima, float desaceleracion_maxima);

// Estructura para velocidades diferenciales
struct VelocidadesDiferenciales {
    int vel_izq;
    int vel_der;
    float giro_normalizado;
};

// Función para calcular velocidades diferenciales
VelocidadesDiferenciales calcular_velocidades_diferenciales(float v_lineal, float angulo_relativo, float max_v);

// ===== FUNCIONES DE CONTROL PRINCIPAL =====

// Estructura para pasar datos directamente desde main.cpp
struct CarroData {
    float distancias[3];        // Distancias UWB [S1, S2, S3]
    float power_data[3];        // [BAT_C, ML_C, MR_C] 
    float imu_data[6];          // [PITCH, ROLL, YAW, MOV, VEL, ACCEL_Z]
    float tag_sensors[3];       // [S1, S2, S3] sensores del tag
    float control_data[2];      // [BAT_TAG, MODO]
    float buttons_data[4];      // [JOY_D, JOY_A, JOY_I, JOY_T]
    bool data_valid;            // Indica si los datos son válidos
};

// Callbacks para PWM (implementadas en main.cpp)
typedef void (*PWMCallback)(float vel_izq, float vel_der);
typedef void (*StopCallback)();

// Funciones del sistema de control
void control_init();
void control_main(CarroData* data, PWMCallback enviar_pwm, StopCallback detener);

// Declaraciones de los setters de Kalman
void kalman_set_Q(float Q);
void kalman_set_R(float R);
float kalman_get_Q();

// Declaración de la nueva función de Filtros A/B
void aplicar_filtros_inteligentes(float* mediciones_crudas, float* mediciones_anteriores, float* resultado, int modo, float q_orig, float r_orig, float q_nuevo, float r_nuevo, float max_delta);

#endif
