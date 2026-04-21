#ifndef CONTROL_H
#define CONTROL_H

#include "calculos.h"

// Callback types para control de motores
typedef void (*PWMCallback)(float vel_izq, float vel_der);
typedef void (*StopCallback)();

// Funciones principales del sistema de control
void control_init();
void control_main(CarroData* data, PWMCallback enviar_pwm, StopCallback detener);

#endif // CONTROL_H