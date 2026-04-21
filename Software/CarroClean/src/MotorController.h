#pragma once

#include <Arduino.h>
//#include "RPiComm.h"

// Funciones públicas del controlador
void setupMotorController();




// ===== NUEVAS FUNCIONES PARA CONTROL DIRECTO =====
// Funciones para ser usadas como callbacks desde control.cpp
void motor_enviar_pwm(int vel_izq, int vel_der);
void motor_detener();
void setupMotorControlDirect();