#ifndef BLEMANAGER_H
#define BLEMANAGER_H

#include <Arduino.h>
#include "calculos.h" // Para usar CarroData

// Funciones principales
void BLEManager_init();
void BLEManager_update(CarroData* data, int vel_L, int vel_R);

// Permite saber si el celular está enviando comandos manuales por BLE
bool BLEManager_isManualOverride(int* out_L, int* out_R);

#endif