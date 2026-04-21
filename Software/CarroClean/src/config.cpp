#include "config.h"

// #################################################################
// POSICIONES DE SENSORES
// #################################################################
// Formato: {x, y, z} en milímetros
// Sensor 0: (-280, 0, 0)
// Sensor 1: (280, 0, 0) 
// Sensor 2: (165, -230, 0)
const float SENSOR_POSICIONES[3][3] = {
    {280.0, 0.0, 0.0},   // Sensor 0: izquierda
    {-280.0, 0.0, 0.0},    // Sensor 1: derecha
    {0.0, -230.0, 0.0}  // Sensor 2: frente-derecha
};
