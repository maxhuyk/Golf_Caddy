#pragma once
#define NUM_ANCHORS 3 
// Estructura para datos raw de UWB
struct UWBRawData {
    float distance1;
    float distance2; 
    float distance3;
    bool valid1;
    bool valid2;
    bool valid3;
    unsigned long timestamp;
};

// Variables compartidas entre cores (thread-safe)
extern volatile UWBRawData g_uwb_raw_data;
extern volatile bool g_uwb_data_ready;
extern volatile unsigned long g_uwb_measurement_count;
extern volatile unsigned long g_uwb_last_measurement_time;

// Funciones para Core 0 (UWB dedicado)
void UWBCore_setup();
void UWBCore_task(void* parameter);

// Funciones para Core 1 (procesamiento)
void UWBCore_startTask();
void UWBCore_getDistances(float distances[NUM_ANCHORS]);
void UWBCore_getAnchorStatus(bool status[NUM_ANCHORS]);
bool UWBCore_update();
unsigned long UWBCore_getMeasurementCount();