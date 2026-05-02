#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"
#include "esp_dds.h"
#include "esp_timer.h"
#include <AceSorting.h>
#include <vector>
#include <math.h>

extern HardwareSerial SerialDebug;

// All position globals are double — conversion to long long happens internally
extern double BASE_LON;
extern double BASE_LAT;
extern double BASE_EXIT_LON;
extern double BASE_EXIT_LAT;

// Tuning parameters
extern int MOWER_OVERLAP;
extern int MAX_DEVIATION;

std::vector<double> AlgorithmNextPoint();
void AlgorithmMotionSetTargetPoint(double tLon, double tLat);
void AlgorithmAbort(bool full_abort);
String AlgorithmGetPathString();
bool   AlgorithmPopulatePathFromString(String& readData);

// --- FreeRTOS task entry point ---
void algorithm_task(void* parameter);

#endif // ALGORITHM_H