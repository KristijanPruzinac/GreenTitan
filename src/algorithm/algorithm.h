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

extern int MOWER_OVERLAP;
extern int MAX_DEVIATION;
extern long long BASE_LON;
extern long long BASE_LAT;
extern long long BASE_EXIT_LON;
extern long long BASE_EXIT_LAT;

void ClearOutlines();
void FindTerrainBounds();
void ClearInterference();
bool FindOutlineIntersections();
bool GeneratePaths();
bool GenerateGcode();
int ShortestOutlinePath(int outline_index, int current_point, int target_point);
int OutlineTraverseInc(int outline_index, int current_point, int amount);
int OutlineTraverseDec(int outline_index, int current_point, int amount);
std::vector<double> AlgorithmNextPoint();
void AlgorithmAbort(bool full_abort);

void AlgorithmCaptureStart();
void AlgorithmCaptureBasePoint();
void AlgorithmCaptureBaseExitPoint();
void AlgorithmCaptureNewOutline();
void AlgorithmCaptureNewPoint();
void AlgorithmCaptureSetNewPoint(long long lon, long long lat);
bool AlgorithmCaptureRemoveOutline();
bool AlgorithmCaptureRemovePoint();
bool AlgorithmCaptureEnd();

void AlgorithmMotionSetTargetPoint(double tLon, double tLat);
String AlgorithmGetPathString();
bool AlgorithmPopulatePathFromString(String& readData);

#endif