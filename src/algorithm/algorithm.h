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
extern double BASE_LON;
extern double BASE_LAT;
extern int64_t BASE_EXIT_X_CM;
extern int64_t BASE_EXIT_Y_CM;

extern bool algorithmDone;

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
void AlgorithmSetCurrentPosition(long long x_cm, long long y_cm);
void AlgorithmReset();

void AlgorithmCaptureStart();
void AlgorithmCaptureBaseExitPoint(int64_t x_cm, int64_t y_cm);
void AlgorithmCaptureNewOutline();
void AlgorithmCaptureNewPoint();
void AlgorithmCaptureSetNewPoint(long long lon, long long lat);
bool AlgorithmCaptureRemoveOutline();
bool AlgorithmCaptureRemovePoint();
bool AlgorithmCaptureEnd();

void AlgorithmMotionSetTargetPoint(double tLon, double tLat);
void AlgorithmSeedStartPosition();

String AlgorithmGetPathString();
bool AlgorithmPopulatePathFromString(String& readData);

const std::vector<std::vector<std::vector<long long>>>& AlgorithmGetExtOutlines();
const std::vector<std::vector<std::vector<long long>>>& AlgorithmGetIntersectionPaths();

#endif