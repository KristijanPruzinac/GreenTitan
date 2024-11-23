#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "Arduino.h"
#include "Defines.h"

#include <AceSorting.h>
#include <vector>
#include <math.h>

extern int MOWER_OVERLAP;
extern int MAX_DEVIATION;
extern int BASE_LON;
extern int BASE_LAT;
extern int BASE_EXIT_LON;
extern int BASE_EXIT_LAT;

//Gps
extern long GpsGetLon();
extern long GpsGetLat();

//Motion
extern void MotionSetTarget(long long tLon, long long tLat);
extern void MotionMoveToTarget();

//Motor
extern void MotorDriveAngle(float angle, bool forward, float speedFactor);
extern void MotorMainOn();
extern void MotorMainOff();


//extern void MainChargingStart();

void ClearOutlines();
void FindTerrainBounds();
void ClearInterference();
bool FindOutlineIntersections();
bool GeneratePaths();
bool GenerateGcode();
int ShortestOutlinePath(int outline_index, int current_point, int target_point);
int OutlineTraverseInc(int outline_index, int current_point, int amount);
int OutlineTraverseDec(int outline_index, int current_point, int amount);
std::vector<long long> AlgorithmNextPoint();
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

String AlgorithmGetPathString();
bool AlgorithmPopulatePathFromString(String& readData);

#endif