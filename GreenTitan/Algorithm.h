#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "Arduino.h"
#include "Defines.h"
#include "Motion.h"
#include "MainTask.h"

#include <AceSorting.h>
#include <Array.h>
#include <math.h>

extern int MOWER_OVERLAP;
extern int MAX_DEVIATION;
extern int BASE_LON;
extern int BASE_LAT;

void FindTerrainBounds();
void ClearInterference();
bool FindOutlineIntersections();
bool GeneratePaths();
bool GenerateGcode();
int ShortestOutlinePath(int outline_index, int current_point, int target_point);
int OutlineTraverseInc(int outline_index, int current_point, int amount);
int OutlineTraverseDec(int outline_index, int current_point, int amount);
Array<int, 2> AlgorithmNextPoint();
void AlgorithmAbort(bool full_abort);

#endif