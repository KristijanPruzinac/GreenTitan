#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "Arduino.h"
#include "Defines.h"
#include "Motion.h"
#include "MainTask.h"

#include <AceSorting.h>
#include <Array.h>
#include <math.h>

void FindTerrainBounds();
void ClearInterference();
void FindOutlineIntersections();
void GeneratePaths();
void GenerateGcode();
long ShortestOutlinePath(int outline_index, int current_point, int target_point);
int OutlineTraverseInc(int outline_index, int current_point, int amount);
int OutlineTraverseDec(int outline_index, int current_point, int amount);
Array<long, 2> AlgorithmNextPoint();
void AlgorithmAbort(bool full_abort);

#endif