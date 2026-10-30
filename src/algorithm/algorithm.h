#ifndef ALGORITHM_H
#define ALGORITHM_H

// The mower's side of the coverage planner (coverage.h, lawn_map.h): outline capture and its NVS
// string, the plan built from it, and waypoints out to the motion task. The controller decides
// the goal (mow, home) and tells where the robot stopped; the planner only says where to go.

#include "Arduino.h"
#include "../definitions.h"
#include "../functions.h"
#include "../navigation/nav_path.h"
#include "coverage.h"

#include "esp_dds.h"

#include <vector>
#include <math.h>

#include "../debug/debug_serial.h"

extern float MOWER_OVERLAP;

// Capture: outline 0 is the lawn boundary (its first point the base), the rest islands; cm
void AlgorithmCaptureStart();
void AlgorithmCaptureNewOutline();
void AlgorithmCaptureNewPoint();                            // at the current position
void AlgorithmCaptureSetNewPoint(long long x_cm, long long y_cm);
bool AlgorithmCaptureRemoveOutline();
bool AlgorithmCaptureRemovePoint();
bool AlgorithmCaptureEnd();                                 // builds the plan; false: no lawn

String AlgorithmGetPathString();
bool AlgorithmPopulatePathFromString(String& readData);

void AlgorithmSetCurrentPosition(long long x_cm, long long y_cm);

// Mission
void AlgorithmReset();                       // nothing mowed, goal idle, robot at the base
void AlgorithmSetGoal(coverage_goal goal);   // COVERAGE_MOW / COVERAGE_HOME / COVERAGE_IDLE
// The robot stopped where it is now (the motion task dropped its path): waypoints it did not
// reach are not mowed, the next one starts from here. skip: give up the piece it was on.
void AlgorithmStoppedAt(bool skip);
void AlgorithmReplan();                      // constraints changed: plan again from what was handed out
bool AlgorithmMowingDone();
// Mow only rows first .. first + count - 1 (CoveragePlan::only_rows), until the next plan build;
// count <= 0: everything. Returns the number of rows that are mowed.
int AlgorithmOnlyRows(int first, int count);
std::vector<double> AlgorithmNextPoint();    // {x m, y m, tool (nav_tool), edge}; empty = goal reached
void AlgorithmMotionSetTargetPoint(double x_m, double y_m, uint8_t tool, int8_t edge);   // edge: see motion_command_t
void AlgorithmSeedStartPosition();           // the next leg starts at the current position

const CoveragePlan& AlgorithmPlan();

#endif
