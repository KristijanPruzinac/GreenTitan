#include "algorithm.h"
#include <new>

static volatile long long current_x_cm = 0;
static volatile long long current_y_cm = 0;

// Where the leg handed out last starts and ends (the motion task tracks from one to the other)
static double prev_x_m = 0, prev_y_m = 0;
static double target_x_m = 0, target_y_m = 0;

static std::vector<std::vector<std::vector<long long>>> outlines;   // cm, as captured
static CoveragePlan plan;

static void plan_log(const char* message) {
    SerialDebug.printf("[PLAN] %s\r\n", message);
}

static double now_x_m() { return current_x_cm / 100.0; }
static double now_y_m() { return current_y_cm / 100.0; }

// ---------------------------------------------------------------- MISSION

void AlgorithmReset() {
    plan.reset();
    prev_x_m = target_x_m = plan.base().x / 1000.0;
    prev_y_m = target_y_m = plan.base().y / 1000.0;
}

void AlgorithmSetGoal(coverage_goal goal) {
    plan.set_goal(goal);
}

void AlgorithmStoppedAt(bool skip) {
    plan.stopped_at(now_x_m(), now_y_m(), skip);
}

void AlgorithmReplan() {
    plan.replan();
}

bool AlgorithmMowingDone() {
    return plan.mowed();
}

std::vector<double> AlgorithmNextPoint() {
    CoveragePoint p;
    uint32_t t0 = micros();
    bool ok = plan.next(&p);
    uint32_t us = micros() - t0;
    if (us > 5000) SerialDebug.printf("[ALGO] next point took %lu ms\r\n", (unsigned long)(us / 1000));
    if (!ok) return std::vector<double>();
    uint8_t tool = p.blade == COVERAGE_BLADE_ON  ? NAV_TOOL_ON  :
                   p.blade == COVERAGE_BLADE_OFF ? NAV_TOOL_OFF : NAV_TOOL_KEEP;
    prev_x_m = target_x_m;
    prev_y_m = target_y_m;
    target_x_m = p.x;
    target_y_m = p.y;
    return std::vector<double>{ p.x, p.y, (double)tool, (double)p.edge };
}

void AlgorithmMotionSetTargetPoint(double x_m, double y_m, uint8_t tool, int8_t edge) {
    motion_command_t cmd = { MOVING, prev_x_m, prev_y_m, x_m, y_m, tool, edge };
    dds_result_t result = DDS_PUBLISH("/motion/command", cmd);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[ALGO] Failed to send motion command: %d\r\n", result);
    }
}

void AlgorithmSeedStartPosition() {
    prev_x_m = target_x_m = now_x_m();
    prev_y_m = target_y_m = now_y_m();
}

void AlgorithmSetCurrentPosition(long long x_cm, long long y_cm) {
    current_x_cm = x_cm;
    current_y_cm = y_cm;
}

const CoveragePlan& AlgorithmPlan() {
    return plan;
}

// ---------------------------------------------------------------- CAPTURE

void AlgorithmCaptureStart() {
    outlines.clear();
}

void AlgorithmCaptureNewOutline() {
    outlines.push_back(std::vector<std::vector<long long>>());
}

void AlgorithmCaptureNewPoint() {
    AlgorithmCaptureSetNewPoint(current_x_cm, current_y_cm);
}

void AlgorithmCaptureSetNewPoint(long long x_cm, long long y_cm) {
    if (outlines.empty()) AlgorithmCaptureNewOutline();
    outlines.back().push_back(std::vector<long long>{ x_cm, y_cm });
}

bool AlgorithmCaptureRemoveOutline() {
    if (outlines.empty()) return false;
    outlines.pop_back();
    return true;
}

bool AlgorithmCaptureRemovePoint() {
    if (outlines.empty() || outlines.back().empty()) return false;
    outlines.back().pop_back();
    return true;
}

bool AlgorithmCaptureEnd() {
    MapPaths paths;
    for (const auto& outline : outlines) {
        if (outline.size() < 3) continue;   // a stray "new outline" or a point or two
        MapPath path;
        for (const auto& q : outline) path.push_back(MapPoint(q[0] * 10, q[1] * 10));
        paths.push_back(path);
    }
    CoverageConfig cfg = {
        MOWER_EDGE_INSET, MOWER_TURN_INSET, MOWER_BLADE_DIAMETER - MOWER_OVERLAP / 100.0, MOTION_MIN_CORNER_RADIUS, PLANNER_MIN_ROW, PLANNER_MARGIN,
        PLANNER_CORNER_TURN, PLANNER_LAP_JOIN, PLANNER_TOLERANCE, plan_log
    };
    uint32_t t0 = millis();
    uint32_t heap0 = ESP.getFreeHeap();
    bool ok = false;
    plan = CoveragePlan();   // the old plan's memory back first: the heap holds one plan at a time
    try {
        ok = !paths.empty() && plan.build(paths, cfg);
    } catch (const std::bad_alloc&) {
        plan = CoveragePlan();   // out of memory fails the capture instead of rebooting the mower
        SerialDebug.println("[ALGO] Out of memory building the plan");
    }
    SerialDebug.printf("[ALGO] Plan built in %lu ms, %u pieces, heap free %u -> %u B (largest block %u B)\r\n",
                       (unsigned long)(millis() - t0), (unsigned)plan.pieces().size(), (unsigned)heap0,
                       (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
    AlgorithmReset();
    return ok;
}

String AlgorithmGetPathString() {
    String s = "";
    for (const auto& outline : outlines) {
        s += "O\n";
        for (const auto& q : outline) s += String((long)q[0]) + " " + String((long)q[1]) + "\n";
    }
    return s;
}

bool AlgorithmPopulatePathFromString(String& readData) {
    AlgorithmCaptureStart();
    int newlineIndex;
    while ((newlineIndex = readData.indexOf('\n')) != -1) {
        String line = readData.substring(0, newlineIndex);
        readData = readData.substring(newlineIndex + 1);
        if (line.length() == 0) continue;
        if (line[0] == 'O') {
            AlgorithmCaptureNewOutline();
        } else {
            int spaceIndex = line.indexOf(' ');
            if (spaceIndex != -1) {
                AlgorithmCaptureSetNewPoint(line.substring(0, spaceIndex).toInt(), line.substring(spaceIndex + 1).toInt());
            }
        }
    }
    return AlgorithmCaptureEnd();
}
