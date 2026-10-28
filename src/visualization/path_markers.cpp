#include "path_markers.h"

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

#include "../debug/debug_serial.h"

// 200 doubles = 1.6 kB, inside one reliable micro-ROS message (MTU 512 x 4 output slots).
// A plan that doesn't fit is sent truncated (whole rings first, then as many rows as fit).
#define MAX_PATH_DATA_DOUBLES  200

static double path_buffer[MAX_PATH_DATA_DOUBLES];
static std_msgs__msg__Float64MultiArray path_msg;

void publish_path_markers(rcl_publisher_t* path_pub) {
    const CoveragePlan& plan = AlgorithmPlan();
    if (!plan.built()) return;

    int idx = 0;
    bool truncated = false;

    // Outlines: the lawn, then the turn zone (where the rows end)
    int outline_count_idx = idx++;
    int outline_count = 0;
    const MapPaths* ring_sets[2] = { &plan.map().lawn(), &plan.map().turn_zone().area };
    for (int k = 0; k < 2 && !truncated; k++) {
        for (const MapPath& ring : *ring_sets[k]) {
            int n = (int)ring.size();
            if (idx + 1 + n * 2 + 1 > MAX_PATH_DATA_DOUBLES) { truncated = true; break; }  // +1: scan line count
            path_buffer[idx++] = (double)n;
            for (const MapPoint& q : ring) {
                path_buffer[idx++] = q.x / 1000.0;
                path_buffer[idx++] = q.y / 1000.0;
            }
            outline_count++;
        }
    }
    path_buffer[outline_count_idx] = (double)outline_count;

    // Scan lines: the infill rows
    int scan_count_idx = idx++;
    int scan_count = 0;
    for (const CoveragePlan::Piece& p : plan.pieces()) {
        if (p.lap) continue;
        if (idx + 1 + 4 > MAX_PATH_DATA_DOUBLES) { truncated = true; break; }
        path_buffer[idx++] = 2.0;
        for (int c = 0; c < 2; c++) {
            path_buffer[idx++] = p.pts[c].x / 1000.0;
            path_buffer[idx++] = p.pts[c].y / 1000.0;
        }
        scan_count++;
    }
    path_buffer[scan_count_idx] = (double)scan_count;

    // Init message fields
    path_msg.layout.dim.data = NULL;
    path_msg.layout.dim.size = 0;
    path_msg.layout.dim.capacity = 0;
    path_msg.layout.data_offset = 0;

    path_msg.data.data = path_buffer;
    path_msg.data.size = idx;
    path_msg.data.capacity = idx;

    RCSOFTCHECK(rcl_publish(path_pub, &path_msg, NULL));
    SerialDebug.printf("[MARKERS] Path published (%d doubles%s)\r\n", idx, truncated ? ", truncated" : "");
}
