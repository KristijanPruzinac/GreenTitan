#include "path_markers.h"

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

#include "../debug/debug_serial.h"

// 200 doubles = 1.6 kB, inside one reliable micro-ROS message (MTU 512 x 4 output slots).
// A path that doesn't fit is sent truncated (whole outlines first, then as many scan lines as fit).
#define MAX_PATH_DATA_DOUBLES  200

static double path_buffer[MAX_PATH_DATA_DOUBLES];
static std_msgs__msg__Float64MultiArray path_msg;

void publish_path_markers(rcl_publisher_t* path_pub) {
    const std::vector<std::vector<std::vector<long long>>>& extOutlines = AlgorithmGetExtOutlines();
    const std::vector<std::vector<std::vector<long long>>>& intersectionPaths = AlgorithmGetIntersectionPaths();

    if (extOutlines.empty()) return;

    int idx = 0;
    bool truncated = false;

    // Outlines
    int outline_count_idx = idx++;
    int outline_count = 0;
    for (size_t o = 0; o < extOutlines.size(); o++) {
        int n = (int)extOutlines[o].size();
        if (idx + 1 + n * 2 + 1 > MAX_PATH_DATA_DOUBLES) { truncated = true; break; }  // +1: scan line count
        path_buffer[idx++] = (double)n;
        for (int p = 0; p < n; p++) {
            path_buffer[idx++] = extOutlines[o][p][0] / 100.0;
            path_buffer[idx++] = extOutlines[o][p][1] / 100.0;
        }
        outline_count++;
    }
    path_buffer[outline_count_idx] = (double)outline_count;

    // Scan lines (they index into the outlines, so only while all outlines were sent)
    int scan_count_idx = idx++;
    int scan_count = 0;
    for (size_t s = 0; s < intersectionPaths.size() && !truncated; s++) {
        int m = (int)intersectionPaths[s].size();
        if (idx + 1 + m * 2 > MAX_PATH_DATA_DOUBLES) { truncated = true; break; }
        path_buffer[idx++] = (double)m;
        for (int c = 0; c < m; c++) {
            int o = (int)intersectionPaths[s][c][0];
            int p = (int)intersectionPaths[s][c][1];
            path_buffer[idx++] = extOutlines[o][p][0] / 100.0;
            path_buffer[idx++] = extOutlines[o][p][1] / 100.0;
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
