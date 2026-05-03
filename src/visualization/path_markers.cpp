#include "path_markers.h"

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

extern HardwareSerial SerialDebug;

#define MAX_PATH_DATA_DOUBLES  512

static double path_buffer[MAX_PATH_DATA_DOUBLES];
static std_msgs__msg__Float64MultiArray path_msg;

void publish_path_markers(rcl_publisher_t* path_pub) {
    const std::vector<std::vector<std::vector<long long>>>& extOutlines = AlgorithmGetExtOutlines();
    const std::vector<std::vector<std::vector<long long>>>& intersectionPaths = AlgorithmGetIntersectionPaths();

    if (extOutlines.empty()) return;

    int idx = 0;

    // Outlines
    path_buffer[idx++] = (double)extOutlines.size();
    for (size_t o = 0; o < extOutlines.size(); o++) {
        int n = (int)extOutlines[o].size();
        if (idx + 1 + n * 2 > MAX_PATH_DATA_DOUBLES) return;
        path_buffer[idx++] = (double)n;
        for (int p = 0; p < n; p++) {
            path_buffer[idx++] = extOutlines[o][p][0] / 100.0;
            path_buffer[idx++] = extOutlines[o][p][1] / 100.0;
        }
    }

    // Scan lines
    path_buffer[idx++] = (double)intersectionPaths.size();
    for (size_t s = 0; s < intersectionPaths.size(); s++) {
        int m = (int)intersectionPaths[s].size();
        if (idx + 1 + m * 2 > MAX_PATH_DATA_DOUBLES) return;
        path_buffer[idx++] = (double)m;
        for (int c = 0; c < m; c++) {
            int o = (int)intersectionPaths[s][c][0];
            int p = (int)intersectionPaths[s][c][1];
            path_buffer[idx++] = extOutlines[o][p][0] / 100.0;
            path_buffer[idx++] = extOutlines[o][p][1] / 100.0;
        }
    }

    // Init message fields
    path_msg.layout.dim.data = NULL;
    path_msg.layout.dim.size = 0;
    path_msg.layout.dim.capacity = 0;
    path_msg.layout.data_offset = 0;

    path_msg.data.data = path_buffer;
    path_msg.data.size = idx;
    path_msg.data.capacity = idx;

    RCSOFTCHECK(rcl_publish(path_pub, &path_msg, NULL));
    SerialDebug.printf("[MARKERS] Path published (%d doubles)\r\n", idx);
}