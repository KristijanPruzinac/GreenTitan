#pragma once
#include <rcl/rcl.h>
#include <std_msgs/msg/float64_multi_array.h>
#include "algorithm/algorithm.h"

void publish_path_markers(rcl_publisher_t* path_pub);