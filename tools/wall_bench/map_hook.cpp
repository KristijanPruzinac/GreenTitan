// Wall bench: prints the firmware's ToF map for bench.py (command M). Built only when
// src/sensor/tof_map.h exists; build.py leaves it out (with a warning) if it doesn't compile, e.g.
// when MapObject's fields differ from the ones assumed here: adapt the printf line, keeping the
// output "map <kind> <x0> <y0> <x1> <y1>" (world metres, kind 0 wall, 1 fence, 2 low, 3 hedge, 4 post: x0 y0 centre, x1 radius).
#include <stdio.h>
#include "sensor/tof_map.h"

void bench_map_print() {
  static MapObject objects[128];
  int n = tof_map_copy(objects, 128);
  for (int i = 0; i < n; i++) {
    const MapObject& o = objects[i];
    printf("map %d %.3f %.3f %.3f %.3f\n", (int)o.kind, (double)o.x0, (double)o.y0, (double)o.x1, (double)o.y1);
  }
}
