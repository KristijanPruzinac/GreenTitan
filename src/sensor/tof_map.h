#ifndef TOF_MAP_H
#define TOF_MAP_H

// The ToF object map (ARCHITECTURE §6, wall assist): walls, fences, low edges, hedges and posts
// the two ToF sensors have confirmed over several frames, in the world (EKF odom) frame. Built in
// tof_task; any task may take a snapshot.

#include <stdint.h>

enum MapKind : uint8_t { MAP_WALL = 0, MAP_FENCE = 1, MAP_LOW = 2, MAP_HEDGE = 3, MAP_POST = 4 };
struct MapObject { float x0, y0, x1, y1; float height; uint8_t kind; uint8_t top_seen; uint16_t hits; uint32_t t_ms; };  // world m; post: x0,y0 centre, x1 radius
int tof_map_copy(MapObject* out, int max);   // thread-safe snapshot, returns the count
void tof_map_clear();

#endif
