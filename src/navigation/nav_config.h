#ifndef NAV_CONFIG_H
#define NAV_CONFIG_H

// Robot description for the navigation core (src/navigation). The core knows nothing about
// mowers: it drives a differential-drive base so that a *tool point* (here the blade, could be a
// sensor or nothing) follows a path of waypoints. Fill this in once per robot.
struct NavConfig {
    // Geometry and limits of the drive
    float wheel_base;         // m, distance between the drive wheels
    float tool_offset;        // m, tool point behind the wheel axle (0 = on the axle)
    float v_max;              // m/s, path speed
    float v_min;              // m/s, creep speed, so the last centimetres still close
    float w_max;              // rad/s, turn rate
    float wheel_v_max;        // m/s, per wheel
    float accel;              // m/s^2 along the path; stay under what the drive can ramp

    // Corners of the waypoint path: rounded into arcs of the tool path where they fit, else the
    // robot stops and turns on the spot (with the axle on the corner). min_radius must exceed
    // tool_offset: on an arc of radius r the axle runs at sqrt(r^2 - tool_offset^2), inside the
    // tool. Entering a curve the axle first swings out, since a trailing tool moves outward as
    // soon as the axle turns. With tool_offset = 0 the axle follows the path itself.
    float corner_radius;      // m, preferred tool-path radius
    float min_radius;         // m, tighter corners turn on the spot
    float max_rounded_turn;   // rad, sharper corners turn on the spot
    float path_tolerance;     // m, waypoints that bend the path less than this are merged away

    // Tracking (MPC per metre of path). Errors larger than max_heading_error are outside what
    // the linear model covers; the robot turns on the spot back onto the path heading first.
    float max_heading_error;  // rad
    float q_lateral;          // weight on lateral error
    float q_heading;          // weight on heading error
    float q_curvature;        // weight on curvature deviating from the path's
    float r_curvature_rate;   // weight on changing curvature (smoothness)
};

#endif
