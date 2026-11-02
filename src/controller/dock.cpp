#include "dock.h"
#include "../functions.h"
#include "../hil/hil.h"
#include "../debug/debug_serial.h"
#include "../configuration/configuration.h"

// -------------------------------------------------------- POWER ---------------------------------------------------------------

static float dock_v = 0.0f, battery_v = 0.0f;
static bool  battery_known = false;

void power_init() {
#if POWER_ADC_FITTED
    analogSetPinAttenuation(DOCK_SENSE_PIN, ADC_11db);
    analogSetPinAttenuation(BATTERY_LEVEL_PIN, ADC_11db);
#endif
}

void power_poll() {
    if (HIL_ENABLED) {
        float d, b;
        if (hil_power_get(&d, &b)) {
            dock_v = d;
            battery_v = b;
            battery_known = b > 1.0f;
        } else {
            dock_v = 0.0f;
            battery_known = false;
        }
        return;
    }
#if POWER_ADC_FITTED
    dock_v = analogReadMilliVolts(DOCK_SENSE_PIN) * 0.001f * POWER_DIVIDER_RATIO;
    battery_v = analogReadMilliVolts(BATTERY_LEVEL_PIN) * 0.001f * POWER_DIVIDER_RATIO;
    battery_known = true;
#else
    dock_v = 0.0f;
    battery_known = false;
#endif
}

float power_dock_v() { return dock_v; }
bool  power_contact() { return dock_v > DOCK_V_MIN; }
bool  power_battery_known() { return battery_known; }
float power_battery_v() { return battery_v; }
int   power_battery_pct() {
    if (!battery_known) return -1;
    float p = (battery_v - BATTERY_V_EMPTY) / (BATTERY_V_FULL - BATTERY_V_EMPTY) * 100.0f;
    return (int)fmaxf(0.0f, fminf(100.0f, roundf(p)));
}

// -------------------------------------------------------- TRACK ---------------------------------------------------------------

static dock_sample_t* track = nullptr;   // on the heap: static DRAM is full
static int track_n = 0;

void dock_track_reset() {
    if (!track) track = new dock_sample_t[DOCK_TRACK_SAMPLES];
    track_n = 0;
}

bool dock_track_add(double x, double y, double gyro) {
    if (!track || track_n >= DOCK_TRACK_SAMPLES) return false;
    track[track_n++] = { x, y, gyro };
    return true;
}

int dock_track_count() { return track_n; }
const dock_sample_t& dock_track_at(int i) { return track[i]; }

bool dock_track_heading(int ref, bool reverse, float yaw_guess, float* yaw, float* spread_deg, float* chord_m,
                        float* drift_deg) {
    int n = track_n;
    if (!track || n < 3 || ref < 0 || ref >= n) return false;
    const double g0 = track[ref].gyro;
    const float L = GNSS_ANTENNA_BEHIND_AXLE;
    float est = yaw_guess;
    float spread = 0.0f, longest = 0.0f, drift = 0.0f;
    int count = 0;
    for (int pass = 0; pass < 2; pass++) {
        // Turned-back cumulative axle positions: P[k] in the frame of the heading at ref
        double px = 0.0, py = 0.0, rx = 0.0, ry = 0.0;
        double* cx = new double[n];
        double* cy = new double[n];
        cx[0] = 0.0; cy[0] = 0.0;
        for (int k = 1; k < n; k++) {
            double ha = est + (track[k - 1].gyro - g0), hb = est + (track[k].gyro - g0);
            double ax0 = track[k - 1].x + L * cos(ha), ay0 = track[k - 1].y + L * sin(ha);
            double ax1 = track[k].x + L * cos(hb), ay1 = track[k].y + L * sin(hb);
            double dx = ax1 - ax0, dy = ay1 - ay0;
            double turn = -(0.5 * (track[k - 1].gyro + track[k].gyro) - g0);   // back to the heading at ref
            double c = cos(turn), s = sin(turn);
            px += c * dx - s * dy;
            py += s * dx + c * dy;
            cx[k] = px; cy[k] = py;
        }
        rx = cx[ref]; ry = cy[ref];
        // Each chord sample -> ref in time order gives the direction of travel at ref
        double sx = 0.0, sy = 0.0;
        count = 0;
        longest = 0.0f;
        float* ests = new float[n];
        float* lens = new float[n];
        for (int k = 0; k < n; k++) {
            if (k == ref) continue;
            double vx = (k < ref) ? rx - cx[k] : cx[k] - rx;
            double vy = (k < ref) ? ry - cy[k] : cy[k] - ry;
            float len = (float)hypot(vx, vy);
            if (len > longest) longest = len;
            if (len < DOCK_MIN_CHORD) continue;
            float h = (float)atan2(vy, vx) + (reverse ? (float)M_PI : 0.0f);
            lens[count] = len;
            ests[count++] = h;
            sx += cos(h);
            sy += sin(h);
        }
        delete[] cx;
        delete[] cy;
        if (count < 3) { delete[] ests; delete[] lens; return false; }
        est = (float)atan2(sy, sx);
        double ss = 0.0, near_s = 0.0, far_s = 0.0;
        int near_n = 0, far_n = 0;
        const float split = 0.5f * (DOCK_MIN_CHORD + longest);
        for (int i = 0; i < count; i++) {
            float d = wrap_pi(ests[i] - est);
            ss += (double)d * d;
            if (lens[i] < split) { near_s += d; near_n++; } else { far_s += d; far_n++; }
        }
        spread = (float)sqrt(ss / count) * (float)RAD_TO_DEG;
        drift = (near_n >= 3 && far_n >= 3) ? (float)(far_s / far_n - near_s / near_n) * (float)RAD_TO_DEG : 0.0f;
        delete[] ests;
        delete[] lens;
    }
    *yaw = est;
    *spread_deg = spread;
    *chord_m = longest;
    if (drift_deg) *drift_deg = drift;
    return true;
}

// -------------------------------------------------------- STATION ESTIMATE ---------------------------------------------------------------

dock_station_t DOCK_STATION = { false, 0.0, 0.0, 0.0f, 0 };

// Residuals of the last accepted measurements (RAM only: after a reboot the gate floors apply)
#define DOCK_RECENT 8
static float recent_pos[DOCK_RECENT], recent_yaw[DOCK_RECENT];
static int recent_n = 0, recent_i = 0;
// Held-back measurements in a row
struct Meas { double x, y; float yaw; };
static Meas held[DOCK_MOVE_COUNT];
static int held_n = 0;

void dock_station_init(double x, double y, float yaw) {
    DOCK_STATION = { true, x, y, wrap_pi(yaw), 1 };
    recent_n = recent_i = 0;
    held_n = 0;
}

static float rms(const float* v, int n) {
    double s = 0.0;
    for (int i = 0; i < n; i++) s += (double)v[i] * v[i];
    return n ? (float)sqrt(s / n) : 0.0f;
}

int dock_station_measure(double x, double y, float yaw) {
    dock_station_t& S = DOCK_STATION;
    float c = cosf(S.yaw), s = sinf(S.yaw);
    double dx = x - S.x, dy = y - S.y;
    float along = (float)(c * dx + s * dy), lat = (float)(-s * dx + c * dy);
    float dpos = hypotf(along, lat);
    float dyaw = wrap_pi(yaw - S.yaw);
    float gate_pos = fmaxf(DOCK_GATE_MIN_POS, DOCK_GATE_SIGMAS * (recent_n >= 3 ? rms(recent_pos, recent_n) : 0.0f));
    float gate_yaw = fmaxf(DOCK_GATE_MIN_YAW_DEG * (float)DEG_TO_RAD,
                           DOCK_GATE_SIGMAS * (recent_n >= 3 ? rms(recent_yaw, recent_n) : 0.0f));
    if (dpos <= gate_pos && fabsf(dyaw) <= gate_yaw) {
        S.x += DOCK_EST_WEIGHT * dx;
        S.y += DOCK_EST_WEIGHT * dy;
        S.yaw = wrap_pi(S.yaw + DOCK_EST_WEIGHT * dyaw);
        if (S.n < 65535) S.n++;
        recent_pos[recent_i] = dpos;
        recent_yaw[recent_i] = dyaw;
        recent_i = (recent_i + 1) % DOCK_RECENT;
        if (recent_n < DOCK_RECENT) recent_n++;
        held_n = 0;
        SerialDebug.printf("[DOCK] Measurement %.1f cm along, %.1f cm lateral, %.2f deg: blended in\r\n",
                           along * 100.0f, lat * 100.0f, dyaw * RAD_TO_DEG);
        return DOCK_MEAS_ACCEPTED;
    }
    // Outside the gate: held back. A run of them that agree with each other = the station moved.
    if (held_n > 0) {
        const Meas& h0 = held[0];
        bool agree = hypot(x - h0.x, y - h0.y) <= DOCK_MOVE_AGREE_POS &&
                     fabsf(wrap_pi(yaw - h0.yaw)) <= DOCK_MOVE_AGREE_YAW_DEG * (float)DEG_TO_RAD;
        if (!agree) held_n = 0;
    }
    held[held_n++] = { x, y, yaw };
    SerialDebug.printf("[DOCK] Measurement %.1f cm along, %.1f cm lateral, %.2f deg: outside the gate "
                       "(%.1f cm, %.2f deg), held back (%d in a row)\r\n", along * 100.0f, lat * 100.0f,
                       dyaw * RAD_TO_DEG, gate_pos * 100.0f, gate_yaw * RAD_TO_DEG, held_n);
    if (held_n < DOCK_MOVE_COUNT) return DOCK_MEAS_GATED;
    double mx = 0.0, my = 0.0, sx = 0.0, sy = 0.0;
    for (int i = 0; i < held_n; i++) {
        mx += held[i].x; my += held[i].y;
        sx += cos(held[i].yaw); sy += sin(held[i].yaw);
    }
    uint16_t n = S.n;
    dock_station_init(mx / held_n, my / held_n, (float)atan2(sy, sx));
    S.n = n;
    SerialDebug.printf("[DOCK] Station moved: adopted x=%.3f y=%.3f yaw=%.2f deg (%d docks agreed)\r\n",
                       S.x, S.y, S.yaw * RAD_TO_DEG, DOCK_MOVE_COUNT);
    return DOCK_MEAS_ADOPTED;
}
