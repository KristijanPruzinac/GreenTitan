#include "functions.h"

void Warning(String message){
  Serial.println("WARNING: " + message);
}

void Error(String message){
  Serial.println("ERROR: " + message);

  //TODO: Implement additional user feedback and error handling
  
  while (1){
    delay(100);
  }
  
}

float gaussian_noise(float stddev) {
    float u1 = (float)(esp_random()) / (float)(UINT32_MAX);
    float u2 = (float)(esp_random()) / (float)(UINT32_MAX);
    return stddev * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
}

static bool status_led_on = false;
void toggle_status_led() {
  pinMode(LED_BUILTIN, OUTPUT);
  status_led_on = !status_led_on;
  digitalWrite(LED_BUILTIN, status_led_on);
}

void error() {
  toggle_status_led();
  delay(200);
  toggle_status_led();
  delay(200);

  toggle_status_led();
  delay(200);
  toggle_status_led();
  delay(200);

  toggle_status_led();
  delay(200);
  toggle_status_led();
  delay(800);

  while (1)
  {
    delay(1000);
  }
  
}

void warning() {
  toggle_status_led();
  delay(200);
  toggle_status_led();
  delay(800);
}

float AngleBetweenPoints(double start_x, double start_y, double end_x, double end_y) {
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    return normalize_angle(atan2f(dy, dx));
}

double DistanceFromLine(double x,   double y,
                       double start_x, double start_y,
                       double end_x,   double end_y) {
    double dx = end_x - start_x;
    double dy = end_y - start_y;

    double length = sqrtf(dx * dx + dy * dy);
    if (length < 1e-6f) return 0.0f;  // degenerate line (start == end)

    // Cross product of line vector and point vector
    // Negated so that RIGHT of line = positive
    return (dy * (x - start_x) - dx * (y - start_y)) / length;
}

double DistanceBetweenPoints(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrtf(dx*dx + dy*dy);
}

bool has_passed_goal(double x, double y, double start_x, double start_y, double end_x, double end_y) {
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double length_sq = dx*dx + dy*dy;
    
    if (length_sq < 0.001f) return true;
    
    double rx = x - start_x;
    double ry = y - start_y;
    double t = (rx*dx + ry*dy) / length_sq;
    
    return (t >= 1.0f);  // Passed the endpoint
}

float normalize_angle(float radians) {
    radians = fmod(radians, 2.0 * M_PI);
    if (radians < 0) radians += 2.0 * M_PI;
    return radians;
}

float angle_diff(float current, float target) {
    float diff = fmod(target - current, 2.0 * M_PI);
    if (diff > M_PI)  diff -= 2.0f * M_PI;
    if (diff < -M_PI) diff += 2.0f * M_PI;
    return diff;
}

int sign_of(double value) {
    if (value > 0.0) return  1;
    if (value < 0.0) return -1;
    return 0;
}