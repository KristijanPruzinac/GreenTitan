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

float AngleBetweenPoints(float start_x, float start_y, float end_x, float end_y) {
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    return normalize_angle(atan2f(dy, dx));
}

float DistanceFromLine(float x, float y,
                       float start_x, float start_y,
                       float end_x,   float end_y) {
    float dx = end_x - start_x;
    float dy = end_y - start_y;

    float length = sqrtf(dx * dx + dy * dy);
    if (length < 1e-6f) return 0.0f;  // degenerate line (start == end)

    // Cross product of line vector and point vector
    // Negated so that RIGHT of line = positive
    return (dy * (x - start_x) - dx * (y - start_y)) / length;
}

float DistanceBetweenPoints(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx*dx + dy*dy);
}

bool has_passed_goal(float x, float y, float start_x, float start_y, float end_x, float end_y) {
    float dx = end_x - start_x;
    float dy = end_y - start_y;
    float length_sq = dx*dx + dy*dy;
    
    if (length_sq < 0.001f) return true;
    
    float rx = x - start_x;
    float ry = y - start_y;
    float t = (rx*dx + ry*dy) / length_sq;
    
    return (t >= 1.0f);  // Passed the endpoint
}

float normalize_angle(float radians) {
    radians = fmodf(radians, 2.0f * M_PI);
    if (radians < 0) radians += 2.0f * M_PI;
    return radians;
}

float angle_diff(float current, float target) {
    float diff = fmodf(target - current, 2.0f * M_PI);
    if (diff > M_PI)  diff -= 2.0f * M_PI;
    if (diff < -M_PI) diff += 2.0f * M_PI;
    return diff;
}

int sign_of(float value) {
    if (value > 0.0f) return  1;
    if (value < 0.0f) return -1;
    return 0;
}