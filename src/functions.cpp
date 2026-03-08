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