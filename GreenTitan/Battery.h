//Sampling and voltage divider
const int battery_sampling_count = 100;
const float battery_divider_factor = 1.0 / 11.0;
int battery_readings[battery_sampling_count];

//Operating range
float batteryLevelMin = 16;
float batteryLevelMax = 18;

//Last read battery data
float batteryCurrentLevel = 0;
float batteryCurrentPercentage = 0;

//Power states
#define BATTERY_LEVEL_LOW 8

void InitBattery(){
  pinMode(BATTERY_level_pin, INPUT);

  for (int i = 0; i < battery_sampling_count; i++){
    battery_readings[i] = analogRead(BATTERY_level_pin);
  }
}

float BatteryRead(){
  //Average over sampling data
  int avg = 0;
  for (int i = 0; i < battery_sampling_count; i++){
    avg += battery_readings[i];
  }
  avg /= battery_sampling_count;

  return batteryCurrentLevel = ((avg / 4096.0) * 3.3) / battery_divider_factor;
}

int BatteryReadPercentage(){
  float val = BatteryRead();

  if (val <= batteryLevelMin){return 0;}
  if (val >= batteryLevelMax){return 100;}

  return batteryCurrentPercentage = (int) ((val - batteryLevelMin) / (batteryLevelMax - batteryLevelMin) * 101.0);
}

void BatteryUpdate(){
  //Shift values back
  for (int i = 0; i < battery_sampling_count - 1; i++){
    battery_readings[i] = battery_readings[i + 1];
  }

  //Read new value
  battery_readings[battery_sampling_count - 1] = analogRead(BATTERY_level_pin);

  int BatPercentage = BatteryReadPercentage();
  if (BatPercentage < BATTERY_LEVEL_LOW){
    STATUS_BATTERY_LOW = true;
  }
  else {
    STATUS_BATTERY_LOW = false;
  }

  //Save last ADC and percentage values
  BatteryReadPercentage();
}
