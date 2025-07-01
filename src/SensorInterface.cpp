#include "SensorInterface.h"

float IMUGetRotationSpeedRaw(){
    xSemaphoreTake(IMUMutex, portMAX_DELAY);
    float rotSpeed = IMUGetGyroZ();
    xSemaphoreGive(IMUMutex);
    return rotSpeed;
}

float IMUGetRotationSpeed(){
    xSemaphoreTake(IMUMutex, portMAX_DELAY);
    float rotSpeed = IMURotSpeed;
    xSemaphoreGive(IMUMutex);
    return rotSpeed;
}
float IMUGetRotationAcceleration(){
    xSemaphoreTake(IMUMutex, portMAX_DELAY);
    float rotAcc = IMURotAcc;
    xSemaphoreGive(IMUMutex);
    return rotAcc;
}

//DEBUG
void PlotIMUData(){
    xSemaphoreTake(IMUMutex, portMAX_DELAY);
    Serial.print(">");
    Serial.print("IMURotSpeed:");
    Serial.print(IMURotSpeed);
    Serial.print(",IMURotAcc:");
    Serial.print(IMURotAcc);
    Serial.print("IMUHeadingChange:");
    Serial.print(IMUHeadingChange);
    Serial.print("\r\n");
    xSemaphoreGive(IMUMutex);
}