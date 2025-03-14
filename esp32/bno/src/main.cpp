#include "BNO055Sensor.h"

BNO055Sensor bno(32, 33); // SDA, SCL

void setup() {
    Serial.begin(115200);
    
    if (!bno.begin()) {
        Serial.println("BNO055 initialization failed");
        while (1);
    }
    
    Serial.println("BNO055 initialized successfully");
}

void loop() {
    if (!bno.isCalibrated()) {
        Serial.println("Sensor not fully calibrated");
    }

    float angle_y = bno.getAngleY();
    Serial.print("Angle between Z axis and vertical: ");
    Serial.print(angle_y);
    Serial.println(" degrees");
    
    delay(100);
}
