// BNO055Sensor.cpp
#include "BNO055Sensor.h"

BNO055Sensor::BNO055Sensor(uint8_t sda_pin, uint8_t scl_pin, uint32_t i2c_freq)
    : I2CBNO(1), bno(55, BNO055_ADDRESS_A, &I2CBNO), _sda_pin(sda_pin), _scl_pin(scl_pin), _i2c_freq(i2c_freq) {}

bool BNO055Sensor::begin() {
    I2CBNO.begin(_sda_pin, _scl_pin, _i2c_freq);
    delay(10000);
    
    if (!bno.begin()) {
        return false;
    }
    
    bno.setMode(OPERATION_MODE_NDOF);
    delay(1000);
    
    if (bno.getMode() != OPERATION_MODE_NDOF) {
        return false;
    }
    
    bno.setExtCrystalUse(true);
    return true;

    measurementPeriod = 1000; // Increase to 100ms
    lastMeasurementTime = 0;
    lastPitch = 0;
    palstance = 0;
    angleY = 0;

}

void BNO055Sensor::update() {
    
    sensors_event_t event;
    bno.getEvent(&event);
    angleY = event.orientation.y;
    updatePalstance();
    //palstance = event.gyro.;
    // This method is left empty as the sensor updates automatically
}



int BNO055Sensor::calibration() {
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    return (system);
}


void BNO055Sensor::updatePalstance() {
    unsigned long currentTime = millis();
    if (currentTime - lastMeasurementTime >= measurementPeriod) {
        //float timeInSeconds = measurementPeriod / 1000.0f;
        float deltaY = (angleY - lastPitch) * PI / 180;
        
        // Only update palstance if the change is significant
        //if (abs(deltaY) > 0.01) { // Adjust this threshold as needed
        palstance = deltaY / (measurementPeriod / 1000.0f);
        // } else {
        //     palstance = 0; // Set to 0 if change is insignificant
        // }
       // Serial.printf("Delta: %f, Period: %f, angleY: %f", deltaY, measurementPeriod, angleY);
       // Serial.println("");

        lastMeasurementTime = currentTime;
        lastPitch = angleY;
    }
}


