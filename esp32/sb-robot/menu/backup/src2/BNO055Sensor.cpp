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
}

void BNO055Sensor::update() {
    // This method is left empty as the sensor updates automatically
}

float BNO055Sensor::getAngleY() {
    sensors_event_t event;
    bno.getEvent(&event);
    return event.orientation.y;
}

bool BNO055Sensor::isCalibrated() {
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    return (system >= 3);
}