#include "ToFSensor.h"

ToFSensor::ToFSensor(TwoWire &wire, int pwrEn, int lpn, int i2c_rst, int interrupt, uint8_t customAddress) : 
    i2c(wire),
    pwrEn_pin(pwrEn),
    lpn_pin(lpn),
    i2c_rst_pin(i2c_rst),
    int_pin(interrupt),
    isInitialized(false),
    _castomAddress(customAddress)
    {}

bool ToFSensor::begin() {}

bool ToFSensor::readData(VL53L5CX_ResultsData& results) {}

bool ToFSensor::setSensorAddress(uint8_t address) {}

bool ToFSensor::setIdle() {}

bool ToFSensor::setLowPower() {}

bool ToFSensor::setRanging() {}

bool ToFSensor::setReset() {}