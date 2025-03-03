// BNO055Sensor.h
#ifndef BNO055SENSOR_H
#define BNO055SENSOR_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



class BNO055Sensor {
public:
    BNO055Sensor(uint8_t sda_pin, uint8_t scl_pin, uint32_t i2c_freq = 40000);
    bool begin();
    void update();
    float getAngleY(){return angleY;};
    int calibration();
    void setMeasurementPeriod(int period){measurementPeriod = period;};
    void updatePalstance();
    float getPalstance(){return palstance;};

private:
    TwoWire I2CBNO;
    Adafruit_BNO055 bno;
    uint8_t _sda_pin;
    uint8_t _scl_pin;
    uint32_t _i2c_freq;
    unsigned long lastMeasurementTime;
    int measurementPeriod; // in milliseconds
    double lastPitch;
    double palstance;
    double angleY;
    double lastPalstance;
};

#endif // BNO055SENSOR_H