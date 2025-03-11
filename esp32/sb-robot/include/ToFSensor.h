#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

class ToFSensor {
private:
    SparkFun_VL53L5CX_IO sensor;
    TwoWire &i2c;
    int pwrEn_pin;
    int lpn_pin;
    int i2c_rst_pin;
    int int_pin;
    bool isInitialized;

    int imageResolution = 0; //Used to pretty print output
    int imageWidth = 0; //Used to pretty print output       
    
    // Default I2C address for VL53L5CX
    uint8_t _castomAddress;
    
    // Sensor data
    VL53L5CX_ResultsData _results;

public:
    ToFSensor(TwoWire &wire, int pwrEn = -1, int lpn = -1, int i2c_rst = -1, int interrupt = -1, uint8_t customAddress = NULL) : 
        i2c(wire),
        pwrEn_pin(pwrEn),
        lpn_pin(lpn),
        i2c_rst_pin(i2c_rst),
        int_pin(interrupt),
        isInitialized(false),
        _castomAddress(customAddress)
        {}
    
    bool begin();
    bool setSensorAddress(uint8_t address);
    bool setIdle();
    bool setLowPower();
    bool setRanging();
    bool setReset();
     
    
    
    bool readData(VL53L5CX_ResultsData& results); 
    
};
