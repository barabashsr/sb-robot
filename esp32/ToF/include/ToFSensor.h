#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

class ToFSensor
{
private:
    SparkFun_VL53L5CX _sensor;
    TwoWire &_i2c;
    int _pwrEn_pin;
    int _lpn_pin;
    int _i2c_rst_pin;
    int _int_pin;
    bool _isInitialized;
    unsigned int _resolution;
    unsigned int _frequency;

    int _imageResolution = 0; // Used to pretty print output
    int _imageWidth = 0;      // Used to pretty print output

    // Default I2C address for VL53L5CX
    uint8_t _castomAddress;

    // Sensor data
    VL53L5CX_ResultsData _results;

    // Static interrupt handler
    static void IRAM_ATTR interruptHandler();
    static volatile bool _dataReady;
    static ToFSensor* _instance;

public:
public:
    ToFSensor(TwoWire &wire, 
        int pwrEn = -1, 
        int lpn = -1, 
        int i2c_rst = -1, 
        int interrupt = -1, 
        uint8_t customAddress = -1,
        unsigned int resolution = 64,
        unsigned int frequency = 15
    );

  

    bool begin();
    bool setSensorAddress(uint8_t address);
    bool setIdle();
    bool setLowPower();
    bool setRanging();
    bool setReset();
    // Interrupt handling methods
    bool setupInterrupt();
    bool didInterrupt();
    bool readDataOnInterrupt(VL53L5CX_ResultsData& results);

    bool readData(VL53L5CX_ResultsData &results);

    // Additional methods
    bool setResolution(int resolution);
    bool setRangingFrequency(int frequency);
    void printData();
    int getIntegrationTime();
    bool setIntegrationTime(int integrationTimeMs);
    bool isConnected();
};