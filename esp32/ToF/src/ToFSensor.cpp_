#include "ToFSensor.h"

ToFSensor::ToFSensor(TwoWire &wire, int pwrEn, int lpn, int i2c_rst, int interrupt, uint8_t customAddress, unsigned int resolution,
    unsigned int frequency) : 
    _i2c(wire),
    _pwrEn_pin(pwrEn),
    _lpn_pin(lpn),
    _i2c_rst_pin(i2c_rst),
    _int_pin(interrupt),
    _isInitialized(false),
    _castomAddress(customAddress),
    _resolution(resolution)
    {


    }


    bool ToFSensor::begin() {
        // Initialize pins if provided
        if (_pwrEn_pin >= 0) {
            pinMode(_pwrEn_pin, OUTPUT);
            digitalWrite(_pwrEn_pin, HIGH);
            delay(10); // Give time for power to stabilize
        }
        
        if (_lpn_pin >= 0) {
            pinMode(_lpn_pin, OUTPUT);
            digitalWrite(_lpn_pin, HIGH);
            delay(10); // Reset the sensor
            digitalWrite(_lpn_pin, LOW);
            delay(10);
            digitalWrite(_lpn_pin, HIGH);
            delay(10);
        }
        
        if (_i2c_rst_pin >= 0) {
            pinMode(_i2c_rst_pin, OUTPUT);
            digitalWrite(_i2c_rst_pin, HIGH);
            delay(10);
            digitalWrite(_i2c_rst_pin, LOW);
            delay(10);
            digitalWrite(_i2c_rst_pin, HIGH);
            delay(10);
        }
        
        if (_int_pin >= 0) {
            pinMode(_int_pin, INPUT_PULLUP);
        }
        
        // Initialize sensor with custom address if provided
      
            if (!_sensor.begin(0x29,_i2c)) {
                Serial.println("VL53L5CX not detected. Check wiring.");
                return false;
            }
        
        
        // Set default resolution to 8x8
        _sensor.setResolution(_resolution);
        _imageResolution = _sensor.getResolution();
        _imageWidth = sqrt(_imageResolution);
        
        // Set default ranging frequency to 15Hz (max for 8x8 mode)
        _sensor.setRangingFrequency(_frequency);
        
        // Set target order to closest by default
        _sensor.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);

        _isInitialized = true;
        if (_int_pin >= 0) {
            if (!setupInterrupt()) {
            Serial.println("Failed to setup interrupt!");
            while (1) delay(10);
        } else {
        Serial.println("OK to setup interrupt!");
        }
    }
        _sensor.startRanging();
        
        
        return true;
    }
    

    bool ToFSensor::readData(VL53L5CX_ResultsData& results) {
        if (!_isInitialized) {
            Serial.println("Sensor not initialized. Call begin() first.");
            return false;
        }
        
        // Check if data is ready
        if (!_sensor.isDataReady()) {
            return false;
        }
        
        // Get ranging data
        if (!_sensor.getRangingData(&results)) {
            Serial.println("Error getting ranging data.");
            return false;
        } else {
            Serial.println("!!Got ranging data.");

        }
        
        // Copy data to internal results structure for later use if needed
        memcpy(&_results, &results, sizeof(VL53L5CX_ResultsData));
        
        return true;
    }
    

    bool ToFSensor::setSensorAddress(uint8_t address) {
        if (!_isInitialized) {
            Serial.println("Sensor not initialized. Call begin() first.");
            return false;
        }
        
        if (address < 0x08 || address > 0x77) {
            Serial.println("Invalid I2C address. Must be between 0x08 and 0x77.");
            return false;
        }
        
        if (!_sensor.setAddress(address)) {
            Serial.println("Failed to set new address.");
            return false;
        }
        
        _castomAddress = address;
        Serial.print("Sensor address changed to 0x");
        Serial.println(address, HEX);
        return true;
    }
    

    bool ToFSensor::setIdle() {
        if (!_isInitialized) {
            Serial.println("Sensor not initialized. Call begin() first.");
            return false;
        }
        
        // Stop ranging if it's active
        if (!_sensor.stopRanging()) {
            Serial.println("Failed to stop ranging.");
            return false;
        }
        
        return true;
    }
    
    bool ToFSensor::setLowPower() {
        if (!_isInitialized) {
            Serial.println("Sensor not initialized. Call begin() first.");
            return false;
        }
        
        // First stop ranging
        if (!setIdle()) {
            return false;
        }
        
        // Set to sleep mode
        if (!_sensor.setPowerMode(SF_VL53L5CX_POWER_MODE::SLEEP)) {
            Serial.println("Failed to set low power mode.");
            return false;
        }
        
        return true;
    }
    
    bool ToFSensor::setRanging() {
        if (!_isInitialized) {
            Serial.println("Sensor not initialized. Call begin() first.");
            return false;
        }
        
        // Wake up if in sleep mode
        SF_VL53L5CX_POWER_MODE currentMode = _sensor.getPowerMode();
        if (currentMode == SF_VL53L5CX_POWER_MODE::SLEEP) {
            if (!_sensor.setPowerMode(SF_VL53L5CX_POWER_MODE::WAKEUP)) {
                Serial.println("Failed to wake up sensor.");
                return false;
            }
            delay(10); // Give time to wake up
        }
        
        // Start ranging
        if (!_sensor.startRanging()) {
            Serial.println("Failed to start ranging.");
            return false;
        }
        
        return true;
    }
    
    bool ToFSensor::setReset() {
        // Use hardware reset if pins are available
        if (_lpn_pin >= 0) {
            digitalWrite(_lpn_pin, LOW);
            delay(10);
            digitalWrite(_lpn_pin, HIGH);
            delay(10);
            
            // Re-initialize the sensor
            _isInitialized = false;
            return begin();
        } else if (_i2c_rst_pin >= 0) {
            digitalWrite(_i2c_rst_pin, LOW);
            delay(10);
            digitalWrite(_i2c_rst_pin, HIGH);
            delay(10);
            
            // Re-initialize the sensor
            _isInitialized = false;
            return begin();
        } else {
            // No hardware reset pins available
            Serial.println("No hardware reset pins configured.");
            return false;
        }
    }

    // Set the resolution (4x4 or 8x8)
bool ToFSensor::setResolution(int resolution) {
    // if (!_isInitialized) {
    //     Serial.println("Sensor not initialized. Call begin() first.");
    //     return false;
    // }
    
    // // Check if resolution is valid (4x4=16 or 8x8=64)
    // if (resolution != 16 && resolution != 64) {
    //     Serial.println("Invalid resolution. Must be 16 (4x4) or 64 (8x8).");
    //     return false;
    // }
    // Serial.println("Stop ranging2");
    
    // // Stop ranging before changing configuration
    // sensor.stopRanging();
    // Serial.println("Stop ranging");
    
    
    // Set resolution
    bool success = _sensor.setResolution(resolution);
    // Serial.println("set rez");
    
    // //Update internal variables
    // if (success) {
    //     _imageResolution = sensor.getResolution();
    //     imageWidth = sqrt(_imageResolution);
        
    //     // Update ranging frequency limits based on resolution
    //     // 4x4 allows up to 60Hz, 8x8 allows up to 15Hz
    //     int maxFreq = (resolution == 16) ? 60 : 15;
    //     int currentFreq = sensor.getRangingFrequency();
    //     if (currentFreq > maxFreq) {
    //         sensor.setRangingFrequency(maxFreq);
    //     }
    // }
    
    // // // Restart ranging if it was active
   
    // sensor.startRanging();
    // Serial.println("Start ranging");

    
    return success;
}

// Set the ranging frequency (Hz)
bool ToFSensor::setRangingFrequency(int frequency) {
    // if (!_isInitialized) {
    //     Serial.println("Sensor not initialized. Call begin() first.");
    //     return false;
    // }
    
    // // Check frequency limits based on current resolution
    // int maxFreq = (_imageResolution == 16) ? 60 : 15;
    // if (frequency < 1 || frequency > maxFreq) {
    //     Serial.print("Invalid frequency. Must be between 1 and ");
    //     Serial.print(maxFreq);
    //     Serial.println(" Hz for current resolution.");
    //     return false;
    // }
    
    return _sensor.setRangingFrequency(frequency);
}

// Print data to serial in a formatted grid
void ToFSensor::printData() {
    //VL53L5CX_ResultsData results;
    
    // if (!readData(_results)) {
    //     return; // No data available
    // }
    
    Serial.println("VL53L5CX Distance Data (mm):");
    
    // Print the data in a grid format
    for (int y = 0; y < _imageWidth; y++) {
        for (int x = 0; x < _imageWidth; x++) {
            // Calculate index in the distance array
            // The data is transposed from the zone mapping shown in datasheet
            int idx = (_imageWidth - 1 - x) + y * _imageWidth;
            
            Serial.print("\t");
            Serial.print(_results.distance_mm[idx]);
        }
        Serial.println();
    }
    Serial.println();
}

// Get integration time
int ToFSensor::getIntegrationTime() {
    if (!_isInitialized) {
        Serial.println("Sensor not initialized. Call begin() first.");
        return -1;
    }
    
    return _sensor.getIntegrationTime();
}

// Set integration time
bool ToFSensor::setIntegrationTime(int integrationTimeMs) {
    if (!_isInitialized) {
        Serial.println("Sensor not initialized. Call begin() first.");
        return false;
    }
    
    return _sensor.setIntegrationTime(integrationTimeMs);
}

// Check if sensor is connected
bool ToFSensor::isConnected() {
    if (!_isInitialized) {
        return false;
    }
    
    return _sensor.isConnected();
}

// Static member initialization
volatile bool ToFSensor::_dataReady = false;
ToFSensor* ToFSensor::_instance = nullptr;

void IRAM_ATTR ToFSensor::interruptHandler() {
    // Set the flag in ISR - keep it minimal
    _dataReady = true;
}

bool ToFSensor::setupInterrupt() {
    if (!_isInitialized || _int_pin < 0) {
        Serial.println("Cannot setup interrupt: sensor not initialized or INT pin not defined");
        return false;
    }
    
    // Store _instance pointer for the static handler
    _instance = this;
    
    // Configure interrupt pin
    pinMode(_int_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_int_pin), interruptHandler, FALLING);
    
    Serial.println("Interrupt handler configured");
    return true;
}

bool ToFSensor::didInterrupt() {
    return _dataReady;
}

bool ToFSensor::readDataOnInterrupt(VL53L5CX_ResultsData& results) {
    Serial.println("Interrupt");
    if (!_dataReady) {
        return false;
    }
    
    // Reset flag before reading data
    _dataReady = false;
    
    // Get ranging data
    if (!_sensor.getRangingData(&results)) {
        Serial.println("Error getting ranging data on interrupt");
        return false;
    }
    
    // Copy data to internal results structure
    memcpy(&_results, &results, sizeof(VL53L5CX_ResultsData));
    
    return true;
}

