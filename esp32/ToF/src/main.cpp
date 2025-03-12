#include <Wire.h>
#include "ToFSensor.h"

// Define pins for VL53L5CX
#define PWREN_PIN -1
#define LPN_PIN -1
#define I2C_RST_PIN -1
#define INT_PIN 42

#define SDA_PIN 8
#define SCL_PIN 9

//TwoWire I2C = TwoWire(0);
ToFSensor tof(Wire, PWREN_PIN, LPN_PIN, I2C_RST_PIN, INT_PIN, 0x29);
VL53L5CX_ResultsData results;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("VL53L5CX ToF Sensor Interrupt Test");
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000); // 1 MHz
    
    // Initialize sensor
    if (!tof.begin()) {
        Serial.println("Failed to initialize VL53L5CX sensor!");
        while (1) delay(10);
    } else {
      Serial.println("OK to initialize VL53L5CX sensor!");
    }
    
    // Set resolution to 8x8
    //tof.setResolution(64); // 8x8 = 64 zones
    //Serial.printf("OK to setup resolution!");

    
    // Set ranging frequency to 15Hz (max for 8x8 mode)
    //tof.setRangingFrequency(15);
    //Serial.printf("OK to setup frequency!");
    
    // Setup interrupt handling
    // if (!tof.setupInterrupt()) {
    //     Serial.println("Failed to setup interrupt!");
    //     while (1) delay(10);
    // } else {
    //   Serial.println("OK to setup interrupt!");
    // }
    
    // Start ranging
    //tof.setRanging();
    //Serial.printf("OK to setup ranging!");
}

void loop() {
    //Check if new data is available via interrupt
    if (tof.didInterrupt()) {
        if (tof.readDataOnInterrupt(results)) {
            //Serial.println(tof.getIntegrationTime());
            tof.printData();
            // Process and print the data
            // Serial.println("VL53L5CX Distance Data (mm):");
            
            // int imageWidth = 8; // For 8x8 resolution
            
            // // Print the data in a grid format
            // for (int y = 0; y < imageWidth; y++) {
            //     for (int x = 0; x < imageWidth; x++) {
            //         // Calculate index in the distance array
            //         int idx = (imageWidth - 1 - x) + y * imageWidth;
                    
            //         Serial.print("\t");
            //         Serial.print(results.distance_mm[idx]);
            //     }
            //     Serial.println();
            // }
            // Serial.println();
        }
    }
    //tof.readData(results);
    //tof.printData();
    
    // Do other tasks here without blocking
    // ...
    
    delay(5); // Small delay
}
