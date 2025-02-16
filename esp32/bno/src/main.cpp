#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define I2C_SDA 19
#define I2C_SCL 18
#define BNO055_ADDRESS 0x28

TwoWire I2CBNO = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS, &I2CBNO);

void setup() {
  Serial.begin(115200);
  I2CBNO.begin(I2C_SDA, I2C_SCL, 40000);
  
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring and I2C address.");
    while (1);
  }
  
  // Set to NDOF mode
  bno.setMode(OPERATION_MODE_NDOF);
  delay(1000);  // Short delay to allow mode change
  Serial.println(bno.getMode());

  // Check if mode was set correctly
  if (bno.getMode() != OPERATION_MODE_NDOF) {
    Serial.println("Failed to set NDOF mode!");
  } else {
    Serial.println("NDOF mode set successfully");
  }

  bno.setExtCrystalUse(true);
  Serial.println("BNO055 initialized!");
  delay(1000);
}

void loop() {
  // Check for system errors
  uint8_t system_status, self_test_result, system_error;
/*   bno.getSystemStatus(&system_status, &self_test_result, &system_error);
  if (system_error != 0) {
    Serial.print("System error detected: ");
    Serial.println(system_error);
    delay(1000);
    return;
  } */

  // Check calibration status
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  if (system < 3) {
    Serial.println("System not fully calibrated. Please calibrate the sensor.");
    Serial.print("Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);
    delay(1000);
    //return;
  }

  // Read orientation data
  sensors_event_t event;
  if (bno.getEvent(&event)) {
    // The Y value represents the angle between Z axis and vertical
    float angle_y = event.orientation.y;
    
    Serial.print("Angle between Z axis and vertical: ");
    Serial.print(angle_y);
    Serial.println(" degrees");
  } else {
    Serial.println("Error reading sensor data");
  }
  
  delay(100);
}
