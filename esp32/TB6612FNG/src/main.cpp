#include <Arduino.h>
#include <SparkFun_TB6612.h>

// Motor A connections
#define AIN1 27
#define AIN2 26
#define PWMA 25

// Motor B connections
#define BIN1 19
#define BIN2 18
#define PWMB 14

// Standby pin
#define STBY 23

// These constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors
Motor motorA = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorB = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup() {
  Serial.begin(115200);
  Serial.println("TB6612FNG Motor Driver Test");
}

void loop() {
  // Test sequence
  Serial.println("Moving forward");
  motorA.drive(255);
  motorB.drive(255);
  delay(2000);

  Serial.println("Stopping");
  motorA.brake();
  motorB.brake();
  delay(1000);

  Serial.println("Moving backward");
  motorA.drive(-255);
  motorB.drive(-255);
  delay(2000);

  Serial.println("Stopping");
  motorA.brake();
  motorB.brake();
  delay(1000);

  Serial.println("Turning right");
  motorA.drive(200);
  motorB.drive(-200);
  delay(1000);

  Serial.println("Stopping");
  motorA.brake();
  motorB.brake();
  delay(1000);

  Serial.println("Turning left");
  motorA.drive(-200);
  motorB.drive(200);
  delay(1000);

  Serial.println("Stopping");
  motorA.brake();
  motorB.brake();
  delay(2000);

  Serial.println("Test sequence complete. Restarting...\n");
}
