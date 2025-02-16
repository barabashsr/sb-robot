#include "motorcontroller.h"
#include <Arduino.h>

// Pin definitions
const int MOTOR_A_IN1 = 27;
const int MOTOR_A_IN2 = 26;
const int MOTOR_A_ENABLE = 25;
const int MOTOR_B_IN1 = 33;
const int MOTOR_B_IN2 = 32;
const int MOTOR_B_ENABLE = 14;
const int MOTOR_A_ENC_A = 34;
const int MOTOR_A_ENC_B = 35;
const int MOTOR_B_ENC_A = 4;
const int MOTOR_B_ENC_B = 5;

MotorController motors(
    MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_ENABLE,
    MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_ENABLE,
    MOTOR_A_ENC_A, MOTOR_A_ENC_B,
    MOTOR_B_ENC_A, MOTOR_B_ENC_B
);

void setup() {
    Serial.begin(115200);
    motors.init();
    Serial.println("MotorController initialized");
}

void loop() {
    // Test motor A
    Serial.println("Testing Motor A");
    motors.setSpeedA(5.0);
    delay(2000);
    motors.setSpeedA(0);
    delay(1000);

    // Test motor B
    Serial.println("Testing Motor B");
    motors.setSpeedB(5.0);
    delay(2000);
    motors.setSpeedB(0);
    delay(1000);

    // Test both motors
    Serial.println("Testing both motors");
    motors.setSpeeds(3.0, 3.0);
    delay(2000);
    motors.stop();
    delay(1000);

    Serial.println("Test complete");
    delay(5000);
}
