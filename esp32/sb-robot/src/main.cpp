#include <Arduino.h>
#include "motorcontroller.h"

// Pin definitions
const int MOTOR_A_IN1 = 27;
const int MOTOR_A_IN2 = 26;
const int MOTOR_A_ENABLE = 25;
const int MOTOR_B_IN1 = 33;
const int MOTOR_B_IN2 = 32;
const int MOTOR_B_ENABLE = 14;

// Encoder pins
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

unsigned long lastEncoderPrint = 0;
const int ENCODER_PRINT_INTERVAL = 100;
String inputString = "";
bool stringComplete = false;

void processCommand(String command) {
    command.trim();
    
    if (command.startsWith("ma ")) {
        int speed = command.substring(3).toInt();
        motors.setMotorA(speed);
        Serial.printf("Setting Motor A to %d%%\n", speed);
    }
    else if (command.startsWith("mb ")) {
        int speed = command.substring(3).toInt();
        motors.setMotorB(speed);
        Serial.printf("Setting Motor B to %d%%\n", speed);
    }
    else if (command.startsWith("both ")) {
        int speed = command.substring(5).toInt();
        motors.setSpeeds(speed, speed);
        Serial.printf("Setting both motors to %d%%\n", speed);
    }
    else if (command == "stop") {
        motors.stop();
        Serial.println("Stopping all motors");
    }
    else if (command == "reset") {
        motors.resetEncoders();
        Serial.println("Resetting encoder counts");
    }
    else if (command == "help") {
        Serial.println("Available commands:");
        Serial.println("ma <speed>  - Set Motor A speed (-100 to 100)");
        Serial.println("mb <speed>  - Set Motor B speed (-100 to 100)");
        Serial.println("both <speed> - Set both motors speed");
        Serial.println("stop       - Stop all motors");
        Serial.println("reset      - Reset encoder counts");
        Serial.println("help       - Show this help");
    }
    else {
        Serial.println("Unknown command. Type 'help' for available commands");
    }
}

void setup() {
    Serial.begin(115200);
    motors.init();
    Serial.println("\nMotor Controller Test with Encoders");
    Serial.println("Type 'help' for available commands");
    inputString.reserve(200);
}

void loop() {
    // Process serial input
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r') {
            if (inputString.length() > 0) {
                processCommand(inputString);
                inputString = "";
            }
        } else {
            inputString += inChar;
        }
    }

    // Print encoder values periodically
    unsigned long currentMillis = millis();
    if (currentMillis - lastEncoderPrint >= ENCODER_PRINT_INTERVAL) {
        lastEncoderPrint = currentMillis;
        Serial.printf("Encoders - A: %ld (%.2f rad), B: %ld (%.2f rad)\n", 
            motors.getEncoderA(), 
            motors.getPositionA(),
            motors.getEncoderB(),
            motors.getPositionB()
        );
    }
}
