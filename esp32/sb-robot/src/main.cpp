#include <Arduino.h>
#include "motorcontroller.h"

// Pin definitions
const int MOTOR_A_IN1 = 27;
const int MOTOR_A_IN2 = 26;
const int MOTOR_A_ENABLE = 25;
const int MOTOR_B_IN1 = 33;
const int MOTOR_B_IN2 = 32;
const int MOTOR_B_ENABLE = 14;

MotorController motors(
    MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_ENABLE,
    MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_ENABLE
);

void processCommand(String command) {
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
    else if (command == "help") {
        Serial.println("Available commands:");
        Serial.println("ma <speed>  - Set Motor A speed (-100 to 100)");
        Serial.println("mb <speed>  - Set Motor B speed (-100 to 100)");
        Serial.println("both <speed> - Set both motors speed");
        Serial.println("stop       - Stop all motors");
        Serial.println("help       - Show this help");
    }
    else {
        Serial.println("Unknown command. Type 'help' for available commands.");
    }
}

void setup() {
    Serial.begin(115200);
    motors.init();
    Serial.println("\nMotor Controller Test");
    Serial.println("Type 'help' for available commands");
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        processCommand(command);
    }
}
