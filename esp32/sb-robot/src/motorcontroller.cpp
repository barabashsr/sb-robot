#include "motorcontroller.h"

MotorController::MotorController(
    int motorA_in1, int motorA_in2, int motorA_enable,
    int motorB_in1, int motorB_in2, int motorB_enable
) {
    motorA = {motorA_in1, motorA_in2, motorA_enable, 0};
    motorB = {motorB_in1, motorB_in2, motorB_enable, 1};
}

void MotorController::initMotor(const MotorPins& motor) {
    pinMode(motor.in1, OUTPUT);
    pinMode(motor.in2, OUTPUT);
    pinMode(motor.enable, OUTPUT);
    
    ledcSetup(motor.pwmChannel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(motor.enable, motor.pwmChannel);
}

void MotorController::setMotorSpeed(const MotorPins& motor, int speed) {
    int pwmSpeed = abs(speed) * 255 / 100;
    
    if (speed > 0) {
        digitalWrite(motor.in1, HIGH);
        digitalWrite(motor.in2, LOW);
    } else if (speed < 0) {
        digitalWrite(motor.in1, LOW);
        digitalWrite(motor.in2, HIGH);
    } else {
        digitalWrite(motor.in1, LOW);
        digitalWrite(motor.in2, LOW);
    }
    
    ledcWrite(motor.pwmChannel, pwmSpeed);
}

void MotorController::init() {
    initMotor(motorA);
    initMotor(motorB);
    stop();
}

void MotorController::setMotorA(int speed) {
    speed = constrain(speed, -100, 100);
    setMotorSpeed(motorA, speed);
}

void MotorController::setMotorB(int speed) {
    speed = constrain(speed, -100, 100);
    setMotorSpeed(motorB, speed);
}

void MotorController::setSpeeds(int speedA, int speedB) {
    setMotorA(speedA);
    setMotorB(speedB);
}

void MotorController::stop() {
    setSpeeds(0, 0);
}
