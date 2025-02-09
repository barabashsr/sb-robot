#include "motorcontroller.h"

MotorController* MotorController::instance = nullptr;

MotorController::MotorController(
    int motorA_in1, int motorA_in2, int motorA_enable,
    int motorB_in1, int motorB_in2, int motorB_enable,
    int motorA_encA, int motorA_encB,
    int motorB_encA, int motorB_encB,
    bool motorA_reversed,
    bool motorB_reversed,
    int motorA_ticks,
    int motorB_ticks
) {
    motorA = {motorA_in1, motorA_in2, motorA_enable, 0, motorA_reversed, 
              motorA_encA, motorA_encB, 0, motorA_ticks};
    motorB = {motorB_in1, motorB_in2, motorB_enable, 1, motorB_reversed, 
              motorB_encA, motorB_encB, 0, motorB_ticks};
    instance = this;
}

void IRAM_ATTR MotorController::encoderISR_A() {
    if (instance) {
        instance->handleEncoderA();
    }
}

void IRAM_ATTR MotorController::encoderISR_B() {
    if (instance) {
        instance->handleEncoderB();
    }
}

void MotorController::handleEncoderA() {
    if (digitalRead(motorA.encoderA) == digitalRead(motorA.encoderB)) {
        motorA.encoderCount++;
    } else {
        motorA.encoderCount--;
    }
}

void MotorController::handleEncoderB() {
    if (digitalRead(motorB.encoderA) == digitalRead(motorB.encoderB)) {
        motorB.encoderCount++;
    } else {
        motorB.encoderCount--;
    }
}

void MotorController::initMotor(MotorPins& motor) {
    pinMode(motor.in1, OUTPUT);
    pinMode(motor.in2, OUTPUT);
    pinMode(motor.enable, OUTPUT);
    pinMode(motor.encoderA, INPUT_PULLUP);
    pinMode(motor.encoderB, INPUT_PULLUP);
    
    ledcSetup(motor.pwmChannel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(motor.enable, motor.pwmChannel);
}

void MotorController::setMotorSpeed(const MotorPins& motor, int speed) {
    int pwmSpeed = abs(speed) * 255 / 100;
    
    if (motor.reversed) {
        speed = -speed;
    }

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
    
    attachInterrupt(digitalPinToInterrupt(motorA.encoderA), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motorB.encoderA), encoderISR_B, CHANGE);
    
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

long MotorController::getEncoderA() {
    return motorA.encoderCount;
}

long MotorController::getEncoderB() {
    return motorB.encoderCount;
}

void MotorController::resetEncoders() {
    motorA.encoderCount = 0;
    motorB.encoderCount = 0;
}

// Add new methods for position in radians
float MotorController::getPositionA() {
    return (float)motorA.encoderCount * 2 * PI / motorA.ticksPerRevolution;
}

float MotorController::getPositionB() {
    return (float)motorB.encoderCount * 2 * PI / motorB.ticksPerRevolution;
}
