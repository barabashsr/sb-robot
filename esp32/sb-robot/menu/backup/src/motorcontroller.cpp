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
               motorA_encA, motorA_encB, 0, motorA_ticks, 0, 0, 0, 0, 0, 0};
    motorB = {motorB_in1, motorB_in2, motorB_enable, 1, motorB_reversed,
               motorB_encA, motorB_encB, 0, motorB_ticks, 0, 0, 0, 0, 0, 0};
    instance = this;

    Kp = 20.0;
    Ki = 0.5;
    Kd = 2.0;
    Ko = 1.5;
    safetyDelay = 50;
    moving = false;
    nextPID = 0;
    lastMotorCommand = 0;
    PID_INTERVAL = 10;
    AUTO_STOP_INTERVAL = 2000;
    minPwmThreshold = 40;
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

float MotorController::calculateCurrentSpeed(MotorPins& motor) {
    long deltaTicks = motor.encoderCount - motor.prevCount;
    float deltaTime = PID_INTERVAL / 1000.0;
    float speed = (float)(deltaTicks * 2.0 * PI) / (motor.ticksPerRevolution * deltaTime);
    motor.prevCount = motor.encoderCount;
    return speed;
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

void MotorController::init() {
    initMotor(motorA);
    initMotor(motorB);
    attachInterrupt(digitalPinToInterrupt(motorA.encoderA), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motorB.encoderA), encoderISR_B, CHANGE);
    stop();
}

void MotorController::setMotorSpeed(MotorPins& motor, int pwmValue) {
    pwmValue = constrain(pwmValue, -255, 255);
    if (motor.reversed) {
        pwmValue = -pwmValue;
    }

    if (abs(pwmValue) < minPwmThreshold && abs(pwmValue) > 0) {
        pwmValue = (pwmValue > 0) ? minPwmThreshold : -minPwmThreshold;
    }

    if (pwmValue > 0) {
        digitalWrite(motor.in1, HIGH);
        digitalWrite(motor.in2, LOW);
    } else if (pwmValue < 0) {
        digitalWrite(motor.in1, LOW);
        digitalWrite(motor.in2, HIGH);
    } else {
        digitalWrite(motor.in1, LOW);
        digitalWrite(motor.in2, LOW);
    }

    int mappedPWM = mapSpeed(abs(pwmValue));
    ledcWrite(motor.pwmChannel, mappedPWM);
}

void MotorController::updateMotorPID(MotorPins& motor) {
    if (abs(motor.currentSpeed) < 0.1 && abs(motor.targetSpeed) > 0.1) {
        if (motor.startTime == 0) {
            motor.startTime = millis();
            return;
        }
        if (millis() - motor.startTime < safetyDelay) {
            return;
        }
    }

    motor.currentSpeed = calculateCurrentSpeed(motor);
    if (abs(motor.targetSpeed) < 0.1) {
        motor.startTime = 0;
    }

    float error = motor.targetSpeed - motor.currentSpeed;
    float pTerm = Kp * error;
    motor.iTerm = constrain(motor.iTerm + Ki * error * (PID_INTERVAL / 1000.0), -100, 100);
    float dTerm = Kd * (error - motor.prevError) / (PID_INTERVAL / 1000.0);
    float feedForward = motor.targetSpeed * 0.5;
    int output = (pTerm + motor.iTerm + dTerm + feedForward) * Ko;
    motor.prevError = error;
    setMotorSpeed(motor, output);
}

void MotorController::update() {
    unsigned long currentMillis = millis();
    if (currentMillis > nextPID) {
        if (moving) {
            updateMotorPID(motorA);
            updateMotorPID(motorB);
        }
        nextPID = currentMillis + PID_INTERVAL;
    }

    if ((currentMillis - lastMotorCommand) > AUTO_STOP_INTERVAL) {
        stop();
    }
}

void MotorController::setSpeedA(float speedRads) {
    lastMotorCommand = millis();
    motorA.targetSpeed = speedRads;
    moving = (motorA.targetSpeed != 0) || (motorB.targetSpeed != 0);
}

void MotorController::setSpeedB(float speedRads) {
    lastMotorCommand = millis();
    motorB.targetSpeed = speedRads;
    moving = (motorA.targetSpeed != 0) || (motorB.targetSpeed != 0);
}

void MotorController::setSpeeds(float speedA_rads, float speedB_rads) {
    setSpeedA(speedA_rads);
    setSpeedB(speedB_rads);
}

void MotorController::setSafetyDelay(unsigned long delay_ms) {
    safetyDelay = delay_ms;
}

float MotorController::getCurrentSpeedA() {
    return motorA.currentSpeed;
}

float MotorController::getCurrentSpeedB() {
    return motorB.currentSpeed;
}

void MotorController::stop() {
    moving = false;
    motorA.targetSpeed = 0;
    motorB.targetSpeed = 0;
    motorA.iTerm = 0;
    motorB.iTerm = 0;
    motorA.currentSpeed = 0;
    motorB.currentSpeed = 0;
    motorA.startTime = 0;
    motorB.startTime = 0;
    setMotorSpeed(motorA, 0);
    setMotorSpeed(motorB, 0);
}

void MotorController::setPID(float kp, float ki, float kd, float ko) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    Ko = ko;
    motorA.iTerm = 0;
    motorB.iTerm = 0;
}

long MotorController::getEncoderA() {
    return motorA.encoderCount;
}

long MotorController::getEncoderB() {
    return motorB.encoderCount;
}

float MotorController::getPositionA() {
    return (float)motorA.encoderCount * 2 * PI / motorA.ticksPerRevolution;
}

float MotorController::getPositionB() {
    return (float)motorB.encoderCount * 2 * PI / motorB.ticksPerRevolution;
}

void MotorController::resetEncoders() {
    motorA.encoderCount = 0;
    motorB.encoderCount = 0;
    motorA.prevCount = 0;
    motorB.prevCount = 0;
}

int MotorController::mapSpeed(float speed) {
    float maxSpeed = 30.0;
    float normalizedSpeed = speed / maxSpeed;
    return (int)(255.0 * pow(abs(normalizedSpeed), 0.7)) * (normalizedSpeed >= 0 ? 1 : -1);
}
