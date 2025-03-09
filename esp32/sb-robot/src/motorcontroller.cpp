#include "motorcontroller.h"

MotorController* MotorController::instance = nullptr;

MotorController::MotorController(
    int motorA_in1, int motorA_in2, int motorA_pwm,
    int motorB_in1, int motorB_in2, int motorB_pwm,
    int standby,
    int motorA_encA, int motorA_encB,
    int motorB_encA, int motorB_encB,
    int motorA_ticks,
    int motorB_ticks
): motorA{
    
    .in1 = motorA_in1,
    .in2 = motorA_in2,
    .pwm = motorA_pwm,
    .encoderA = motorA_encA,
    .encoderB = motorA_encB,
    .encoderCount = 0,
    .ticksPerRevolution = motorA_ticks   
},
motorB{
    
    .in1 = motorB_in1,
    .in2 = motorB_in2,
    .pwm = motorB_pwm,
    .encoderA = motorB_encA,
    .encoderB = motorB_encB,
    .encoderCount = 0,
    .ticksPerRevolution = motorB_ticks   
},
standbyPin(standby)

 {
    instance = this;
}

void IRAM_ATTR MotorController::encoderISR_C() {
    if (instance) {
        instance->handleEncoderA();
    }
}

void IRAM_ATTR MotorController::encoderISR_D() {
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
    pinMode(motor.pwm, OUTPUT);
    pinMode(motor.encoderA, INPUT_PULLUP);
    pinMode(motor.encoderB, INPUT_PULLUP);
}

void MotorController::setMotorSpeed(const MotorPins& motor, int speed) {
    speed = constrain(speed, -255, 255);
    int pwmValue = abs(speed);

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

    analogWrite(motor.pwm, pwmValue);
}

void MotorController::init() {
    pinMode(standbyPin, OUTPUT);
    digitalWrite(standbyPin, HIGH);  // Enable the driver

    initMotor(motorA);
    initMotor(motorB);
    attachInterrupt(digitalPinToInterrupt(motorA.encoderA), encoderISR_C, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motorB.encoderA), encoderISR_D, CHANGE);
    lastMeasurementTime = 0;
    measurementPeriod = 20; // Default to 10ms
    lastEncoderCountA = 0;
    lastEncoderCountB = 0;
    threshold_A = 40;
    threshold_B = 40;
    stop();
    Serial.println("motors initialised");
}

void MotorController::setMotorA(int speed) {
    if (abs(speedA) < 0.01){
        setMotorSpeed(motorA, ((speed > 0) - (speed < 0)) * threshold_A);
        return;

    }
    setMotorSpeed(motorA, speed);
}

void MotorController::setMotorB(int speed) {
    if (abs(speedB) < 0.01){
        setMotorSpeed(motorB, ((speed > 0) - (speed < 0)) * threshold_B);
        return;

    }
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
    lastEncoderCountA =0;
    lastEncoderCountB = 0;
    motorA.encoderCount = 0;
    motorB.encoderCount = 0;
}

float MotorController::getPositionA() {
    return (float)motorA.encoderCount * 2 * PI / motorA.ticksPerRevolution;
}

float MotorController::getPositionB() {
    return (float)motorB.encoderCount * 2 * PI / motorB.ticksPerRevolution;
}

void MotorController::setMeasurementPeriod(int period) {
    measurementPeriod = period;
}

void MotorController::updateSpeeds() {
    unsigned long currentTime = millis();
    if (currentTime - lastMeasurementTime >= measurementPeriod) {
        long currentCountA = getEncoderA();
        long currentCountB = getEncoderB();
        float timeInSeconds = measurementPeriod / 1000.0f;

        float rotationsA = (currentCountA - lastEncoderCountA) / (float)motorA.ticksPerRevolution;
        speedA = (rotationsA * 2 * PI) / timeInSeconds; // radians per second

        float rotationsB = (currentCountB - lastEncoderCountB) / (float)motorB.ticksPerRevolution;
        speedB = (rotationsB * 2 * PI) / timeInSeconds; // radians per second

        lastMeasurementTime = currentTime;
        lastEncoderCountA = currentCountA;
        lastEncoderCountB = currentCountB;
    }
}
/* 
float MotorController::getSpeedA() {
    long currentCount = getEncoderA();
    float rotations = (currentCount - lastEncoderCountA) / (float)motorA.ticksPerRevolution;
    float timeInSeconds = measurementPeriod / 1000.0f;
    return (rotations * 2 * PI) / timeInSeconds; // radians per second
}

float MotorController::getSpeedB() {
    long currentCount = getEncoderB();
    float rotations = (currentCount - lastEncoderCountB) / (float)motorB.ticksPerRevolution;
    float timeInSeconds = measurementPeriod / 1000.0f;
    return (rotations * 2 * PI) / timeInSeconds; // radians per second
} */