#include "motorcontroller.h"

MotorController* MotorController::instance = nullptr;

MotorController::MotorController(
    int motorA_in1, int motorA_in2, int motorA_enable,
    int motorB_in1, int motorB_in2, int motorB_enable,
    int motorA_encA, int motorA_encB,
    int motorB_encA, int motorB_encB,
    int stbyPin,
    bool motorA_reversed,
    bool motorB_reversed,
    int motorA_ticks,
    int motorB_ticks
) {
    motorA = {
        motorA_in1, motorA_in2, motorA_enable, motorA_reversed,
        motorA_encA, motorA_encB, 0, motorA_ticks, 0, 0, 0, 0, 0
    };
    motorB = {
        motorB_in1, motorB_in2, motorB_enable, motorB_reversed,
        motorB_encA, motorB_encB, 0, motorB_ticks, 0, 0, 0, 0, 0
    };
    this->stbyPin = stbyPin;

    instance = this;
    Kp = 20.0;
    Ki = 0.5;
    Kd = 2.0;
    Ko = 1.5;
    moving = false;
    lastMotorCommand = 0;
    PID_INTERVAL = 10; // ms
    AUTO_STOP_INTERVAL = 2000;
    minPwmThreshold = 70;
    autoStopEnabled = false;
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
    float instantSpeed = (float)(deltaTicks * 2.0f * PI) / (motor.ticksPerRevolution * deltaTime);
    motor.prevCount = motor.encoderCount;
    motor.speedBuffer[motor.bufferIndex] = instantSpeed;
    motor.bufferIndex = (motor.bufferIndex + 1) % SPEED_FILTER_SIZE;
    float averageSpeed = 0;
    for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
        averageSpeed += motor.speedBuffer[i];
    }
    averageSpeed /= SPEED_FILTER_SIZE;
    return averageSpeed;
}

void MotorController::init() {
    pinMode(motorA.encoderA, INPUT_PULLUP);
    pinMode(motorA.encoderB, INPUT_PULLUP);
    pinMode(motorB.encoderA, INPUT_PULLUP);
    pinMode(motorB.encoderB, INPUT_PULLUP);
    pinMode(stbyPin, OUTPUT);
    digitalWrite(stbyPin, HIGH);

    sfMotorA = new Motor(motorA.in1, motorA.in2, motorA.enable, motorA.reversed ? -1 : 1, stbyPin);
    sfMotorB = new Motor(motorB.in1, motorB.in2, motorB.enable, motorB.reversed ? -1 : 1, stbyPin);

    attachInterrupt(digitalPinToInterrupt(motorA.encoderA), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motorB.encoderA), encoderISR_B, CHANGE);
    stop();

    xTaskCreatePinnedToCore(
        controllerTaskCode,
        "ControllerTask",
        10000,
        this,
        1,
        &controllerTask,
        1  // Pin to Core 1
    );
}

void MotorController::controllerTaskCode(void* parameter) {
    MotorController* controller = static_cast<MotorController*>(parameter);
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(controller->PID_INTERVAL);
    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        controller->update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void MotorController::updateMotorPID(MotorPins& motor) {
    motor.currentSpeed = calculateCurrentSpeed(motor);
    float error = motor.targetSpeed - motor.currentSpeed;
    float pTerm = Kp * error;
    motor.iTerm = constrain(
        motor.iTerm + (Ki * error * (PID_INTERVAL / 1000.0f)),
        -100, 100
    );
    float dTerm = Kd * (error - motor.prevError) / (PID_INTERVAL / 1000.0f);
    float feedForward = motor.targetSpeed * 0.5f;
    int output = (int)((pTerm + motor.iTerm + dTerm + feedForward) * Ko);
    motor.prevError = error;
    
    if (&motor == &motorA) {
        sfMotorA->drive(output);
    } else {
        sfMotorB->drive(output);
    }
}

void MotorController::update() {
    if (moving) {
        updateMotorPID(motorA);
        updateMotorPID(motorB);
    }
    if (autoStopEnabled && (millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
        stop();
    }
}

void MotorController::setSpeedA(float speedRads) {
    lastMotorCommand = millis();
    motorA.targetSpeed = speedRads;
    if (fabs(speedRads) < 0.0001f) {
        motorA.iTerm = 0;
        sfMotorA->brake();
    }
    moving = (motorA.targetSpeed != 0.0f) || (motorB.targetSpeed != 0.0f);
}

void MotorController::setSpeedB(float speedRads) {
    lastMotorCommand = millis();
    motorB.targetSpeed = speedRads;
    if (fabs(speedRads) < 0.0001f) {
        motorB.iTerm = 0;
        sfMotorB->brake();
    }
    moving = (motorA.targetSpeed != 0.0f) || (motorB.targetSpeed != 0.0f);
}

void MotorController::setSpeeds(float speedA_rads, float speedB_rads) {
    setSpeedA(speedA_rads);
    setSpeedB(speedB_rads);
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
    motorA.prevError = 0;
    motorB.prevError = 0;
    motorA.currentSpeed = 0;
    motorB.currentSpeed = 0;
    sfMotorA->brake();
    sfMotorB->brake();
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
    return (float)motorA.encoderCount * (2.0f * PI / motorA.ticksPerRevolution);
}

float MotorController::getPositionB() {
    return (float)motorB.encoderCount * (2.0f * PI / motorB.ticksPerRevolution);
}

void MotorController::resetEncoders() {
    motorA.encoderCount = 0;
    motorB.encoderCount = 0;
    motorA.prevCount = 0;
    motorB.prevCount = 0;
}

void MotorController::setPWMA(int pwm) {
    sfMotorA->drive(pwm);
}

void MotorController::setPWMB(int pwm) {
    sfMotorB->drive(pwm);
}
