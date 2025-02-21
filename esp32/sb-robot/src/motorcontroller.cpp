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
        motorA_encA, motorA_encB, 0, motorA_ticks, 0, 0, 0, 0
    };
    motorB = {
        motorB_in1, motorB_in2, motorB_enable, motorB_reversed,
        motorB_encA, motorB_encB, 0, motorB_ticks, 0, 0, 0, 0
    };
    this->stbyPin = stbyPin;

    instance = this;
    lastUpdateTime = 0;

    pidA = new PID(&motorA.currentSpeed, &motorA.pidOutput, &motorA.targetSpeed, 1, 0, 0, DIRECT);
    pidB = new PID(&motorB.currentSpeed, &motorB.pidOutput, &motorB.targetSpeed, 1, 0, 0, DIRECT);
    pidA->SetSampleTime(PID_INTERVAL);
    pidB->SetSampleTime(PID_INTERVAL);
    pidA->SetOutputLimits(-255, 255);
    pidB->SetOutputLimits(-255, 255);
    pidA->SetMode(AUTOMATIC);
    pidB->SetMode(AUTOMATIC);
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
    const TickType_t xFrequency = pdMS_TO_TICKS(PID_INTERVAL);
    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        controller->update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void MotorController::updateMotorSpeed(MotorPins& motor) {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastUpdateTime;
    
    if (deltaTime >= PID_INTERVAL) {
        long deltaTicks = motor.encoderCount - motor.prevCount;
        motor.currentSpeed = (double)deltaTicks * (1000.0 / deltaTime); // ticks per second
        motor.prevCount = motor.encoderCount;
        lastUpdateTime = currentTime;
        
        // Debug print
        Serial.print("Delta ticks: ");
        Serial.print(deltaTicks);
        Serial.print(", Delta time: ");
        Serial.print(deltaTime);
        Serial.print(", Current speed: ");
        Serial.println(motor.currentSpeed);
    }
}

void MotorController::update() {
    updateMotorSpeed(motorA);
    updateMotorSpeed(motorB);

    pidA->Compute();
    pidB->Compute();

    sfMotorA->drive(motorA.pidOutput);
    sfMotorB->drive(motorB.pidOutput);

    // Debug print
    Serial.print("PID output A: ");
    Serial.print(motorA.pidOutput);
    Serial.print(", PID output B: ");
    Serial.println(motorB.pidOutput);
}

void MotorController::setSpeedA(float speedTicks) {
    motorA.targetSpeed = speedTicks; // Now in ticks per second
    Serial.print("Setting speed A to: ");
    Serial.println(speedTicks);
}

void MotorController::setSpeedB(float speedTicks) {
    motorB.targetSpeed = speedTicks; // Now in ticks per second
    Serial.print("Setting speed B to: ");
    Serial.println(speedTicks);
}

void MotorController::setSpeeds(float speedA_ticks, float speedB_ticks) {
    setSpeedA(speedA_ticks);
    setSpeedB(speedB_ticks);
}

float MotorController::getCurrentSpeedA() {
    return motorA.currentSpeed;
}

float MotorController::getCurrentSpeedB() {
    return motorB.currentSpeed;
}

void MotorController::stop() {
    motorA.targetSpeed = 0;
    motorB.targetSpeed = 0;
    sfMotorA->brake();
    sfMotorB->brake();
}

void MotorController::setPID(float kp, float ki, float kd) {
    pidA->SetTunings(kp, ki, kd);
    pidB->SetTunings(kp, ki, kd);
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
    return (float)motorB.encoderCount * (2.0f * PI / motorA.ticksPerRevolution);
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
