#include "motorcontroller.h"

MotorController* MotorController::instance = nullptr;

MotorController::MotorController(
    int motorA_in1, int motorA_in2, int motorA_pwm,
    int motorB_in1, int motorB_in2, int motorB_pwm,
    int standby,
    int motorA_encA, int motorA_encB,
    int motorB_encA, int motorB_encB,
<<<<<<< .merge_file_VGYz13
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
=======
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

>>>>>>> .merge_file_U1ZhNx
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

<<<<<<< .merge_file_VGYz13
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
=======
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
>>>>>>> .merge_file_U1ZhNx
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
    lastEncoderCountA =0;
    lastEncoderCountB = 0;
    motorA.encoderCount = 0;
    motorB.encoderCount = 0;
    motorA.prevCount = 0;
    motorB.prevCount = 0;
}

<<<<<<< .merge_file_VGYz13
float MotorController::getPositionA() {
    return (float)motorA.encoderCount * 2 * PI / motorA.ticksPerRevolution;
=======
void MotorController::setPWMA(int pwm) {
    sfMotorA->drive(pwm);
>>>>>>> .merge_file_U1ZhNx
}

void MotorController::setPWMB(int pwm) {
    sfMotorB->drive(pwm);
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