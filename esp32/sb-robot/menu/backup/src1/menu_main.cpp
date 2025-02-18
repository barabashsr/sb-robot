#include "menu_menu.h"
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



void CALLBACK_FUNCTION SetSpeedA(int id) {
    float speed = menuSetA.getAsFloatingPointValue();
    motors.setSpeedA(speed);
}

void CALLBACK_FUNCTION SetSpeedB(int id) {
    float speed = menuSetB.getAsFloatingPointValue();
    Serial.println(speed);
    motors.setSpeedB(speed);
}

void CALLBACK_FUNCTION StopA(int id) {
/*     motors.setSpeedA(0);
    menuSpeedA.setFloatValue(0); */
}

void CALLBACK_FUNCTION StopB(int id) {
/*     motors.setSpeedB(0);
    menuSpeedB.setFloatValue(0); */
}

void CALLBACK_FUNCTION SetKp(int id) {
    float kp = menuKp.getAsFloatingPointValue();
    float ki = menuKi.getAsFloatingPointValue();
    float kd = menuKd.getAsFloatingPointValue();
    float ko = menuKo.getAsFloatingPointValue();
    motors.setPID(kp, ki, kd, ko);
}

void CALLBACK_FUNCTION SetKi(int id) {
    float kp = menuKp.getAsFloatingPointValue();
    float ki = menuKi.getAsFloatingPointValue();
    float kd = menuKd.getAsFloatingPointValue();
    float ko = menuKo.getAsFloatingPointValue();
    motors.setPID(kp, ki, kd, ko);
}

void CALLBACK_FUNCTION SetKd(int id) {
    float kp = menuKp.getAsFloatingPointValue();
    float ki = menuKi.getAsFloatingPointValue();
    float kd = menuKd.getAsFloatingPointValue();
    float ko = menuKo.getAsFloatingPointValue();
    motors.setPID(kp, ki, kd, ko);
}

void CALLBACK_FUNCTION SetKo(int id) {
    float kp = menuKp.getAsFloatingPointValue();
    float ki = menuKi.getAsFloatingPointValue();
    float kd = menuKd.getAsFloatingPointValue();
    float ko = menuKo.getAsFloatingPointValue();
    motors.setPID(kp, ki, kd, ko);
}

void CALLBACK_FUNCTION SetPIDHz(int id) {
    unsigned long interval = menuPIDInterval.getCurrentValue();
    motors.setPIDInterval(interval);
}

/* void CALLBACK_FUNCTION setAutoStopInterval(int id) {
    unsigned long interval = menuAutoStopInterval.getCurrentValue();
    motors.setAutoStopInterval(interval);
} */

/* void CALLBACK_FUNCTION setMinPwmThreshold(int id) {
    int threshold = menuMinPwmThreshold.getCurrentValue();
    motors.setMinPwmThreshold(threshold);
} */

void CALLBACK_FUNCTION ResetTicsA(int id) {
    motors.resetEncoders();
    menuTicksA.setFloatValue(0);
    menuTicksB.setFloatValue(0);
}

void CALLBACK_FUNCTION ResetTicsB(int id) {
    motors.resetEncoders();
    menuTicksA.setFloatValue(0);
    menuTicksB.setFloatValue(0);
}

void updateMenuValues() {
    menuTicksA.setFloatValue(motors.getEncoderA());
    menuTicksB.setFloatValue(motors.getEncoderB());
    menuSpeedA.setFloatValue(motors.getCurrentSpeedA());
    menuSpeedA.setFloatValue(motors.getCurrentSpeedB());
}


void setup() {
    Wire.begin();
    Serial.begin(115200);
    Serial.println("serial started");
    motors.init();
    setupMenu();
    taskManager.scheduleFixedRate(100, updateMenuValues);
}

void loop() {
    taskManager.runLoop();
}

