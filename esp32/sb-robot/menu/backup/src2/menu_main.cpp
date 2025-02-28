#include "menu_menu.h"
#include "motorcontroller.h"
#include "BNO055Sensor.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>


#define EEPROM_SIZE 512
#ifndef MENU_MAGIC_KEY
#define MENU_MAGIC_KEY 0xfad0
#endif 



// Pin definitions
const int MOTOR_A_IN1 = 27;
const int MOTOR_A_IN2 = 26;
const int MOTOR_A_ENABLE = 25;
const int MOTOR_B_IN1 = 18;
const int MOTOR_B_IN2 = 19;
const int MOTOR_B_ENABLE = 14;

// Encoder pins
const int MOTOR_A_ENC_A = 34;
const int MOTOR_A_ENC_B = 35;
const int MOTOR_B_ENC_A = 4;
const int MOTOR_B_ENC_B = 5;

//bno055 pins


const int BNO_SDA = 32;
const int BNO_SCL = 33;

BNO055Sensor bno(BNO_SDA, BNO_SCL);

MotorController motors(
    MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_ENABLE,
    MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_ENABLE,
    MOTOR_A_ENC_A, MOTOR_A_ENC_B,
    MOTOR_B_ENC_A, MOTOR_B_ENC_B
);



void CALLBACK_FUNCTION SetSpeedA(int id) {
    float speed = menuSetA.getAsFloatingPointValue();
    Serial.println(speed);
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

void CALLBACK_FUNCTION SetThreshold(int id) {
    motors.setMinPWMThreshold(menuThreshold.getCurrentValue());
    // TODO - your menu change code
}



void CALLBACK_FUNCTION SavePID(int id) {
    menuMgr.save();
    EEPROM.commit();
    Serial.println("Saved!!!");
    // TODO - your menu change code
}


void updateMenuValues() {
    menuTicksA.setFloatValue(motors.getEncoderA(), false);
    menuTicksB.setFloatValue(motors.getEncoderB(), false);
    menuSpeedA.setFloatValue(motors.getCurrentSpeedA(), false);
    menuSpeedA.setFloatValue(motors.getCurrentSpeedB(), false);
    //Serial.println("updated");
}

void updateParams(){

    float angle_y = bno.getAngleY();
 /*    Serial.print("Angle between Z axis and vertical: ");
    Serial.print(angle_y);
    Serial.print(" degrees"); */
/*     if (!bno.isCalibrated()) {
        Serial.println("Sensor not fully calibrated");
    } */
    motors.setSpeeds(menuSetA.getAsFloatingPointValue(), menuSetB.getAsFloatingPointValue());
    
};

void printSpeedA(){
    Serial.print(">SpeedA:");
    Serial.print(motors.getCurrentSpeedA());
    Serial.print(",");
    Serial.print("TargetA:");
    Serial.println(menuSetA.getAsFloatingPointValue());
    

}
void printSpeedB(){
    Serial.print(">SpeedB:");
    Serial.print(motors.getCurrentSpeedB());
    Serial.print(",");
    Serial.print("TargetB:");
    Serial.println(menuSetB.getAsFloatingPointValue());
    

}



void setup() {
    Wire.begin();
    Serial.begin(115200);
    Serial.println("serial started");

    EEPROM.begin(EEPROM_SIZE);
    Serial.println("EEPROM initialized");

    if (!bno.begin()) {
        Serial.println("BNO055 initialization failed");
        while (1);
    }
    
    Serial.println("BNO055 initialized successfully");

    motors.init();
    setupMenu();
    menuMgr.load(MENU_MAGIC_KEY);
    taskManager.scheduleFixedRate(1000, updateParams);
}

void loop() {
    taskManager.runLoop();
    motors.update();
    if (menuSetA.getAsFloatingPointValue() != 0.0){
        printSpeedA();

    };
    if (menuSetB.getAsFloatingPointValue() != 0.0){
        printSpeedB();

    };


    //printAngle();
    
}



