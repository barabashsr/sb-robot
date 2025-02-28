#include "menu_menu.h"
#include <Arduino.h>
#include "motorcontroller.h"
#include "BNO055Sensor.h"
#include <PID_v1.h>
#include <EEPROM.h>
#include <Preferences.h>

#define EEPROM_SIZE 512
#ifndef MENU_MAGIC_KEY
#define MENU_MAGIC_KEY 0xfade
#endif 


float ofsetAngle = 1.7;
float pitchAngle;
float speedA = 0.0; //in rad per second
float speedB = 0.0; //in radian per second
float positionA = 0; //in radians
float positionB = 0; //in radians
float x_vel = 0;  //in meters per second
float yaw_rate = 0; //in rad per second
const float wheelRadius = 0.0325; //in meters
const float wheelSeparation = 0.172; //in meters

const int STATE_PRINT_INTERVAL = 200;
unsigned long lastEncoderPrint = 0;



const int MOTOR_A_IN1 = 1;
const int MOTOR_A_IN2 = 2;
const int MOTOR_A_PWM = 3;
const int MOTOR_B_IN1 = 11;
const int MOTOR_B_IN2 = 12;
const int MOTOR_B_PWM = 13;
const int STANDBY = 10;

// Encoder pins
const int MOTOR_A_ENC_A = 17;
const int MOTOR_A_ENC_B = 18;
const int MOTOR_B_ENC_A = 4;
const int MOTOR_B_ENC_B = 5;

// BNO055 pins
const int BNO_SDA = 15;
const int BNO_SCL = 16;

//#define RGB_BUILTIN 48

//PID
//double originalSetpoint = 173;
//double setpoint = originalSetpoint;
double movingAngleOffset = 0;
double input, output;

//adjust these values to fit your own design
double KpPitch = 30;   
double KdPitch = 0.5;
double KiPitch = 10;
int pitchSampleTime = 10;

//target angle
double target_angle = 0;
int speed_set = 0;

PID pidPitch(&input, &output, &target_angle, KpPitch, KiPitch, KdPitch, REVERSE);


BNO055Sensor bno(BNO_SDA, BNO_SCL);

MotorController motors(
    MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM,
    MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM,
    STANDBY,
    MOTOR_A_ENC_A, MOTOR_A_ENC_B,
    MOTOR_B_ENC_A, MOTOR_B_ENC_B
);



void setLedColor(int calibration) {
    switch (calibration) {
      case 0:
        neopixelWrite(RGB_BUILTIN, 64, 0, 0); // Red
        break;
      case 1:
        neopixelWrite(RGB_BUILTIN, 64, 40, 0); // Orange
        break;
      case 2:
        neopixelWrite(RGB_BUILTIN, 64, 64, 0); // Yellow
        break;
      case 3:
        neopixelWrite(RGB_BUILTIN, 0, 64, 0); // Green
        break;
      default:
        neopixelWrite(RGB_BUILTIN, 0, 0, 0); // Off
        break;
    }
  }


void angleControl(){
    pitchAngle = bno.getAngleY();
    input = pitchAngle + ofsetAngle;
    pidPitch.Compute();
    speed_set = output;
    motors.setSpeeds(speed_set, speed_set);
    
}


void updateMenuValues(){

    unsigned long currentMillis = millis();
    if (currentMillis - lastEncoderPrint >= STATE_PRINT_INTERVAL) {
        lastEncoderPrint = currentMillis;
        Serial.printf("A: Pose: %.1f (Vel: %.1f rad/s), B: Pose: %.1f (Vel: %.1f rad/s)\n",
            positionA,
            speedA,
            positionB,
            speedB
        );
        //float angle_y = bno.getAngleY();
        Serial.print("Input : ");
        Serial.print(input);
        float error = input - target_angle;
        Serial.print(" error: ");
        Serial.print(error);
        Serial.print(" output: ");
        Serial.print(speed_set);
    //float angleY = bno.getAngleY();
    menuPitch.setFloatValue(pitchAngle);
    int bnoCalib = bno.calibration();
    menuBNOCalib.setCurrentValue(bnoCalib);

    menuVel.setFloatValue(x_vel);
    menuYawRate.setFloatValue(yaw_rate);

    setLedColor(bnoCalib);
    }
}

void calculateVelocities(){
    motors.updateSpeeds();
    speedA = motors.getSpeedA();
    speedB = motors.getSpeedB();
    positionA = motors.getPositionA();
    positionB = motors.getPositionB();
    x_vel = (speedA+speedB) * wheelRadius /2;
    yaw_rate = (speedA - speedB) * wheelRadius / wheelSeparation;





    


}

void setup() {
    

    Serial.begin(115200);

    Serial.println("serial started");

    EEPROM.begin(EEPROM_SIZE);
    Serial.println("EEPROM initialized"); 
    
    //EEPROM.begin(EEPROM_SIZE);
    delay(100);
    motors.init();
    Serial.println("\nMotor Controller Test with TB6612FNG");
    setupMenu();
    menuMgr.load(MENU_MAGIC_KEY);
    KpPitch = menuKpPitch.getAsFloatingPointValue();
    KdPitch = menuKdPitch.getAsFloatingPointValue();
    KiPitch = menuKiPitch.getAsFloatingPointValue();
    pitchSampleTime = menuPeriodP.getCurrentValue();
    ofsetAngle = menuPitchOfset.getAsFloatingPointValue();

    


 
    pinMode(RGB_BUILTIN, OUTPUT);

    //setup bno
    if (!bno.begin()) {
        Serial.println("BNO055 initialization failed");
        while (1);
    } else {
        Serial.println("BNO055 initialized successfully");

    }

    //setup PID
    pidPitch.SetMode(AUTOMATIC);  
    pidPitch.SetOutputLimits(-100, 100); 




    pidPitch.SetSampleTime(pitchSampleTime);
    pidPitch.SetTunings(KpPitch, KiPitch, KdPitch);
    
    Serial.printf("Loaded PID values - Kp: %f, Ki: %f, Kd: %f\n", KpPitch, KiPitch, KdPitch);

}

void loop() {
    taskManager.runLoop();
    calculateVelocities();
    angleControl();
    updateMenuValues();
    
    
    //setLedColor(bno.calibration());
    //taskManager.scheduleFixedRate(1000, updateMenuValues);

}


void CALLBACK_FUNCTION StopB(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKo(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION StopA(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKp(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION ResetTicsA(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKd(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION ResetTicsB(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetPIDHz(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKi(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetSpeedA(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetThreshold(int id) {
    // TODO - your menu change code
}



void CALLBACK_FUNCTION SavePID(int id) {
    menuMgr.save(MENU_MAGIC_KEY);
    EEPROM.commit();
    Serial.println("Saved!!!");
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetSpeedB(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKpVel(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKiYaw(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setPitchPIDPeriod(int id) {
    pitchSampleTime = menuPeriodP.getCurrentValue();
    pidPitch.SetSampleTime(pitchSampleTime);
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKdVel(int id) {
    // TODO - your menu change code
}



void CALLBACK_FUNCTION setTargetYawRa(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetPitchOfset(int id) {
    ofsetAngle = menuPitchOfset.getAsFloatingPointValue();
    Serial.print("Pitch ofset changed to");
    Serial.println(ofsetAngle);
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKiPitch(int id) {
    KiPitch = menuKiPitch.getAsFloatingPointValue();
    pidPitch.SetTunings(KpPitch, KiPitch, KdPitch);
    Serial.println("Pitch Ki changed");
    // TODO - your menu change code
}


void CALLBACK_FUNCTION getBNOCalib(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKiVel(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKpPitch(int id) {
    KpPitch = menuKpPitch.getAsFloatingPointValue();
    pidPitch.SetTunings(KpPitch, KiPitch, KdPitch);
    Serial.println("Pitch Kp changed");
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKpYaw(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setVelPIDPeriod(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKdPitch(int id) {
    KdPitch = menuKdPitch.getAsFloatingPointValue();
    pidPitch.SetTunings(KpPitch, KiPitch, KdPitch);
    Serial.println("Pitch Kd changed");

    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKdYaw(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setYawPIDPeriod(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setTargetVel(int id) {
    // TODO - your menu change code
}
