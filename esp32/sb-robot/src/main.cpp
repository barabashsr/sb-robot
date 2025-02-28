#include <Arduino.h>
#include "motorcontroller.h"
#include "BNO055Sensor.h"
#include <PID_v1.h>
//#include "esp_mac.h"
// Pin definitions for TB6612FNG

#define OFSET_ANGLE_Y -0.2;

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
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 20;   
double Kd = 0.5;
double Ki = 40;

//target angle
double target_angle = 0;
int speed_set = 0;

PID pid(&input, &output, &target_angle, Kp, Ki, Kd, REVERSE);


BNO055Sensor bno(BNO_SDA, BNO_SCL);

MotorController motors(
    MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM,
    MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM,
    STANDBY,
    MOTOR_A_ENC_A, MOTOR_A_ENC_B,
    MOTOR_B_ENC_A, MOTOR_B_ENC_B
);

unsigned long lastEncoderPrint = 0;
const int STATE_PRINT_INTERVAL = 1000;

String inputString = "";
bool stringComplete = false;


void printState(){

    unsigned long currentMillis = millis();
    if (currentMillis - lastEncoderPrint >= STATE_PRINT_INTERVAL) {
        lastEncoderPrint = currentMillis;
        Serial.printf("Encoders - A: %ld (%.2f rad), B: %ld (%.2f rad)\n",
            motors.getEncoderA(),
            motors.getPositionA(),
            motors.getEncoderB(),
            motors.getPositionB()
        );
        //float angle_y = bno.getAngleY();
        Serial.print("Input : ");
        Serial.print(input);
        float error = input - target_angle;
        Serial.print(" error: ");
        Serial.print(error);
        Serial.print(" output: ");
        Serial.print(speed_set);

        //Serial.println(bno.calibration());
        
    }
    
};

void processCommand(String command) {
    command.trim();
    if (command.startsWith("ma ")) {
        int speed = command.substring(3).toInt();
        motors.setMotorA(speed);
        Serial.printf("Setting Motor A to %d%%\n", speed);
    }
    else if (command.startsWith("mb ")) {
        int speed = command.substring(3).toInt();
        motors.setMotorB(speed);
        Serial.printf("Setting Motor B to %d%%\n", speed);
    }
    else if (command.startsWith("both ")) {
        int speed = command.substring(5).toInt();
        motors.setSpeeds(speed, speed);
        Serial.printf("Setting both motors to %d%%\n", speed);
    }
    else if (command == "stop") {
        motors.stop();
        Serial.println("Stopping all motors");
    }
    else if (command == "reset") {
        motors.resetEncoders();
        Serial.println("Resetting encoder counts");
    }
    else if (command == "help") {
        Serial.println("Available commands:");
        Serial.println("ma <speed> - Set Motor A speed (-100 to 100)");
        Serial.println("mb <speed> - Set Motor B speed (-100 to 100)");
        Serial.println("both <speed> - Set both motors speed");
        Serial.println("stop - Stop all motors");
        Serial.println("reset - Reset encoder counts");
        Serial.println("help - Show this help");
    }
    else {
        Serial.println("Unknown command. Type 'help' for available commands");
    }


}

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
    input = bno.getAngleY() + OFSET_ANGLE_Y;
    pid.Compute();
    speed_set = output;
    motors.setSpeeds(speed_set, speed_set);
    
}



void setup() {
    Serial.begin(115200);
    delay(1000);
    motors.init();
    Serial.println("\nMotor Controller Test with TB6612FNG");
    Serial.println("Type 'help' for available commands");
    inputString.reserve(200);
    pinMode(RGB_BUILTIN, OUTPUT);

    //setup bno
    if (!bno.begin()) {
        Serial.println("BNO055 initialization failed");
        while (1);
    } else {
        Serial.println("BNO055 initialized successfully");

    }

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-100, 100); 
}

void loop() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r') {
            if (inputString.length() > 0) {
                processCommand(inputString);
                inputString = "";
            }
        } else {
            inputString += inChar;
        }
    }
    printState();
    setLedColor(bno.calibration());
    angleControl();
    
   


    


}


