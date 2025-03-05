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

//#define BT_RX 19
//#define BT_TX 20



double ofsetAngle = 1.7;
double pitchAngle;
float speedA = 0.0; //in rad per second
float speedB = 0.0; //in radian per second
float positionA = 0; //in radians
float positionB = 0; //in radians
double x_vel = 0.0;  //in meters per second
double yaw_rate = 0.0; //in rad per second
const float wheelRadius = 0.0325; //in meters
const float wheelSeparation = 0.172; //in meters
float manualOfsetAngle = 0.77;

const int STATE_PRINT_INTERVAL = 200;
unsigned long lastPrintTime = 0;


//Bluetooth serial communication
//String inputString = "";
//bool stringComplete = false;



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


//PID controllers vars

double control_output = 0.0;

//PID values for Palstance control
double currentPalst; //palst pid input
double targetPalst;

double KpPalst = 20;   
double KdPalst = 0.6;
double KiPalst = 40.9;
double OutputPalst = 0;
float KoPalst = 1;
int palstSampleTime = 10;
bool palstPidOn = true;
const int pidLimitPalst = 255;


//PID values for Pitch control
double pitchInput, pitchOutput;
double KpPitch = 20;   
double KdPitch = 0.6;
double KiPitch = 40.9;
int pitchSampleTime = 10;
bool pitchPidOn = true;
const int pidLimitPitch = 255;

double target_angle = 0;

//PID values for linear velocity control
double KpVel = 20;   
double KdVel = 0.0;
double KiVel = 10;
float KoVel = 1;
double OutputVel = 0;
int velSampleTime = 20;
bool velPidOn = false;
const float velAngleLimit = 4.0;

//target angle
int speed_set = 0;

//target velocity

double target_vel = 0;

//PID values for linear velocity control
double KpYaw = 10;   
double KdYaw = 0.1;
double KiYaw = 5;
int yawSampleTime = 200;
bool yawPidOn = false;
const float yawRateLimit = 200;

double yawDelta = 0;
double target_YawRate = 0;

//PID pidPalst(&currentPalst, &OutputPalst, &targetPalst, KpPalst, KiPalst, KdPalst, REVERSE); //never gets to the saturation

PID pidPitch(&pitchAngle, &pitchOutput, &target_angle, KpPitch, KiPitch, KdPitch, REVERSE); //should be inversed? no

PID pidVel(&x_vel, &OutputVel, &target_vel, KpVel, KiVel, KdVel, REVERSE); //should be inversed? yes
PID pidYaw(&yaw_rate, &yawDelta, &target_YawRate, KpYaw, KiYaw, KdYaw, REVERSE);



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

void  calculateVelocities(){
    motors.updateSpeeds();
    speedA = motors.getSpeedA();
    speedB = motors.getSpeedB();
    positionA = motors.getPositionA();
    positionB = motors.getPositionB();
    x_vel = (speedA+speedB) * wheelRadius /2;
    yaw_rate = (speedA - speedB) * wheelRadius / wheelSeparation;





    


}

void angleControl(){
    
    pitchAngle = bno.getAngleY();
    
    //pitchInput = pitchAngle + ofsetAngle;
    if(pitchPidOn){
    pidPitch.Compute();
    control_output = pitchOutput;
    motors.setSpeeds(control_output+yawDelta, control_output-yawDelta);
    }
}
/*     void palstanceControl(){
    
    if(palstPidOn){
    bno.update();
    currentPalst = bno.getPalstance();
    pidPalst.Compute();
    } else {
        control_output = 0;
    }
    //speed_set = c;
    control_output = constrain(OutputPalst * KoPalst, -pidLimitPalst, pidLimitPalst);
    motors.setSpeeds(control_output+yawDelta, control_output-yawDelta);
    
} */




void speedControl(){
    calculateVelocities();
    if(velPidOn){
    //calculateVelocities();
    pidVel.Compute();
    target_angle = OutputVel; // * KoVel;
    
    //speed_set = pitchOutput;
    //motors.setSpeeds(speed_set, speed_set); 
    } else {
        //target_angle = manualOfsetAngle;
    }
}

void yawRateControl(){

    if(yawPidOn){
        pidYaw.Compute();
        
        //speed_set = pitchOutput;
        //motors.setSpeeds(speed_set, speed_set); 
    } else {
        //yawDelta = 0;
    }

}

void robotControl(){
    angleControl();
    speedControl();
    yawRateControl();

};


void updateMenuValues(){

    unsigned long currentMillis = millis();
    if (currentMillis - lastPrintTime >= STATE_PRINT_INTERVAL) {
        lastPrintTime = currentMillis;
        /* Serial.printf("A: Pose: %.1f (Vel: %.1f rad/s), B: Pose: %.1f (Vel: %.1f rad/s)\n",
            positionA,
            speedA,
            positionB,
            speedB
        );
        //float angle_y = bno.getAngleY();
        Serial.print("Input : ");
        Serial.print(pitchInput);
        float error = pitchInput - target_angle;
        Serial.print(" error: ");
        Serial.print(error);
        Serial.print(" pitchOutput: ");
        Serial.print(speed_set); */
    //float angleY = bno.getAngleY();
    menuPitch.setFloatValue(pitchAngle);
    int bnoCalib = bno.calibration();
    menuBNOCalib.setCurrentValue(bnoCalib);
    currentPalst = bno.getPalstance();

    menuVel.setFloatValue(x_vel);
    menuYawRate.setFloatValue(yaw_rate);
    menuPalstance.setFloatValue(currentPalst);
    //Serial.println(currentPalst);
    menuPitchOfset.setFromFloatingPointValue(target_angle);
    Serial.printf(" T_vel: %.2f, Vel: %.2f, T_angle: %.2f, Angle: %.2f output: %.2f\n", 
        target_vel,
        x_vel,
        target_angle, 
        pitchAngle, 
        control_output
    );
    

    setLedColor(bnoCalib);
    }
}




void processCommand(String command) {
    command.trim();
    /* if (command.startsWith("ma ")) {
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
    } */

    if (command.startsWith("kpp ")) {
        KpPitch = command.substring(4).toFloat();
        menuKpPitch.setFromFloatingPointValue(KpPitch);
        Serial.printf("Setting kp to %f\n Setting kd to %f\n Setting ki to %f\n", pidPitch.GetKp(), pidPitch.GetKd(), pidPitch.GetKi());
    
    }
    else if (command.startsWith("kip ")) {
        KiPitch = command.substring(4).toFloat();
        menuKiPitch.setFromFloatingPointValue(KiPitch);
        //Serial1.printf("Setting kp to %f\n Setting kd to %f\n Setting ki to %f\n", pidPitch.GetKp(), pidPitch.GetKd(), pidPitch.GetKi());
    
    }
    else if (command.startsWith("kdp ")) {
        KdPitch = command.substring(4).toFloat();
        menuKdPitch.setFromFloatingPointValue(KdPitch);
        //Serial1.printf("Setting kp to %f\n Setting kd to %f\n Setting ki to %f\n", pidPitch.GetKp(), pidPitch.GetKd(), pidPitch.GetKi());
    
    }
    else if (command.startsWith("stp ")) {
        pitchSampleTime = command.substring(4).toInt();
        menuPeriodP.setCurrentValue(pitchSampleTime);
        //Serial1.printf("Setting pitch loop to %d%%\n", pitchSampleTime);
    
    }

    else if (command.startsWith("oa ")) {
        if(!velPidOn){
            ofsetAngle = command.substring(3).toFloat();
        }
        
    }
    else if (command == "save") {
        menuSaveValues.changeOccurred(false);


        Serial.println("PID values saved to EEPROM");
        //Serial1.println("PID values saved to EEPROM");
      }

    else {
        Serial.println("Unknown command. Type 'kpp', 'kip', 'kdp' for change");
        //Serial1.println("Unknown command. Type 'kpp', 'kip', 'kdp' for change");
    }
    // pidPitch.SetTunings(KpPitch, KiPitch, KdPitch);
    // Serial.printf("Setting kp to %f\n Setting kd to %f\n Setting ki to %f\n", pid.GetKp(), pid.GetKd(), pid.GetKi());


}


volatile TickType_t palstancePeriod = pdMS_TO_TICKS(5);
volatile TickType_t robotControlPeriod = pdMS_TO_TICKS(10);
volatile TickType_t speedPeriod = pdMS_TO_TICKS(50);
volatile TickType_t yawRatePeriod = pdMS_TO_TICKS(50);


    //Functions to implementt PID controls on core1
/*     void palstanceControlTask(void * parameter) {
        TickType_t xLastWakeTime;
        //const TickType_t xFrequency = pdMS_TO_TICKS(5);
        xLastWakeTime = xTaskGetTickCount();
      
        for(;;) {
          palstanceControl();
          vTaskDelayUntil(&xLastWakeTime, palstancePeriod);
        }
      } */
      
      void robotControlTask(void * parameter) {
        TickType_t xLastWakeTime;
        //const TickType_t xFrequency = pdMS_TO_TICKS(10);
        xLastWakeTime = xTaskGetTickCount();
      
        for(;;) {
          robotControl();
          vTaskDelayUntil(&xLastWakeTime, robotControlPeriod);
        }
      }
      /* 
      void speedControlTask(void * parameter) {
        TickType_t xLastWakeTime;
        //const TickType_t xFrequency = pdMS_TO_TICKS(50);
        xLastWakeTime = xTaskGetTickCount();
      
        for(;;) {
          speedControl();
          vTaskDelayUntil(&xLastWakeTime, speedPeriod);
        }
      }
      
      void yawRateControlTask(void * parameter) {
        TickType_t xLastWakeTime;
        //const TickType_t xFrequency = pdMS_TO_TICKS(50);
        xLastWakeTime = xTaskGetTickCount();
      
        for(;;) {
          yawRateControl();
          vTaskDelayUntil(&xLastWakeTime, yawRatePeriod);
        }
      }
      
 */
    


void setup() {
    

    Serial.begin(115200);
    //Serial1.begin(9600, SERIAL_8N1, BT_RX, BT_TX); // Speed, Mode, RX pin, TX pin

    Serial.println("Serial started");
    //Serial1.println("//Serial1 started");

    EEPROM.begin(EEPROM_SIZE);
    Serial.println("EEPROM initialized"); 
    //Serial1.println("EEPROM initialized"); 
    
    //EEPROM.begin(EEPROM_SIZE);
    delay(100);
    motors.init();
    //inputString.reserve(200);
    
    //Serial.println("\nMotor Controller Test with TB6612FNG");
    setupMenu();
    menuMgr.load(MENU_MAGIC_KEY);

    //Assign Pitc PID values
    // KpPitch = menuKpPitch.getAsFloatingPointValue();
    // KdPitch = menuKdPitch.getAsFloatingPointValue();
    // KiPitch = menuKiPitch.getAsFloatingPointValue();
    // pitchSampleTime = menuPeriodP.getCurrentValue();
    // ofsetAngle = menuPitchOfset.getAsFloatingPointValue();
    //manualOfsetAngle = ofsetAngle;
    
    //Assign Palst PID values
/*     menuKpPalst.triggerCallback();
    menuKdPalst.triggerCallback();
    menuKiPalst.triggerCallback();
    menuKoPalst.triggerCallback();
    menuPeriodPalst.triggerCallback();
    menuPalstPIDToggle.triggerCallback(); */

    //Assign Pitch PID values
    menuKpPitch.triggerCallback();
    menuKdPitch.triggerCallback();
    menuKiPitch.triggerCallback();
    menuPeriodP.triggerCallback();
    //ofsetAngle = menuPitchOfset.getAsFloatingPointValue();
    menuPitchPIDToggle.triggerCallback();

    //Assign Velocity PID values
    menuKpVel.triggerCallback();
    menuKdVel.triggerCallback();
    menuKiVel.triggerCallback();
    menuKoVel.triggerCallback();
    menuPeriodV.triggerCallback();
    menuVelPIDToggle.triggerCallback();

    //Assign Yaw raye PID values
    menuKpYaw.triggerCallback();
    menuKdYaw.triggerCallback();
    menuKiYaw.triggerCallback();
    menuPeriodY.triggerCallback();
    menuYawPIDToggle.triggerCallback();

    //set motors thresholds
    menuThresholdA.triggerCallback();
    menuThresholdB.triggerCallback();




 
    pinMode(RGB_BUILTIN, OUTPUT);

    //setup bno
    if (!bno.begin()) {
        Serial.println("BNO055 initialization failed");
        //Serial1.println("BNO055 initialization failed");
        while (1);
    } else {
        Serial.println("BNO055 initialized successfully");
        //Serial1.println("BNO055 initialized successfully");

    }
    bno.setMeasurementPeriod(STATE_PRINT_INTERVAL);

/* 
    //setup Palst PID
    pidPalst.SetMode(AUTOMATIC);  
    pidPalst.SetOutputLimits(-pidLimitPalst, pidLimitPalst); 
    Serial.printf("Loaded Palst PID values - Kp: %f, Ki: %f, Kd: %f\n", KpPalst, KiPalst, KdPalst);
 */

    //setup Pitch PID
    pidPitch.SetMode(AUTOMATIC);  
    pidPitch.SetOutputLimits(-pidLimitPitch, pidLimitPitch); 
    // pidPitch.SetSampleTime(pitchSampleTime);
    // pidPitch.SetTunings(KpPitch, KiPitch, KdPitch);
    Serial.printf("Loaded Pitch PID values - Kp: %f, Ki: %f, Kd: %f\n", KpPitch, KiPitch, KdPitch);
    //Serial1.printf("Loaded Pitch PID values - Kp: %f, Ki: %f, Kd: %f\n", KpPitch, KiPitch, KdPitch);

     //setup Velocity PID
     pidVel.SetMode(AUTOMATIC);  
     pidVel.SetOutputLimits(-velAngleLimit, velAngleLimit); 
    //  pidVel.SetSampleTime(velSampleTime);
    //  pidVel.SetTunings(KpVel, KiVel, KdVel);
     Serial.printf("Loaded Velocity PID values - Kp: %f, Ki: %f, Kd: %f\n", KpVel, KiVel, KdVel);
     //Serial1.printf("Loaded Velocity PID values - Kp: %f, Ki: %f, Kd: %f\n", KpVel, KiVel, KdVel);

     //setup Yaw rate PID
     pidYaw.SetMode(AUTOMATIC);  
     pidYaw.SetOutputLimits(-yawRateLimit, yawRateLimit); 
    //  pidYaw.SetSampleTime(yawSampleTime);
    //  pidYaw.SetTunings(KpYaw, KiYaw, KdYaw);
     Serial.printf("Loaded Velocity PID values - Kp: %f, Ki: %f, Kd: %f\n", KpYaw, KiYaw, KdYaw);
     //Serial1.printf("Loaded Velocity PID values - Kp: %f, Ki: %f, Kd: %f\n", KpVel, KiVel, KdVel);

     motors.setMeasurementPeriod(velSampleTime);


     // Put PID functions on core 1
/* 
     xTaskCreatePinnedToCore(
        palstanceControlTask,   // Function to implement the task
        "PalstanceControl",     // Name of the task
        10000,                  // Stack size in words
        NULL,                   // Task input parameter
        4,                      // Priority of the task (highest)
        NULL,                   // Task handle
        1);                     // Core where the task should run (1)
     */
      xTaskCreatePinnedToCore(robotControlTask, "AngleControl", 10000, NULL, 3, NULL, 1);
      //xTaskCreatePinnedToCore(speedControlTask, "SpeedControl", 10000, NULL, 2, NULL, 1);
      //xTaskCreatePinnedToCore(yawRateControlTask, "YawRateControl", 10000, NULL, 1, NULL, 1);
 

}

void loop() {
    taskManager.runLoop();
    
    bno.update();
    //bno.update();
    // palstanceControl();
    // angleControl();
    //speedControl();
    // yawRateControl();

    updateMenuValues();
    /* while (//Serial1.available()) {
        char inChar = (char)//Serial1.read();
        if (inChar == '\n' || inChar == '\r') {
            if (inputString.length() > 0) {
                processCommand(inputString);
                inputString = "";
            }
        } else {
            inputString += inChar;
        }
    } */
    
    
    //setLedColor(bno.calibration());
    //taskManager.scheduleFixedRate(1000, updateMenuValues);

}



/* 

Palstance control pid loop


 */

 void CALLBACK_FUNCTION SetKdPalst(int id) {
/*     KdPalst = menuKdPitch.getAsFloatingPointValue();
    pidPalst.SetTunings(KpPalst, KiPalst, KdPalst);
    Serial.println("Palst Kd changed"); */
    // TODO - your menu change code
}



void CALLBACK_FUNCTION SetKiPalst(int id) {
/*     KiPalst = menuKiPitch.getAsFloatingPointValue();
    pidPalst.SetTunings(KpPalst, KiPalst, KdPalst);
    Serial.println("Palst Ki changed"); */
    // TODO - your menu change code
}


void CALLBACK_FUNCTION togglePalstPid(int id) {
/*     if (menuPalstPIDToggle.getBoolean()){
        palstPidOn = true;
        pidPalst.SetMode(AUTOMATIC);
    } else {
        palstPidOn = false;
        //menuPitchOfset.setFromFloatingPointValue(manualOfsetAngle);
        pidPalst.SetMode(MANUAL);
        
    } */
}


void CALLBACK_FUNCTION SetKpPalst(int id) {
/*     KpPalst = menuKpPitch.getAsFloatingPointValue();
    pidPalst.SetTunings(KpPalst, KiPalst, KdPalst);
    Serial.println("Palst Kp changed");
    // TODO - your menu change code */
}

void CALLBACK_FUNCTION SetKoPalst(int id) {
/*     KoPalst = menuKoPalst.getAsFloatingPointValue();
    Serial.println("Palst Ko changed");
    // TODO - your menu change code */
}


void CALLBACK_FUNCTION setPalstPIDPeriod(int id) {
/*     palstSampleTime = menuPeriodPalst.getCurrentValue()+1;
    pidPalst.SetSampleTime(palstSampleTime);
    palstancePeriod = pdMS_TO_TICKS(palstSampleTime);

    bno.setMeasurementPeriod(palstSampleTime); */
    // TODO - your menu change code
}

/* 

Pitch control pid loop


 */


void CALLBACK_FUNCTION togglePitchPid(int id) {
    if (menuPitchPIDToggle.getBoolean()){
        pitchPidOn = true;
        pidPitch.SetMode(AUTOMATIC);
    } else {
        pitchPidOn = false;
        //menuPitchOfset.setFromFloatingPointValue(manualOfsetAngle);
        pidPitch.SetMode(MANUAL);
        targetPalst = 0;
        
    }
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKiPitch(int id) {
    KiPitch = menuKiPitch.getAsFloatingPointValue();
    pidPitch.SetTunings(KpPitch, KiPitch, KdPitch);
    Serial.println("Pitch Ki changed");
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKpPitch(int id) {
    KpPitch = menuKpPitch.getAsFloatingPointValue();
    pidPitch.SetTunings(KpPitch, KiPitch, KdPitch);
    Serial.println("Pitch Kp changed");
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKdPitch(int id) {
    KdPitch = menuKdPitch.getAsFloatingPointValue();
    pidPitch.SetTunings(KpPitch, KiPitch, KdPitch);
    Serial.println("Pitch Kd changed");

    // TODO - your menu change code
}


void CALLBACK_FUNCTION setPitchPIDPeriod(int id) {
    pitchSampleTime = menuPeriodP.getCurrentValue()+1;
    robotControlPeriod = pdMS_TO_TICKS(pitchSampleTime);
    pidPitch.SetSampleTime(pitchSampleTime);
    // TODO - your menu change code
}


/* 

Velocity control pid loop


 */


 void CALLBACK_FUNCTION toggleVelPid(int id) {
    if (menuVelPIDToggle.getBoolean()){
        velPidOn = true;
        pidVel.SetMode(AUTOMATIC);
    } else {
        velPidOn = false;
        menuPitchOfset.setFromFloatingPointValue(manualOfsetAngle);
        target_angle = manualOfsetAngle;
        pidVel.SetMode(MANUAL);
        
    }
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKpVel(int id) {
    KpVel = menuKpVel.getAsFloatingPointValue();
    pidVel.SetTunings(KpVel, KiVel, KdVel);
    Serial.printf("Vel Kp: %2f, Vel Ki: %2f,Vel Kd: %2f\n", pidVel.GetKp(), pidVel.GetKi(),pidVel.GetKd());

}


void CALLBACK_FUNCTION SetKdVel(int id) {
    KdVel = menuKdVel.getAsFloatingPointValue();
    pidVel.SetTunings(KpVel, KiVel, KdVel);
    Serial.printf("Vel Kp: %2f, Vel Ki: %2f,Vel Kd: %2f\n", pidVel.GetKp(), pidVel.GetKi(),pidVel.GetKd());
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKiVel(int id) {
    KiVel = menuKiVel.getAsFloatingPointValue();
    pidVel.SetTunings(KpVel, KiVel, KdVel);
    Serial.printf("Vel Kp: %2f, Vel Ki: %2f,Vel Kd: %2f\n", pidVel.GetKp(), pidVel.GetKi(),pidVel.GetKd());
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setVelPIDPeriod(int id) {
    velSampleTime = menuPeriodV.getCurrentValue()+1;
    //speedPeriod = pdMS_TO_TICKS(velSampleTime);
    pidVel.SetSampleTime(pitchSampleTime);
    //motors.setMeasurementPeriod(velSampleTime);
    if (yawSampleTime>velSampleTime){
        motors.setMeasurementPeriod(velSampleTime);

    }
    // TODO - your menu change code
}

void CALLBACK_FUNCTION SetKoVel(int id) {
    KoVel =  menuKoVel.getAsFloatingPointValue();
    // TODO - your menu change code
}



/* 

Yaw rate control pid loop


 */




void CALLBACK_FUNCTION SetKiYaw(int id) {
    KiYaw = menuKiPitch.getAsFloatingPointValue();
    pidYaw.SetTunings(KpYaw, KiYaw, KdYaw);
    Serial.println("Yaw Ki changed");
    // TODO - your menu change code
}



void CALLBACK_FUNCTION SetKpYaw(int id) {
    KpYaw = menuKpPitch.getAsFloatingPointValue();
    pidYaw.SetTunings(KpYaw, KiYaw, KdYaw);
    Serial.println("Yaw Kp changed");
    // TODO - your menu change code
}



void CALLBACK_FUNCTION SetKdYaw(int id) {
    KdYaw = menuKdPitch.getAsFloatingPointValue();
    pidYaw.SetTunings(KpYaw, KiYaw, KdYaw);
    Serial.println("Yaw Kp changed");
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setYawPIDPeriod(int id) {
    yawSampleTime = menuPeriodY.getCurrentValue() + 1;
    //yawRatePeriod = pdMS_TO_TICKS(yawSampleTime);
    pidYaw.SetSampleTime(yawSampleTime);
    if (yawSampleTime<velSampleTime){
        motors.setMeasurementPeriod(yawSampleTime);

    }
    // TODO - your menu change code
}


void CALLBACK_FUNCTION toggleYawPID(int id) {
    if (menuYawPIDToggle.getBoolean()){
        yawPidOn = true;
        pidYaw.SetMode(AUTOMATIC);
    } else {
        yawPidOn = false;
        yawDelta = 0.0;
        pidYaw.SetMode(MANUAL);
        
    }
    
}


/* 

Utilities


 */

void CALLBACK_FUNCTION SavePID(int id) {
    menuMgr.save(MENU_MAGIC_KEY);
    EEPROM.commit();
    Serial.println("Saved!!!");
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetPitchOfset(int id) {
    if (!velPidOn){
    manualOfsetAngle = menuPitchOfset.getAsFloatingPointValue();
    target_angle = manualOfsetAngle;
    Serial.print("Pitch ofset changed to");
    Serial.println(ofsetAngle);
    
    } else {
        //menuPitchOfset.setFromFloatingPointValue(ofsetAngle);
    }
    // TODO - your menu change code
}


void CALLBACK_FUNCTION getBNOCalib(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setTargetYawRa(int id) {
    target_YawRate = menuSetYaw.getAsFloatingPointValue();
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setTargetVel(int id) {
    target_vel = menuSetVel.getAsFloatingPointValue();
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setThresholdB(int id) {
    int threshold = menuThresholdA.getCurrentValue();
    motors.setThresholdA(threshold);
    Serial.printf("Treshold motor A changet to: %3d", threshold);
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setThresholdA(int id) {
    int threshold = menuThresholdB.getCurrentValue();
    motors.setThresholdB(threshold);
    Serial.printf("Treshold motor A changet to: %3d", threshold);
    // TODO - your menu change code
}


