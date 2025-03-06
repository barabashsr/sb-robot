#include "menu_menu.h"
#include <Arduino.h>
#include "motorcontroller.h"
#include "BNO055Sensor.h"
#include <PID_v1.h>
#include <EEPROM.h>
#include <Preferences.h>
#include "BalanceController.h"
#include "utils.h"
#include <WiFi.h>



#define EEPROM_SIZE 512
#ifndef MENU_MAGIC_KEY
#define MENU_MAGIC_KEY 0xfade
#endif 




//#define BT_RX 19
//#define BT_TX 20




const float wheelRadius = 0.0325; //in meters
const float wheelSeparation = 0.172; //in meters
// float manualOfsetAngle = 0.77;

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




BNO055Sensor bno(BNO_SDA, BNO_SCL);

MotorController motors(
    MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM,
    MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM,
    STANDBY,
    MOTOR_A_ENC_A, MOTOR_A_ENC_B,
    MOTOR_B_ENC_A, MOTOR_B_ENC_B
);

pidParams paramsPitch{0.0, 0.0, 0.0, 0.0f, 0.0f, 7, false, true};
pidParams paramsVel{0.0, 0.0, 0.0, 0.0f, 0.0f, 7, false, true}, paramsYaw{0.0, 0.0, 0.0, 0.0f, 0.0f, 7, false, true};

controllerState contrState;

double targetVel = 0.0;
double targetYawRate = 0.0;
double targetPitch = 0.7;

int BalanceTaskCore = 0;
int bnoTaskPeriod = 3;


BalanceController controller(motors, 
                            bno, 
                            paramsPitch, 
                            paramsVel, 
                            paramsYaw,
                            contrState,
                            wheelRadius,
                            wheelSeparation,
                            targetVel,
                            targetYawRate,
                            targetPitch,
                            BalanceTaskCore,
                            bnoTaskPeriod
                        );



///ros setup



#include <std_msgs/msg/int32.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <stdio.h>

#define ROS_DOMAIN_ID 77    
IPAddress agent_ip; //(192, 168, 1, 74);
size_t agent_port = 8888;

String localIP = "192.168.1.74";

// char ssid[] = "Beeline_5G_F13F37";
// char psk[]= "1122334455667788";

String ssid = "Beeline_2G_F13F37";
String password = "1122334455667788";

char ssidR[] = "Beeline_2G_F13F37";
char passwordR[] = "1122334455667788";

// int argc;
// const char * const * argv;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msgIn, msgOut;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return;}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// Error handle loop
void error_loop() {
  while(1) {
    //printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
      RCSOFTCHECK(rcl_publish(&publisher, &msgOut, NULL));
      msgOut.data++;
      //Serial.println(msg.data);
    }
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	Serial.printf("Received: %d\n", msg->data);
}



// void connectToWiFi(const String& ssid, const String& password, IPAddress agent_ip, uint16_t agent_port) {
//     // Convert String to char array
//     char ssidR[ssid.length() + 1];
//     char passwordR[password.length() + 1];
    
//     // Copy the contents of the String to the char array
//     strcpy(ssidR, ssid.c_str());
//     strcpy(passwordR, password.c_str());
    
//     // Now use these char arrays with set_microros_wifi_transports
//     set_microros_wifi_transports(ssidR, passwordR, agent_ip, agent_port);
// }

void connectToWiFi(const String& ssid, const String& password, const String& agent_ip_str, uint16_t agent_port) {
    char ssidR[ssid.length() + 1];
    char passwordR[password.length() + 1];
    
    strcpy(ssidR, ssid.c_str());
    strcpy(passwordR, password.c_str());
    
    IPAddress agent_ip;
    if (!agent_ip.fromString(agent_ip_str)) {
        Serial.println("Invalid IP address format");
        return;
    }
    
    set_microros_wifi_transports(ssidR, passwordR, agent_ip, agent_port);
}




///ros setup




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
    controller.updateState();
    menuPitch.setFloatValue(static_cast<float>(contrState.currentPitch));
    int bnoCalib = bno.calibration();
    menuBNOCalib.setCurrentValue(bnoCalib);
    //currentPalst = bno.getPalstance();

    menuVel.setFloatValue(static_cast<float>(contrState.currentVel));
    menuYawRate.setFloatValue(static_cast<float>(contrState.currentYawRate));
    //menuPalstance.setFloatValue(currentPalst);
    //Serial.println(currentPalst);
    if (contrState.velPIDOn){
        menuPitchOfset.setFromFloatingPointValue(static_cast<float>(contrState.targetPitch));
    }
    double Kp, Ki, Kd;
    controller.getPitchPID(Kp, Ki, Kd);
    
    /* Serial.printf(" Pitch: Kp: %.2f, Kd: %.2f, Ki: %.2f, output:  %.2f, pitch:  %.2f\n", 
       Kp, Ki, Kd, contrState.controlOutput, contrState.currentPitch
    ); */


    

    setLedColor(bnoCalib);
    }
}




void processCommand(String command) {
    // command.trim();
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

    // if (command.startsWith("kpp ")) {
    //     KpPitch = command.substring(4).toFloat();
    //     menuKpPitch.setFromFloatingPointValue(KpPitch);
    //     Serial.printf("Setting kp to %f\n Setting kd to %f\n Setting ki to %f\n", pidPitch.GetKp(), pidPitch.GetKd(), pidPitch.GetKi());
    
    // }
    // else if (command.startsWith("kip ")) {
    //     KiPitch = command.substring(4).toFloat();
    //     menuKiPitch.setFromFloatingPointValue(KiPitch);
    //     //Serial1.printf("Setting kp to %f\n Setting kd to %f\n Setting ki to %f\n", pidPitch.GetKp(), pidPitch.GetKd(), pidPitch.GetKi());
    
    // }
    // else if (command.startsWith("kdp ")) {
    //     KdPitch = command.substring(4).toFloat();
    //     menuKdPitch.setFromFloatingPointValue(KdPitch);
    //     //Serial1.printf("Setting kp to %f\n Setting kd to %f\n Setting ki to %f\n", pidPitch.GetKp(), pidPitch.GetKd(), pidPitch.GetKi());
    
    // }
    // else if (command.startsWith("stp ")) {
    //     pitchSampleTime = command.substring(4).toInt();
    //     menuPeriodP.setCurrentValue(pitchSampleTime);
    //     //Serial1.printf("Setting pitch loop to %d%%\n", pitchSampleTime);
    
    // }

    // else if (command.startsWith("oa ")) {
    //     if(!velPidOn){
    //         ofsetAngle = command.substring(3).toFloat();
    //     }
        
    // }
    // else if (command == "save") {
    //     menuSaveValues.changeOccurred(false);


    //     Serial.println("PID values saved to EEPROM");
    //     //Serial1.println("PID values saved to EEPROM");
    //   }

    // else {
    //     Serial.println("Unknown command. Type 'kpp', 'kip', 'kdp' for change");
    //     //Serial1.println("Unknown command. Type 'kpp', 'kip', 'kdp' for change");
    // }
    // // pidPitch.SetTunings(KpPitch, KiPitch, KdPitch);
    // // Serial.printf("Setting kp to %f\n Setting kd to %f\n Setting ki to %f\n", pid.GetKp(), pid.GetKd(), pid.GetKi());


}






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


/* 
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    Serial.printf("Connected to WiFi IP: %s", WiFi.localIP());

 */

    motors.init();
    //inputString.reserve(200);
    
    //Serial.println("\nMotor Controller Test with TB6612FNG");
    setupMenu();
    menuMgr.load(MENU_MAGIC_KEY);


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

    paramsPitch.direct = false;
    paramsPitch.max = 255;
    paramsPitch.min = -255;
    
    paramsVel.direct = false;
    paramsVel.max = 4.0;
    paramsVel.min = -4.0;
   
    paramsYaw.direct = false;
    paramsYaw.max = 200;
    paramsYaw.max = -200;


    controller.begin();
    Serial.println("Balance controller started");


    ///MICRO ROS SETUP

    //set_microros_wifi_transports(ssidR, passwordR, agent_ip, agent_port);
    connectToWiFi(ssid, password, localIP, agent_port);



    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();

    // Initialize init options
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    // Set ROS_DOMAIN_ID
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

    // Initialize support with options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    //create init_options
    //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "micro_ros_platformio_node_publisher"));
    

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "micro_ros_platformio_node_publisher"));

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor

    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    delay(1000);
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msgIn, &subscription_callback, ON_NEW_DATA));
    

    msgOut.data = 0;





    //MICRO ROS SETUP


}

void loop() {
    taskManager.runLoop();
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    //bno.update();
    //controller.update();

    updateMenuValues();
}




 void CALLBACK_FUNCTION SetKdPalst(int id) {

}



void CALLBACK_FUNCTION SetKiPalst(int id) {

}


void CALLBACK_FUNCTION togglePalstPid(int id) {

}


void CALLBACK_FUNCTION SetKpPalst(int id) {

}

void CALLBACK_FUNCTION SetKoPalst(int id) {

}


void CALLBACK_FUNCTION setPalstPIDPeriod(int id) {

}

/* 

Pitch control pid loop


 */


void CALLBACK_FUNCTION togglePitchPid(int id) {
   controller.setPitchPIDOn(menuPitchPIDToggle.getBoolean());

}


void CALLBACK_FUNCTION SetKiPitch(int id) {
    paramsPitch.Ki = menuKiPitch.getAsFloatingPointValue();
    controller.setTuningsPitch();
    Serial.println("Pitch Ki changed");
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKpPitch(int id) {
    paramsPitch.Kp = menuKpPitch.getAsFloatingPointValue();
    controller.setTuningsPitch();
    Serial.println("Pitch Kp changed");
    // TODO - your menu change code
}


void CALLBACK_FUNCTION SetKdPitch(int id) {
    paramsPitch.Kd = menuKdPitch.getAsFloatingPointValue();
    controller.setTuningsPitch();
    Serial.println("Pitch Kd changed");

    // TODO - your menu change code
}


void CALLBACK_FUNCTION setPitchPIDPeriod(int id) {
    paramsPitch.period = menuPeriodP.getCurrentValue() + 1;
    controller.updatePitchPID();
    
    // TODO - your menu change code
}


/* 

Velocity control pid loop


 */


 void CALLBACK_FUNCTION toggleVelPid(int id) {
    bool state = menuVelPIDToggle.getBoolean();
    if (!state){
        menuPitchOfset.setFromFloatingPointValue(targetPitch);

    }

    controller.setVelPIDOn(state);
}


void CALLBACK_FUNCTION SetKpVel(int id) {
    paramsVel.Kp = menuKpVel.getAsFloatingPointValue();
    controller.setTuningsVel();
    Serial.printf("Vel Kp: %2f, Vel Ki: %2f,Vel Kd: %2f\n", paramsVel.Kp, paramsVel.Ki, paramsVel.Kd);

}


void CALLBACK_FUNCTION SetKdVel(int id) {
    paramsVel.Kd = menuKdVel.getAsFloatingPointValue();
    controller.setTuningsVel();
    Serial.printf("Vel Kp: %2f, Vel Ki: %2f,Vel Kd: %2f\n", paramsVel.Kp, paramsVel.Ki, paramsVel.Kd);
}


void CALLBACK_FUNCTION SetKiVel(int id) {
    paramsVel.Ki = menuKiVel.getAsFloatingPointValue();
    controller.setTuningsVel();
    Serial.printf("Vel Kp: %2f, Vel Ki: %2f,Vel Kd: %2f\n", paramsVel.Kp, paramsVel.Ki, paramsVel.Kd);
}


void CALLBACK_FUNCTION setVelPIDPeriod(int id) {
    paramsVel.period = menuPeriodV.getCurrentValue() + 1;
    controller.updateVelPID();
}

void CALLBACK_FUNCTION SetKoVel(int id) {
    
    // TODO - your menu change code
}



/* 

Yaw rate control pid loop


 */




void CALLBACK_FUNCTION SetKiYaw(int id) {
    paramsYaw.Ki = menuKiYaw.getAsFloatingPointValue();
    controller.setTuningsYaw();
    Serial.println("Yaw Ki changed");
    // TODO - your menu change code
}



void CALLBACK_FUNCTION SetKpYaw(int id) {
    paramsYaw.Kp = menuKpYaw.getAsFloatingPointValue();
    controller.setTuningsYaw();
    Serial.println("Yaw Kp changed");
    // TODO - your menu change code
}



void CALLBACK_FUNCTION SetKdYaw(int id) {
    paramsYaw.Kd = menuKdYaw.getAsFloatingPointValue();
    controller.setTuningsYaw();
    Serial.println("Yaw Kp changed");
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setYawPIDPeriod(int id) {
    paramsYaw.period = menuPeriodY.getCurrentValue() + 1;
    controller.updateYawPID();
   
}


void CALLBACK_FUNCTION toggleYawPID(int id) {

        controller.setPitchPIDOn(menuYawPIDToggle.getBoolean());
    
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
    if(!contrState.velPIDOn){
    targetPitch = menuPitchOfset.getAsFloatingPointValue();
    }
    // TODO - your menu change code
}


void CALLBACK_FUNCTION getBNOCalib(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setTargetYawRa(int id) {
    targetYawRate = menuSetYaw.getAsFloatingPointValue();
    // TODO - your menu change code
}


void CALLBACK_FUNCTION setTargetVel(int id) {
    targetVel = menuSetVel.getAsFloatingPointValue();
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


