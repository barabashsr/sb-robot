#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>

class MotorController {
private:
    struct MotorPins {
        int in1;
        int in2;
        int enable;
        int pwmChannel;
    };
    
    MotorPins motorA;
    MotorPins motorB;
    
    static const int PWM_FREQUENCY = 5000;
    static const int PWM_RESOLUTION = 8;

    void initMotor(const MotorPins& motor);
    void setMotorSpeed(const MotorPins& motor, int speed);

public:
    MotorController(
        int motorA_in1, int motorA_in2, int motorA_enable,
        int motorB_in1, int motorB_in2, int motorB_enable
    );

    void init();
    void setMotorA(int speed);
    void setMotorB(int speed);
    void setSpeeds(int speedA, int speedB);
    void stop();
};

#endif
