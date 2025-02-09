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
        bool reversed;
        int encoderA;
        int encoderB;
        volatile long encoderCount;
        int ticksPerRevolution;
    };
    
    MotorPins motorA;
    MotorPins motorB;
    
    static const int PWM_FREQUENCY = 5000;
    static const int PWM_RESOLUTION = 8;

    void initMotor(MotorPins& motor);
    void setMotorSpeed(const MotorPins& motor, int speed);

    static void IRAM_ATTR encoderISR_A();
    static void IRAM_ATTR encoderISR_B();
    static MotorController* instance;

public:
    MotorController(
        int motorA_in1, int motorA_in2, int motorA_enable,
        int motorB_in1, int motorB_in2, int motorB_enable,
        int motorA_encA, int motorA_encB,
        int motorB_encA, int motorB_encB,
        bool motorA_reversed = false,
        bool motorB_reversed = false,
        int motorA_ticks = 585,
        int motorB_ticks = 585
    );

    void init();
    void setMotorA(int speed);
    void setMotorB(int speed);
    void setSpeeds(int speedA, int speedB);
    void stop();

    long getEncoderA();
    long getEncoderB();
    float getPositionA(); // Returns position in radians
    float getPositionB(); // Returns position in radians
    void resetEncoders();
    
    // Methods for ISR
    void handleEncoderA();
    void handleEncoderB();
};

#endif
