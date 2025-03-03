#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>

class MotorController {
private:
    struct MotorPins {
        int in1;
        int in2;
        int pwm;
        int encoderA;
        int encoderB;
        volatile long encoderCount;
        int ticksPerRevolution;

        
    };

    MotorPins motorA;
    MotorPins motorB;
    int standbyPin;

    static const int PWM_FREQUENCY = 5000;
    static const int PWM_RESOLUTION = 8;

    void initMotor(MotorPins& motor);
    void setMotorSpeed(const MotorPins& motor, int speed);
    static void IRAM_ATTR encoderISR_C();
    static void IRAM_ATTR encoderISR_D();
    static MotorController* instance;

    unsigned long lastMeasurementTime;
    int measurementPeriod; // in milliseconds
    long lastEncoderCountA;
    long lastEncoderCountB;
    float speedA;
    float speedB;
    int threshold_A;    //initial start threshold 0-255
    int threshold_B;    //initial start threshold 0-255

public:
    MotorController(
        int motorA_in1, int motorA_in2, int motorA_pwm,
        int motorB_in1, int motorB_in2, int motorB_pwm,
        int standby,
        int motorA_encA, int motorA_encB,
        int motorB_encA, int motorB_encB,
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
    float getPositionA();
    float getPositionB();
    void resetEncoders();
    void handleEncoderA();
    void handleEncoderB();
    void setMeasurementPeriod(int period);
    float getSpeedA(){return speedA;};
    float getSpeedB(){return speedB;};
    void updateSpeeds();
    void setThresholdA(int threshold){threshold_A = threshold;};
    void setThresholdB(int threshold){threshold_B = threshold;};
};

#endif
