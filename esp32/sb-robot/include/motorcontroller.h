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
        long prevCount;
        float targetSpeed; // Target speed in rad/s
        long prevError; // For PID calculation
        float iTerm; // Integral term
        float currentSpeed; // Current speed in rad/s
        unsigned long startTime; // For soft start
    };

    MotorPins motorA;
    MotorPins motorB;

    static const int PWM_FREQUENCY = 20000; // Increased from 5000 to 20000 Hz
    static const int PWM_RESOLUTION = 8;
    const unsigned long PID_INTERVAL = 10; // 100Hz update rate
    unsigned long nextPID;
    unsigned long lastMotorCommand;
    static const unsigned long AUTO_STOP_INTERVAL = 2000;
    unsigned long safetyDelay; // Delay before motor start

    // PID parameters
    float Kp;
    float Ki;
    float Kd;
    float Ko; // Output scaling

    bool moving;

    void initMotor(MotorPins& motor);
    void setMotorSpeed(MotorPins& motor, int pwmValue);
    void updateMotorPID(MotorPins& motor);
    float calculateCurrentSpeed(MotorPins& motor);
    int mapSpeed(float speed);

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
    void update();
    void setSpeedA(float speedRads);
    void setSpeedB(float speedRads);
    void setSpeeds(float speedA_rads, float speedB_rads);
    void setPID(float kp, float ki, float kd, float ko);
    void setSafetyDelay(unsigned long delay_ms);
    void stop();
    long getEncoderA();
    long getEncoderB();
    float getPositionA();
    float getPositionB();
    float getCurrentSpeedA();
    float getCurrentSpeedB();
    void resetEncoders();
    void handleEncoderA();
    void handleEncoderB();
};

#endif