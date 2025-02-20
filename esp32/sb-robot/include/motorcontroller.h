#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include <SparkFun_TB6612.h>

#define SPEED_FILTER_SIZE 5

class MotorController {
private:
    struct MotorPins {
        int in1;
        int in2;
        int enable;
        bool reversed;
        int encoderA;
        int encoderB;
        volatile long encoderCount;
        int ticksPerRevolution;
        long prevCount;
        float targetSpeed;
        long prevError;
        float iTerm;
        float currentSpeed;
        float speedBuffer[SPEED_FILTER_SIZE];
        int bufferIndex;
    };

    MotorPins motorA;
    MotorPins motorB;
    Motor* sfMotorA;
    Motor* sfMotorB;
    int stbyPin;

    unsigned long PID_INTERVAL;
    unsigned long lastMotorCommand;
    unsigned long AUTO_STOP_INTERVAL;
    float Kp;
    float Ki;
    float Kd;
    float Ko;
    bool moving;
    int minPwmThreshold;
    bool autoStopEnabled;

    static void IRAM_ATTR encoderISR_A();
    static void IRAM_ATTR encoderISR_B();
    static MotorController* instance;

    float calculateCurrentSpeed(MotorPins& motor);
    void updateMotorPID(MotorPins& motor);
    static void controllerTaskCode(void* parameter);
    TaskHandle_t controllerTask;

public:
    MotorController(
        int motorA_in1, int motorA_in2, int motorA_enable,
        int motorB_in1, int motorB_in2, int motorB_enable,
        int motorA_encA, int motorA_encB,
        int motorB_encA, int motorB_encB,
        int stbyPin,
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
    void setPWMA(int pwm);
    void setPWMB(int pwm);

    float getKp() const { return Kp; }
    float getKi() const { return Ki; }
    float getKd() const { return Kd; }
    float getKo() const { return Ko; }
    void setPIDInterval(unsigned long interval) { PID_INTERVAL = interval; }
    void setAutoStopInterval(unsigned long interval) { AUTO_STOP_INTERVAL = interval; }
    void setMinPwmThreshold(int threshold) { minPwmThreshold = threshold; }
    unsigned long getPIDInterval() const { return PID_INTERVAL; }
    unsigned long getAutoStopInterval() const { return AUTO_STOP_INTERVAL; }
    int getMinPwmThreshold() const { return minPwmThreshold; }
    void setAutoStopEnabled(bool enabled) { autoStopEnabled = enabled; }
    bool isAutoStopEnabled() const { return autoStopEnabled; }
};

#endif
