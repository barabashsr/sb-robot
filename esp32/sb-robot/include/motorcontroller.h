#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <PID_v1.h>

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
        double targetSpeed;
        double currentSpeed;
        double pidOutput;
    };

    MotorPins motorA;
    MotorPins motorB;
    Motor* sfMotorA;
    Motor* sfMotorB;
    int stbyPin;

    PID* pidA;
    PID* pidB;

    static const unsigned long PID_INTERVAL = 10; // ms
    unsigned long lastUpdateTime;

    static void IRAM_ATTR encoderISR_A();
    static void IRAM_ATTR encoderISR_B();
    static MotorController* instance;

    void updateMotorSpeed(MotorPins& motor);
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
    void setSpeedA(float speedTicks);
    void setSpeedB(float speedTicks);
    void setSpeeds(float speedA_ticks, float speedB_ticks);
    void setPID(float kp, float ki, float kd);
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

    // Unused methods and properties (kept for compatibility)
/*     float getKp() const { return 0; }
    float getKi() const { return 0; }
    float getKd() const { return 0; }
    float getKo() const { return 0; }
    void setPIDInterval(unsigned long interval) {}
    void setAutoStopInterval(unsigned long interval) {}
    void setMinPwmThreshold(int threshold) {}
    unsigned long getPIDInterval() const { return PID_INTERVAL; }
    unsigned long getAutoStopInterval() const { return 0; }
    int getMinPwmThreshold() const { return 0; }
    void setAutoStopEnabled(bool enabled) {}
    bool isAutoStopEnabled() const { return false; } */
};

#endif
