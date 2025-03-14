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
<<<<<<< .merge_file_nsEwFd
        int pwm;
=======
        int enable;
        bool reversed;
>>>>>>> .merge_file_OUQ7wx
        int encoderA;
        int encoderB;
        volatile long encoderCount;
        int ticksPerRevolution;
<<<<<<< .merge_file_nsEwFd

        
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
=======
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
>>>>>>> .merge_file_OUQ7wx

public:
    MotorController(
        int motorA_in1, int motorA_in2, int motorA_pwm,
        int motorB_in1, int motorB_in2, int motorB_pwm,
        int standby,
        int motorA_encA, int motorA_encB,
        int motorB_encA, int motorB_encB,
<<<<<<< .merge_file_nsEwFd
        int motorA_ticks,
        int motorB_ticks
=======
        int stbyPin,
        bool motorA_reversed = false,
        bool motorB_reversed = false,
        int motorA_ticks = 585,
        int motorB_ticks = 585
>>>>>>> .merge_file_OUQ7wx
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
<<<<<<< .merge_file_nsEwFd
    void resetEncoders();
    void handleEncoderA();
    void handleEncoderB();
    void setMeasurementPeriod(int period);
    float getSpeedA(){return speedA;};
    float getSpeedB(){return speedB;};
    void updateSpeeds();
    void setThresholdA(int threshold){threshold_A = threshold;};
    void setThresholdB(int threshold){threshold_B = threshold;};
    void setTicksPerRevolution(int value){motorA.ticksPerRevolution = value; motorB.ticksPerRevolution = value;};
=======
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
>>>>>>> .merge_file_OUQ7wx
};

#endif
