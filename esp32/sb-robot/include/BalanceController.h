#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include "motorcontroller.h"
#include "BNO055Sensor.h"
#include <PID_v1.h>
#include "utils.h"



class BalanceController {
public:
    // struct pidParams
    //         {
    //             double Kp;
    //             double Ki;
    //             double Kd;
    //             float max;
    //             float min;
    //             int period;
    //             bool direct;
    //             bool modeAuto;
    //             /* data */
    //         };
    // struct controllerState 
    //         {
    //             float speedA;
    //             float speedB;
    //             float positionA;
    //             float positionB;
    //             double currentVel;
    //             double currentYawRate;
    //             double currentPitch;
    //             double targetPitch;
    //             double controlOutput;
    //             bool pitchPIDOn;
    //             bool velPIDOn;
    //             bool yawPIDOn;
                

    //         };

    BalanceController(MotorController &motors, 
                        BNO055Sensor &bno, 
                        pidParams &pitchParams, 
                        pidParams &velParams, 
                        pidParams &yawParams,
                        controllerState &controllerState, 
                        const float wheelRadius, 
                        const float wheelSeparation, 
                        double &targetVelocity, 
                        double &targetYawRate,
                        double &targetAngle,
                        int coreNum,
                        int bnoTaskPeriod
                    );
    
    void begin();
    void update();
    void stop();
    
    // Pitch PID methods
    void setTuningsPitch();
    void getPitchPID(double& kp, double& ki, double& kd);
    // void setPitchPIDLimits(double lower, double upper);
    // void setPitchPIDSampleTime(int sampleTime);
    // void setPitchPIDDirection(bool reverse);
    void updatePitchPID();
    
    // Velocity PID methods
    void setTuningsVel();
    void getVelocityPID(double& kp, double& ki, double& kd);
    // void setVelocityPIDLimits(double lower, double upper);
    // void setVelocityPIDSampleTime(int sampleTime);
    // void setVelocityPIDDirection(bool reverse);
    void updateVelPID();
    
    // Yaw PID methods
    void setTuningsYaw();
    void getYawPID(double& kp, double& ki, double& kd);
    // void setYawPIDLimits(double lower, double upper);
    // void setYawPIDSampleTime(int sampleTime);
    // void setYawPIDDirection(bool reverse);
    void updateYawPID();
    
    void updateState();
    void updateBno();

    // Other control methods
    // void setTargetVelocity(double& velocity);
    // void setTargetYawRate(double& yawRate);

    // void getWheelSpeeds(float& speedA, float& speedB);
    // void getWheelPositions(float& positionA, float& positionB);
    void setTargetAngle(float angle);
    void setTaskPeriod(int period);
    void setPitchPIDOn(bool state);
    void setVelPIDOn(bool state);
    void setYawPIDOn(bool state);
    void setBNOTaskPeriod(int period);
    void setLWSreshold(int value);
    void setRWSreshold(int value);
    void resetMotors();

   

 
    
private:
    MotorController& _motors;
    BNO055Sensor& _bno;
    
    PID _pitchPID;
    PID _velPID;
    PID _yawPID;

    pidParams& _pidParamsPitch;
    pidParams& _pidParamsVel;
    pidParams& _pidParamsYaw;

    controllerState& _state;

    int _coreNum;
    
    double _currentPitch, _pitchOutput, _targetPitch;
    double _currentVel, _velocityOutput;
    double _currentYawRate, _yawOutput;
    double& _targetYawRate, _targetVelocity, _manualAngle;
    float _positionA, _positionB, _speedA, _speedB;
    bool _pitchPidOn, _velPidOn, _yawPidOn;

    volatile TickType_t _controllerUpdatetaskPeriod;
    volatile TickType_t _bnoUpdatetaskPeriod;

    float _wheelRadius;
    float _wheelSeparation;

   // float& _manualAngle;
    void updatePitchControl();
    void updateVelocityControl();
    void updateYawControl();
    void calculateVelocities();
    
    void static controlTask(void * parameter);
    void static bnoTask(void * parameter);


};

#endif // BALANCE_CONTROLLER_H
