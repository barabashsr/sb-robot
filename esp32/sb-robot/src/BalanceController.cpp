#include "BalanceController.h"

BalanceController::BalanceController(MotorController& motors, 
                                        BNO055Sensor& bno, 
                                        pidParams& pitchParams, 
                                        pidParams& velParams, 
                                        pidParams& yawParams,
                                        controllerState& controllerState, 
                                        float wheelRadius, 
                                        float wheelSeparation,
                                        double& targetVelocity,
                                        double& targetYawRate,
                                        float& targetAngle
                                    )
    : _motors(motors), _bno(bno), _targetVelocity(targetVelocity), _targetYawRate(targetYawRate), _manualAngle(targetAngle), _state(controllerState),
      _pidParamsPitch(pitchParams), _pidParamsVel(velParams), _pidParamsYaw(yawParams),
      _pitchPID(&_currentPitch, &_pitchOutput, &_targetPitch, pitchParams.Kp, pitchParams.Ki, pitchParams.Kd, pitchParams.direct ? DIRECT : REVERSE),
      _velPID(&_currentVel, &_velocityOutput, &_targetVelocity, velParams.Kp, velParams.Ki, velParams.Kd, velParams.direct ? DIRECT : REVERSE),
      _yawPID(&_currentYawRate, &_yawOutput, &_targetYawRate, yawParams.Kp, yawParams.Ki, yawParams.Kd, yawParams.direct ? DIRECT : REVERSE)
      
{ 


    _wheelRadius = wheelRadius;
    _wheelSeparation = wheelSeparation;
    

    _state.speedA = &_speedA;
    _state.speedB = &_speedB;
    _state.positionA = &_positionA;
    _state.positionB = &_positionB;
    _state.currentVel = &_currentVel;
    _state.currentYawRate = &_currentYawRate;
    _state.currentPitch = &_currentPitch;
    _state.currentVel = &_currentVel;
    _state.targetPitch = &_targetPitch;
    _state.pitchPIDOn = &_pitchPidOn;
    _state.yawPIDOn = &_yawPidOn;
    _state.velPIDOn = &_velPidOn;



    

    
    
}

void BalanceController::updateYawPID(){
    _yawPID.SetMode(_pidParamsYaw.modeAuto ? AUTOMATIC : MANUAL);
    _yawPID.SetOutputLimits(_pidParamsYaw.min, _pidParamsYaw.max);
    _yawPID.SetSampleTime(_pidParamsYaw.period);
    _yawPID.SetControllerDirection(_pidParamsYaw.direct ? DIRECT : REVERSE);
};

void BalanceController::updateVelPID(){
    _velPID.SetMode(_pidParamsVel.modeAuto ? AUTOMATIC : MANUAL);
    _velPID.SetOutputLimits(_pidParamsVel.min, _pidParamsVel.max);
    _velPID.SetSampleTime(_pidParamsVel.period);
    _velPID.SetControllerDirection(_pidParamsVel.direct ? DIRECT : REVERSE);
};

void BalanceController::updatePitchPID(){
    _velPID.SetMode(_pidParamsPitch.modeAuto ? AUTOMATIC : MANUAL);
    _velPID.SetOutputLimits(_pidParamsPitch.min, _pidParamsPitch.max);
    _velPID.SetSampleTime(_pidParamsPitch.period);
    _velPID.SetControllerDirection(_pidParamsPitch.direct ? DIRECT : REVERSE);
    _taskPeriod = pdMS_TO_TICKS(_pidParamsPitch.period);
};

void BalanceController::setTuningsPitch(){
    _pitchPID.SetTunings(_pidParamsPitch.Kp, _pidParamsPitch.Ki, _pidParamsPitch.Kd);

}

void BalanceController::setTuningsVel(){
    _velPID.SetTunings(_pidParamsVel.Kp, _pidParamsVel.Ki, _pidParamsVel.Kd);

}

void BalanceController::setTuningsYaw(){
    _velPID.SetTunings(_pidParamsYaw.Kp, _pidParamsYaw.Ki, _pidParamsYaw.Kd);

}




void BalanceController::begin(pidParams& pitchParams, pidParams& velParams, pidParams& yawParams) {

    updatePitchPID();  
    setTuningsPitch();
    updateVelPID();
    setTuningsVel();
    updateYawPID();
    setTuningsYaw();
    xTaskCreatePinnedToCore(controlTask, "BalanceControl", 10000, NULL, 3, NULL, 1);

}

void BalanceController::update() {
    _currentPitch = _bno.getAngleY();
    updatePitchControl();

    calculateVelocities();

    updateVelocityControl();
    updateYawControl();
     
}

void BalanceController::controlTask(void * parameter) {
    BalanceController* controller = static_cast<BalanceController*>(parameter);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
  
    for(;;) {
      controller->update();
      vTaskDelayUntil(&xLastWakeTime, controller->_taskPeriod);
    }
}


void BalanceController::stop(){
    //just in case
}


void BalanceController::updatePitchControl() {
 
    //calculate motor output
    if (_pitchPidOn) {
        _pitchPID.Compute();
        double leftMotorOutput = _pitchOutput + _yawOutput;
        double rightMotorOutput = _pitchOutput - _yawOutput;
        _motors.setSpeeds(leftMotorOutput, rightMotorOutput);
    } else {
        _motors.stop();
    }

}

void BalanceController::updateVelocityControl() {
    if(_velPidOn) {
        _velPID.Compute();
        _targetPitch = _velocityOutput;
    } else {
        _targetPitch = _manualAngle;
    }
}

void BalanceController::updateYawControl() {
    if(_yawPidOn) {
        _yawPID.Compute();
    } else {
        _yawOutput = 0;
    }
}

// Pitch PID methods
// void BalanceController::setPitchPID(double kp, double ki, double kd) {
//     _pitchPID.SetTunings(kp, ki, kd);
// }

void BalanceController::getPitchPID(double& kp, double& ki, double& kd) {
    kp = _pitchPID.GetKp();
    ki = _pitchPID.GetKi();
    kd = _pitchPID.GetKd();
}

// void BalanceController::setPitchPIDLimits(double lower, double upper) {
//     _pitchPID.SetOutputLimits(lower, upper);
// }

// void BalanceController::setPitchPIDSampleTime(int sampleTime) {
//     _pitchPID.SetSampleTime(sampleTime);
// }

// void BalanceController::setPitchPIDDirection(bool reverse) {
//     _pitchPID.SetControllerDirection(reverse ? REVERSE : DIRECT);
// }

// Velocity PID methods
// void BalanceController::setVelocityPID(double kp, double ki, double kd) {
//     _velocityPID.SetTunings(kp, ki, kd);
// }

void BalanceController::getVelocityPID(double& kp, double& ki, double& kd) {
    kp = _velPID.GetKp();
    ki = _velPID.GetKi();
    kd = _velPID.GetKd();
}

// void BalanceController::setVelocityPIDLimits(double lower, double upper) {
//     _velocityPID.SetOutputLimits(lower, upper);
// }

// void BalanceController::setVelocityPIDSampleTime(int sampleTime) {
//     _velocityPID.SetSampleTime(sampleTime);
// }

// void BalanceController::setVelocityPIDDirection(bool reverse) {
//     _velocityPID.SetControllerDirection(reverse ? REVERSE : DIRECT);
// }

// Yaw PID methods
// void BalanceController::setYawPID(double kp, double ki, double kd) {
//     _yawPID.SetTunings(kp, ki, kd);
// }

void BalanceController::getYawPID(double& kp, double& ki, double& kd) {
    kp = _yawPID.GetKp();
    ki = _yawPID.GetKi();
    kd = _yawPID.GetKd();
}

// void BalanceController::setYawPIDLimits(double lower, double upper) {
//     _yawPID.SetOutputLimits(lower, upper);
// }

// void BalanceController::setYawPIDSampleTime(int sampleTime) {
//     _yawPID.SetSampleTime(sampleTime);
// }

// void BalanceController::setYawPIDDirection(bool reverse) {
//     _yawPID.SetControllerDirection(reverse ? REVERSE : DIRECT);
// }

// Other control methods
// void BalanceController::setTargetVelocity(double& velocity) {
//     _targetVelocity = velocity;
// }

// void BalanceController::setTargetYawRate(double& yawRate) {
//     _targetYawRate = yawRate;
// }

// void BalanceController::getWheelSpeeds(float& speedA, float& speedB){
//     speedA = _speedA;
//     speedB = _speedB;

// }

// void BalanceController::getWheelPositions(float& positionA, float& positionB){
//     positionA = _positionA;
//     positionB = _positionB;
// }

void BalanceController::calculateVelocities(){

    _motors.updateSpeeds();
    _speedA = _motors.getSpeedA();
    _speedB = _motors.getSpeedB();
    _positionA = _motors.getPositionA();
    _positionB = _motors.getPositionB();
    _currentVel = (_speedA + _speedB) * _wheelRadius /2; // Assuming this gives linear velocity
    _currentYawRate = (_speedA - _speedB) * _wheelRadius / _wheelSeparation; // Assuming this gives yaw rate
    

};

void BalanceController::setTargetAngle(float angle) {
    _manualAngle = angle;
};

void BalanceController::setTaskPeriod(int period){
    _taskPeriod = pdMS_TO_TICKS(period);
};

void BalanceController::setPitchPIDOn(bool state) {
    _pitchPidOn = state;
    _pitchPID.SetMode(state ? AUTOMATIC : MANUAL);
}

void BalanceController::setVelPIDOn(bool state) {
    _velPidOn = state;
    _velPID.SetMode(state ? AUTOMATIC : MANUAL);
}

void BalanceController::setYawPIDOn(bool state) {
    _yawPidOn = state;
    _yawPID.SetMode(state ? AUTOMATIC : MANUAL);

}

void BalanceController::setPitchPIDOn(bool state) {_velPidOn = state;};
void BalanceController::setPitchPIDOn(bool state) {_yawPidOn = state;};