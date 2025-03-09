#include "BalanceController.h"

BalanceController::BalanceController(MotorController &motors,
    BNO055Sensor &bno,
    pidParams &pitchParams,
    pidParams &velParams,
    pidParams &yawParams,
    controllerState &controllerState,
    float wheelRadius,
    float wheelSeparation,
    double &targetVelocity,
    double &targetYawRate,
    double &targetAngle,
    int coreNum, 
    int bnoTaskPeriod)
    
    : _motors(motors), _bno(bno), 
      _pidParamsPitch(pitchParams), _pidParamsVel(velParams), _pidParamsYaw(yawParams),
      _targetVelocity(targetVelocity), _targetYawRate(targetYawRate), _manualAngle(targetAngle),
      _wheelRadius(wheelRadius), _wheelSeparation(wheelSeparation),
      _coreNum(coreNum),
      _currentPitch(0), _pitchOutput(0), _targetPitch(0),
      _currentVel(0), _velocityOutput(0),
      _currentYawRate(0), _yawOutput(0),
      _state(controllerState),
      _positionA(0), _positionB(0), _speedA(0), _speedB(0),
      _pitchPidOn(false), _velPidOn(false), _yawPidOn(false),
      _controllerUpdatetaskPeriod(0),
      _pitchPID(&_currentPitch, &_pitchOutput, &_targetPitch, pitchParams.Kp, pitchParams.Ki, pitchParams.Kd, DIRECT),
      _velPID(&_currentVel, &_velocityOutput, &targetVelocity, velParams.Kp, velParams.Ki, velParams.Kd, REVERSE),
      _yawPID(&_currentYawRate, &_yawOutput, &targetYawRate, yawParams.Kp, yawParams.Ki, yawParams.Kd, REVERSE)
  
      
{ 
    setBNOTaskPeriod(bnoTaskPeriod);


    _wheelRadius = wheelRadius;
    _wheelSeparation = wheelSeparation;
    Serial.println("constructed");
    
/* 
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
    _state.controlOutput = &_pitchOutput;

 */

 

    

    
    
}

void BalanceController::updateYawPID(){
    _yawPID.SetMode(_pidParamsYaw.modeAuto ? AUTOMATIC : MANUAL);
    _yawPID.SetOutputLimits(_pidParamsYaw.min, _pidParamsYaw.max);
    _yawPID.SetSampleTime(_pidParamsYaw.period);
    _yawPID.SetControllerDirection(_pidParamsYaw.direct ? DIRECT : REVERSE);
    setYawPIDOn(_pidParamsYaw.modeAuto);
};

void BalanceController::updateVelPID(){
    _velPID.SetMode(_pidParamsVel.modeAuto ? AUTOMATIC : MANUAL);
    _velPID.SetOutputLimits(_pidParamsVel.min, _pidParamsVel.max);
    _velPID.SetSampleTime(_pidParamsVel.period);
    _velPID.SetControllerDirection(_pidParamsVel.direct ? DIRECT : REVERSE);
    _motors.setMeasurementPeriod(_pidParamsVel.period);
    setVelPIDOn(_pidParamsVel.modeAuto);
};

void BalanceController::updatePitchPID(){
    _pitchPID.SetMode(_pidParamsPitch.modeAuto ? AUTOMATIC : MANUAL);
    _pitchPID.SetOutputLimits(_pidParamsPitch.min, _pidParamsPitch.max);
    _pitchPID.SetSampleTime(_pidParamsPitch.period);
    _pitchPID.SetControllerDirection(_pidParamsPitch.direct ? DIRECT : REVERSE);
    setPitchPIDOn(_pidParamsPitch.modeAuto);
    _controllerUpdatetaskPeriod = pdMS_TO_TICKS(_pidParamsPitch.period);
};

void BalanceController::setTuningsPitch(){
    _pitchPID.SetTunings(_pidParamsPitch.Kp, _pidParamsPitch.Ki, _pidParamsPitch.Kd);

}

void BalanceController::setTuningsVel(){
    _velPID.SetTunings(_pidParamsVel.Kp, _pidParamsVel.Ki, _pidParamsVel.Kd);

}

void BalanceController::setTuningsYaw(){
    _yawPID.SetTunings(_pidParamsYaw.Kp, _pidParamsYaw.Ki, _pidParamsYaw.Kd);

}




void BalanceController::begin() {

    updatePitchPID();  
    setTuningsPitch();

    updateVelPID();
    setTuningsVel();

    updateYawPID();
    setTuningsYaw();
    Serial.println("init");

    xTaskCreatePinnedToCore(controlTask, "BalanceControl", 10000, this, 3, NULL, static_cast<BaseType_t>(_coreNum));
    xTaskCreatePinnedToCore(bnoTask, "BNOUpdateTask", 10000, this, 3, NULL, static_cast<BaseType_t>(_coreNum));

}

void BalanceController::update() {
    _currentPitch = _bno.getAngleY();
    updatePitchControl();
    //Serial.printf("pitch updated: current: %f, target: %f, output \n", _currentPitch, _targetPitch,  _pitchOutput);

    calculateVelocities();
    //Serial.println("vel calculated");

    updateVelocityControl();
    //Serial.println("vel updated");
    updateYawControl();
    //Serial.println("yaw updated");
     
}

void BalanceController::updateBno(){
    _bno.update();
}


void BalanceController::controlTask(void * parameter) {
    BalanceController* controller = static_cast<BalanceController*>(parameter);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
  
    for(;;) {
      controller->update();
      vTaskDelayUntil(&xLastWakeTime, controller->_controllerUpdatetaskPeriod);
    }
}

void BalanceController::bnoTask(void * parameter) {
    BalanceController* controller = static_cast<BalanceController*>(parameter);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
  
    for(;;) {
      controller->updateBno();
      vTaskDelayUntil(&xLastWakeTime, controller->_bnoUpdatetaskPeriod);
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

void BalanceController::getPitchPID(double& kp, double& ki, double& kd) {
    kp = _pitchPID.GetKp();
    ki = _pitchPID.GetKi();
    kd = _pitchPID.GetKd();
}

void BalanceController::getVelocityPID(double& kp, double& ki, double& kd) {
    kp = _velPID.GetKp();
    ki = _velPID.GetKi();
    kd = _velPID.GetKd();
}

void BalanceController::getYawPID(double& kp, double& ki, double& kd) {
    kp = _yawPID.GetKp();
    ki = _yawPID.GetKi();
    kd = _yawPID.GetKd();
}

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
    _controllerUpdatetaskPeriod = pdMS_TO_TICKS(period);
};

void BalanceController::setBNOTaskPeriod(int period){
    _bnoUpdatetaskPeriod = pdMS_TO_TICKS(period);
};



void BalanceController::setPitchPIDOn(bool state) {
    _pitchPidOn = state;
    _state.pitchPIDOn = state;
    _pitchPID.SetMode(state ? AUTOMATIC : MANUAL);
    Serial.printf("Pitch PID set to %d\n", _state.pitchPIDOn);
}

void BalanceController::setVelPIDOn(bool state) {
    _velPidOn = state;
    _state.velPIDOn = state;
    _velPID.SetMode(state ? AUTOMATIC : MANUAL);
    Serial.printf("Vel PID set to %d\n", _state.velPIDOn);
    
}

void BalanceController::setYawPIDOn(bool state) {
    _yawPidOn = state;
    _state.yawPIDOn = state;
    _yawPID.SetMode(state ? AUTOMATIC : MANUAL);
    Serial.printf("Yaw PID set to %d\n", _state.yawPIDOn);

}

void BalanceController::updateState(){
    
    _state.speedA = _speedA;
    _state.speedB = _speedB;
    _state.positionA = _positionA;
    _state.positionB = _positionB;
    _state.currentVel = _currentVel;
    _state.currentYawRate = _currentYawRate;
    _state.currentPitch = _currentPitch;
    _state.targetPitch = _targetPitch;
    _state.controlOutput = _pitchOutput;
    _state.pitchPIDOn = _pitchPidOn;
    _state.velPIDOn = _velPidOn;
    _state.yawPIDOn = _yawPidOn;
    _state.targetVel = _targetVelocity;
};

void BalanceController::setLWSreshold(int value){
    _motors.setThresholdB(value);

};
void BalanceController::setRWSreshold(int value){
    _motors.setThresholdA(value);

};

void BalanceController::resetMotors(){
    _motors.resetEncoders();
    
    this->calculateVelocities();
    this->updateState();
};

void BalanceController::setTicksPerRevolution(int value){
    _motors.setTicksPerRevolution(value);
};