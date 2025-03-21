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
      _currentPitch(0.0), _pitchOutput(0.0), _targetPitch(0.0),
      _currentVel(0.0), _velocityOutput(0.0),
      _currentYawRate(0.0), _yawOutput(0.0),
      _state(controllerState),
      _positionA(0), _positionB(0), _speedA(0), _speedB(0),
      _pitchPidOn(false), _velPidOn(false), _yawPidOn(false),
      _controllerUpdatetaskPeriod(0), _x(0), _y(0), _theta(0), _lastPosA(0), _lastPosB(0), 
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
    static bool firstRun = true;
    
    if (firstRun) {
        // On first run, use manual angle instead of PID output
        _targetPitch = _manualAngle;
        firstRun = false;
        return;
    }
    if (_velPidOn) {
        // Check for valid velocity before computing
        if (!isnan(_currentVel) && !isinf(_currentVel)) {
            _velPID.Compute();
            _targetPitch = _velocityOutput;
        } else {
            // If velocity is invalid, use manual angle
            _targetPitch = _manualAngle;
        }
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
    _currentVel = (_speedA + _speedB) * _wheelRadius /2; 
    _currentYawRate = (_speedA - _speedB) * _wheelRadius / _wheelSeparation; 
    

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


void BalanceController::calculateTransform(transform& tf) {
    // Get current wheel positions in radians
    double currentPosA = _state.positionA;  // Left wheel position
    double currentPosB = _state.positionB;  // Right wheel position
    
    // Calculate wheel position changes since last update
    double deltaA = currentPosA - _lastPosA;  // Left wheel delta
    double deltaB = currentPosB - _lastPosB;  // Right wheel delta
    
    // Convert wheel rotation to distance traveled
    double distanceA = deltaA * _wheelRadius;  // Left wheel distance
    double distanceB = deltaB * _wheelRadius;  // Right wheel distance
    
    // Calculate robot movement
    double distanceCenter = (distanceA + distanceB) / 2.0;  // Linear distance
    double deltaTheta = (distanceB - distanceA) / _wheelSeparation;  // Angular change
    
    // Update robot pose
    // First rotate, then translate
    _theta += deltaTheta;
    
    // Normalize theta to [-π, π]
    while (_theta > M_PI) _theta -= 2 * M_PI;
    while (_theta < -M_PI) _theta += 2 * M_PI;
    
    // Calculate displacement in global frame
    _x += distanceCenter * cos(_theta);
    _y += distanceCenter * sin(_theta);
    double cy = cos(_theta * 0.5);
    double sy = sin(_theta * 0.5);

    tf.t_x = _x;
    tf.t_y = _y;
    tf.t_z = 0;
    tf.r_x = 0;
    tf.r_y = 0;
    tf.r_z = sy;
    tf.r_w = cy;

    
    // Store current positions for next update
    _lastPosA = currentPosA;
    _lastPosB = currentPosB;
}
