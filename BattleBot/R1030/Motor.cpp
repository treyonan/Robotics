#include "Motor.h"

void Motor::Output(int signalLevel, String MotorName) {
  writeMicroseconds(signalLevel); 
  _Output = signalLevel;  
  _MotorName = MotorName;
}
void Motor::RampMotor(String Direction, int PT, String MotorName) {
  Motor::RampTimer.EN = true;    
  if (Direction == "UP") {   
    StartFlag = true;   
    _ScaledHigh = 2000; // Ramping up to 2000
    if (_State == STOPPED) {
      Motor::RampTimer.RES();
      Motor::RampTimer.PT = PT;  
      _ScaledLow = 1000; // Starting at 1000    
      _State = RAMP_UP;      
    }
    if (_State == RAMP_DOWN) { // If switched to ramp up before ramp down is complete
      Motor::RampTimer.RES();
      Motor::RampTimer.PT = ((_RampLevel-1000)/1000)*PT; // Calculate new PT based on current ramplevel
      _ScaledLow = _RampLevel; // Start at current ramp level
      _State = RAMP_UP;
      //Motor::PrintMotorInfo();
    }    
  }
  else {        
    _ScaledHigh = 1000; // Ramping down to 1000
    if (_State == RUNNING) {
      Motor::RampTimer.RES();
      Motor::RampTimer.PT = PT;    
      _ScaledLow = 2000; // Starting at 2000
      _State = RAMP_DOWN;
    }
    if (_State == RAMP_UP) { // If switched to ramp down before ramp up is complete
      Motor::RampTimer.RES();
      Motor::RampTimer.PT = ((_RampLevel-1000)/1000)*PT; // Calculate new PT based on current ramplevel
      _ScaledLow = _RampLevel; // Start at current ramp level
      _State = RAMP_DOWN;   
    }    
  }   
  if (Motor::RampTimer.DN()) {
    if (_State == RAMP_UP) {
      _State = RUNNING;
    }
    else if (_State == RAMP_DOWN) {
      _State = STOPPED;
    }
  }
  if (StartFlag) {
    _RampLevel = constrain(map(Motor::RampTimer.ET, 0, Motor::RampTimer.PT, _ScaledLow, _ScaledHigh), 1000, 2000);
  }
  else {
    _RampLevel = 1000;
  }
  Motor::Output(_RampLevel, MotorName); 
  Motor::PrintMotorInfo(); 
}
void Motor::PrintMotorInfo() {
  Serial.print("State: ");  
  Serial.print(_State);
  Serial.print(", ");
  Serial.print("Ramp Level: ");
  Serial.print(_RampLevel);
  Serial.print(", ");  
  Serial.print("PT Calc: ");
  Serial.println(Motor::RampTimer.PT);    
}

