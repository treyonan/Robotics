#include "Drive.h"

Drive::Drive(Motor& DriveMotorA, Motor& DriveMotorB, int MotorAChannel, int MotorBChannel, int CalibrateChannel, Channel& Channels, IMU& myIMU)
  : _DriveMotorA(DriveMotorA), _DriveMotorB(DriveMotorB), _MotorAChannel(MotorAChannel), _MotorBChannel(MotorBChannel), _CalibrateChannel(CalibrateChannel), _Channels(Channels), _myIMU(myIMU) {
  _DriveMotorA_CutOff = _Channels.CutOff[_MotorAChannel];
  _DriveMotorB_CutOff = _Channels.CutOff[_MotorBChannel];
}
void Drive::Init() {
  _PID.SetCV(_Channels.ScaledMin[_MotorAChannel], _Channels.ScaledMax[_MotorAChannel]);
  _PID.SetGain(.50, 0, 0);
}
bool Drive::StartCalibration() {  // 3 second button press or more
  _Calibrate.ButtonState = _Channels.Reading[_CalibrateChannel];
  if (_Calibrate.ButtonState == 2000 && _Calibrate.ButtonState != _Calibrate.ButtonPreviousState) {
    _Calibrate.ButtonOnStartTime = millis();
  }
  _Calibrate.ButtonPreviousState = _Calibrate.ButtonState;

  if (_Calibrate.ButtonState == 2000 && (millis() - _Calibrate.ButtonOnStartTime) > 3000) {
    return true;
  } else {
    return false;
  }
}
bool Drive::StopCalibration() {  // 500 ms button press or less
  _Calibrate.ButtonState = _Channels.Reading[_CalibrateChannel];
  if (_Calibrate.ButtonState == 2000 && _Calibrate.ButtonState != _Calibrate.ButtonPreviousState) {
    _Calibrate.ButtonOnStartTime = millis();
    _Calibrate.ButtonPressed = true;
  }
  if (_Calibrate.ButtonState == 1000 && _Calibrate.ButtonState != _Calibrate.ButtonPreviousState) {
    _Calibrate.ButtonOffStartTime = millis();
  }
  _Calibrate.ButtonPreviousState = _Calibrate.ButtonState;
  if ((_Calibrate.ButtonOffStartTime - _Calibrate.ButtonOnStartTime) <= 500 && _Calibrate.ButtonPressed) {
    _Calibrate.ButtonPressed = false;
    return true;
  } else {
    return false;
  }
}
bool Drive::Calibrated() {
  _Channels.OverrideReading(_MotorAChannel, 1750);
  _Channels.OverrideReading(_MotorBChannel, 1750);
  Drive::TankDrive(0);
  if ((millis() - _Calibrate.CheckTimePrevious) > 1000) {
    _Calibrate.CheckTimePrevious = millis();
    pv = _myIMU.GyroX();  // CHECK FOR CORRECT ONE
    PID_Output = _PID.Output(pv);
    if (pv > pvMin && pv < pvMax) {
      _Calibrated = true;
    } else if (pv < pvMin) {
      _Channels.ScaleFactor[_MotorAChannel] = _Channels.ScaleFactor[_MotorAChannel] - abs(PID_Output);  // CHECK IF THIS IS CORRECT
    } else if (pv > pvMax) {
      _Channels.ScaleFactor[_MotorBChannel] = _Channels.ScaleFactor[_MotorBChannel] - abs(PID_Output);  // CHECK IF THIS IS CORRECT
    }
  }
  Serial.print("Calibrating:");
  Serial.print("  Channel A Scale Factor: ");
  Serial.print(_Channels.ScaleFactor[_MotorAChannel]);
  Serial.print("  Channel B Scale Factor: ");
  Serial.println(_Channels.ScaleFactor[_MotorBChannel]);
  return _Calibrated;
}
void Drive::ResetCalibration() {  // Set process variable minimum and maximum for PID loop based on current orientation
  bandwidth = 10;
  sp = _myIMU.GyroX();           // CHECK FOR CORRECT ONE
  pvMin = sp - (bandwidth / 2);  // CHECK MATH
  pvMax = sp + (bandwidth / 2);
  _PID.SetSP(sp);
  _PID.SetPV(pvMin, pvMax);
  _PID.ResetIntegral();
  _Calibrated = false;
  _Calibrate.CheckTimePrevious = millis();
  Serial.print("Reset calibration");
}
void Drive::TankDrive(int Direction) {
  if (Direction == 0) {
    _DriveMotorA.Output(_Channels.Map(_MotorAChannel) * _Channels.ScaleFactor[_MotorAChannel], "A");
    _DriveMotorB.Output(_Channels.Map(_MotorBChannel) * _Channels.ScaleFactor[_MotorBChannel], "B");
  }
  else { // Change logic to flip controls
    _DriveMotorA.Output(_Channels.Map(_MotorAChannel) * _Channels.ScaleFactor[_MotorAChannel], "A");
    _DriveMotorB.Output(_Channels.Map(_MotorBChannel) * _Channels.ScaleFactor[_MotorBChannel], "B");
  }
}
void Drive::ArcadeDrive(int Direction) {
  _Ch0 = _Channels.Map(0);
  _Ch1 = _Channels.Map(1);
  // Left motor = Y + X
  // Right motor = Y - X  
  _ArcadeDriveLeft = map(_Ch1, 1000, 2000, -500, 500) + map(_Ch0, 1000, 2000, -500, 500);
  _ArcadeDriveLeft = map(_ArcadeDriveLeft, -500, 500, 1000, 2000);
  _ArcadeDriveRight = map(_Ch1, 1000, 2000, -500, 500) - map(_Ch0, 1000, 2000, -500, 500);
  _ArcadeDriveRight = map(_ArcadeDriveRight, -500, 500, 1000, 2000);  
  _ArcadeDriveLeftFlipped = map(_ArcadeDriveLeft, 1000, 2000, 2000, 1000);
  _ArcadeDriveRightFlipped = map(_ArcadeDriveRight, 1000, 2000, 2000, 1000);
  if (Direction == 0) {    
    _DriveMotorA.Output(_ArcadeDriveRight * _Channels.ScaleFactor[_MotorAChannel], "A"); 
    _DriveMotorB.Output(_ArcadeDriveLeft * _Channels.ScaleFactor[_MotorBChannel], "B"); 
  }
  else { 
    _DriveMotorA.Output(_ArcadeDriveLeftFlipped * _Channels.ScaleFactor[_MotorAChannel], "A"); 
    _DriveMotorB.Output(_ArcadeDriveRightFlipped * _Channels.ScaleFactor[_MotorBChannel], "B");     
  }
}

