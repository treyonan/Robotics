#include "PID.h"

PID::PID() {
}
void PID::SetCV(float cvMin, float cvMax) {
  _cvMin = cvMin;
  _cvMax = cvMax;
}
void PID::SetPV(float pvMin, float pvMax) {
  _pvMin = pvMin;
  _pvMax = pvMax;
}
void PID::SetSP(float sp) {
  _sp = sp;
}

void PID::SetGain(float Kp, float Ki, float Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}
void PID::ResetIntegral() {
  _integral = 0;
  _lasterror = 0;
}
float PID::Output(float pv) {
  _pv = pv;
  _error = _sp - _pv;
  _proportional = _error * _Kp;
  _integral = _integral + (_error * _Ki);
  _derivative = (_error - _lasterror) * _Kd;
  _lasterror = _error;
  _output = _proportional + _integral + _derivative;
  return _output;
}
