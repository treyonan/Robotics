#ifndef PID_h
#define PID_h

class PID {
private:
  float _error;
  float _lasterror;
  float _sp;
  float _pv;
  float _cv;
  float _pvMin;
  float _pvMax;
  float _cvMin;
  float _cvMax;
  float _output;
  float _Kp;
  float _Ki;
  float _Kd;
  float _proportional;  // proportional gain
  float _integral;      // integral gain
  float _derivative;    // derivative gain
public:
  PID();
  void SetCV(float cvMin, float cvMax);
  void SetPV(float pvMin, float pvMax);
  void SetSP(float sp);
  void SetGain(float Kp, float Ki, float Kd);
  void ResetIntegral();
  float Output(float pv);
};

#endif