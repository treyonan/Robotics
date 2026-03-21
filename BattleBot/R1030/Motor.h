#ifndef Motor_h
#define Motor_h
#include <Arduino.h>
#include <Servo.h>
#include "Timer.h"

class Motor : public Servo {
private:
  int _Output;   
  float _RampLevel;    
  String _MotorName;
  enum State {
    STOPPED,
    RAMP_UP,
    RAMP_DOWN,
    RUNNING
  };
  State _State = STOPPED;   
  int _ScaledLow;
  int _ScaledHigh;   
public:  
  void Output(int signalLevel, String MotorName);
  void RampMotor(String Direction, int PT, String MotorName);    
  void PrintMotorInfo();  
  bool StartFlag = false;   
  Timer RampTimer;  
};

#endif