#include "Timer.h"

void Timer::Init(int _PT) {
  PT = _PT; 
  Timer::RES();  
}
bool Timer::DN() {
  if (!EN) {    
    Timer::RES();
  }
  else {    
    _CT = millis();
    ET = _CT - _RES_Time;    
    _DN = (ET >= PT)? true:false;
  }
  return _DN;
}
void Timer::RES() {
  _RES_Time = millis();
  _DN = false;  
  ET = 0;  
}
