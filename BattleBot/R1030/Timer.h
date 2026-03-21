#ifndef Timer_h
#define Timer_h
#include <Arduino.h>

class Timer {
  private:            
    bool _DN;
    long _CT;
    long _RES_Time;
  public:     
    bool EN;
    int ET;
    int PT;       
    void Init(int PT);
    bool DN();
    void RES();
};

#endif