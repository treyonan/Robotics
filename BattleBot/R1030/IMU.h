#ifndef IMU_h
#define IMU_h
#include <Arduino.h>
#include "LSM6DS3.h"
#include "Wire.h"

class IMU : public LSM6DS3 {
private:
  float _AvgWeight;
  float _InputWeight;
  float _GyroX;
  float _GyroY;
  float _GyroZ;
  float _AccelX;
  float _AccelY;
  float _AccelZ;
  long _PreviousCheckTime;
public:
  IMU(uint8_t busType, uint8_t address);
  float GyroX();
  float GyroY();
  float GyroZ();
  float AccelX();
  float AccelY();
  float AccelZ();
  void printGyroInfo();
  void printAccelInfo();
  bool IsUpsideDown();  
  class IMU_Orientation {
  private:
    bool state;
    bool lastState = false;
    bool value = false;
    unsigned long debounceDelay;
    unsigned long lastDebounceTime = 0;
  public:
    bool Orientation(float Reading, float MinThreshold, float MaxThreshold, int Delay);
  };
  class Average {
  private:
    float _AVG;
    float _InputRaw;
    float _AvgWeight;
    float _InputWeight;
  public:    
    void SetWeights(float AvgWeight, float InputWeight);
    float AVG(float InputRaw);
  };  
  IMU_Orientation Z;
  Average AverageGyroX;
  Average AverageGyroY;
  Average AverageGyroZ;
  Average AverageAccelX;
  Average AverageAccelY;
  Average AverageAccelZ;
};

#endif