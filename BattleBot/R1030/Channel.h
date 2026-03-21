#ifndef Channel_h
#define Channel_h
#include <Arduino.h>
#include <IBusBM.h>

class Channel : public IBusBM {
public:
  Channel();
  void ReadChannels();
  void Init(int channelNum, float _RawMin, float _RawMax, float _ScaledMin, float _ScaledMax, float _CutOff, float _ScaleFactor);
  void OverrideReading(int channelNum, int value);
  float Map(int channelNum);
  float Reading[8];
  float RawMin[8];
  float RawMax[8];
  float ScaledMin[8];
  float ScaledMax[8];
  float CutOff[8];
  float Output[8];
  float ScaleFactor[8];
};

#endif