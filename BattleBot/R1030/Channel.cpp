#include "Channel.h"

Channel::Channel()
  : IBusBM() {
}
void Channel::ReadChannels() {
  Channel::loop();
  for (int i = 0; i < 8; i++) {
    Reading[i] = Channel::readChannel(i);
  }
}
void Channel::Init(int channelNum, float _RawMin, float _RawMax, float _ScaledMin, float _ScaledMax, float _CutOff, float _ScaleFactor) {
  RawMin[channelNum] = _RawMin;
  RawMax[channelNum] = _RawMax;
  ScaledMin[channelNum] = _ScaledMin;
  ScaledMax[channelNum] = _ScaledMax;
  CutOff[channelNum] = _CutOff;  
  ScaleFactor[channelNum] = _ScaleFactor;  
}
void Channel::OverrideReading(int channelNum, int value) {
  Reading[channelNum] = value;
}
float Channel::Map(int channelNum) {
  Output[channelNum] = constrain(map(Reading[channelNum], RawMin[channelNum], RawMax[channelNum], ScaledMin[channelNum], ScaledMax[channelNum]), 1000, 2000);
  if (Output[channelNum] > (CutOff[channelNum] - 50) && Output[channelNum] < (CutOff[channelNum] + 50)) {
    Output[channelNum] = CutOff[channelNum];
    return Output[channelNum];
  }
  return Output[channelNum];
}
