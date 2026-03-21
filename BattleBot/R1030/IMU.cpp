#include "IMU.h"

IMU::IMU(uint8_t busType, uint8_t address)
  : LSM6DS3(busType, address) { 
    _AvgWeight = .999;
    _InputWeight = (1.000 - _AvgWeight);
    AverageGyroX.SetWeights(_AvgWeight, _InputWeight); 
    AverageGyroY.SetWeights(_AvgWeight, _InputWeight);   
    AverageGyroZ.SetWeights(_AvgWeight, _InputWeight);   
    AverageAccelX.SetWeights(_AvgWeight, _InputWeight);   
    AverageAccelY.SetWeights(_AvgWeight, _InputWeight);   
    AverageAccelZ.SetWeights(_AvgWeight, _InputWeight);   
    _PreviousCheckTime = millis();  
}
float IMU::GyroX() {  
  return AverageGyroX.AVG(readFloatGyroX());  
}
float IMU::GyroY() {
  return AverageGyroY.AVG(readFloatGyroY());   
}
float IMU::GyroZ() {
  return AverageGyroZ.AVG(readFloatGyroZ());    
}
float IMU::AccelX() {
  return AverageAccelX.AVG(readFloatAccelX());  
}
float IMU::AccelY() {
  return AverageAccelY.AVG(readFloatAccelY());    
}
float IMU::AccelZ() {
  return AverageAccelZ.AVG(readFloatAccelZ());   
}
void IMU::printGyroInfo() {
  _GyroX = GyroX();
  _GyroY = GyroY();
  _GyroZ = GyroZ();
  if ((millis() - _PreviousCheckTime) > 1000) {
    Serial.print(" GX1 = ");
    Serial.print(_GyroX, 4);
    Serial.print(" GY1 = ");
    Serial.print(_GyroY, 4);
    Serial.print(" GZ1 = ");
    Serial.println(_GyroZ, 4);
    _PreviousCheckTime = millis();
  }
}
void IMU::printAccelInfo() {
  _AccelX = AccelX();
  _AccelY = AccelY();
  _AccelZ = AccelZ();
  if ((millis() - _PreviousCheckTime) > 1000) {
    Serial.print(" AX1 = ");
    Serial.print(_AccelX, 4);
    Serial.print(" AY1 = ");
    Serial.print(_AccelY, 4);
    Serial.print(" AZ1 = ");
    Serial.println(_AccelZ, 4);
    _PreviousCheckTime = millis();
  }
}
bool IMU::IMU_Orientation::Orientation(float Reading, float MinThreshold, float MaxThreshold, int Delay) {
  value = false;
  if (Reading >= MinThreshold && Reading <= MaxThreshold) {
    state = true;
  } else {
    state = false;
  }
  if (state != lastState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > Delay) {
    value = state;
  }
  lastState = state;
  return value;
}
void IMU::Average::SetWeights(float AvgWeight, float InputWeight) {
  _AVG = 0;
  _AvgWeight = AvgWeight;
  _InputWeight = InputWeight;
}
float IMU::Average::AVG(float InputRaw) {
  _AVG = (_AVG * _AvgWeight) + (InputRaw * _InputWeight);
  return _AVG;
}
bool IMU::IsUpsideDown() {
  float Reading = IMU::AccelZ();
  return Z.Orientation(Reading, -1.25, -0.65, 500);
}
