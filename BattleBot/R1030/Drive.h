#ifndef Drive_h
#define Drive_h
#include "Channel.h"
#include "Motor.h"
#include "IMU.h"
#include "PID.h"

class Drive {
private:
  struct Calibrate {
    int ButtonState;
    int ButtonPreviousState;    
    unsigned long ButtonOnStartTime;
    unsigned long ButtonOffStartTime;
    bool ButtonPressed;   
    long CheckTimePrevious;     
  };
  Calibrate _Calibrate;
  Motor& _DriveMotorA;
  Motor& _DriveMotorB;
  int _MotorAChannel;
  int _MotorBChannel;
  int _CalibrateChannel;
  Channel& _Channels;
  IMU& _myIMU;
  float _DriveMotorA_CutOff;
  float _DriveMotorB_CutOff;
  bool _Calibrated;
  PID _PID;
  float PID_Output;
  float bandwidth;
  float sp;
  float pv;
  float pvMin;
  float pvMax;
  int _Ch0;
  int _Ch1;
  int _ArcadeDriveLeft;
  int _ArcadeDriveRight; 
  int _ArcadeDriveLeftFlipped;
  int _ArcadeDriveRightFlipped; 
public:
  Drive(Motor& DriveMotorA, Motor& DriveMotorB, int MotorAChannel, int MotorBChannel, int CalibrateChannel, Channel& Channels, IMU& myIMU);
  void Init();
  bool StartCalibration();
  bool StopCalibration();
  bool Calibrated();
  void ResetCalibration();
  void TankDrive(int Direction);
  void ArcadeDrive(int Direction);  
};

#endif