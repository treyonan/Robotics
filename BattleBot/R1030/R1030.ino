#include "Channel.h"
#include "Motor.h"
#include "IMU.h"
#include "Drive.h"
#include "Timer.h"

// This item toggles debugging behaviour on or off.
// Comment the item out to disable debugging messages.
/*
#define debug
// Set up debugging behaviours
#ifdef debug
#define debug_print(x) Serial.print (x)
#define debug_println(x) Serial.println (x)
#else
#define debug_print(x)
#define debug_println(x)
#endif
*/

#define MotorAChannel 1
#define MotorAPin 9
#define MotorBChannel 2
#define MotorBPin 8  // CHECK
#define CalibrateChannel 5
#define WeaponChannel 6
#define WeaponPin 10  // CHECK

Channel Channels;
Motor WeaponMotor;
Motor DriveMotorA;
Motor DriveMotorB;

IMU myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A

Drive Drive(DriveMotorA, DriveMotorB, MotorAChannel, MotorBChannel, CalibrateChannel, Channels, myIMU);

float _Reading[8];
long CalibrateTimeout;
int Direction = 0; // 0 = Right side up, 1 = Upside down. Update with IMU direction
int cnt_rec = 0;
int cnt_rec_previous = -1;
Timer CheckRxTimer;
bool RxState;
int RxCheckCount = 0;
int DisconnectedCount = 0;

void ReadChannels() {
  // Channel 0 - Right joystick left/right, right = 2000, left = 1000
  // Channel 1 - Right joystick up/down, up = 2000, down = 1000
  // Channel 2 - Left joystick up/down, up = 2000, down = 1000
  // Channel 3 - Left joystick left/right, right = 2000, left = 1000
  // Channel 4 - NU
  // Channel 5 - Front button, pressed = 2000, released = 1000
  // Channel 6 - Switch A (Left Switch), up = 1000, middle = cut off value, down = 2000
  // Channel 7 - Switch B (Right Switch), up = 1000, middle = cut off value, down = 2000, Used for FailSafe signal
  Channels.ReadChannels();
  for (int i = 0; i < 8; i++) {
    _Reading[i] = Channels.Map(i);
  }
#ifdef debug
  debug_print(_Reading[0]);
  debug_print("    ");
  debug_print(_Reading[1]);
  debug_print("    ");
  debug_print(_Reading[2]);
  debug_print("    ");
  debug_print(_Reading[3]);
  debug_print("    ");
  debug_print(_Reading[4]);
  debug_print("    ");
  debug_print(_Reading[5]);
  debug_print("    ");
  debug_print(_Reading[6]);
  debug_print("    ");
  debug_println(_Reading[7]);
  delay(100);
#endif
}
void CheckCalibration() {
  if (Drive.StartCalibration()) {
    Drive.ResetCalibration();
    CalibrateTimeout = millis();
    while (true) {
      ReadChannels();
      if (Drive.StopCalibration()) {
        Serial.println("Stop calibration");
        break;
      }
      if (Drive.Calibrated()) {
        Serial.println("Calibrated");
        break;
      }
      if ((millis() - CalibrateTimeout) > 20000) {
        Serial.println("Timed Out");
        break;
      }
    }
  }
}
void RunDriveMotors() {
  if (!myIMU.IsUpsideDown()) {
    //Serial.println("Right Side Up");
    Direction = 0;    
  } else {   
    //Serial.println("Upside Down"); 
    Direction = 1;
  }
  //Drive.TankDrive(Direction);
  Drive.ArcadeDrive(Direction);
}
void RunWeaponMotor() {
  if (_Reading[WeaponChannel] == 2000) {
    WeaponMotor.RampMotor("UP", 3000, "Weapon");    
  }
  else {
    WeaponMotor.RampMotor("DOWN", 1000, "Weapon");    
  }
}
void ArmESC() {
  DriveMotorA.Output(1500, "A");
  DriveMotorB.Output(1500, "B");
  WeaponMotor.Output(1000, "Weapon");
  delay(3000);
}
bool RxDisconnected() {    
  if (CheckRxTimer.DN()) {
    CheckRxTimer.RES();
    cnt_rec = Channels.cnt_rec;  
    if (cnt_rec_previous == cnt_rec) {
      RxCheckCount ++;  
    }
    else {
      RxCheckCount = 0;
    }
    RxState = (RxCheckCount == 5)? true:false;
    cnt_rec_previous = cnt_rec;    
  }
  return RxState;
}
void ShutDownMotors() {
  DriveMotorA.Output(1500, "A");
  DriveMotorB.Output(1500, "B");
  WeaponMotor.Output(1000, "Weapon");
  WeaponMotor.StartFlag = false;
}

void setup() {
  Serial.begin(115200);
  Channels.begin(Serial1, IBUSBM_NOTIMER);
  Channels.Init(0, 1000, 2000, 1000, 2000, 1500, 1.00);
  Channels.Init(1, 1000, 2000, 1000, 2000, 1500, 1.00);
  Channels.Init(2, 1000, 2000, 1000, 2000, 1500, 1.00);
  Channels.Init(3, 1000, 2000, 1000, 2000, 1500, 1.00);
  Channels.Init(4, 1000, 2000, 1000, 2000, 1500, 1.00);
  Channels.Init(5, 1000, 2000, 1000, 2000, 1500, 1.00);
  Channels.Init(6, 1000, 2000, 1000, 2000, 1000, 1.00);
  Channels.Init(7, 1000, 2000, 1000, 2000, 1500, 1.00);

  Drive.Init();
  
  WeaponMotor.RampTimer.Init(3000);
  CheckRxTimer.Init(100);
  CheckRxTimer.EN = true;

  // Attach Motor ESCs
  DriveMotorA.attach(MotorAPin);  
  DriveMotorB.attach(MotorBPin);  
  WeaponMotor.attach(WeaponPin);
  ArmESC();

  //while (!Serial); // Need to comment when connected to external power other than USB-C    
    
  //Call .begin() to configure the IMUs

  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("Device OK!");
  }  
}
void loop() {

  ReadChannels();
  if (_Reading[CalibrateChannel] == 2000) { 
    Serial.println("Ready");   
    while (true) {
      //myIMU.printGyroInfo();
      //myIMU.printAccelInfo();
      //DriveMotorA.PrintMotorInfo();
      //DriveMotorB.PrintMotorInfo();
      //WeaponMotor.PrintMotorInfo();        
      ReadChannels();
      CheckCalibration();
      RunDriveMotors();         
      RunWeaponMotor();  
      if (RxDisconnected()) {
        DisconnectedCount ++;
        Serial.print("Disconnected: ");
        Serial.println(DisconnectedCount);
        DisconnectedCount = 0;
        ShutDownMotors();
        break;
      }      
    }
  }  
}