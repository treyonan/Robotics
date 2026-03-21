// Edited by Trey O'Nan - Revision 1.4
// 1.0 - Added functions to convert RC signal to PWM for DC motor outputs
// 1.4 - Added 2 more DC motor channels
//
// MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
//

// include the pinchangeint library - see the links in the related topics section above for details
#include <PinChangeInt.h>


#include <Servo.h>

// RC channel in pins
#define CH1_IN_PIN 7 // Aileron. Right joystick, left/right
#define CH2_IN_PIN 8 // Elevator. Right joystick, up/down
#define CH3_IN_PIN 12 // Throttle. Left joystick, up/down
#define CH4_IN_PIN 13 // Aux

// Servo motor channel out pins. If more than 2 servos are needed, you will have to use one of the DC motor pins
#define CH1_OUT_PIN 10
#define CH2_OUT_PIN 11

// DC Motor driver pins
const int AIN1_PIN = A0;
const int AIN2_PIN = A1;
const int APWM_PIN = 3;
const int BIN1_PIN = A2;
const int BIN2_PIN = A3;
const int BPWM_PIN = 5;
const int CIN1_PIN = A4;
const int CIN2_PIN = A5;
const int CPWM_PIN = 6;
const int DIN1_PIN = 2;
const int DIN2_PIN = 4;
const int DPWM_PIN = 9;

// Servo objects generate the signals expected by Electronic Speed Controllers and Servos
// We will use the objects to output the signals we read in
// this example code provides a straight pass through of the signal with no custom processing
Servo servoCH1;
Servo servoCH2;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define CH1_FLAG 1
#define CH2_FLAG 2
#define CH3_FLAG 4
#define CH4_FLAG 8

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unCH1InShared;
volatile uint16_t unCH2InShared;
volatile uint16_t unCH3InShared;
volatile uint16_t unCH4InShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulCH1Start;
uint32_t ulCH2Start;
uint32_t ulCH3Start;
uint32_t ulCH4Start;

bool printRadioPulse = 0;

void setup()
{
  Serial.begin(9600);
 
  Serial.println("multiChannels");

  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers 
  //servoCH1.attach(CH1_OUT_PIN);
  //servoCH2.attach(CH2_OUT_PIN);    

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(CH1_IN_PIN, calcCH1,CHANGE);
  PCintPort::attachInterrupt(CH2_IN_PIN, calcCH2,CHANGE);
  PCintPort::attachInterrupt(CH3_IN_PIN, calcCH3,CHANGE);
	PCintPort::attachInterrupt(CH4_IN_PIN, calcCH4,CHANGE);
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unCH1In;
  static uint16_t unCH2In;
  static uint16_t unCH3In;
	static uint16_t unCH4In;
  
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
        
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
   
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
   
    if(bUpdateFlags & CH1_FLAG)
    {
      unCH1In = unCH1InShared;
    }
   
    if(bUpdateFlags & CH2_FLAG)
    {
      unCH2In = unCH2InShared;
    }
   
    if(bUpdateFlags & CH3_FLAG)
    {
      unCH3In = unCH3InShared;
    }

		if(bUpdateFlags & CH4_FLAG)
    {
      unCH4In = unCH4InShared;
    }
    
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;   
        
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }
 
  // do any processing from here onwards
  // only use the local values unCH1In, unCH3In and unCH2In, the shared
  // variables unCH1InShared, unCH3InShared, unCH2InShared are always owned by
  // the interrupt routines and should not be used in loop

  // Serial.println(unCH1In);  // For debugging channel Min and Max)

  // Convert to PWM value (-255 to 255)
  int PWM_1 = pulseToPWM(unCH1In, 980, 1964, -255, 255, 100); // Aileron. Right joystick, left/right
  int PWM_2 = pulseToPWM(unCH2In, 1000, 2000, -255, 255, 100); // Elevator. Right joystick, up/down
  int PWM_3 = pulseToPWM(unCH3In, 1004, 1992, -255, 255, 100); // Throttle. Left joystick, up/down
	int PWM_4 = pulseToPWM(unCH4In, 980, 1984, 0, 255, 0); // Aux. Currently set to VARA

  if (printRadioPulse == 1) {
    Serial.print(unCH1In);
    Serial.print(", ");
    Serial.print(unCH3In);
    Serial.print(", ");
    Serial.println(unCH4In);
  }
  
  int left;
  int right;
  
  unsigned long currentMillis;
  unsigned long timeDelay = 50;

  bool arcadeDrive = 1;

  if (arcadeDrive) {
    left = PWM_3 + PWM_1;
    right = PWM_3 - PWM_1;   
  }
  else {
    left = PWM_3;
    right = PWM_2;   
  }

  motorC(PWM_4);
  drive(left, right); // Drive motor
   
 
  // the following code provides simple pass through
  // this is a good initial test, the Arduino will pass through
  // receiver input as if the Arduino is not there.
  // This should be used to confirm the circuit and power
  // before attempting any custom processing in a project.
 
  // we are checking to see if the channel value has changed, this is indicated 
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.  
 
  if(bUpdateFlags & CH1_FLAG)
  {
    if(servoCH1.readMicroseconds() != unCH1In)
    {
      servoCH1.writeMicroseconds(unCH1In);
    }
  }	

	if(bUpdateFlags & CH2_FLAG)
  {
    if(servoCH2.readMicroseconds() != unCH2In)
    {
      servoCH2.writeMicroseconds(unCH2In);
    }
  }	 
 
  bUpdateFlags = 0;
}

// simple interrupt service routine
void calcCH1()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(CH1_IN_PIN) == HIGH)
  {
    ulCH1Start = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unCH1InShared = (uint16_t)(micros() - ulCH1Start);
    // use set the CH1 flag to indicate that a new CH1 signal has been received
    bUpdateFlagsShared |= CH1_FLAG;

  }
}

void calcCH2()
{
  if(digitalRead(CH2_IN_PIN) == HIGH)
  {
    ulCH2Start = micros();
  }
  else
  {
    unCH2InShared = (uint16_t)(micros() - ulCH2Start);
    bUpdateFlagsShared |= CH2_FLAG;

  }
}

void calcCH3()
{
  if(digitalRead(CH3_IN_PIN) == HIGH)
  {
    ulCH3Start = micros();
  }
  else
  {
    unCH3InShared = (uint16_t)(micros() - ulCH3Start);
    bUpdateFlagsShared |= CH3_FLAG;

  }
}

void calcCH4()
{
  if(digitalRead(CH4_IN_PIN) == HIGH)
  {
    ulCH4Start = micros();
  }
  else
  {
    unCH4InShared = (uint16_t)(micros() - ulCH4Start);
    bUpdateFlagsShared |= CH4_FLAG;

  }
}

// Convert RC pulse value to motor PWM value
int pulseToPWM(int pulse, int rawMin, int rawMax, int scaledMin, int scaledMax, int deadzone) {
  
  // If we're receiving numbers, convert them to motor PWM
  if ( pulse >= (rawMin - 20) && pulse <= (rawMax + 20)) {   
    pulse = map(pulse, rawMin, rawMax, (scaledMin-10), (scaledMax+10));   
    pulse = constrain(pulse, scaledMin, scaledMax);    
  } else {
    pulse = 0;
  }

  // Anything in deadzone should stop the motor
  if ( abs(pulse) <= deadzone) {
    pulse = 0;
  }

  return pulse;
}

// Positive for forward, negative for reverse
void drive(int speed_a, int speed_b) {

  // Limit speed between -255 and 255
  speed_a = constrain(speed_a, -255, 255);
  speed_b = constrain(speed_b, -255, 255);

  // Set direction for motor A
  if ( speed_a == 0 ) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, LOW);
  } else if ( speed_a > 0 ) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
  }

  // Set direction for motor B
  if ( speed_b == 0 ) {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, LOW);
  } else if ( speed_b > 0 ) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  }

  // Set speed
  analogWrite(APWM_PIN, abs(speed_a));
  analogWrite(BPWM_PIN, abs(speed_b)); 

  if (printRadioPulse == 0) {
    Serial.print("Motor A: ");
    Serial.print(speed_a);
    Serial.print(",  Motor B: ");
    Serial.println(speed_b);    
  }
  
   
}

void motorC(int speed_c) {

  // Limit speed between -255 and 255
  speed_c = constrain(speed_c, 0, 255  ); 

  // Set direction for motor C
  if ( speed_c <= 0 ) {
    digitalWrite(CIN1_PIN, LOW);
    digitalWrite(CIN2_PIN, LOW);
    //speed_c = 0;  
  } else {
    digitalWrite(CIN1_PIN, HIGH);
    digitalWrite(CIN2_PIN, LOW);
  }  

  // Set speed
  analogWrite(CPWM_PIN, abs(speed_c));   

  if (printRadioPulse == 0) {
    Serial.print("Motor C: ");
    Serial.println(speed_c);     
  }
  
   
}
