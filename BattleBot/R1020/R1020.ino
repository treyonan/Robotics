/* Demonstrates one way to read signal data from a
   radio control receiver. We use the EnableInterrupt
   library.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY
   OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
   TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
   PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
   THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
   DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
   CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
   OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
   OTHER DEALINGS IN THE SOFTWARE.
*/
#include <EnableInterrupt.h>
// aileron
volatile int rc_aileron_pulsewidth_in = -100;
volatile long rc_aileron_pulse_start;
// The pin ID connected to the aileron signal wire. Aileron is right joystick, left/right
#define rc_aileron_pin_in 7
// Elevator
volatile int rc_elevator_pulsewidth_in = -100;
volatile long rc_elevator_pulse_start;
// The pin ID connected to the elevator input signal wire. Elevator is right joystick, up/down
#define rc_elevator_pin_in 8
// Throttle
volatile int rc_throttle_pulsewidth_in = -100;
volatile long rc_throttle_pulse_start;
// The pin ID connected to the throttle input signal wire. Throttle is left joystick, up/down
#define rc_throttle_pin_in 12
// Aux_1
volatile int rc_aux_1_pulsewidth_in = -100;
volatile long rc_aux_1_pulse_start;
// The pin ID connected to the aux 1 input signal wire.
#define rc_aux_1_pin_in 13
// Aux_2
volatile int rc_aux_2_pulsewidth_in = -100;
volatile long rc_aux_2_pulse_start;
// The pin ID connected to the aux 2 input signal wire.
#define rc_aux_2_pin_in 10

// DC Motor driver pins
const int BIN1_PIN = A0;
const int BIN2_PIN = A1;
const int BPWM_PIN = 3;
const int AIN1_PIN = A2;
const int AIN2_PIN = A3;
const int APWM_PIN = 5;
const int CIN2_PIN = A4;
const int CIN1_PIN = A5;
const int CPWM_PIN = 6;
const int DIN1_PIN = 2;
const int DIN2_PIN = 4;
const int DPWM_PIN = 9;

//PWM signal to DC motors
int PWM_1;
int PWM_2;
int PWM_3;
int PWM_4;

//Variables for drive control
int left;
int right;
bool arcadeDrive = 1;

// Timing
long time_start;

// This item toggles debugging behaviour on or off.
// Comment the item out to disable debugging messages.
#define debug
// Set up debugging behaviours
#ifdef debug
#define debug_print(x) Serial.print (x)
#define debug_println(x) Serial.println (x)
#else
#define debug_print(x)
#define debug_println(x)
#endif

int Ch1_Reading;
int Ch2_Reading;
int Ch3_Reading;
int Ch4_Reading;
int Ch5_Reading;

int direction;

bool debugMode = 0;

void setup() {
  // Set the pin modes
  pinMode(rc_aileron_pin_in, INPUT);
  pinMode(rc_elevator_pin_in, INPUT);
  pinMode(rc_throttle_pin_in, INPUT);
  pinMode(rc_aux_1_pin_in, INPUT);
  pinMode(rc_aux_2_pin_in, INPUT);
  // Set the initial interrupts
  enableInterrupt(rc_aileron_pin_in, isrAileronRising, RISING);
  enableInterrupt(rc_elevator_pin_in, isrElevatorRising, RISING);
  enableInterrupt(rc_throttle_pin_in, isrThrottleRising, RISING);
  enableInterrupt(rc_aux_1_pin_in, isrAux_1Rising, RISING);
  enableInterrupt(rc_aux_2_pin_in, isrAux_2Rising, RISING);
  // Note the start time
  time_start = micros();
#ifdef debug
  // Start serial communication so we can see the results
  Serial.begin(57600);
#endif  

}

void loop() {
  static long counter = 0;
  long time_loop_start;
  float cps;
  static long time_of_last_message = micros();
  time_loop_start = micros();

  counter++;

  // Convert to PWM value (-255 to 255)
  PWM_1 = pulseToPWM(rc_aileron_pulsewidth_in, 980, 1964, -255, 255, 100); // Aileron. Right joystick, left/right
  PWM_2 = pulseToPWM(rc_elevator_pulsewidth_in, 1000, 2000, -255, 255, 100); // Elevator. Right joystick, up/down
  PWM_3 = pulseToPWM(rc_throttle_pulsewidth_in, 1004, 1992, -255, 255, 100); // Throttle. Left joystick, up/down
  PWM_4 = pulseToPWM(rc_aux_1_pulsewidth_in, 1004, 1992, -255, 255, 100); // Aux 1  

  Ch5_Reading = rc_aux_2_pulsewidth_in;  

  if (arcadeDrive) {
    left = (PWM_3 + PWM_1);
    right = PWM_3 - PWM_1;
  }
  else {
    left = PWM_3;
    right = PWM_2;
  }

  if (Ch5_Reading <= 1500) {
    direction = 0;
  }
  else {
    direction = 1;
  }

  motorC(PWM_4);
  drive(left, right, direction); // Drive motor


#ifdef debug
  // Print some details, but not every iteration.
  // Just every half-second or so.
  if (time_loop_start - time_of_last_message > 500000) {
    if (debugMode) {
      Ch1_Reading = rc_aileron_pulsewidth_in;
      Ch2_Reading = rc_elevator_pulsewidth_in;
      Ch3_Reading = rc_throttle_pulsewidth_in;
      Ch4_Reading = rc_aux_1_pulsewidth_in;
      Ch5_Reading;
      
    }
    else {
      Ch1_Reading = left;
      Ch2_Reading = right;
      Ch3_Reading = PWM_3;
      Ch4_Reading = PWM_4;
      Ch5_Reading;
    }
    debug_print("Counter: ");
    debug_print(counter);
    debug_print("\tAileron: ");
    debug_print(Ch1_Reading);
    debug_print("\tElevator: ");
    debug_print(Ch2_Reading);
    debug_print("\tThrottle: ");
    debug_print(Ch3_Reading);
    debug_print("\tAux: ");
    debug_print(Ch4_Reading);
    debug_print("\tAux 2: ");
    debug_print(Ch5_Reading);
    debug_print("\tCycles/sec: ");
    cps = (1000000.0 * counter) /
          (time_loop_start - time_start);
    debug_println(cps);
    time_of_last_message = micros();
  }
#endif

}
