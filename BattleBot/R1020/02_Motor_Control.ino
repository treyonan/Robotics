// Convert RC pulse value to motor PWM value
int pulseToPWM(int pulse, int rawMin, int rawMax, int scaledMin, int scaledMax, int deadzone) {

  // If we're receiving numbers, convert them to motor PWM
  if ( pulse >= (rawMin - 20) && pulse <= (rawMax + 20)) {
    pulse = map(pulse, rawMin, rawMax, (scaledMin - 10), (scaledMax + 10));
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
void drive(int speed_a, int speed_b, int _direction) {
  
  // Limit speed between -255 and 255
  speed_a = constrain(speed_a, -255, 255);
  speed_b = constrain(speed_b, -255, 255);

  // Set direction for motor A
  if ( speed_a == 0 ) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, LOW);
  } 
  else if ( speed_a > 0 ) {
      if (_direction == 0) {                
        digitalWrite(AIN1_PIN, HIGH);
        digitalWrite(AIN2_PIN, LOW);
      } 
      else {        
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
      }    
  } 
  else {
      if (_direction == 0) {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
      }
      else {
        digitalWrite(AIN1_PIN, HIGH);
        digitalWrite(AIN2_PIN, LOW);
      }      
  }

  // Set direction for motor B
  if ( speed_b == 0 ) {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, LOW);
  } 
  else if ( speed_b > 0 ) {
      if (_direction == 0) {
        digitalWrite(BIN1_PIN, HIGH);
        digitalWrite(BIN2_PIN, LOW);
      }
      else {
        digitalWrite(BIN1_PIN, LOW);
        digitalWrite(BIN2_PIN, HIGH);
      }    
  } 
  else {
      if (_direction == 0) {
        digitalWrite(BIN1_PIN, LOW);
        digitalWrite(BIN2_PIN, HIGH);
      }
      else {
        digitalWrite(BIN1_PIN, HIGH);
        digitalWrite(BIN2_PIN, LOW);
      }    
  } 

  // Set speed
  analogWrite(APWM_PIN, abs(speed_a));
  analogWrite(BPWM_PIN, abs(speed_b));
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
}
