/* This function is called by an interrupt event when
   the signal voltage on the aileron channel falls.
   'isr' denotes Interrupt Sub-Routine.
*/
void isrAileronFalling() {
  rc_aileron_pulsewidth_in = micros() - rc_aileron_pulse_start;
  disableInterrupt(rc_aileron_pin_in);
  enableInterrupt(rc_aileron_pin_in, isrAileronRising, RISING);
}
/* This function is called by an interrupt event when
   the signal voltage on the aileron channel rises.
   'isr' denotes Interrupt Sub-Routine.
*/
void isrAileronRising() {
  rc_aileron_pulse_start = micros();
  disableInterrupt(rc_aileron_pin_in);
  enableInterrupt(rc_aileron_pin_in, isrAileronFalling, FALLING);
}
/* This function is called by an interrupt event when
   the signal voltage on the elevator channel falls.
   'isr' denotes Interrupt Sub-Routine.
*/
void isrElevatorFalling() {
  rc_elevator_pulsewidth_in = micros() - rc_elevator_pulse_start;
  disableInterrupt(rc_elevator_pin_in);
  enableInterrupt(rc_elevator_pin_in, isrElevatorRising, RISING);
}
/* This function is called by an interrupt event when
   the signal voltage on the elevator channel rises.
   'isr' denotes Interrupt Sub-Routine.
*/
void isrElevatorRising() {
  rc_elevator_pulse_start = micros();
  disableInterrupt(rc_elevator_pin_in);
  enableInterrupt(rc_elevator_pin_in, isrElevatorFalling, FALLING);
}
/* This function is called by an interrupt event when
   the signal voltage on the throttle channel falls.
   'isr' denotes Interrupt Sub-Routine.
*/
void isrThrottleFalling() {
  rc_throttle_pulsewidth_in = micros() - rc_throttle_pulse_start;
  disableInterrupt(rc_throttle_pin_in);
  enableInterrupt(rc_throttle_pin_in, isrThrottleRising, RISING);
}
/* This function is called by an interrupt event when
   the signal voltage on the throttle channel rises.
   'isr' denotes Interrupt Sub-Routine.
*/
void isrThrottleRising() {
  rc_throttle_pulse_start = micros();
  disableInterrupt(rc_throttle_pin_in);
  enableInterrupt(rc_throttle_pin_in, isrThrottleFalling, FALLING);
}

void isrAux_1Falling() {
  rc_aux_1_pulsewidth_in = micros() - rc_aux_1_pulse_start;
  disableInterrupt(rc_aux_1_pin_in);
  enableInterrupt(rc_aux_1_pin_in, isrAux_1Rising, RISING);
}
/* This function is called by an interrupt event when
   the signal voltage on the Aux 1 channel rises.
   'isr' denotes Interrupt Sub-Routine.
*/
void isrAux_1Rising() {
  rc_aux_1_pulse_start = micros();
  disableInterrupt(rc_aux_1_pin_in);
  enableInterrupt(rc_aux_1_pin_in, isrAux_1Falling, FALLING);
}

void isrAux_2Falling() {
  rc_aux_2_pulsewidth_in = micros() - rc_aux_2_pulse_start;
  disableInterrupt(rc_aux_2_pin_in);
  enableInterrupt(rc_aux_2_pin_in, isrAux_2Rising, RISING);
}
/* This function is called by an interrupt event when
   the signal voltage on the Aux 2 channel rises.
   'isr' denotes Interrupt Sub-Routine.
*/
void isrAux_2Rising() {
  rc_aux_2_pulse_start = micros();
  disableInterrupt(rc_aux_2_pin_in);
  enableInterrupt(rc_aux_2_pin_in, isrAux_2Falling, FALLING);
}
