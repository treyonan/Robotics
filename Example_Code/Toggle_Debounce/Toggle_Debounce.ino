const int buttonPin = 2;  // the pin that the pushbutton is attached to
const int ledPin = 13;    // the pin that the LED is attached to

int buttonState = 0;      // variable for reading the button status
int lastButtonState = 0;  // variable to store the previous button state
int ledState = LOW;       // variable to store the current LED state
unsigned long debounceDelay = 50; // debounce delay time in milliseconds
unsigned long lastDebounceTime = 0; // last time the button was pressed

void setup() {
  pinMode(ledPin, OUTPUT);      // initialize the LED pin as an output
  pinMode(buttonPin, INPUT);    // initialize the button pin as an input
}

void loop() {
  // read the state of the button and debounce it
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) { // toggle the LED state
        ledState = !ledState;
      }
    }
  }
  
  digitalWrite(ledPin, ledState);       // set the LED state
  lastButtonState = reading;             // store the previous button state
}
