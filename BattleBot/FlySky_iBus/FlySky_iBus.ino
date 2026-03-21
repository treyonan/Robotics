#include <IBusBM.h>
IBusBM IBUS;

void setup() {
  Serial.begin(115200);
  IBUS.begin(Serial1, IBUSBM_NOTIMER);
}

void loop() {
  IBUS.loop();   
  for (int i=1; i<7 ; i++) {
    Serial.print("Ch");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(IBUS.readChannel(i));
    Serial.print(" - ");
  }
 Serial.println();
 delay(50);
}