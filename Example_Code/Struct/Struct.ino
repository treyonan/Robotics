struct RGB {
  byte r;
  byte g;
  byte b;
};

RGB colorReturned;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  colorReturned = getBlue();
  displayRGB(colorReturned);

}

RGB getBlue() {
  RGB color = {0,0,255};
  return color;
}

void displayRGB(RGB color) {
  Serial.println(color.b);
  delay(1000);
}
