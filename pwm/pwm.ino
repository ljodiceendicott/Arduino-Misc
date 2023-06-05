int LEDPin = 8;
int PBPin= 5;

byte lastButtonState = LOW; // byte stores an 8-bit unsigned number from 0 to 255
byte ledState = LOW;

void setup() {
  pinMode(LEDPin, OUTPUT);
  pinMode(PBPin, INPUT);
}

void loop() {
  byte buttonState = digitalRead(PBPin);
  if (buttonState != lastButtonState) {
  lastButtonState = buttonState;
  if (buttonState == LOW) {
    ledState = (ledState == HIGH) ? LOW: HIGH;
    digitalWrite(LEDPin, ledState);
  }
  }
}
