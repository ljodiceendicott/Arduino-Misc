int PushButton = 7; // create global variable for Pushbutton (PB)

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // set your pushbutton's pin as an input:
  pinMode(PushButton, INPUT);
}

void loop() {
  // read the input pin:
  int buttonState = digitalRead(PushButton);
  // print out the state of the button:
  Serial.println(buttonState);
  delay(1);      // delay in between reads for stability
}
