int ledPin = 9;

int val;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(A0, INPUT);
}

void loop() {
  Serial.begin(9600);
 for (val = 0; val < 100; val++){
  analogWrite(ledPin, val);
  delay(50);
  }
  for (val=100; val>0; val--){
  analogWrite(ledPin, val);
  delay(50);
  }
}
